import time
import threading
import asyncio
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import MarkerArray
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse

from geometry_msgs.msg import Transform

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

from avatar_challenge_msgs.action import Draw
from avatar_challenge_msgs.msg import Point2D
from avatar_challenge.geometry import path_to_markers, transform_vertices

class DrawNode(Node):
    """Node responsible for handling the drawing action, planning Cartesian paths, and visualizing the drawing vertices."""
    def __init__(self):
        super().__init__('draw_node')

        self.declare_parameter('move_group', 'xarm7')
        self.declare_parameter('eef_link', 'link_eef')
        self.declare_parameter('max_step', 0.01)
        self.declare_parameter('jump_threshold', 0.0)

        self.move_group_name = self.get_parameter('move_group').get_parameter_value().string_value
        self.eef_link = self.get_parameter('eef_link').get_parameter_value().string_value
        self.max_step = self.get_parameter('max_step').get_parameter_value().double_value
        self.jump_threshold = self.get_parameter('jump_threshold').get_parameter_value().double_value
        self._drawing_lock = threading.Lock()

        self.cartesian_path_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        self._action_server = ActionServer(
            self,
            Draw,
            'draw',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('draw_node action server ready')
       
        marker_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._marker_pub = self.create_publisher(MarkerArray, 'drawing_markers', marker_qos)

    def goal_callback(self, _goal_request):
        if not self._drawing_lock.acquire(blocking=False):
            self.get_logger().warning('Rejecting draw goal because another drawing is active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.get_logger().info('Draw cancellation requested')
        return CancelResponse.ACCEPT
        
    def ensure_clients(self, timeout_sec: float = 5.0) -> bool:
        """Ensure that the required ROS 2 service and action server are available.
        
        Args:
            timeout_sec (float): The maximum time to wait for the service and action server to become available, in seconds.
        
        Returns:
            bool: True if both the service and action server are available, False otherwise.
        """
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('compute_cartesian_path service not available')
            return False
        if not self.execute_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('execute_trajectory action server not available')
            return False
        return True

    async def plan_cartesian_path(
        self,
        world_T_drawing_tf: Transform,
        drawing_p_vertices_2d: list[Point2D],
    ) -> GetCartesianPath.Response:
        """Plan a Cartesian path for the drawing action.

        This method transforms the 2D drawing vertices into 3D world coordinates,
        publishes them as markers for visualization, and then requests a Cartesian
        path from the MoveIt service based on these waypoints.

        Args:
            world_T_drawing_tf (Transform): The transform from the drawing frame to the world frame.
            drawing_p_vertices_2d (list[Point2D]): The 2D vertices of the drawing shape.

        Returns:
            GetCartesianPath.Response: The response from the compute_cartesian_path service.
        """
        world_p_vertices = transform_vertices(world_T_drawing_tf, drawing_p_vertices_2d)
        self._marker_pub.publish(path_to_markers(world_p_vertices, frame_id='world', timestamp=self.get_clock().now().to_msg()))
        
        path_req = GetCartesianPath.Request()
        path_req.header.frame_id = 'world'
        path_req.start_state.is_diff = True
        path_req.group_name = self.move_group_name
        path_req.link_name = self.eef_link
        path_req.waypoints = world_p_vertices
        path_req.max_step = self.max_step
        path_req.jump_threshold = self.jump_threshold
        path_req.avoid_collisions = True

        response = await self.cartesian_path_client.call_async(path_req)
        
        return response
    
    async def execute_callback(self, drawing_handle: ActionServer) -> Draw.Result:
        """Handle the execution of a draw action goal.

        Args:
            drawing_handle (ActionServer): The action server handling the draw goal.

        Returns:
            Draw.Result: The result of the draw action execution.
        """
        drawing_req = drawing_handle.request
        self.get_logger().info(f'Accepted draw goal with {len(drawing_req.points)} points')
        start_time = time.time()
        result = Draw.Result()

        if not drawing_req.points:
            result.success = False
            result.message = 'Drawing must contain at least one point'
            result.duration = time.time() - start_time
            drawing_handle.abort()
            self._drawing_lock.release()
            return result

        if not self.ensure_clients():
            result.success = False
            result.message = 'MoveIt compute_cartesian_path / execute_trajectory not available'
            result.duration = time.time() - start_time
            drawing_handle.abort()
            self._drawing_lock.release()
            return result

        feedback_msg = Draw.Feedback()
        feedback_msg.state = 'planning'
        drawing_handle.publish_feedback(feedback_msg)

        try:
            path_response = await self.plan_cartesian_path(
                drawing_req.transform, drawing_req.points
            )

            if drawing_handle.is_cancel_requested:
                result.success = False
                result.message = 'Drawing canceled during planning'
                result.duration = time.time() - start_time
                drawing_handle.canceled()
                self._drawing_lock.release()
                return result

            if path_response is None or path_response.fraction < 1.0:
                result.success = False
                fraction = 0.0 if path_response is None else path_response.fraction
                result.message = f'Cartesian path was only {fraction:.1%} complete'
                result.duration = time.time() - start_time
                drawing_handle.abort()
                self._drawing_lock.release()
                return result

            feedback_msg.state = 'executing'
            drawing_handle.publish_feedback(feedback_msg)
            
            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory = path_response.solution

            goal_handle_future = self.execute_client.send_goal_async(exec_goal)
            exec_goal_handle = await goal_handle_future

            if not exec_goal_handle.accepted:
                result.success = False
                result.message = 'execute_trajectory goal rejected'
                result.duration = time.time() - start_time
                drawing_handle.abort()
                self._drawing_lock.release()
                return result

            exec_result_future = exec_goal_handle.get_result_async()
            while not exec_result_future.done():
                if drawing_handle.is_cancel_requested:
                    self.get_logger().info('Canceling active MoveIt trajectory')
                    await exec_goal_handle.cancel_goal_async()
                    result.success = False
                    result.message = 'Drawing canceled during execution'
                    result.duration = time.time() - start_time
                    drawing_handle.canceled()
                    self._drawing_lock.release()
                    return result
                await asyncio.sleep(0.05)

            exec_result = exec_result_future.result()

            feedback_msg.state = 'cleanup'
            drawing_handle.publish_feedback(feedback_msg)

            if exec_result.result.error_code.val == 1:
                result.success = True
                result.message = 'Executed'
                result.duration = time.time() - start_time
                drawing_handle.succeed()
            else:
                result.success = False
                result.message = f'Execution failed with error code {exec_result.result.error_code.val}'
                result.duration = time.time() - start_time
                drawing_handle.abort()
            self._drawing_lock.release()
            return result
        
        except Exception as e:
            self.get_logger().error(f'MoveIt execution failed: {e}')
            result.success = False
            result.message = str(e)
            result.duration = time.time() - start_time
            drawing_handle.abort()
            self._drawing_lock.release()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = DrawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

