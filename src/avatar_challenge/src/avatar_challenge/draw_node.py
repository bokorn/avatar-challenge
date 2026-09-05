import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from geometry_msgs.msg import Point, Pose, Quaternion, Transform

from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.action import ExecuteTrajectory, MoveGroup

from spatialmath import SE3, SO3
from spatialmath.base import q2r
import numpy as np
import numpy.typing as npt

from avatar_challenge_msgs.action import Draw
from avatar_challenge_msgs.msg import Point2D

def transform_to_se3(transform: Transform) -> SE3:
    """Convert a geometry_msgs/Transform's quaternion into a spatialmath SO3."""
    q = transform.rotation
    R = q2r([q.w, q.x, q.y, q.z], order='sxyz')
    t = [transform.translation.x, transform.translation.y, transform.translation.z]
    return SE3.Rt(R, t, check=False)

def transform_vertices(
    robot_T_drawing_tf: Transform,
    drawing_p_vertices_2d: list[Point2D],
) -> list[Pose]:
    """Transform 2D shape points (x, y) into 3D Pose waypoints using the
    given transform's position mapping (only position varies along the path)."""
    robot_T_drawing = transform_to_se3(robot_T_drawing_tf)

    drawing_p_vertices = np.array([[p.x, p.y, 0.0] for p in drawing_p_vertices_2d], dtype=float)
    robot_p_vertices = (robot_T_drawing * drawing_p_vertices.T).T

    poses = []
    for robot_p_vert in robot_p_vertices:
        pose = Pose()
        pose.position.x = float(robot_p_vert[0])
        pose.position.y = float(robot_p_vert[1])
        pose.position.z = float(robot_p_vert[2])
        pose.orientation = robot_T_drawing_tf.rotation
        poses.append(pose)
        
    return poses

class DrawNode(Node):
    def __init__(self):
        super().__init__('draw_node')

        self.declare_parameter('move_group', 'xarm7')
        self.declare_parameter('eef_link', 'link_eef')
        self.declare_parameter('max_step', 0.01)
        self.declare_parameter('jump_threshold', 0.0)
        self.declare_parameter('speed_scaling', 0.2)

        self.move_group_name = self.get_parameter('move_group').get_parameter_value().string_value
        self.eef_link = self.get_parameter('eef_link').get_parameter_value().string_value
        self.max_step = self.get_parameter('max_step').get_parameter_value().double_value
        self.jump_threshold = self.get_parameter('jump_threshold').get_parameter_value().double_value
        self.speed_scaling = self.get_parameter('speed_scaling').get_parameter_value().double_value

        self.cartesian_path_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.move_action_client = ActionClient(self, MoveGroup, 'move_action')

        self._action_server = ActionServer(self, Draw, 'draw', self.execute_callback)
        self.get_logger().info('draw_node action server ready')

    def ensure_clients(self, timeout_sec: float = 5.0) -> bool:
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('compute_cartesian_path service not available')
            return False
        if not self.move_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('move_action action server not available')
            return False
        if not self.fk_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('compute_fk service not available')
            return False
        if not self.execute_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('execute_trajectory action server not available')
            return False
        return True

    async def plan_cartesian_path(
        self,
        world_T_drawing_tf: Transform,
        drawing_p_vertices_2d: list[Point2D],
    ):
        world_p_vertices = transform_vertices(world_T_drawing_tf, drawing_p_vertices_2d)

        path_req = GetCartesianPath.Request()
        path_req.header.frame_id = ''
        path_req.start_state.is_diff = True
        path_req.group_name = self.move_group_name
        path_req.link_name = self.eef_link
        path_req.waypoints = world_p_vertices
        path_req.max_step = self.max_step
        path_req.jump_threshold = self.jump_threshold
        path_req.avoid_collisions = True

        response = await self.cartesian_path_client.call_async(path_req)
        return response, world_p_vertices
    
    async def execute_callback(self, drawing_handle):
        drawing_req = drawing_handle.request
        self.get_logger().info(f'Accepted draw goal with {len(drawing_req.points)} points')

        start_time = time.time()
        result = Draw.Result()

        if not self.ensure_clients():
            result.success = False
            result.message = 'MoveIt compute_cartesian_path / execute_trajectory not available'
            result.duration = time.time() - start_time
            result.points_drawn = 0
            drawing_handle.abort()
            return result

        feedback_msg = Draw.Feedback()
        feedback_msg.current_point = 0
        feedback_msg.fraction_completed = 0.0
        drawing_handle.publish_feedback(feedback_msg)

        try:
            path_response, robot_p_vertices = await self.plan_cartesian_path(
                drawing_req.transform, drawing_req.points
            )
            n = len(drawing_req.points)

            if path_response is None or path_response.fraction <= 0.0:
                result.success = False
                result.message = 'Failed to compute a valid Cartesian path'
                result.duration = time.time() - start_time
                result.points_drawn = 0
                drawing_handle.abort()
                return result

            fraction = path_response.fraction
            self.get_logger().info(f'Cartesian path fraction: {fraction}')

            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory = path_response.solution

            goal_handle_future = self.execute_client.send_goal_async(exec_goal)
            exec_goal_handle = await goal_handle_future

            if not exec_goal_handle.accepted:
                result.success = False
                result.message = 'execute_trajectory goal rejected'
                result.duration = time.time() - start_time
                result.points_drawn = 0
                drawing_handle.abort()
                return result

            exec_result_future = exec_goal_handle.get_result_async()
            exec_result = await exec_result_future

            feedback_msg.current_point = n
            feedback_msg.fraction_completed = 1.0
            drawing_handle.publish_feedback(feedback_msg)

            if exec_result.result.error_code.val == 1:
                result.success = True
                result.message = 'Executed'
                result.duration = time.time() - start_time
                result.points_drawn = n
                drawing_handle.succeed()
            else:
                result.success = False
                result.message = f'Execution failed with error code {exec_result.result.error_code.val}'
                result.duration = time.time() - start_time
                result.points_drawn = 0
                drawing_handle.abort()
            return result
        
        except Exception as e:
            self.get_logger().error(f'MoveIt execution failed: {e}')
            result.success = False
            result.message = str(e)
            result.duration = time.time() - start_time
            result.points_drawn = 0
            drawing_handle.abort()
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

