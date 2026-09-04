import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from geometry_msgs.msg import Point, Pose, Quaternion, Transform

from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, MotionPlanRequest, PositionConstraint
from shape_msgs.msg import SolidPrimitive

from spatialmath import SO3
from spatialmath.base import q2r
import numpy as np
import numpy.typing as npt

from avatar_challenge_msgs.action import Draw
from avatar_challenge_msgs.msg import Point2D


def transform_rotation_to_so3(transform: Transform) -> SO3:
    """Convert a geometry_msgs/Transform's quaternion into a spatialmath SO3."""
    q = transform.rotation
    R = q2r([q.w, q.x, q.y, q.z], order='sxyz')
    return SO3(R, check=False)


def so3_to_quaternion(R: SO3) -> Quaternion:
    q = R.UnitQuaternion()
    return Quaternion(x=float(q.v[0]), y=float(q.v[1]), z=float(q.v[2]), w=float(q.s))


# Since the end-effector's orientation doesn't matter for drawing a shape,
# only its position, we try a handful of fixed (constant-orientation)
# candidates for the whole straight-line path and keep whichever lets
# compute_cartesian_path complete the largest fraction of it. Holding
# orientation perfectly constant (rather than deriving/interpolating one)
# avoids wasting reachability on wrist motion that isn't needed.
HALF_PI = np.pi / 2.0
FIXED_ORIENTATION_CANDIDATES = {
    'identity': so3_to_quaternion(SO3()),
    'rx+90': so3_to_quaternion(SO3.Rx(HALF_PI)),
    'rx-90': so3_to_quaternion(SO3.Rx(-HALF_PI)),
    'ry+90': so3_to_quaternion(SO3.Ry(HALF_PI)),
    'ry-90': so3_to_quaternion(SO3.Ry(-HALF_PI)),
    'rz+90': so3_to_quaternion(SO3.Rz(HALF_PI)),
    'rz-90': so3_to_quaternion(SO3.Rz(-HALF_PI)),
}


def scale_trajectory_time(trajectory, speed_scaling: float):
    """Scale a joint trajectory's timing (and velocities/accelerations) to
    approximate a slower/faster execution speed. speed_scaling in (0, 1]
    slows the motion down; 1.0 leaves it unchanged."""
    if speed_scaling <= 0.0 or speed_scaling == 1.0:
        return trajectory

    factor = 1.0 / speed_scaling
    for point in trajectory.joint_trajectory.points:
        t = point.time_from_start
        scaled_sec_total = (t.sec + t.nanosec * 1e-9) * factor
        point.time_from_start.sec = int(scaled_sec_total)
        point.time_from_start.nanosec = int((scaled_sec_total - int(scaled_sec_total)) * 1e9)
        point.velocities = [v * speed_scaling for v in point.velocities]
        point.accelerations = [a * speed_scaling * speed_scaling for a in point.accelerations]
    return trajectory


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

        # MoveIt2 (Humble) has no moveit_py/moveit_commander Python bindings,
        # so we talk to move_group directly via its service/action interface.
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

    async def get_current_orientation(self) -> Quaternion:
        """Query the end effector's current orientation from move_group via
        compute_fk, using the is_diff=True trick to mean 'current state'
        without needing to track /joint_states ourselves."""
        req = GetPositionFK.Request()
        req.header.frame_id = ''
        req.fk_link_names = [self.eef_link]
        req.robot_state.is_diff = True

        response = await self.fk_client.call_async(req)
        if response.error_code.val != 1 or not response.pose_stamped:
            raise RuntimeError(f'compute_fk failed with error code {response.error_code.val}')
        return response.pose_stamped[0].pose.orientation

    async def move_to_position(self, position: Point, tolerance: float = 0.01) -> bool:
        """Move the eef to the given position with NO orientation constraint,
        letting move_group plan freely (any orientation, any joint-space
        path). This gets the arm from wherever it currently is (which may be
        far away, e.g. its home pose) to the drawing's starting point, without
        forcing a fixed orientation across that whole approach distance."""
        constraint_region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tolerance]
        constraint_region.primitives = [sphere]
        pose = Pose()
        pose.position = position
        pose.orientation.w = 1.0
        constraint_region.primitive_poses = [pose]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = ''
        position_constraint.link_name = self.eef_link
        position_constraint.constraint_region = constraint_region
        position_constraint.weight = 1.0

        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        request.goal_constraints = [Constraints(position_constraints=[position_constraint])]
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0

        goal = MoveGroup.Goal()
        goal.request = request

        goal_handle = await self.move_action_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error('move_action goal rejected')
            return False

        result = await goal_handle.get_result_async()
        if result.result.error_code.val != 1:
            self.get_logger().error(f'move_action failed with error code {result.result.error_code.val}')
            return False
        return True

    def compute_waypoints(
        self,
        robot_T_drawing_tf: Transform,
        drawing_p_vertices_2d: list[Point2D],
        orientation: Quaternion,
    ) -> list[Pose]:
        """Transform 2D shape points (x, y) into 3D Pose waypoints using the
        given transform's position mapping, with every waypoint fixed to the
        given orientation (only position varies along the path)."""
        robot_T_drawing = transform_rotation_to_so3(robot_T_drawing_tf)
        t = robot_T_drawing_tf.translation
        translation = np.array([t.x, t.y, t.z], dtype=float)

        drawing_p_vertices = np.array([[p.x, p.y, 0.0] for p in drawing_p_vertices_2d], dtype=float)
        robot_p_vertices = (robot_T_drawing * drawing_p_vertices.T).T + translation

        poses = []
        for robot_p_vert in robot_p_vertices:
            pose = Pose()
            pose.position.x = float(robot_p_vert[0])
            pose.position.y = float(robot_p_vert[1])
            pose.position.z = float(robot_p_vert[2])
            pose.orientation = orientation
            poses.append(pose)
        return poses

    async def plan_best_cartesian_path(
        self,
        robot_T_drawing_tf: Transform,
        drawing_p_vertices_2d: list[Point2D],
    ):
        """Try a handful of fixed orientation candidates (plus the eef's
        current orientation) and return the compute_cartesian_path response
        and waypoints for whichever candidate completes the largest fraction
        of the straight-line path."""
        candidates = dict(FIXED_ORIENTATION_CANDIDATES)
        try:
            candidates['current'] = await self.get_current_orientation()
        except RuntimeError as e:
            self.get_logger().warning(f'Could not query current orientation: {e}')

        best_name = None
        best_response = None
        best_waypoints = None
        for name, orientation in candidates.items():
            waypoints = self.compute_waypoints(robot_T_drawing_tf, drawing_p_vertices_2d, orientation)

            path_req = GetCartesianPath.Request()
            path_req.header.frame_id = ''
            path_req.start_state.is_diff = True
            path_req.group_name = self.move_group_name
            path_req.link_name = self.eef_link
            path_req.waypoints = waypoints
            path_req.max_step = self.max_step
            path_req.jump_threshold = self.jump_threshold
            path_req.avoid_collisions = True

            response = await self.cartesian_path_client.call_async(path_req)
            self.get_logger().info(f'Orientation candidate "{name}": fraction={response.fraction:.3f}')

            if response.error_code.val == 1 and (best_response is None or response.fraction > best_response.fraction):
                best_name = name
                best_response = response
                best_waypoints = waypoints

            if best_response is not None and best_response.fraction >= 1.0:
                break

        if best_response is not None:
            self.get_logger().info(f'Best orientation candidate: "{best_name}" (fraction={best_response.fraction:.3f})')
        return best_response, best_waypoints

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
            first_waypoint = self.compute_waypoints(
                drawing_req.transform, drawing_req.points[:1], Quaternion(w=1.0)
            )[0]
            if not await self.move_to_position(first_waypoint.position):
                result.success = False
                result.message = 'Failed to move to the drawing start position'
                result.duration = time.time() - start_time
                result.points_drawn = 0
                drawing_handle.abort()
                return result

            path_response, robot_p_vertices = await self.plan_best_cartesian_path(
                drawing_req.transform, drawing_req.points
            )
            n = len(drawing_req.points)

            if path_response is None or path_response.fraction <= 0.0:
                result.success = False
                result.message = 'Failed to compute a valid Cartesian path with any orientation candidate'
                result.duration = time.time() - start_time
                result.points_drawn = 0
                drawing_handle.abort()
                return result

            fraction = path_response.fraction
            self.get_logger().info(f'Cartesian path fraction: {fraction}')

            trajectory = scale_trajectory_time(path_response.solution, self.speed_scaling)

            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory = trajectory

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

