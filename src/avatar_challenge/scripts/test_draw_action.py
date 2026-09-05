#!/usr/bin/env python3
import sys
import os
import yaml
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Point, Transform
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

from avatar_challenge_msgs.action import Draw
from avatar_challenge_msgs.msg import Point2D

from spatialmath import SE3


def matrix_to_transform(robot_T_drawing_np) -> Transform:
    """Convert a 4x4 nested-list matrix into a geometry_msgs/Transform."""
    robot_T_drawing_se3 = SE3(robot_T_drawing_np, check=False)
    q = robot_T_drawing_se3.UnitQuaternion()

    robot_T_drawing = Transform()
    robot_T_drawing.translation.x = float(robot_T_drawing_np[0][3])
    robot_T_drawing.translation.y = float(robot_T_drawing_np[1][3])
    robot_T_drawing.translation.z = float(robot_T_drawing_np[2][3])

    robot_T_drawing.rotation.x = float(q.v[0])
    robot_T_drawing.rotation.y = float(q.v[1])
    robot_T_drawing.rotation.z = float(q.v[2])
    robot_T_drawing.rotation.w = float(q.s)


    return robot_T_drawing


class DrawClient(Node):
    def __init__(self):
        super().__init__('draw_client')
        self._client = ActionClient(self, Draw, 'draw')
        # Transient-local durability so RViz's Marker display still shows the
        # points even if it subscribes after this node publishes them.
        marker_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._marker_pub = self.create_publisher(MarkerArray, 'drawing_markers', marker_qos)

    def publish_vertex_markers(self, world_points, frame_id='world'):
        """Publish the shape's vertices (and the straight-line path between
        them) as RViz markers so they can be visually compared against the
        arm's actual executed path."""
        header_stamp = self.get_clock().now().to_msg()

        points_marker = Marker()
        points_marker.header.frame_id = frame_id
        points_marker.header.stamp = header_stamp
        points_marker.ns = 'drawing_vertices'
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.01
        points_marker.scale.y = 0.01
        points_marker.scale.z = 0.01
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        points_marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in world_points]

        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = header_stamp
        line_marker.ns = 'drawing_path'
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.003
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.points = list(points_marker.points) + [points_marker.points[0]]

        self._marker_pub.publish(MarkerArray(markers=[points_marker, line_marker]))

    def send_goal(self, transform: Transform, points, speed=0.2):
        goal_msg = Draw.Goal()
        goal_msg.transform = transform
        goal_msg.points = points
        goal_msg.speed_scaling = float(speed)

        self._client.wait_for_server()
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        return result


def load_goal_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    shape = data.get('shape', [])
    transform_mat = data.get('transform', [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    points = [Point2D(x=float(p[0]), y=float(p[1])) for p in shape]
    transform = matrix_to_transform(transform_mat)
    return transform, points, transform_mat, shape


def shape_to_world_points(shape, transform_mat):
    """Apply the 4x4 transform matrix to the 2D shape points (z=0 in the
    drawing's local frame) to get their 3D positions in the world frame."""
    robot_T_drawing = SE3(transform_mat, check=False)
    world_points = []
    for p in shape:
        world_point = np.asarray(robot_T_drawing * [float(p[0]), float(p[1]), 0.0]).flatten()
        world_points.append(world_point)
    return world_points


def main(argv=sys.argv[1:]):
    rclpy.init()
    client = DrawClient()

    yaml_file = argv[0] if len(argv) > 0 else 'config/shape_example.yaml'
    if not os.path.isabs(yaml_file):
        yaml_file = os.path.join(get_package_share_directory('avatar_challenge'), yaml_file)

    transform, points, transform_mat, shape = load_goal_from_yaml(yaml_file)
    print(f'Loaded goal from {yaml_file}')
    print(f'Transform: {transform}')
    print(f'Points: {points}')

    world_points = shape_to_world_points(shape, transform_mat)
    client.publish_vertex_markers(world_points)

    res = client.send_goal(transform, points, speed=0.2)
    if res is not None:
        print('Result:', res.success, res.message, res.duration, res.points_drawn)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
