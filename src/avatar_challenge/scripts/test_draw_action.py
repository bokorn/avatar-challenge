#!/usr/bin/env python3
import sys
import os
import yaml
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Transform
from ament_index_python.packages import get_package_share_directory

from avatar_challenge_msgs.action import Draw
from avatar_challenge_msgs.msg import Point2D

from avatar_challenge.geometry import matrix_to_transform



class DrawClient(Node):
    """Client node for sending drawing goals to the Draw action server."""
    def __init__(self):
        super().__init__('draw_client')
        self._client = ActionClient(self, Draw, 'draw')

    def send_goal(self, world_T_drawing: Transform, drawing_p_points: list[Point2D]) -> Draw.Result:
        """Send a drawing goal to the Draw action server and wait for the result.

        Args:
            world_T_drawing (Transform): The transform from the drawing frame to the world frame.
            drawing_p_points (list[Point2D]): The 2D points defining the drawing shape.

        Returns:
            Draw.Result: The result of the drawing action.
        """
        goal_msg = Draw.Goal()
        goal_msg.transform = world_T_drawing
        goal_msg.points = drawing_p_points

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

def load_goal_from_yaml(yaml_file) -> tuple[Transform, list[Point2D]]:
    """Load a drawing goal from a YAML file.

    Args:
        yaml_file (str): The path to the YAML file containing the drawing goal.

    Returns:
        tuple[Transform, list[Point2D]]: The transform from the drawing frame to the world frame and the list of 2D points defining the drawing shape.
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    shape = data.get('shape', [])
    world_T_drawing_np = np.array(data.get('transform', [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

    drawing_p_points = [Point2D(x=float(p[0]), y=float(p[1])) for p in shape]
    world_T_drawing = matrix_to_transform(world_T_drawing_np)
    return world_T_drawing, drawing_p_points

def main(argv=sys.argv[1:]):
    rclpy.init()
    client = DrawClient()

    yaml_file = argv[0] if len(argv) > 0 else 'config/shape_example.yaml'
    if not os.path.isabs(yaml_file):
        yaml_file = os.path.join(get_package_share_directory('avatar_challenge'), yaml_file)

    world_T_drawing, drawing_p_points = load_goal_from_yaml(yaml_file)
    print(f'Loaded goal from {yaml_file}')

    res = client.send_goal(world_T_drawing, drawing_p_points)
    if res is not None:
        print('Result:', res.success, res.message, res.duration)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
