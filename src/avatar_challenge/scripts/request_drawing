#!/usr/bin/env python3
import argparse
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

from avatar_challenge.geometry import compute_circular_arc, matrix_to_transform, compute_b_spline



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
    world_T_drawing = matrix_to_transform(world_T_drawing_np)
    
    drawing_p_points = []
    prev_point = None
    for i, p in enumerate(shape):
        if p[0] == 'v':
            if len(p) != 3:
                raise ValueError(f"Vertex at index {i} must have 2 coordinates")
            drawing_p_points.append(Point2D(x=float(p[1]), y=float(p[2])))
            prev_point = np.array([float(p[1]), float(p[2])])
            
        elif p[0] == 'a':
            if len(p) != 4:
                raise ValueError(f"Arc at index {i} must have 3 coordinates and a radius")
            if prev_point is None:
                raise ValueError(f"Arc at index {i} must be preceded by a vertex")
            
            end_point = np.array([float(p[1]), float(p[2])])
            arc_points = compute_circular_arc(prev_point, end_point, float(p[3]), 0.01)
            print(f"Computed arc points from {prev_point} to {end_point} with radius {p[3]}: {arc_points}")
            drawing_p_points.extend([Point2D(x=pt[0], y=pt[1]) for pt in arc_points])
            prev_point = end_point
        
        elif p[0] == 'b':
            if (len(p)-1) % 2 != 0:
                raise ValueError(f"B-spline curve at index {i} must have an even number of coordinates after the 'b'")
            if prev_point is None:
                raise ValueError(f"B-spline curve at index {i} must be preceded by a vertex")
            
            control_points = [np.array([float(p[j]), float(p[j+1])]) for j in range(1, len(p), 2)]
            control_points = np.concatenate([prev_point[np.newaxis, :], control_points])
            b_spline_points = compute_b_spline(control_points, 0.01)
            print(f"Computed B-spline points from {prev_point} with control points {control_points}: {b_spline_points}")
            drawing_p_points.extend([Point2D(x=pt[0], y=pt[1]) for pt in b_spline_points])
            prev_point = control_points[-1]
        else:
            raise ValueError(f"Unknown shape type '{p[0]}' at index {i}")
    
    return world_T_drawing, drawing_p_points

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='Request a drawing from the avatar challenge.')
    parser.add_argument('yaml_file', nargs='?', default='config/shape_example.yaml', help='Path to the YAML file containing the shape')
    args = parser.parse_args()
    
    
    rclpy.init()
    client = DrawClient()

    yaml_file = args.yaml_file
    if not os.path.isabs(yaml_file):
        yaml_file = os.path.join(get_package_share_directory('avatar_challenge'), yaml_file)

    world_T_drawing, drawing_p_points = load_goal_from_yaml(yaml_file)
    print(f'Loaded goal from {yaml_file} with {len(drawing_p_points)} points')

    res = client.send_goal(world_T_drawing, drawing_p_points)
    if res is not None:
        print('Result:', res.success, res.message, res.duration)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
