from spatialmath import SE3
from spatialmath.base import q2r
from geometry_msgs.msg import Pose, Transform, Point
from avatar_challenge_msgs.msg import Point2D
import numpy as np
import numpy.typing as npt
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time

def transform_to_se3(transform: Transform) -> SE3:
    """Convert a geometry_msgs/Transform's quaternion into a spatialmath SO3.
    
    Args:
        transform (Transform): The geometry_msgs/Transform to convert.

    Returns:
        SE3: The corresponding spatialmath SE3 object.
    """
    q = transform.rotation
    R = q2r([q.w, q.x, q.y, q.z], order='sxyz')
    t = [transform.translation.x, transform.translation.y, transform.translation.z]
    return SE3.Rt(R, t, check=False)

def transform_vertices(
    world_T_drawing_tf: Transform,
    drawing_p_vertices_2d: list[Point2D],
) -> list[Pose]:
    """Transform 2D shape points (x, y) into 3D Pose waypoints using the
    given transform's position mapping (only position varies along the path).

    Args:
        world_T_drawing_tf (Transform): The transform from the drawing frame to the world frame.
        drawing_p_vertices_2d (list[Point2D]): The 2D vertices of the drawing shape.

    Returns:
        list[Pose]: The corresponding 3D Pose waypoints in the world frame.
    """
    world_T_drawing = transform_to_se3(world_T_drawing_tf)

    drawing_p_vertices = np.array([[p.x, p.y, 0.0] for p in drawing_p_vertices_2d], dtype=float)
    world_p_vertices = (world_T_drawing * drawing_p_vertices.T).T

    world_p_poses = []
    for world_p_vert in world_p_vertices:
        pose = Pose()
        pose.position.x = float(world_p_vert[0])
        pose.position.y = float(world_p_vert[1])
        pose.position.z = float(world_p_vert[2])
        pose.orientation = world_T_drawing_tf.rotation
        world_p_poses.append(pose)
        
    return world_p_poses

def matrix_to_transform(world_T_drawing_np: npt.NDArray) -> Transform:
    """Convert a 4x4 nested-list matrix into a geometry_msgs/Transform.

    Args:
        world_T_drawing_np (npt.NDArray): The 4x4 transformation matrix representing the pose of the drawing frame in the world frame.

    Returns:
        Transform: The corresponding geometry_msgs/Transform object.
    """
    world_T_drawing_se3 = SE3(world_T_drawing_np, check=False)
    q = world_T_drawing_se3.UnitQuaternion()

    world_T_drawing = Transform()
    world_T_drawing.translation.x = float(world_T_drawing_np[0][3])
    world_T_drawing.translation.y = float(world_T_drawing_np[1][3])
    world_T_drawing.translation.z = float(world_T_drawing_np[2][3])

    world_T_drawing.rotation.x = float(q.v[0])
    world_T_drawing.rotation.y = float(q.v[1])
    world_T_drawing.rotation.z = float(q.v[2])
    world_T_drawing.rotation.w = float(q.s)

    return world_T_drawing

def path_to_markers(points: list[Pose], frame_id: str = 'world', timestamp: Time =None):
    """Convert a list of 3D Pose waypoints into RViz markers for visualization.

    Args:
        points (list[Pose]): The 3D Pose waypoints to visualize.
        frame_id (str, optional): The reference frame for the markers. Defaults to 'world'.
        timestamp (Time, optional): The timestamp for the markers. Defaults to None.

    Returns:
        MarkerArray: The RViz markers representing the points and the connecting path.
    """
    
    points_marker = Marker()
    points_marker.header.frame_id = frame_id
    if timestamp is not None:
        points_marker.header.stamp = timestamp
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
    points_marker.points = [Point(x=float(p.position.x), y=float(p.position.y), z=float(p.position.z)) for p in points]

    line_marker = Marker()
    line_marker.header.frame_id = frame_id
    if timestamp is not None:
        line_marker.header.stamp = timestamp
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

    return MarkerArray(markers=[points_marker, line_marker])