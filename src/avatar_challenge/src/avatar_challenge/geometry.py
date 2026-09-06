from spatialmath import SE3
from spatialmath.base import q2r
from geometry_msgs.msg import Pose, Transform, Point
from avatar_challenge_msgs.msg import Point2D
import numpy as np
import numpy.typing as npt
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
from scipy.interpolate import splprep, splev

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
    
    if not points:
        return MarkerArray()

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

def compute_circle_center(start_point: tuple[float, float], end_point: tuple[float, float], signed_radius: float) -> tuple[float, float]:
    """Compute the center of a circle given two points on its circumference and the signed radius.

    Args:
        start_point (tuple[float, float]): The starting point on the circle (x, y).
        end_point (tuple[float, float]): The ending point on the circle (x, y).
        signed_radius (float): The signed radius of the circle. Positive for counter-clockwise arcs, negative for clockwise arcs.

    Returns:
        tuple[float, float]: The coordinates of the circle center (x, y).
    """
    x0, y0 = start_point
    x1, y1 = end_point
    dx, dy = x1 - x0, y1 - y0
    q = np.sqrt(dx**2 + dy**2)
    if q == 0:
        raise ValueError("Start point and end point cannot be the same")
    if abs(signed_radius) < q / 2:
        raise ValueError("The absolute radius must be at least half the chord length")
    # midpoint between start and end points
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    # distance from midpoint to center
    d = np.sqrt(signed_radius**2 - (q / 2)**2)
    # direction vector perpendicular to the line segment
    perp_dx, perp_dy = -dy / q, dx / q
    # two possible centers
    cx1, cy1 = mx + d * perp_dx, my + d * perp_dy
    cx2, cy2 = mx - d * perp_dx, my - d * perp_dy
    # choose the center based on the sign of the radius
    if signed_radius > 0:
        return cx1, cy1
    else:
        return cx2, cy2
        
def compute_circular_arc(start_point: tuple[float, float], end_point: tuple[float, float], signed_radius: float, resolution: float = 0.1) -> np.ndarray:
   
    """Compute the coordinates of a circular arc.

    Args:
        start_point (tuple[float, float]): The starting point of the arc (x, y).
        end_point (tuple[float, float]): The ending point of the arc (x, y).
        signed_radius (float): The signed radius of the circular arc. Positive for counter-clockwise arcs, negative for clockwise arcs.
        resolution (float, optional): The spacial resolution of the points along the arc in units of arc length. Defaults to 0.1.

    Returns:
        tuple[np.ndarray, np.ndarray]: The x and y coordinates of the points along the arc.
    """
    
    x0, y0 = start_point
    x1, y1 = end_point
    xc, yc = compute_circle_center(start_point, end_point, signed_radius)
    
    radius = np.sqrt((x0 - xc)**2 + (y0 - yc)**2)
    start_angle = np.arctan2(y0 - yc, x0 - xc)
    end_angle = np.arctan2(y1 - yc, x1 - xc)
    sweep_angle = end_angle - start_angle
    if signed_radius < 0 and sweep_angle > 0:
        sweep_angle -= 2 * np.pi
    elif signed_radius > 0 and sweep_angle < 0:
        sweep_angle += 2 * np.pi
    num_points = max(int(np.abs(sweep_angle) * radius / resolution), 2)
    angles = np.linspace(start_angle, start_angle + sweep_angle, num_points)
    
    arc_x = xc + radius * np.cos(angles)
    arc_y = yc + radius * np.sin(angles)
    
    return np.stack((arc_x, arc_y), axis=-1)

def compute_b_spline(control_points: np.ndarray, resolution: float = 0.1) -> np.ndarray:
    """Compute a B-spline curve from control points.

    Args:
        control_points (np.ndarray): An array of control points of shape (N, 2), where N is the number of control points.
        resolution (float, optional): The spacial resolution of the points along the B-spline curve in units of arc length. Defaults to 0.1.

    Returns:
        np.ndarray: An array of points along the B-spline curve of shape (num_points, 2).
    """

    tck, _ = splprep(control_points.T, s=0)
    num_points = max(int(np.linalg.norm(np.diff(control_points, axis=0), axis=1).sum() / resolution), 2)
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck)
    return np.vstack((x_new, y_new)).T