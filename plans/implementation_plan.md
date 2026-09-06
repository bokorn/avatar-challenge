# Implementation Plan: ROS 2 Cartesian Drawing

## Status

The core implementation is complete and uses a ROS 2 action API. The package
accepts a drawing transform and expanded 2D points, computes a Cartesian path
with MoveIt 2, executes it, publishes RViz markers, and supports cancellation.

## Architecture

The workspace contains two packages:

- `avatar_challenge_msgs`: interfaces-only `ament_cmake` package.
- `avatar_challenge`: `ament_cmake` plus `ament_cmake_python` package containing
  the Python node, geometry helpers, launch file, and request client.

The custom action is `avatar_challenge_msgs/action/Draw`:

```text
Goal:
geometry_msgs/Transform transform
avatar_challenge_msgs/Point2D[] points

Result:
bool success
string message
float32 duration

Feedback:
string state
```

The action name is `draw`.

## Runtime flow

1. `request_drawing.py` loads a YAML file.
2. The client converts the 4x4 transform matrix to
   `geometry_msgs/Transform` using `spatialmath`.
3. Shape records are expanded into `Point2D[]`:
   - vertices are copied directly;
   - circular arcs are sampled at approximately 0.01 m resolution;
   - B-splines are sampled at approximately 0.01 m resolution.
4. The client sends a `Draw` action goal to `draw_node`.
5. `draw_node` transforms every 2D point into a 3D `geometry_msgs/Pose` using
   the supplied rotation and translation.
6. The transformed vertices and connecting path are published as a
   `visualization_msgs/MarkerArray` on `drawing_markers`.
7. `draw_node` calls MoveIt's `/compute_cartesian_path` service with:
   - planning group `xarm7` by default;
   - end-effector link `link_eef` by default;
   - `max_step` defaulting to 0.01 m;
   - `jump_threshold` defaulting to 0.0;
   - collision avoidance enabled.
8. A complete Cartesian path is sent to MoveIt's `/execute_trajectory` action.
9. The draw action returns success or failure and the elapsed duration.

Partial Cartesian paths are rejected and are not executed.

## Cancellation and goal ownership

- `draw_node` accepts one drawing goal at a time using a non-blocking lock.
- Additional goals are rejected while a drawing is active.
- Cancellation requests are accepted by the draw action server.
- Cancellation during planning aborts the draw operation.
- Cancellation during execution is forwarded to the active MoveIt trajectory
  through `cancel_goal_async()`.
- The draw goal is marked canceled after the MoveIt goal cancellation request.

## YAML shape format

Each input file contains `transform` and `shape` fields.

### Transform

`transform` is a homogeneous 4x4 `world_T_drawing` matrix:

```yaml
transform:
  - [r00, r01, r02, tx]
  - [r10, r11, r12, ty]
  - [r20, r21, r22, tz]
  - [0.0, 0.0, 0.0, 1.0]
```

The 3x3 block rotates local drawing coordinates into the `world` frame. The
last column translates the drawing origin in meters. Shape points lie in the
local drawing plane at `z = 0`.

The rotation is expected to be a proper rotation with determinant +1. A
reflection or malformed homogeneous matrix is outside the supported input
assumptions.

### Vertex

```yaml
- [v, x, y]
```

`x` and `y` are local drawing coordinates in meters. Vertices are processed in
file order.

### Circular arc

```yaml
- [a, end_x, end_y, signed_radius]
```

An arc starts at the preceding vertex and ends at `(end_x, end_y)`.
` signed_radius` is in meters:

- positive selects the counter-clockwise solution;
- negative selects the clockwise solution;
- the absolute radius must be at least half the chord length.

An arc must follow a vertex. The parser samples the arc before sending the
resulting points to the action server.

### B-spline

```yaml
- [b, x1, y1, x2, y2, x3, y3]
```

The preceding vertex is automatically prepended as the first control point.
The remaining coordinate pairs are additional control points. A B-spline must
follow a vertex and is sampled before execution.

### Example

```yaml
transform:
  - [0.0, 0.0, 1.0, 0.5]
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.2]
  - [0.0, 0.0, 0.0, 1.0]

shape:
  - [v, 0.0, 0.0]
  - [a, 0.1, 0.0, 0.05]
  - [v, 0.1, 0.1]
  - [b, 0.02, 0.05, 0.01, 0.07, 0.0, 0.1]
  - [v, 0.0, 0.0]
```

## Launch and usage

The current launch entry point is `start.launch.py`. It starts the xArm7 fake
MoveIt stack and `draw_node`:

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/dev_ws/install/setup.bash
ros2 launch avatar_challenge start.launch.py
```

The test/request client runs from another sourced shell:

```bash
ros2 run avatar_challenge request_drawing.py
ros2 run avatar_challenge request_drawing.py /absolute/path/to/shape.yaml
```

RViz displays the xArm scene. The drawing preview is available through the
`drawing_markers` MarkerArray topic with `world` as the fixed frame.

## Assumptions

- The target environment is Ubuntu 22.04 with ROS 2 Humble.
- `xarm_ros2` is available from its Humble branch and its workspace is sourced.
- MoveIt 2 exposes `/compute_cartesian_path` and `/execute_trajectory`.
- The planning frame is named `world`.
- Coordinates, radii, and sampling resolutions are in meters.
- The default robot group is `xarm7`; the default end-effector is `link_eef`.
- The supplied waypoints include orientation copied from the drawing transform.
  Orientation is therefore still part of MoveIt's Cartesian feasibility check.
- Shapes are not implicitly closed; the input must repeat its first point when a
  closed path is desired.
- Empty shapes are invalid.
- The fake xArm launch is for simulation/testing. Real hardware requires the
  appropriate hardware and controller configuration.
- The implementation calls MoveIt 2's native ROS interfaces directly because
  `moveit_commander` is ROS 1-only and `moveit_py` is not available in the
  target Humble installation.

## Validation

The following checks are used after changes:

```bash
python3 -m py_compile \
  src/avatar_challenge/src/avatar_challenge/draw_node.py \
  src/avatar_challenge/src/avatar_challenge/geometry.py \
  src/avatar_challenge/scripts/request_drawing.py

colcon build --packages-select avatar_challenge_msgs avatar_challenge --symlink-install
```

An end-to-end test requires the xArm fake MoveIt launch to be running and then
runs `request_drawing.py` against the `draw` action server.

## Follow-up improvements

- Add dedicated unit tests for transform validation, arc sampling, B-spline
  sampling, and empty input.
- Add a client-side cancellation option to `request_drawing.py` for convenient
  manual testing.
- Consider making the planning frame, marker frame, and sampling resolutions
  configurable parameters.
- Remove or reconcile the legacy `setup.py` if the package continues to use
  `ament_cmake` with `ament_python_install_package`.
