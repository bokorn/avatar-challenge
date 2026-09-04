# Implementation Plan: ROS2 Draw Node (updated)

This document expands the implementation plan to include an action-based drawing API, a custom `Drawing` message, and a test script that loads a YAML and queries the action server.

## Summary
- Implement `draw_node` as an action server that performs drawing requests.
- Add a custom ROS message `Drawing.msg` (and optionally an action definition `Draw.action`).
- Provide a test script that reads a YAML shape file and sends a draw request to the action server.

## New work items (high level)
1. Add an action server interface to `draw_node` that accepts drawing requests and returns success/failure and optional statistics (duration, points_drawn).
2. Add `msg/Drawing.msg` describing a drawing request payload (fields: `string yaml_file`, `float32 speed_scaling`, `bool execute`).
3. Update `package.xml` and `CMakeLists.txt` to generate message artifacts via `rosidl_generate_interfaces` and add `rosidl_default_generators` and related build deps.
4. Optionally create `action/Draw.action` with goal/result/feedback if you prefer using ROS actions directly (recommended for long-running operations with feedback). If using `action/Draw.action`, generate action interfaces and implement an action server instead of a simple topic/service.
5. Add `scripts/test_draw_action.py` that:
   - Loads a YAML file (`config/shape_example.yaml` by default).
   - Constructs a `Drawing` message or action goal and sends it to the server.
   - Waits for the result and prints status and timing.
6. Update `launch/draw.launch.py` to include any message/action generation timeouts and to optionally start a small test node for CI.
7. Update `README.md` with instructions to build, run, and test the action server.

## Message definition proposal
- `msg/Drawing.msg`

  string yaml_file
  float32 speed_scaling
  bool execute

This is a simple request message; if using actions, the `Draw.action` could be:

  # goal
  string yaml_file
  float32 speed_scaling
  ---
  # result
  bool success
  string message
  float32 duration
  int32 points_drawn
  ---
  # feedback
  int32 current_point
  float32 fraction_completed

## Node behavior (action server)
- On goal accept: load the YAML file and parse `shape` and `transform`.
- Convert points to world poses using the transform (using `spatialmath`).
- Compute Cartesian path segments via MoveIt (prefer `compute_cartesian_path`), send partial feedback periodically (if action), and execute the trajectory.
- On completion: return success/failure result containing points drawn and duration.

## Test script (`scripts/test_draw_action.py`)
- Read YAML path from CLI or use package `config/shape_example.yaml`.
- Create and send a `Drawing` message (or action goal) to the `draw_node` action server.
- Wait for result and print debug info.

## Build changes
- Add `rosidl_default_generators` and `rosidl_default_runtime` to `package.xml`.
- In `CMakeLists.txt` add `rosidl_generate_interfaces(${PROJECT_NAME} "msg/Drawing.msg" "action/Draw.action" ...)` and call `ament_export_dependencies(rosidl_default_runtime)` as needed.

## Verification
1. `colcon build --packages-select avatar_challenge` builds message/action types and the node.
2. `ros2 launch avatar_challenge draw.launch.py` starts MoveIt + the `draw_node` action server.
3. `python3 scripts/test_draw_action.py --yaml config/shape_example.yaml` sends a request and prints result.

## Next steps
- Confirm whether you want a simple message+service, or a full ROS `action` interface. I recommend `action/Draw.action` because drawing is long-running and benefits from feedback.
- Once confirmed I'll update `package.xml`/`CMakeLists.txt` for message/action generation and scaffold `msg/Drawing.msg`, `action/Draw.action`, and `scripts/test_draw_action.py`.

## Status Update: MoveIt2 (Humble) implementation

The `action/Draw.action` approach was chosen and implemented. This section documents the
as-built architecture, which differs from the original plan above in one major way: **there
are no Python MoveIt bindings on ROS2 Humble** (neither `moveit_commander`, which is ROS1-only,
nor `moveit_py`/`MoveItPy`, which only exists starting at Iron/Rolling). `draw_node.py` instead
talks to `move_group` directly via its native ROS2 service/action interfaces from `rclpy`.

### Architecture used
- `moveit_msgs/srv/GetCartesianPath` on `/compute_cartesian_path` — plans the straight-line
  path through the shape's waypoints.
- `moveit_msgs/action/ExecuteTrajectory` on `/execute_trajectory` — executes the planned path.
- `moveit_msgs/action/MoveGroup` on `/move_action` — used with a **position-only** goal
  constraint (no orientation constraint) to freely move the arm from wherever it currently is
  to the drawing's starting point, before tracing the shape.
- `moveit_msgs/srv/GetPositionFK` on `/compute_fk` — used with `robot_state.is_diff = True` and
  an empty `joint_state` to query the eef's current pose/orientation without subscribing to
  `/joint_states`.
- All calls are made with `async def`/`await` inside the action server's `execute_callback`,
  which `rclpy` supports natively under the default `SingleThreadedExecutor` (no
  `MultiThreadedExecutor`/`ReentrantCallbackGroup` needed).

### Key files (current state)
- `src/avatar_challenge/draw_node.py` — the action server. Key pieces:
  - `compute_waypoints(transform, points_2d, orientation)`: maps 2D shape points into 3D
    `Pose`s using the transform's rotation **and translation**, with every waypoint fixed to
    one given orientation (position-only variation along the path).
  - `move_to_position(position)`: sends a `MoveGroup` goal with only a `PositionConstraint`
    (a small tolerance sphere around the target, built from `shape_msgs/SolidPrimitive`) and no
    orientation constraint, letting MoveIt plan+execute freely to reach the drawing's first
    point from the arm's current (possibly distant, e.g. home) pose.
  - `plan_best_cartesian_path(transform, points_2d)`: since eef orientation doesn't matter for
    drawing, tries a fixed set of candidate constant orientations (`identity`, `Rx(±90°)`,
    `Ry(±90°)`, `Rz(±90°)`, plus the eef's current orientation via FK) for the whole
    straight-line path, and keeps whichever `compute_cartesian_path` call returns the highest
    completion `fraction`.
  - `execute_callback`: calls `move_to_position` for the first waypoint, then
    `plan_best_cartesian_path` for the shape trace, then executes the resulting trajectory via
    `ExecuteTrajectory` and reports `Draw.Result` (success/message/duration/points_drawn).
- `scripts/test_draw_action.py` — standalone test client; loads shape+transform from YAML
  (`config/shape_example.yaml` by default) and sends a `Draw` action goal. Uses `spatialmath`
  (`SO3`/`UnitQuaternion`, via `.v`/`.s` accessors — **not** `.x/.y/.z/.w`) instead of
  `tf_transformations` (which is broken under `numpy>=1.24`).
- `launch/draw.launch.py` — launches the xarm7 fake MoveIt stack + `draw_node`. Supports
  `use_rviz:=false` for headless environments (bridges to `xarm_moveit_config`'s internal
  `show_rviz` launch configuration).
- `package.xml` — exec_depends: `rclpy`, `spatialmath`, `moveit_msgs`, `shape_msgs`. No
  `moveit_commander`/`tf2_ros`/`tf_transformations` (removed, unused/broken).

### Bugs found and fixed during implementation/testing
1. `tf_transformations`/`transforms3d==0.3.1` crashes with `numpy>=1.24` (`np.float` removed) —
   switched to `spatialmath` throughout.
2. `spatialmath.UnitQuaternion` has no `.x/.y/.z/.w` — correct accessors are `.v` (xyz) and
   `.s` (w).
3. `rclpy.action.server.ServerGoalHandle` has no `.accept()` — goals are auto-accepted by the
   `ActionServer`'s default `goal_callback`; removed the invalid call.
4. `rviz2` crashes in this headless container — added `use_rviz:=false` launch argument.
5. An example YAML transform's rotation submatrix had determinant -1 (a reflection, not a
   valid rotation) — fixed to a proper rotation (determinant +1).
6. An unexplained `SO3.Rz(pi) *` orientation flip in `compute_waypoints` was hurting
   reachability with no justification — removed.
7. **`compute_waypoints` never added the transform's translation** — waypoints were only
   rotated around the origin, never actually moved to the transform's intended position (e.g.
   `(0.5, 0, 0.2)`). This was the root cause of most of the low/inconsistent Cartesian path
   completion fractions observed during testing, and has been fixed.

### Reachability/orientation investigation findings
- Individual target positions were confirmed reachable via `compute_ik`; the limiting factor
  was orientation choice and the distance from the arm's current pose to the drawing area
  (e.g. home is at `(0.206, 0, 0.12)` vs. a drawing area around `(0.5-0.6, 0, 0.2-0.3)`),
  not raw position reachability.
- Forcing one fixed orientation across the *entire* path — including the long approach segment
  from home/current pose to the first shape point — was the dominant limiter on completion
  fraction. Splitting the motion into (1) a free-orientation `move_action` approach to the
  first point, then (2) a fixed-orientation `compute_cartesian_path` trace of the shape itself,
  is the current solution.

### Current status
Implementation complete and rebuilt; pending a fresh end-to-end test run by the user with the
translation-bug fix and two-phase (`move_action` + best-fixed-orientation `compute_cartesian_path`)
approach in place.
