# avatar_challenge

`avatar_challenge` is a ROS 2 Humble package that accepts a drawing request as
a ROS 2 action. The request contains a 2D shape and a 4x4 transform describing
where that shape is located in the robot's `world` frame. The node converts the
shape into 3D Cartesian waypoints, asks MoveIt 2 to compute a straight-line
Cartesian path, and executes the resulting trajectory.

The action server is provided by `draw_node` on the `draw` action name. The
command-line client is `request_drawing.py`.

## Installation

```bash
cd ~ 
git clone git@github.com:bokorn/avatar-challenge.git
```

### Python dependencies

```bash
sudo apt update && sudo apt install python3-pip

pip3 install spatialmath-python[ros-humble]
```

Source the workspaces in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_ws/install/setup.bash
source ~/avatar-challenge/install/setup.bash
```

## Build

From the workspace root:

```bash
colcon build --packages-select avatar_challenge_msgs avatar_challenge --symlink-install
source install/setup.bash
```

## Run

Launch the fake xArm7 hardware, MoveIt 2, and the drawing action server:

```bash
ros2 launch avatar_challenge start.launch.py
```

After `draw_node action server ready` appears, send the default example from a
second sourced shell:

```bash
ros2 run avatar_challenge request_drawing
```

Send another YAML file with:

```bash
ros2 run avatar_challenge request_drawing.py /absolute/path/to/shape.yaml
```

The action supports feedback and cancellation. The node forwards cancellation
to MoveIt's trajectory action and accepts only one drawing goal at a time.

## YAML format

Each file contains a homogeneous transform and a shape sequence:

```yaml
transform:
  - [r00, r01, r02, tx]
  - [r10, r11, r12, ty]
  - [r20, r21, r22, tz]
  - [0.0, 0.0, 0.0, 1.0]

shape:
  - [v, x, y]
```
### Vertices

```yaml
- [v, x, y]
```

`x` and `y` are drawing-frame coordinates in meters. Points are executed in
the order listed.

### Circular arcs

```yaml
- [a, end_x, end_y, signed_radius]
```

The arc starts at the previous vertex and ends at `(end_x, end_y)`.
`signed_radius` is in meters. Positive values select counter-clockwise arcs;
negative values select clockwise arcs. An arc must follow a vertex.

The client samples arcs at approximately 0.01-meter resolution before sending
the action goal.

### B-splines

```yaml
- [b, x1, y1, x2, y2, x3, y3 ...]
```

The previous vertex is automatically prepended as the first control point.
The remaining coordinate pairs are additional control points. A B-spline must
follow a vertex and is sampled at approximately 0.01-meter resolution.

### Complete example

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

## Visualization

`draw_node` publishes the transformed vertices and connecting path as a
`visualization_msgs/MarkerArray` on `drawing_markers`. Add a MarkerArray display
in RViz and use `world` as the fixed frame.
