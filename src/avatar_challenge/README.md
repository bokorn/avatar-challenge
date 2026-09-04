# avatar_challenge

This package provides `draw_node`, a ROS2 action server that accepts a 2D shape
(as a list of points) plus a `geometry_msgs/Transform` describing where to draw
it, and executes a straight-line end-effector Cartesian trajectory through
MoveIt2's `move_group` (via its native `compute_cartesian_path` service and
`execute_trajectory` action — there is no `moveit_commander`/`moveit_py` on
ROS2 Humble, so this package talks to `move_group` directly).

## Requirements

- **Ubuntu 22.04** + **ROS2 Humble** (`/opt/ros/humble`).
- **MoveIt2 for Humble** (apt):
  ```bash
  sudo apt install ros-humble-moveit ros-humble-moveit-ros-planning-interface \
                    ros-humble-moveit-ros-move-group ros-humble-moveit-msgs
  ```
- **xarm_ros2 (`humble` branch)** — built from source, e.g.:
  ```bash
  mkdir -p ~/xarm_ws/src && cd ~/xarm_ws/src
  git clone -b humble https://github.com/xArm-Developer/xarm_ros2.git --recursive
  cd ~/xarm_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build
  ```
  This workspace's `setup.bash` must be sourced (after `/opt/ros/humble`, before
  building/running this package) so that `xarm_moveit_config` and the other
  `xarm_*` packages are on the `AMENT_PREFIX_PATH`:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/xarm_ws/install/setup.bash
  ```
- **Python packages**:
  ```bash
  pip install spatialmath-python
  ```
  (`PyYAML` and `numpy` are typically already present via `python3-yaml`/ROS.)

## Build

```bash
colcon build --packages-select avatar_challenge_msgs avatar_challenge --symlink-install
```

Source overlay:
```bash
source install/setup.bash
```

## Launch

Launches the xArm7 fake MoveIt stack (via `xarm_moveit_config`) plus `draw_node`:
```bash
ros2 launch avatar_challenge draw.launch.py
```

> Note: this includes `xarm_moveit_config`'s `xarm7_moveit_fake.launch.py`,
> which also starts `rviz2`. In a headless/no-display environment (e.g. a
> devcontainer without X11), `rviz2` will fail to start and the launch's exit
> handler will shut down the whole launch. Either provide a display (e.g.
> X11/VNC forwarding) or launch `move_group`/`draw_node` without rviz.

## Test

With the launch running, send a goal from a YAML shape file:
```bash
ros2 run avatar_challenge test_draw_action.py <path/to/shape.yaml>
```
