# diffdrive_aruco_follow

ROS 2 Jazzy workspace for a Gazebo Sim leader-follower demo with two differential-drive robots.

The leader follows a predefined velocity profile and carries ArUco marker ID `42`. The follower detects the marker with its camera, publishes a debug image, and can continue following the stored marker path when the marker is temporarily lost.

## Demo

[![Leader-follower ArUco demo](docs/demo_thumbnail.png)](https://drive.google.com/file/d/1USltDKkg9fu8IxKernQhQIf-0ha4RkoM/view?usp=sharing)

## Packages

```text
src/
├── follower_scene/      # Gazebo world, URDF/Xacro, ros2_control, RViz, bridge config
└── follower_control/    # Leader/follower Python control nodes and helper modules
```

Main launch file:

```text
src/follower_control/launch/run.launch.py
```

It starts Gazebo Sim, RViz2, the Gazebo-ROS bridge, both robots, both controller managers, `rqt_image_view`, and the leader/follower control nodes.

## Dependencies

Target environment:

- ROS 2 Jazzy
- Gazebo Sim
- Python 3 with OpenCV and NumPy

Install the main dependencies:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt-image-view \
  ros-jazzy-tf2-tools \
  ros-jazzy-nav-msgs \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-numpy
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

For a single package:

```bash
colcon build --packages-select follower_scene
colcon build --packages-select follower_control
source install/setup.bash
```

Rebuild after changing launch files, config files, URDF/Xacro files, or Python entry points.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch follower_control run.launch.py
```

Do not launch `follower_scene bringup.launch.py` separately while `follower_control run.launch.py` is running. The main launch already includes the scene launch, so running both can duplicate Gazebo, `/clock`, bridges, or controllers.

If the scene is already running, the nodes can also be started manually:

```bash
ros2 run follower_control leader_control_node
ros2 run follower_control follower_control_node
```

## Runtime Behavior

- Robots run in separate namespaces: `/leader` and `/follower`.
- The leader starts at `(2.0, 0.0, 0.0)` and publishes commands to `/leader/diff_drive_controller/cmd_vel`.
- The follower starts at `(0.0, 0.0, 0.0)`, reads `/follower/camera/image_raw`, and publishes commands to `/follower/diff_drive_controller/cmd_vel`.
- Both diff-drive controllers use `geometry_msgs/msg/TwistStamped`.
- The follower publishes a processed camera stream on `/camera/image_debug`.
- The detected marker TF frame is `detected_aruco_42`.
- The path-memory target TF frame is `path_memory_target`.

Important follower defaults:

```text
marker_id = 42
marker_size = 0.1
desired_distance = 0.3
path_goal_tolerance = 0.15
max_linear_speed = 1.2
max_angular_speed = 2.0
use_path_memory = True
use_kalman_filter = True
```

## Useful Topics

```text
/clock
/follower/camera/image_raw
/camera/image_debug
/follower/scan
/leader/diff_drive_controller/cmd_vel
/follower/diff_drive_controller/cmd_vel
/leader/visual_path
/follower/visual_path
```

The bridge config is in:

```text
src/follower_scene/config/ros_gz_bridge.yaml
```

## Useful Files

```text
src/follower_control/follower_control/leader_control_node.py
src/follower_control/follower_control/follower_control_node.py
src/follower_control/follower_control/modules/aruco_detector.py
src/follower_scene/models/aruco_3/textures/marker_42.png
src/follower_scene/config/diff_leader.yaml
src/follower_scene/config/diff_follower.yaml
src/follower_scene/worlds/map.sdf
```

## Quick Checks

Controllers:

```bash
ros2 control list_controllers -c /leader/controller_manager
ros2 control list_controllers -c /follower/controller_manager
```

Expected:

```text
joint_state_broadcaster active
diff_drive_controller active
```

Camera and commands:

```bash
ros2 topic echo /follower/camera/image_raw --once
ros2 topic echo /camera/image_debug --once
ros2 topic echo /leader/diff_drive_controller/cmd_vel --once
ros2 topic echo /follower/diff_drive_controller/cmd_vel --once
```

TF:

```bash
ros2 run tf2_tools view_frames
```

The TF tree should include `leader/odom -> leader/base_link` and `follower/odom -> follower/base_link`, without duplicated frames such as `leader/leader/base_link`.

## Troubleshooting

### RViz Reports Time Jumps

Gazebo or the `/clock` bridge is probably running more than once. Stop duplicate launches and use only:

```bash
ros2 launch follower_control run.launch.py
```

### Controllers Are Not Active

Wait a few seconds after launch, then check both controller managers. If needed, rebuild and source again:

```bash
colcon build
source install/setup.bash
```

### Follower Does Not Move

Check that:

- The ArUco marker is visible to the follower camera.
- `/follower/camera/image_raw` is publishing.
- `/camera/image_debug` is publishing.
- `/follower/diff_drive_controller/cmd_vel` receives commands.
- The follower controller is active.

### Marker Is Detected but Motion Looks Wrong

Tune the control constants in:

```text
src/follower_control/follower_control/follower_control_node.py
```

Then rebuild:

```bash
colcon build --packages-select follower_control
source install/setup.bash
```
