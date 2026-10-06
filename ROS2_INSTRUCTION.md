# ROS2 gim_control Project - Build and Launch Instructions

## Package Structure

This project has been consolidated into a single ROS2 package:

| Package | Location | Description |
|---------|----------|-------------|
| `gim_control` | `/home/tue/gim_control/` | Motor control interface via CAN bus & URDF robot description (3 DOF arm) |

**Note:** The robot description (URDF and meshes) is integrated directly into the `gim_control` package.

## Package Dependencies

First, ensure you have the required ROS 2 dependencies installed, including MuJoCo and controllers:

```bash
sudo apt update
sudo apt install -y ros-humble-mujoco-ros2-control ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller
```

- Other dependencies: `rclcpp`, `std_msgs`, `sensor_msgs`, `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`

## Building the Project

Build the package:

```bash
cd /home/tue/gim_control
colcon build --packages-select gim_control
```

After building, source the setup script:

```bash
source /home/tue/gim_control/install/setup.bash
```

## Launching the Robot

### Option 1: MuJoCo Physics Simulation (Recommended)

This launches the robot in the MuJoCo physics engine, running `mujoco_ros2_control` with standard ROS 2 controllers:

```bash
ros2 launch gim_control mujoco.launch.py
```

### Option 2: Launch robot description only (URDF + RViz + joint state publisher GUI)

Useful for visual inspection of the kinematic model without physics simulation:

```bash
ros2 launch gim_control display.launch.py
```

### Option 3: Real Hardware / Direct CAN Control

The `gim_control.launch.py` file combines the robot description with the real motor control node:

```bash
ros2 launch gim_control gim_control.launch.py
```

Alternatively, you can run the node standalone:

```bash
ros2 run gim_control gim_control_node
```

## Controlling the Simulation

When running MuJoCo (`mujoco.launch.py`), a `joint_trajectory_controller` is spawned under the name `arm_controller`. You can send trajectory commands to it via the `/arm_controller/joint_trajectory` topic.

For example, to test moving the robot arm:
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint'],
    points: [
      { positions: [0.5, 0.5, 0.5], time_from_start: { sec: 2, nanosec: 0 } }
    ]
  }
}"
```
