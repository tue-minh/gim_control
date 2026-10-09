# ROS2 gim_control Project

## Overview
This package provides a C++ ROS2 Humble library for controlling the GIM6010-8 motor via CAN bus. It wraps the ODrive interface functionality, provides a clean ROS2 API for motor control, and includes full robot descriptions (3-DOF arm), physics simulation (MuJoCo), and trajectory/impedance simulation nodes.

## Package Structure
```text
/home/tue/gim_control/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package manifest
├── include/ros2_gim_control/
│   └── gim_control_interface.hpp  # CAN control header file
├── src/
│   ├── robot_hardware/            # CAN interface implementations
│   ├── robot_kinematic/           # FK, IK, and Jacobian libraries
│   ├── robot_trajectory/          # Trajectory generation and CSV files
│   ├── robot_description/         # URDF, meshes, launch files, RViz configs
│   ├── trajectory_simulator_node.cpp # CSV Trajectory runner 
│   ├── impedance_simulator_node.cpp  # Impedance control simulator
│   └── main.cpp                   # Main hardware node execution
└── reference/              # Reference ODrive interface (existing)
```

## Package Dependencies
Ensure you have the required ROS 2 dependencies installed, including MuJoCo and controllers:
```bash
sudo apt update
sudo apt install -y ros-humble-mujoco-ros2-control ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller
```
- Other dependencies: `rclcpp`, `std_msgs`, `sensor_msgs`, `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`

## Building the Project
```bash
cd /home/tue/gim_control
colcon build --packages-select gim_control
source install/setup.bash
```

## Launching and Simulation

### Option 1: Kinematic Trajectory Simulation (CSV Replay)
To visualize the pre-computed CSV trajectory (`gim_arm_circle_traj.csv`) in RViz without physics:
1. Launch the robot visualization with the GUI disabled (to avoid conflicting joint commands):
   ```bash
   ros2 launch gim_control display.launch.py gui:=false
   ```
2. In another terminal, run the trajectory simulator:
   ```bash
   ros2 run gim_control trajectory_simulator_node
   ```
*Note: If you just want to manually pose the robot with sliders, run `ros2 launch gim_control display.launch.py` (gui defaults to true).*

### Option 2: MuJoCo Physics & Impedance Control Simulation
The `impedance_simulator_node` computes joint efforts based on a PD control law: 
`tau = Kp * (q_des - q_act) + Kd * (qd_des - qd_act)`

It reads the CSV trajectory, subscribes to `/joint_states` (from MuJoCo), and publishes the resulting torque to `/arm_effort_controller/commands`.

1. Launch the robot in the MuJoCo physics engine. (Note: `arm_effort_controller` is loaded as active by default):
   ```bash
   ros2 launch gim_control mujoco.launch.py
   ```
2. In another terminal, run the impedance node to drive the simulated robot:
   ```bash
   ros2 run gim_control impedance_simulator_node
   ```

**Dynamic Parameter Updates:**
You can dynamically adjust the `Kp` and `Kd` gains while the node is running by publishing to `/impedance_gains` (`std_msgs/msg/Float64MultiArray`).
```bash
ros2 topic pub --once /impedance_gains std_msgs/msg/Float64MultiArray "{data: [20.0, 20.0, 20.0, 2.0, 2.0, 2.0]}"
```

### Option 3: Trajectory Control in MuJoCo
If you prefer to use the standard ROS 2 position trajectory controller instead of the impedance node, you can switch the active controllers:
```bash
ros2 control set_controller_state arm_effort_controller inactive
ros2 control set_controller_state arm_controller active
```
Then send trajectory commands to the `/arm_controller/joint_trajectory` topic. Example:
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

### Option 4: Real Hardware / Direct CAN Control
Combine the robot description with the real motor control node:
```bash
ros2 launch gim_control gim_control.launch.py
```
Alternatively, run the hardware node standalone:
```bash
ros2 run gim_control gim_control_node
```

---

## C++ API Reference (`GimControlInterface`)

### Construction
```cpp
GimControlInterface();
```
Initializes the node, opens CAN socket (can0), and starts the CAN reader thread.

### Motor Control Methods

**Velocity Control:**
- `InitVelocityMode(int node_id)` - Initialize velocity control mode
- `SendVelocityCommand(int node_id, float velocity)` - Send velocity command (rad/s)

**Position Control:**
- `InitPositionMode(int node_id)` - Initialize position control mode
- `SendPositionCommand(int node_id, float position, int16_t vel_ff = 0, int16_t torque_ff = 0)` - Send position command (turns)

**Torque Control:**
- `InitTorqueMode(int node_id)` - Initialize torque control mode
- `SendTorqueCommand(int node_id, float torque)` - Send torque command (Nm)

**Reading Data:**
- `ReadEncoder(int node_id)` - Request encoder data via CAN
- `get_encoder_data(int node_id, float &pos, float &vel)` - Get position and velocity
- `get_torque_data(int node_id, float &torque)` - Get measured torque
- `has_encoder_data(int node_id) const` - Check if encoder data is available

**PID & Trajectory Limits:**
- `SetPIDGains(int node_id, float pos_gain, float vel_gain, float vel_integrator_gain)` - Set PID gains
- `SetTrajectoryVelLimit(int node_id, float vel_limit)` - Set velocity limit
- `SetTrajectoryAccelLimit(int node_id, float accel_limit, float decel_limit)` - Set accel/decel limits

### Usage Example
```cpp
#include <ros2_gim_control/gim_control_interface.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_gim_control::GimControlInterface>();
    
    node->InitVelocityMode(0);
    node->SendVelocityCommand(0, 10.0);
    
    float pos, vel;
    node->get_encoder_data(0, pos, vel);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
