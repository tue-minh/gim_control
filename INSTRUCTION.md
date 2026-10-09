# ROS2 gim_control Project

## Overview
This package provides a C++ ROS2 Humble library for controlling the GIM6010-8 motor via CAN bus. It wraps the ODrive interface functionality, provides a clean ROS2 API for motor control, and includes full robot descriptions (3-DOF arm), physics simulation (MuJoCo), a trajectory visualiser, and a real-hardware impedance controller node.

## Package Structure
```text
/home/tue/gim_control/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package manifest
├── include/ros2_gim_control/
│   └── gim_control_interface.hpp  # CAN control header file
├── src/
│   ├── robot_hardware/            # CAN interface (GimControlInterface)
│   ├── robot_kinematic/           # FK, IK, and Jacobian libraries
│   ├── robot_trajectory/          # Trajectory generation and CSV files
│   ├── robot_description/         # URDF, meshes, launch files, RViz configs
│   ├── trajectory_simulator_node.cpp # CSV Trajectory runner 
│   ├── impedance_simulator_node.cpp  # Real-hardware impedance controller (CAN)
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

### Option 2: MuJoCo Physics Simulation
> [!NOTE]
> `impedance_simulator_node` now drives the **real** motors (see Option 4) and no longer publishes to `/arm_effort_controller/commands`. For simulation, launch MuJoCo and drive it with `arm_controller` (Option 3).

Launch the robot in the MuJoCo physics engine (`arm_effort_controller` is loaded as active by default):
   ```bash
   ros2 launch gim_control mujoco.launch.py
   ```

### Option 3: Trajectory Control in MuJoCo
To use the standard ROS 2 position trajectory controller in simulation, switch the active controllers:
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

### Option 4: Real Hardware Impedance Control (`impedance_simulator_node`)
`impedance_simulator_node` drives the **real** motors over `can0` through `GimControlInterface`:
`tau = Kp * (q_ref - q) + Kd * (qd_ref - qd)` (joint space, Nm), sent in torque mode.

> [!WARNING]
> Bring `can0` up first. The defaults (gains, gear ratios, signs) are unverified placeholders: check the sign/scale of `/gim/joint_states` by moving the arm by hand with motors disabled, and test with small gains and the arm free to move. There is no gravity compensation.

**Behaviour**
- Starts with all motors **DISABLED** (idle). It only publishes feedback until you enable it.
- Enabling takes about 10 s (`InitTorqueMode` sleeps per motor); it runs in a worker thread so feedback keeps publishing.
- The reference starts at the current arm pose and moves at `max_speed` (rad/s) toward the target, so the arm never jumps. Targets are clamped to the URDF joint limits.
- Ctrl-C / SIGTERM: torque is zeroed and every motor is set to idle before exit.
- Faults (no encoder reply for `encoder_timeout` s, or a joint > 0.3 rad outside limits) disable all motors and are reported on `/gim/status`.

**Run**
```bash
ros2 run gim_control impedance_simulator_node
ros2 topic echo /gim/joint_states          # measured joint angles (rad)
```

**Enable modes** (`/gim/mode`, `std_msgs/String`)
```bash
# 1) follow joint_state_publisher_gui  (also run: ros2 launch gim_control display.launch.py)
ros2 topic pub --once /gim/mode std_msgs/msg/String "{data: gui}"
# 2) follow the CSV trajectory (first moves slowly to the start pose, then plays it)
ros2 topic pub --once /gim/mode std_msgs/msg/String "{data: trajectory}"
# disable all motors
ros2 topic pub --once /gim/mode std_msgs/msg/String "{data: disable}"
```
Switching between `gui` and `trajectory` while enabled does not re-initialise the motors.

**Topics**

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/gim/mode` | `std_msgs/String` | sub | `disable` \| `gui` \| `trajectory` |
| `/impedance_gains` | `Float64MultiArray` | sub | `[kp0 kp1 kp2 kd0 kd1 kd2]` |
| `/gim/torque_limit` | `Float64MultiArray` | sub | `[t0 t1 t2]` joint torque limit (Nm) |
| `/gim/max_speed` | `std_msgs/Float64` | sub | reference slew speed (rad/s) |
| `/gim/set_zero` | `std_msgs/Empty` | sub | take current pose as joint zero (only while disabled) |
| `/joint_states` (`gui_topic`) | `sensor_msgs/JointState` | sub | GUI target (from `joint_state_publisher_gui`) |
| `/gim/joint_states` | `sensor_msgs/JointState` | pub | measured angle (rad), velocity, commanded torque |
| `/gim/motor_raw` | `Float64MultiArray` | pub | `[turns0..2, turns/s0..2]` raw encoder values |
| `/gim/status` | `std_msgs/String` | pub | `DISABLED` \| `ENABLING` \| `GUI` \| `TRAJECTORY` (+ last fault) |

```bash
ros2 topic pub --once /impedance_gains std_msgs/msg/Float64MultiArray "{data: [5,10,5, 0.2,0.4,0.2]}"
ros2 topic pub --once /gim/torque_limit std_msgs/msg/Float64MultiArray "{data: [5,40,5]}"
ros2 topic pub --once /gim/max_speed std_msgs/msg/Float64 "{data: 0.3}"
ros2 topic pub --once /gim/set_zero std_msgs/msg/Empty "{}"
```

**Parameters** (`ros2 run ... --ros-args -p name:=value`)

| Parameter | Default | Description |
|---|---|---|
| `node_ids` | `[0,1,2]` | CAN node id per joint |
| `joint_names` | `[base_joint, shoulder_joint, elbow_joint]` | joint names |
| `gear_ratio` | `[8,64,8]` | rotor turns per joint turn |
| `invert_direction` | `[true,false,true]` | flips position/velocity sign |
| `torque_sign` | `[-1,1,-1]` | sign applied to the motor torque command |
| `torque_gear_ratio` | `[1,8,1]` | motor torque = joint torque / ratio |
| `torque_limit` | `[5,40,5]` | joint torque saturation (Nm) |
| `joint_lower` / `joint_upper` | URDF limits | reference clamp (rad) |
| `kp` / `kd` | `[5,10,5]` / `[0.2,0.4,0.2]` | impedance gains |
| `max_speed` | `0.3` | reference slew speed (rad/s) |
| `control_rate_hz` | `100` | control loop rate |
| `encoder_timeout` | `0.2` | encoder watchdog (s) |
| `loop_trajectory` | `false` | loop CSV trajectory (otherwise hold last pose) |
| `gui_topic` | `/joint_states` | GUI target topic |
| `csv_file_path` | `.../gim_arm_circle_traj.csv` | trajectory CSV |

Joint zero is the encoder value at power-on unless you call `/gim/set_zero`.

### Option 5: Real Hardware / Direct CAN Control (raw interface)
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

**Disabling:**
- `DisableMotor(int node_id)` - Zero the torque command and set `AXIS_STATE_IDLE`. Non-blocking (no sleeps), safe to call from shutdown paths. Sent twice for reliability.

**Reading Data:**
- `ReadEncoder(int node_id)` - Request encoder data via CAN
- `get_encoder_data(int node_id, float &pos, float &vel)` - Get position (turns) and velocity (turns/s)
- `get_torque_data(int node_id, float &torque)` - Get measured torque
- `has_encoder_data(int node_id) const` - Check if encoder data is available
- `encoder_sample_count(int node_id) const` - Number of encoder replies received so far (freshness watchdog)

*Notes: per-command `Send*Command` logs are at DEBUG level (they run at control-loop rate). The CAN reader uses a 100 ms receive timeout so it exits cleanly on shutdown.*

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
