# ROS2 Humble Library for GIM6010-8 Motor Control

## Overview
This package provides a C++ ROS2 Humble library for controlling the GIM6010-8 motor via CAN bus. It wraps the ODrive interface functionality and provides a clean ROS2 API for motor control.

## Folder Structure
```
/home/tue/gim_control/
├── CMakeLists.txt          # Build configuration
├── include/ros2_gim_control/
│   └── gim_control_interface.hpp  # Main header file
├── src/
│   └── gim_control_interface.cpp  # Implementation file
├── package.xml             # ROS2 package manifest
└── reference/              # Reference ODRIave interface (existing)
    ├── odrive_interface.hpp
    └── odrive_interface.cpp
```

## API Reference

### Class: `GimControlInterface`
Inherits from `rclcpp::Node`.

#### Construction
```cpp
GimControlInterface();
```
Initializes the node, opens CAN socket (can0), and starts the CAN reader thread.

#### Motor Control Methods

**Velocity Control:**
- `InitVelocityMode(int node_id)` - Initialize velocity control mode
- `SendVelocityCommand(int node_id, float velocity)` - Send velocity command (rad/s)

**Position Control:**
- `InitPositionMode(int node_id)` - Initialize position control mode
- `SendPositionCommand(int node_id, float position, int16_t vel_ff = 0, int16_t torque_ff = 0)` - Send position command (turns)
- `SendPositionCommandRad(int node_id, float position_rad, ...)` - Send position in radians

**Torque Control:**
- `InitTorqueMode(int node_id)` - Initialize torque control mode
- `InitTorqueMode(int node_id, int input_mode)` - Initialize with specific input mode (1=PASSTHROUGH, 6=TORQUE_RAMP)
- `SendTorqueCommand(int node_id, float torque)` - Send torque command (Nm)

**Reading Data:**
- `ReadEncoder(int node_id)` - Request encoder data via CAN
- `get_encoder_data(int node_id, float &pos, float &vel)` - Get position and velocity
- `get_torque_data(int node_id, float &torque)` - Get measured torque
- `has_encoder_data(int node_id) const` - Check if encoder data is available

**PID Gains:**
- `SetPosGain(int node_id, float pos_gain)` - Set position gain
- `SetVelGains(int node_id, float vel_gain, float vel_integrator_gain)` - Set velocity gains
- `SetPIDGains(int node_id, float pos_gain, float vel_gain, float vel_integrator_gain)` - Set both gains

**Trajectory Control:**
- `SetTrajectoryVelLimit(int node_id, float vel_limit)` - Set velocity limit
- `SetTrajectoryAccelLimit(int node_id, float accel_limit, float decel_limit)` - Set acceleration/deceleration limits
- `SetTrajectoryInertia(int node_id, float inertia)` - Set inertia

**Utility:**
- `SaveConfiguration(int node_id)` - Save ODrive configuration
- `Reboot(int node_id)` - Reboot ODrive

## Configuration
- Default CAN interface: `can0`
- Motor KV constant: 170.0 (used for torque conversion)
- RAD_TO_REV: 4.0 / π
- REV_TO_RAD: π / 4

## Building
```bash
cd /home/tue/gim_control
colcon build --packages-select gim_control
```

## Usage Example
```cpp
#include <ros2_gim_control/gim_control_interface.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_gim_control::GimControlInterface>();
    
    // Initialize motor 0 in velocity mode
    node->InitVelocityMode(0);
    
    // Send 10 rad/s velocity command
    node->SendVelocityCommand(0, 10.0);
    
    // Read encoder data
    float pos, vel;
    node->get_encoder_data(0, pos, vel);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```