#include <can_odrive_interface/odrive_interface.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include "std_msgs/msg/float32.hpp"
#include <unordered_map>


#define KV 170.0  // Giá trị KV của động cơ

namespace can_odrive_interface {

const double REV_TO_RAD = M_PI / 4.0; // 1 turn = π/4 radian

// use_global_arguments(false): ODriveInterface được tạo làm member bên trong các
// node khác (ImpedanceControllerNode, ImpedanceResistanceNode...). Nếu dùng
// global arguments mặc định, remap "-r __node:=X" của tiến trình cha sẽ đổi luôn
// tên node này trùng với node cha -> 2 node cùng tên "/X", khiến ros2 param
// set/get có thể gửi nhầm sang node sai.
ODriveInterface::ODriveInterface()
: Node("odrive_interface", rclcpp::NodeOptions().use_global_arguments(false)),
  running_(true) {
    RCLCPP_INFO(this->get_logger(), "ODrive Interface Initialized!");

    // Lấy joint_number từ parameters (nếu có)
    int joint_number = 3; // Mặc định là 4, có thể lấy từ ROS2 parameters
    declare_parameter("joint_number", 3);
    get_parameter("joint_number", joint_number);
    encoder_data_.resize(joint_number);

    // Tạo socket CAN
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể mở socket CAN!");
        rclcpp::shutdown();
        return;
    }

    const int recv_own_msgs = 0;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể tắt nhận lại CAN frame do chính socket gửi!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể lấy chỉ mục CAN interface!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể kết nối CAN!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    // Khởi tạo reader thread để đọc tin nhắn CAN
    reader_thread_ = std::thread(&ODriveInterface::read_can_messages, this);

    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> torque_publishers_;

}

ODriveInterface::~ODriveInterface() {
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    if (sock_ >= 0) {
        close(sock_);
    }
}

void ODriveInterface::initialize_odrive(int node_id) {
    // Đặt trạng thái IDLE
    if (send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_IDLE for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_IDLE", node_id);
    usleep(2000000);
}

// void ODriveInterface::InitVelocityMode(int node_id) {
//     RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Velocity Control Mode", node_id);

//     initialize_odrive(node_id);

//     // Đặt chế độ điều khiển VELOCITY_CONTROL (0x02) và PASSTHROUGH (0x01)
//     if (send_can_frame(node_id, 0x0B, {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set VELOCITY_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to VELOCITY_CONTROL with PASSTHROUGH", node_id);
//     usleep(2000000);

//     // Kích hoạt CLOSED_LOOP_CONTROL
//     if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
//     usleep(2000000);

//     RCLCPP_INFO(this->get_logger(), "Node %d Velocity Mode initialization completed", node_id);
// }

void ODriveInterface::InitVelocityMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Velocity Control Mode", node_id);

    initialize_odrive(node_id);

    // Đặt chế độ điều khiển VELOCITY_CONTROL (0x02) và PASSTHROUGH (0x01)
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B; // Set_Controller_Modes
    frame.can_dlc = 8;
    int32_t control_mode = 2; // Velocity Control
    int32_t input_mode = 2;   // 1:Pass-through 5:Trajectory
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Set_Controller_Modes cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Velocity Control Mode cho Node %d", node_id);
    usleep(2000000);

    // Kích hoạt CLOSED_LOOP_CONTROL
    frame.can_id = (node_id << 5) | 0x007; // Set_Axis_Requested_State
    frame.can_dlc = 4;
    int32_t axis_state = 8; // Closed Loop Control
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Closed Loop Control cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Closed Loop Control cho Node %d", node_id);
    usleep(2000000);

    RCLCPP_INFO(this->get_logger(), "Node %d Velocity Mode initialization completed", node_id);
}

void ODriveInterface::InitPositionMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Position Control Mode", node_id);

    initialize_odrive(node_id);

    // Đặt chế độ điều khiển POSITION_CONTROL (0x03) và PASSTHROUGH (0x01)
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B; // Set_Controller_Modes
    frame.can_dlc = 8;
    int32_t control_mode = 3; // Position Control
    int32_t input_mode = 1;   // 1:Pass-through 5:Trajectory
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Set_Controller_Modes cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Position Control Mode cho Node %d", node_id);
    usleep(2000000);

    // Kích hoạt CLOSED_LOOP_CONTROL
    frame.can_id = (node_id << 5) | 0x007; // Set_Axis_Requested_State
    frame.can_dlc = 4;
    int32_t axis_state = 8; // Closed Loop Control
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Closed Loop Control cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Closed Loop Control cho Node %d", node_id);
    usleep(2000000);

    RCLCPP_INFO(this->get_logger(), "Node %d Position Mode initialization completed", node_id);
}

void ODriveInterface::InitPositionFilterMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Position Control + POS_FILTER", node_id);

    initialize_odrive(node_id);

    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B; // Set_Controller_Modes
    frame.can_dlc = 8;
    int32_t control_mode = 3; // Position Control
    int32_t input_mode = 3;   // Pos Filter
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Set_Controller_Modes POS_FILTER cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Position Control + POS_FILTER cho Node %d", node_id);
    usleep(2000000);

    frame.can_id = (node_id << 5) | 0x007; // Set_Axis_Requested_State
    frame.can_dlc = 4;
    int32_t axis_state = 8; // Closed Loop Control
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Closed Loop Control cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Closed Loop Control cho Node %d", node_id);
    usleep(2000000);

    RCLCPP_INFO(this->get_logger(), "Node %d Position POS_FILTER initialization completed", node_id);
}

// void ODriveInterface::InitTorqueMode(int node_id) {
//     RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Torque Control Mode", node_id);

//     initialize_odrive(node_id);

//     // Đặt chế độ điều khiển TORQUE_CONTROL (0x01) và PASSTHROUGH (0x01)
//     if (send_can_frame(node_id, 0x0B, {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set TORQUE_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to TORQUE_CONTROL with PASSTHROUGH", node_id);
//     usleep(2000000);

//     // Kích hoạt CLOSED_LOOP_CONTROL
//     if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
//     usleep(2000000);

//     RCLCPP_INFO(this->get_logger(), "Node %d Torque Mode initialization completed", node_id);
// }

void ODriveInterface::InitTorqueMode(int node_id) {
    InitTorqueMode(node_id, 6);  // default: TORQUE_RAMP
}

void ODriveInterface::InitTorqueMode(int node_id, int input_mode) {
    const char* input_mode_name = (input_mode == 1) ? "PASSTHROUGH" :
                                  (input_mode == 6) ? "TORQUE_RAMP" : "CUSTOM";
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d: Torque %s Mode", node_id, input_mode_name);
    initialize_odrive(node_id);  // → IDLE + delay 2s

    struct can_frame frame;

    // Bước 1: Set TORQUE_CONTROL + selected input mode.
    frame.can_id = (node_id << 5) | 0x00B;
    frame.can_dlc = 8;
    int32_t control_mode = 1; // TORQUE_CONTROL
    int32_t input_mode_i32 = input_mode;
    std::memcpy(frame.data,     &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode_i32, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi Set_Controller_Modes Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d: TORQUE_CONTROL + %s", node_id, input_mode_name);
    usleep(500000); // 500ms

    // Bước 2: CLOSED_LOOP_CONTROL
    // frame.can_id = (node_id << 5) | 0x007;
    // frame.can_dlc = 4;
    // int32_t axis_state = 8;
    // std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    // std::memset(frame.data + 4, 0, 4);
    frame.can_id = (node_id << 5) | 0x007;
    frame.can_dlc = 8;  // ← đổi từ 4 thành 8
    int32_t axis_state = 8;
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi Closed Loop Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d: CLOSED_LOOP_CONTROL", node_id);
    usleep(3000000); // 3s chờ axis vào closed loop

    RCLCPP_INFO(this->get_logger(), "Node %d Torque %s Mode ready", node_id, input_mode_name);
}

void ODriveInterface::SendVelocityCommand(int node_id, float velocity) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0D; // Set_Input_Vel
    frame.can_dlc = 8;
    std::memcpy(frame.data, &velocity, sizeof(float));
    std::memset(frame.data + 4, 0, 4);
    if ( write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh CAN cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Gửi vận tốc %.2f đến ODrive ID %d", velocity, node_id);
    }
}

void ODriveInterface::SendVelocityCommand_nolog(int node_id, float velocity) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0D; // Set_Input_Vel
    frame.can_dlc = 8;
    std::memcpy(frame.data, &velocity, sizeof(float));
    std::memset(frame.data + 4, 0, 4);
    if ( write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh CAN cho động cơ %d!", node_id);
    } 
}

// void ODriveInterface::SendPositionCommand(int node_id, float position) {
//     float position_in_turns = position; // Giả sử position đã ở đơn vị turns
//     struct can_frame frame;
//     frame.can_id = (node_id << 5) | 0x0C; // Set_Input_Pos
//     frame.can_dlc = 8;
//     int16_t vel_ff = 0; // Vận tốc feedforward
//     int16_t torque_ff = 0; // Mô-men xoắn feedforward
//     std::memcpy(frame.data, &position_in_turns, sizeof(float));
//     std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
//     std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh vị trí CAN cho động cơ %d!", node_id);
//     } else {
//         RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f turns đến ODrive ID %d", position_in_turns, node_id);
//     }
// }

void ODriveInterface::SendPositionCommand(int node_id, float position, int16_t vel_ff, int16_t torque_ff) {//torque có đơn vị là Nm/1000
    float position_in_turns = position; // Giả sử position đã ở đơn vị turns
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0C; // Set_Input_Pos
    frame.can_dlc = 8;
    std::memcpy(frame.data, &position_in_turns, sizeof(float));
    std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
    std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh vị trí CAN cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f turns đến ODrive ID %d", position_in_turns, node_id);
    }
}

void ODriveInterface::SendPositionCommand_nolog(int node_id, float position, int16_t vel_ff, int16_t torque_ff) {//torque có đơn vị là Nm/1000
    float position_in_turns = position; // Giả sử position đã ở đơn vị turns
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0C; // Set_Input_Pos
    frame.can_dlc = 8;
    std::memcpy(frame.data, &position_in_turns, sizeof(float));
    std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
    std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh vị trí CAN cho động cơ %d!", node_id);
    } 
}

void ODriveInterface::SendTorqueCommand(int node_id, float torque) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0E; // Set_Input_Torque
    frame.can_dlc = sizeof(float);
    std::memcpy(frame.data, &torque, sizeof(float));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node %d", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sent Torque to Node %d: %.5f Nm over CAN", node_id, torque);
    }
}


void ODriveInterface::SendTorqueCommand_nolog(int node_id, float torque) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0E; // Set_Input_Torque
    frame.can_dlc = sizeof(float);
    std::memcpy(frame.data, &torque, sizeof(float));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node %d", node_id);
    } 
}

void ODriveInterface::SendPositionCommandRad(int node_id, float position_rad, int16_t vel_ff, int16_t torque_ff) {
    float position_in_turns = position_rad * RAD_TO_REV;
    SendPositionCommand_nolog(node_id, position_in_turns, vel_ff, torque_ff);
}

void ODriveInterface::SetPosGain(int node_id, float pos_gain) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x01A;  // Set_Pos_Gain
    frame.can_dlc = 4;
    std::memcpy(frame.data, &pos_gain, sizeof(float));

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Set_Pos_Gain cho Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Đặt Pos_Gain = %.3f cho Node %d", pos_gain, node_id);
    }
}

void ODriveInterface::SetVelGains(int node_id, float vel_gain, float vel_integrator_gain) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x01B;  // Set_Vel_Gains
    frame.can_dlc = 8;
    std::memcpy(frame.data, &vel_gain, sizeof(float));
    std::memcpy(frame.data + 4, &vel_integrator_gain, sizeof(float));

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Set_Vel_Gains cho Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Đặt Vel_Gain = %.3f, Vel_Integrator_Gain = %.3f cho Node %d",
                    vel_gain, vel_integrator_gain, node_id);
    }
}

void ODriveInterface::SetPIDGains(int node_id, float pos_gain, float vel_gain, float vel_integrator_gain) {
    SetPosGain(node_id, pos_gain);
    usleep(100000); // Đợi 100ms giữa hai lệnh
    SetVelGains(node_id, vel_gain, vel_integrator_gain);
}

// void ODriveInterface::process_torque(int node_id, uint8_t* data) {
//     float torque_setpoint = 0.0f;
//     float torque_actual = 0.0f;

//     std::memcpy(&torque_setpoint, data, sizeof(float));
//     std::memcpy(&torque_actual, data + 4, sizeof(float));

//     encoder_data_[node_id].torque = torque_actual;

//     RCLCPP_INFO(this->get_logger(),
//         "Node %d | Torque_Setpoint = %.3f Nm | Torque_Actual = %.3f Nm",
//         node_id, torque_setpoint, torque_actual);
// }


//===================Trajectory control========================

void ODriveInterface::SetTrajectoryVelLimit(int node_id, float vel_limit){
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x011;  // Set_Trajectory_Vel_Limit
    frame.can_dlc = 4;
    std::memcpy(frame.data, &vel_limit, sizeof(float));

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Set_Trajectory_Vel_Limit cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Đặt Trajectory_Vel_Limit = %.3f cho động cơ %d", vel_limit, node_id);
    }
}

void ODriveInterface::SetTrajectoryAccelLimit(int node_id, float accel_limit, float decel_limit){
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x012;  // Set_Trajectory_Accel_Limit
    frame.can_dlc = 8;
    std::memcpy(frame.data, &accel_limit, sizeof(float));
    std::memcpy(frame.data + 4, &decel_limit, sizeof(float)); 

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Set_Trajectory_Accel_Limit cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Đặt Trajectory_Accel_Limit = %.3f và Trajectory_Decel_Limit = %.3f cho động cơ %d", accel_limit, decel_limit, node_id);
    }
}

void ODriveInterface::SetTrajectoryInertia(int node_id, float inertia){
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x013;  // Set_Trajectory_Inertia
    frame.can_dlc = 4;
    std::memcpy(frame.data, &inertia, sizeof(float));

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi Set_Trajectory_Inertia cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Đặt Trajectory_Inertia = %.3f cho động cơ %d", inertia, node_id);
    }
}

//==========================Lưu cấu hình và khởi động lại=======================
void ODriveInterface::SaveConfiguration(int node_id) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x01F;  // Save_Configuration
    frame.can_dlc = 0;  // Không có dữ liệu

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Save_Configuration to Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sending Save_Configuration to Node %d successfully!", node_id);
    }

    // Theo khuyến nghị, cần đợi một chút để flash ghi xong (~2s)
    usleep(2000000);
}

void ODriveInterface::Reboot(int node_id) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x016;  // Reboot
    frame.can_dlc = 0;  // Không có dữ liệu

    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Reboot to Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sending Reboot to Node %d successfully!", node_id);
    }

    // Sau khi gửi reboot, nên chờ ODrive khởi động lại (~5s)
    usleep(5000000);
}

// void ODriveInterface::ReadEncoder() {
//     // Gửi yêu cầu dữ liệu encoder cho cả hai node
//     send_can_frame(0, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
//     send_can_frame(1, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
// }

void ODriveInterface::ReadEncoder() {
    for (size_t i = 0; i < encoder_data_.size(); ++i) {
        send_can_remote_frame(static_cast<int>(i), 0x009);
    }
}

void ODriveInterface::ReadEncoder(int node_id) {
    send_can_remote_frame(node_id, 0x009);
}
void ODriveInterface::ReadTorque(int node_id) {
    // for (size_t i = 0; i < encoder_data_.size(); ++i) {
    //     // ODrive yêu cầu 8 byte 0 để request Get_Iq
    //     send_can_frame(i, 0x014, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    // }
    send_can_remote_frame(node_id, 0x014);
}

// void ODriveInterface::ReadTorque() {
//     for (size_t i = 0; i < encoder_data_.size(); ++i) {
//         send_can_frame(i, 0x01c, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
//     }
// }

// void ODriveInterface::read_can_messages() {
//     struct can_frame frame;
//     while (running_ && rclcpp::ok()) {
//         ssize_t nbytes = read(sock_, &frame, sizeof(struct can_frame));
//         if (nbytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Lỗi đọc dữ liệu CAN!");
//             continue;
//         }
//         if (nbytes == sizeof(struct can_frame)) {
//             int node_id = frame.can_id >> 5;
//             if ((frame.can_id & 0x1F) == 0x009 && static_cast<size_t>(node_id) < encoder_data_.size()) {
//                 process_params(node_id, frame.data);
//             }

//         }
//     }
// }

void ODriveInterface::read_can_messages() {
    struct can_frame frame;
    while (running_ && rclcpp::ok()) {
        ssize_t nbytes = read(sock_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi đọc dữ liệu CAN!");
            continue;
        }

        if (nbytes != sizeof(struct can_frame))
            continue;

        int node_id    = frame.can_id >> 5;
        uint8_t cmd_id = frame.can_id & 0x1F;

        if (node_id < 0 || static_cast<size_t>(node_id) >= encoder_data_.size())
            continue;

        switch (cmd_id) {
            case 0x09:   // Get_Encoder_Estimates → pos/vel
                process_params(node_id, frame.data);
                break;

            case 0x14:   // Get_Iq → dùng để tính torque
                process_torque(node_id, frame.data);
                break;

            default:
                // Bỏ qua các command khác
                break;
        }
    }
}





void ODriveInterface::get_encoder_data(int node_id, float &pos, float &vel) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        pos = encoder_data_[node_id].position;
        vel = encoder_data_[node_id].velocity;
    } else {
        pos = 0.0;
        vel = 0.0;
        RCLCPP_ERROR(this->get_logger(), "Invalid node_id: %d", node_id);
    }
}

void ODriveInterface::get_torque_data(int node_id, float &torque) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        torque = encoder_data_[node_id].torque;
    } else {
        torque = 0.0;
        RCLCPP_ERROR(this->get_logger(), "Invalid node_id: %d", node_id);
    }
}

void ODriveInterface::get_encoder_data_nolog(int node_id, float &pos, float &vel) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        pos = encoder_data_[node_id].position;
        vel = encoder_data_[node_id].velocity;
    }
}

bool ODriveInterface::has_encoder_data(int node_id) const {
    return node_id >= 0 &&
           static_cast<size_t>(node_id) < encoder_data_.size() &&
           encoder_data_[node_id].has_encoder_sample;
}

void ODriveInterface::get_torque_data_nolog(int node_id, float &torque) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        torque = encoder_data_[node_id].torque;
    } 
}

void ODriveInterface::process_params(int node_id, uint8_t* data) {
    float pos_estimate, vel_estimate;
    std::memcpy(&pos_estimate, data, sizeof(float));
    std::memcpy(&vel_estimate, data + 4, sizeof(float));

    // Lưu dữ liệu vào encoder_data_
    encoder_data_[node_id].position = pos_estimate;
    encoder_data_[node_id].velocity = vel_estimate;
    encoder_data_[node_id].has_encoder_sample = true;

    // RCLCPP_INFO(this->get_logger(), "Node %d: Pos_Estimate = %.3f turns, Vel_Estimate = %.3f turns/s",
    //             node_id, pos_estimate, vel_estimate);
}

void ODriveInterface::process_torque(int node_id, uint8_t* data) {
    float iq_set  = 0.0f;
    float iq_meas = 0.0f;

    std::memcpy(&iq_set,  data,     sizeof(float));
    std::memcpy(&iq_meas, data + 4, sizeof(float));

    // Chuyển Iq (A) -> torque (Nm), nếu bạn dùng chuẩn ODrive: T = Iq * 8.27 / KV
    float torque_actual = (iq_meas * 8.27f) / KV;   // KV phải là hằng số bạn định nghĩa

    encoder_data_[node_id].torque = torque_actual;

    // RCLCPP_INFO(this->get_logger(),
    //     "Node %d | iq_meas = %.3f A -> Torque_Actual = %.3f Nm",
    //     node_id, iq_meas, torque_actual);
}

int ODriveInterface::send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | cmd_id;
    frame.can_dlc = 8;
    size_t i = 0;
    for (auto byte : data) {
        if (i < 8) frame.data[i++] = byte;
    }
    while (i < 8) frame.data[i++] = 0;
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        // RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
        return -1;
    }
    // RCLCPP_INFO(this->get_logger(), "Gửi frame CAN cho node %d, cmd_id 0x%02x", node_id, cmd_id);
    return 0;
}

int ODriveInterface::send_can_remote_frame(int node_id, uint8_t cmd_id, uint8_t dlc) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = ((node_id << 5) | cmd_id) | CAN_RTR_FLAG;
    frame.can_dlc = dlc;
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return -1;
    }
    return 0;
}

}  // namespace can_odrive_interface
