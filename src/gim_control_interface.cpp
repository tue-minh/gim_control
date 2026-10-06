#include <ros2_gim_control/gim_control_interface.hpp>
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

#define KV 170.0

namespace ros2_gim_control {

const double REV_TO_RAD = M_PI / 4.0;

GimControlInterface::GimControlInterface()
: Node("gim_control_interface", rclcpp::NodeOptions().use_global_arguments(false)),
  running_(true) {
    RCLCPP_INFO(this->get_logger(), "GIM Control Interface Initialized!");

    int joint_number = 3;
    declare_parameter("joint_number", 3);
    get_parameter("joint_number", joint_number);
    encoder_data_.resize(joint_number);

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CAN socket!");
        rclcpp::shutdown();
        return;
    }

    const int recv_own_msgs = 0;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot set CAN socket option!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot get CAN interface index!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot bind CAN socket!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    reader_thread_ = std::thread(&GimControlInterface::read_can_messages, this);
}

GimControlInterface::~GimControlInterface() {
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    if (sock_ >= 0) {
        close(sock_);
    }
}

void GimControlInterface::initialize_odrive(int node_id) {
    if (send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_IDLE for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_IDLE", node_id);
    usleep(2000000);
}

void GimControlInterface::InitVelocityMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Velocity Control Mode", node_id);
    initialize_odrive(node_id);

    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B;
    frame.can_dlc = 8;
    int32_t control_mode = 2;
    int32_t input_mode = 2;
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Controller_Modes for Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Set Velocity Control Mode for Node %d", node_id);
    usleep(2000000);

    frame.can_id = (node_id << 5) | 0x007;
    frame.can_dlc = 4;
    int32_t axis_state = 8;
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Closed Loop Control for Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Set Closed Loop Control for Node %d", node_id);
    usleep(2000000);
}

void GimControlInterface::InitPositionMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Position Control Mode", node_id);
    initialize_odrive(node_id);

    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B;
    frame.can_dlc = 8;
    int32_t control_mode = 3;
    int32_t input_mode = 1;
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Controller_Modes for Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Set Position Control Mode for Node %d", node_id);
    usleep(2000000);

    frame.can_id = (node_id << 5) | 0x007;
    frame.can_dlc = 4;
    int32_t axis_state = 8;
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Closed Loop Control for Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Set Closed Loop Control for Node %d", node_id);
    usleep(2000000);
}

void GimControlInterface::InitTorqueMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Torque Control Mode", node_id);
    initialize_odrive(node_id);

    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B;
    frame.can_dlc = 8;
    int32_t control_mode = 1;
    int32_t input_mode = 6;
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Controller_Modes for Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Set Torque Control Mode for Node %d", node_id);
    usleep(500000);

    frame.can_id = (node_id << 5) | 0x007;
    frame.can_dlc = 8;
    int32_t axis_state = 8;
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Closed Loop Control for Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Set Closed Loop Control for Node %d", node_id);
    usleep(3000000);
}

void GimControlInterface::SendVelocityCommand(int node_id, float velocity) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0D;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &velocity, sizeof(float));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending velocity command for motor %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sent velocity %.2f to ODrive ID %d", velocity, node_id);
    }
}

void GimControlInterface::SendPositionCommand(int node_id, float position, int16_t vel_ff, int16_t torque_ff) {
    float position_in_turns = position;
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0C;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &position_in_turns, sizeof(float));
    std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
    std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending position command for motor %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sent position %.2f turns to ODrive ID %d", position_in_turns, node_id);
    }
}

void GimControlInterface::SendTorqueCommand(int node_id, float torque) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0E;
    frame.can_dlc = sizeof(float);
    std::memcpy(frame.data, &torque, sizeof(float));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending torque command for Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sent Torque to Node %d: %.5f Nm", node_id, torque);
    }
}

void GimControlInterface::ReadEncoder(int node_id) {
    send_can_remote_frame(node_id, 0x009);
}

void GimControlInterface::get_encoder_data(int node_id, float &pos, float &vel) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        pos = encoder_data_[node_id].position;
        vel = encoder_data_[node_id].velocity;
    } else {
        pos = 0.0;
        vel = 0.0;
        RCLCPP_ERROR(this->get_logger(), "Invalid node_id: %d", node_id);
    }
}

void GimControlInterface::get_torque_data(int node_id, float &torque) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        torque = encoder_data_[node_id].torque;
    } else {
        torque = 0.0;
        RCLCPP_ERROR(this->get_logger(), "Invalid node_id: %d", node_id);
    }
}

bool GimControlInterface::has_encoder_data(int node_id) const {
    return node_id >= 0 &&
           static_cast<size_t>(node_id) < encoder_data_.size() &&
           encoder_data_[node_id].has_encoder_sample;
}

void GimControlInterface::SetPosGain(int node_id, float pos_gain) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x01A;
    frame.can_dlc = 4;
    std::memcpy(frame.data, &pos_gain, sizeof(float));
    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Pos_Gain for Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Set Pos_Gain = %.3f for Node %d", pos_gain, node_id);
    }
}

void GimControlInterface::SetVelGains(int node_id, float vel_gain, float vel_integrator_gain) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x01B;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &vel_gain, sizeof(float));
    std::memcpy(frame.data + 4, &vel_integrator_gain, sizeof(float));
    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Vel_Gains for Node %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Set Vel_Gain = %.3f, Vel_Integrator_Gain = %.3f for Node %d",
                    vel_gain, vel_integrator_gain, node_id);
    }
}

void GimControlInterface::SetPIDGains(int node_id, float pos_gain, float vel_gain, float vel_integrator_gain) {
    SetPosGain(node_id, pos_gain);
    usleep(100000);
    SetVelGains(node_id, vel_gain, vel_integrator_gain);
}

void GimControlInterface::SetTrajectoryVelLimit(int node_id, float vel_limit) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x011;
    frame.can_dlc = 4;
    std::memcpy(frame.data, &vel_limit, sizeof(float));
    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Trajectory_Vel_Limit for motor %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Set Trajectory_Vel_Limit = %.3f for motor %d", vel_limit, node_id);
    }
}

void GimControlInterface::SetTrajectoryAccelLimit(int node_id, float accel_limit, float decel_limit) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x012;
    frame.can_dlc = 8;
    std::memcpy(frame.data, &accel_limit, sizeof(float));
    std::memcpy(frame.data + 4, &decel_limit, sizeof(float));
    if (write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending Set_Trajectory_Accel_Limit for motor %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Set Trajectory_Accel_Limit = %.3f, Decel_Limit = %.3f for motor %d",
                    accel_limit, decel_limit, node_id);
    }
}

void GimControlInterface::read_can_messages() {
    struct can_frame frame;
    while (running_ && rclcpp::ok()) {
        ssize_t nbytes = read(sock_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN read error!");
            continue;
        }
        if (nbytes != sizeof(struct can_frame))
            continue;

        int node_id    = frame.can_id >> 5;
        uint8_t cmd_id = frame.can_id & 0x1F;

        if (node_id < 0 || static_cast<size_t>(node_id) >= encoder_data_.size())
            continue;

        switch (cmd_id) {
            case 0x09:
                process_params(node_id, frame.data);
                break;
            case 0x14:
                process_torque(node_id, frame.data);
                break;
            default:
                break;
        }
    }
}

void GimControlInterface::process_params(int node_id, uint8_t* data) {
    float pos_estimate, vel_estimate;
    std::memcpy(&pos_estimate, data, sizeof(float));
    std::memcpy(&vel_estimate, data + 4, sizeof(float));

    encoder_data_[node_id].position = pos_estimate;
    encoder_data_[node_id].velocity = vel_estimate;
    encoder_data_[node_id].has_encoder_sample = true;
}

void GimControlInterface::process_torque(int node_id, uint8_t* data) {
    float iq_set = 0.0f;
    float iq_meas = 0.0f;

    std::memcpy(&iq_set,  data,     sizeof(float));
    std::memcpy(&iq_meas, data + 4, sizeof(float));

    float torque_actual = (iq_meas * 8.27f) / KV;
    encoder_data_[node_id].torque = torque_actual;
}

int GimControlInterface::send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | cmd_id;
    frame.can_dlc = 8;
    size_t i = 0;
    for (auto byte : data) {
        if (i < 8) frame.data[i++] = byte;
    }
    while (i < 8) frame.data[i++] = 0;
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return -1;
    }
    return 0;
}

int GimControlInterface::send_can_remote_frame(int node_id, uint8_t cmd_id, uint8_t dlc) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = ((node_id << 5) | cmd_id) | CAN_RTR_FLAG;
    frame.can_dlc = dlc;
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return -1;
    }
    return 0;
}

}  // namespace ros2_gim_control