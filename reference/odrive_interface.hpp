#ifndef CAN_ODRIVE_INTERFACE_HPP
#define CAN_ODRIVE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <unordered_map>
#include <std_msgs/msg/float32.hpp>

namespace can_odrive_interface {

class ODriveInterface : public rclcpp::Node {
public:
    static constexpr double RAD_TO_REV = 4.0 / M_PI; 
    ODriveInterface();
    ~ODriveInterface();

    void InitVelocityMode(int node_id);
    void InitPositionMode(int node_id);
    void InitPositionFilterMode(int node_id);
    void InitTorqueMode(int node_id);
    void InitTorqueMode(int node_id, int input_mode);
    void SendVelocityCommand(int node_id, float velocity);
    void SendVelocityCommand_nolog(int node_id, float velocity);
    void SendPositionCommand(int node_id, float position, int16_t vel_ff, int16_t torque_ff);
    void SendPositionCommand_nolog(int node_id, float position, int16_t vel_ff, int16_t torque_ff);
    void SendPositionCommandRad(int node_id, float position_rad, int16_t vel_ff, int16_t torque_ff);
    void SendTorqueCommand(int node_id, float torque);
    void SendTorqueCommand_nolog(int node_id, float torque);
    void ReadEncoder();
    void ReadEncoder(int node_id);
    void ReadTorque(int node_id);
    void get_encoder_data(int node_id, float &pos, float &vel);
    void get_torque_data(int node_id, float &torque);
    void get_encoder_data_nolog(int node_id, float &pos, float &vel);
    void get_torque_data_nolog(int node_id, float &torque);
    bool has_encoder_data(int node_id) const;
    void process_torque(int node_id, uint8_t* data);

    // set parameters PID
    void SetPosGain(int node_id, float pos_gain);
    void SetVelGains(int node_id, float vel_gain, float vel_integrator_gain);
    void SetPIDGains(int node_id, float pos_gain, float vel_gain, float vel_integrator_gain);
    // trajectory control
    void SetTrajectoryVelLimit(int node_id, float vel_limit);
    void SetTrajectoryAccelLimit(int node_id, float accel_limit, float decel_limit);
    void SetTrajectoryInertia(int node_id, float inertia);
    
    void SaveConfiguration(int node_id);
    void Reboot(int node_id);


private:
    struct EncoderData {
        float position = 0.0;
        float velocity = 0.0;
        float torque = 0.0;
        bool has_encoder_sample = false;
    };
    // EncoderData encoder_data_[2];
    std::vector<EncoderData> encoder_data_; // Thay bằng vector
    void process_params(int node_id, uint8_t* data);
    void initialize_odrive(int node_id);
    // void request_params();
    void read_can_messages();
    // void read_can_torque_messages();
    // void process_params(int node_id, uint8_t* data);
    int send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data);
    int send_can_remote_frame(int node_id, uint8_t cmd_id, uint8_t dlc = 8);

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread reader_thread_;
    bool running_;
    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> torque_publishers_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_0_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub_0_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_1_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub_1_;
};

}  // namespace can_odrive_interface

#endif  // CAN_ODRIVE_INTERFACE_HPP
