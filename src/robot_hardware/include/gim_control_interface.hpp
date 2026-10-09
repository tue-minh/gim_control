#ifndef GIM_CONTROL_INTERFACE_HPP
#define GIM_CONTROL_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <std_msgs/msg/float32.hpp>

namespace ros2_gim_control {

class GimControlInterface : public rclcpp::Node {
public:
    static constexpr double RAD_TO_REV = 4.0 / M_PI;
    static constexpr double REV_TO_RAD = M_PI / 4.0;
    static constexpr double KV = 170.0;

    GimControlInterface();
    ~GimControlInterface();

    void InitVelocityMode(int node_id);
    void InitPositionMode(int node_id);
    void InitTorqueMode(int node_id);
    void SendVelocityCommand(int node_id, float velocity);
    void SendPositionCommand(int node_id, float position, int16_t vel_ff = 0, int16_t torque_ff = 0);
    void SendTorqueCommand(int node_id, float torque);
    // Zero the torque command and put the axis into AXIS_STATE_IDLE (motor disabled).
    // Non-blocking (no sleeps) so it is safe to call from shutdown paths.
    void DisableMotor(int node_id);
    void ReadEncoder(int node_id);
    void get_encoder_data(int node_id, float &pos, float &vel);
    void get_torque_data(int node_id, float &torque);
    bool has_encoder_data(int node_id) const;
    // Number of encoder replies received so far (used as a freshness watchdog).
    uint64_t encoder_sample_count(int node_id) const;

    void SetPosGain(int node_id, float pos_gain);
    void SetVelGains(int node_id, float vel_gain, float vel_integrator_gain);
    void SetPIDGains(int node_id, float pos_gain, float vel_gain, float vel_integrator_gain);
    void SetTrajectoryVelLimit(int node_id, float vel_limit);
    void SetTrajectoryAccelLimit(int node_id, float accel_limit, float decel_limit);

private:
    struct EncoderData {
        float position = 0.0;
        float velocity = 0.0;
        float torque = 0.0;
        bool has_encoder_sample = false;
        uint64_t sample_count = 0;
    };

    int sock_;
    std::thread reader_thread_;
    std::atomic<bool> running_;
    std::vector<EncoderData> encoder_data_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> torque_publishers_;

    void initialize_odrive(int node_id);
    void read_can_messages();
    void process_params(int node_id, uint8_t* data);
    void process_torque(int node_id, uint8_t* data);
    int send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data);
    int send_can_remote_frame(int node_id, uint8_t cmd_id, uint8_t dlc = 8);
};

}  // namespace ros2_gim_control

#endif  // GIM_CONTROL_INTERFACE_HPP