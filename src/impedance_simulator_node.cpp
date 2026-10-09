#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <algorithm>

using namespace std::chrono_literals;

struct TrajectoryPoint {
    double t;
    std::array<double, 3> q;
    std::array<double, 3> qd;
};

class ImpedanceSimulatorNode : public rclcpp::Node {
public:
    ImpedanceSimulatorNode() : Node("impedance_simulator_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_effort_controller/commands", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ImpedanceSimulatorNode::joint_state_callback, this, std::placeholders::_1));

        // Topic to update Kp and Kd gains dynamically: [kp1, kp2, kp3, kd1, kd2, kd3]
        gains_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/impedance_gains", 10, std::bind(&ImpedanceSimulatorNode::gains_callback, this, std::placeholders::_1));

        this->declare_parameter<std::string>("csv_file_path", "/home/tue/gim_control/src/robot_trajectory/gim_arm_circle_traj.csv");
        std::string csv_file_path = this->get_parameter("csv_file_path").as_string();
        
        this->declare_parameter<std::vector<double>>("kp", {0, 0, 0});
        this->declare_parameter<std::vector<double>>("kd", {0, 0, 0});
        kp_ = this->get_parameter("kp").as_double_array();
        kd_ = this->get_parameter("kd").as_double_array();

        load_trajectory(csv_file_path);
        
        start_time_ = this->now();
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ImpedanceSimulatorNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Impedance Simulator Node Started.");
    }

private:
    void gains_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            for (int i = 0; i < 3; ++i) {
                kp_[i] = msg->data[i];
                kd_[i] = msg->data[i + 3];
            }
            RCLCPP_INFO(this->get_logger(), "Updated Gains - Kp: [%.2f, %.2f, %.2f], Kd: [%.2f, %.2f, %.2f]",
                        kp_[0], kp_[1], kp_[2], kd_[0], kd_[1], kd_[2]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid gains array size. Expected 6 elements [kp1, kp2, kp3, kd1, kd2, kd3]. Received %zu.", msg->data.size());
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.size() >= 3 && msg->position.size() >= 3) {
            for (size_t i = 0; i < 3; ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                if (it != msg->name.end()) {
                    int idx = std::distance(msg->name.begin(), it);
                    q_act_[i] = msg->position[idx];
                    if (msg->velocity.size() > static_cast<size_t>(idx)) {
                        qd_act_[i] = msg->velocity[idx];
                    }
                }
            }
        }
    }

    void load_trajectory(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) return;

        std::string line;
        if (std::getline(file, line)) {}

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string cell;
            TrajectoryPoint pt;
            try {
                std::getline(ss, cell, ','); pt.t = std::stod(cell);
                std::getline(ss, cell, ','); pt.q[0] = std::stod(cell);
                std::getline(ss, cell, ','); pt.q[1] = std::stod(cell);
                std::getline(ss, cell, ','); pt.q[2] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qd[0] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qd[1] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qd[2] = std::stod(cell);
                trajectory_data_.push_back(pt);
            } catch (...) { continue; }
        }
    }

    void timer_callback() {
        if (trajectory_data_.empty()) return;

        double t = (this->now() - start_time_).seconds();
        if (t > trajectory_data_.back().t) {
            start_time_ = this->now();
            t = 0.0;
        }

        TrajectoryPoint pt = trajectory_data_.back();
        auto it = std::lower_bound(trajectory_data_.begin(), trajectory_data_.end(), t,
            [](const TrajectoryPoint& p, double time) { return p.t < time; });
            
        if (it == trajectory_data_.begin()) {
            pt = *it;
        } else if (it != trajectory_data_.end()) {
            auto prev = std::prev(it);
            double ratio = (t - prev->t) / (it->t - prev->t);
            pt.t = t;
            for (size_t i = 0; i < 3; ++i) {
                pt.q[i] = prev->q[i] + ratio * (it->q[i] - prev->q[i]);
                pt.qd[i] = prev->qd[i] + ratio * (it->qd[i] - prev->qd[i]);
            }
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(3);
        for (size_t i = 0; i < 3; ++i) {
            msg.data[i] = kp_[i] * (pt.q[i] - q_act_[i]) + kd_[i] * (pt.qd[i] - qd_act_[i]);
        }
        
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gains_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<TrajectoryPoint> trajectory_data_;
    rclcpp::Time start_time_;

    std::vector<double> kp_;
    std::vector<double> kd_;
    std::array<std::string, 3> joint_names_ = {"base_joint", "shoulder_joint", "elbow_joint"};
    std::array<double, 3> q_act_ = {0.0, 0.0, 0.0};
    std::array<double, 3> qd_act_ = {0.0, 0.0, 0.0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImpedanceSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
