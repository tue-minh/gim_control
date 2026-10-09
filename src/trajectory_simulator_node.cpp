#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <stdexcept>

using namespace std::chrono_literals;

struct TrajectoryPoint {
    double t;
    std::array<double, 3> q;
    std::array<double, 3> qd;
    std::array<double, 3> qdd;
    std::array<double, 3> ee;
};

class TrajectorySimulatorNode : public rclcpp::Node {
public:
    TrajectorySimulatorNode() : Node("trajectory_simulator_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        this->declare_parameter<std::string>("csv_file_path", "/home/tue/gim_control/src/robot_trajectory/gim_arm_circle_traj.csv");
        std::string csv_file_path = this->get_parameter("csv_file_path").as_string();
        
        load_trajectory(csv_file_path);
        
        start_time_ = this->now();
        
        timer_ = this->create_wall_timer(
            20ms, std::bind(&TrajectorySimulatorNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Trajectory Simulator Node Started.");
    }

private:
    void load_trajectory(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file: %s", filename.c_str());
            return;
        }

        std::string line;
        // Read header
        if (std::getline(file, line)) {
            // Header skipped
        }

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            
            std::stringstream ss(line);
            std::string cell;
            TrajectoryPoint pt;
            
            try {
                // t(s)
                std::getline(ss, cell, ','); pt.t = std::stod(cell);
                // q_base, q_shoulder, q_elbow
                std::getline(ss, cell, ','); pt.q[0] = std::stod(cell);
                std::getline(ss, cell, ','); pt.q[1] = std::stod(cell);
                std::getline(ss, cell, ','); pt.q[2] = std::stod(cell);
                // qd_base, qd_shoulder, qd_elbow
                std::getline(ss, cell, ','); pt.qd[0] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qd[1] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qd[2] = std::stod(cell);
                // qdd_base, qdd_shoulder, qdd_elbow
                std::getline(ss, cell, ','); pt.qdd[0] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qdd[1] = std::stod(cell);
                std::getline(ss, cell, ','); pt.qdd[2] = std::stod(cell);
                // ee_x, ee_y, ee_z
                std::getline(ss, cell, ','); pt.ee[0] = std::stod(cell);
                std::getline(ss, cell, ','); pt.ee[1] = std::stod(cell);
                std::getline(ss, cell, ','); pt.ee[2] = std::stod(cell);
                
                trajectory_data_.push_back(pt);
            } catch (const std::exception& e) {
                continue;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points.", trajectory_data_.size());
    }

    void timer_callback() {
        if (trajectory_data_.empty()) return;

        double t = (this->now() - start_time_).seconds();
        
        // Loop the trajectory if it exceeds the maximum time
        if (t > trajectory_data_.back().t) {
            start_time_ = this->now();
            t = 0.0;
        }

        TrajectoryPoint pt = trajectory_data_.back();
        auto it = std::lower_bound(trajectory_data_.begin(), trajectory_data_.end(), t,
            [](const TrajectoryPoint& p, double time) {
                return p.t < time;
            });
            
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

        // Publish to JointState
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"base_joint", "shoulder_joint", "elbow_joint"};
        msg.position = {pt.q[0], pt.q[1], pt.q[2]};
        msg.velocity = {pt.qd[0], pt.qd[1], pt.qd[2]};
        
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<TrajectoryPoint> trajectory_data_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
