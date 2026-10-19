// Real-hardware multi-joint POSITION trajectory execution node for the 3-DOF GIM arm
// (CAN bus, via GimControlInterface).
//
// Follows the design and safety patterns of:
//   - position_control_node.cpp  (real-time CAN control, safety watchdog, position mode)
//   - impedance_trajectory_node.cpp (topic interface, trajectory loading, logging, terminal CLI)
//
// This node loads the tuned motor/PID parameters from config/motor_params.yaml and executes
// 3-DOF trajectories (such as src/robot_trajectory/gim_arm_circle_traj.csv) across all 3 joints
// simultaneously in plain POSITION mode:
//
//     q_cmd = q_ref(t)                              (ODrive position PID does the tracking)
//
// After running the trajectory once:
//   1. All motors hold the last trajectory point (the shipped circle trajectory ends at 0).
//   2. Full system info (time, actual/target pos, actual/target vel, errors, measured torque)
//      is recorded and automatically saved to:
//      config/calibration_logs/position_trajectory_logs/pos_traj_<timestamp>.csv
//      and plotted into:
//      config/calibration_logs/position_trajectory_logs/pos_traj_<timestamp>.png
//   3. To run again, the user must issue the 'start' command again.
//
// Topics:
//   sub  /gim/enable              std_msgs/Empty              Enable all motors in position mode
//   sub  /gim/disable             std_msgs/Empty              Disable all motors (AXIS_STATE_IDLE)
//   sub  /gim/safety_stop         std_msgs/Empty              Emergency stop (instant disable + fault latch)
//   sub  /gim/emergency_stop      std_msgs/Empty              Alias for safety_stop
//   sub  /gim/reset_fault         std_msgs/Empty              Clear safety fault latch
//   sub  /gim/echo_params         std_msgs/Empty              Log and republish motor parameters
//   sub  /gim/reload_params       std_msgs/Empty              Reload config/motor_params.yaml live
//   sub  /gim/start_trajectory    std_msgs/Empty              Start executing loaded trajectory (run once)
//   sub  /gim/pause_trajectory    std_msgs/Empty              Pause / resume trajectory
//   sub  /gim/stop_trajectory     std_msgs/Empty              Stop trajectory and hold the current pose
//   sub  /gim/plot                std_msgs/Empty              Re-plot the latest recorded trajectory
//   sub  /gim/load_trajectory     std_msgs/String             Load a different CSV trajectory file
//   sub  /gim/set_loop            std_msgs/Bool               Enable / disable trajectory loop mode
//   sub  /gim/pid_gains           std_msgs/Float64MultiArray  Live PID update [joint_idx, p, v, i]
//   sub  /gim/position_limits     std_msgs/Float64MultiArray  Live limits [joint_idx, vel, accel, (decel)]
//   sub  /gim/command             std_msgs/String             Text command line interface
//   pub  /gim/joint_states        sensor_msgs/JointState      Measured pos/vel + measured torque
//   pub  /gim/motor_state         std_msgs/Float64MultiArray  Realtime [q, qd, q_ref, qd_ref, q_cmd, tau_meas]
//   pub  /gim/status              std_msgs/String             Latched node state string
//   pub  /gim/trajectory_status   std_msgs/String             Latched trajectory state: IDLE|RUNNING|PAUSED|...
//   pub  /gim/trajectory_progress std_msgs/Float64MultiArray  Realtime [t_cur, t_total, progress_ratio]
//   pub  /gim/motor_params        std_msgs/String             Latched parameter dump (YAML)
//   pub  /gim/motor_params_data   std_msgs/Float64MultiArray  Latched parameter dump (numeric)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>

#include "gim_control_interface.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

constexpr size_t kN = 3;
using Vec3 = std::array<double, kN>;

// Smooth (cosine) interpolation between two positions: s(0)=0, s(1)=1.
double s_curve(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return 0.5 - 0.5 * std::cos(M_PI * u);
}

// d/du of s_curve(u).
double s_curve_dot(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return 0.5 * M_PI * std::sin(M_PI * u);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

std::string stamp_now() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

struct TrajectoryPoint {
    double t = 0.0;
    Vec3 q{};
    Vec3 qd{};
};

struct TrajectorySample {
    double t = 0.0;
    Vec3 q_act{};
    Vec3 q_ref{};
    Vec3 qd_act{};
    Vec3 qd_ref{};
    Vec3 e_p{};
    Vec3 e_v{};
    Vec3 tau_meas{};
};

enum class TrajectoryState {
    IDLE,
    APPROACHING,
    RUNNING,
    PAUSED,
    COMPLETED,
    ABORTED
};

const char* to_string(TrajectoryState s) {
    switch (s) {
        case TrajectoryState::IDLE: return "IDLE";
        case TrajectoryState::APPROACHING: return "APPROACHING";
        case TrajectoryState::RUNNING: return "RUNNING";
        case TrajectoryState::PAUSED: return "PAUSED";
        case TrajectoryState::COMPLETED: return "COMPLETED";
        case TrajectoryState::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
}

}  // namespace

class PositionTrajectoryNode : public rclcpp::Node {
public:
    PositionTrajectoryNode() : Node("position_trajectory_node") {
        // ---------------- ROS Parameters ----------------
        declare_parameter<std::vector<std::string>>("joint_names", {"base_joint", "shoulder_joint", "elbow_joint"});
        declare_parameter<std::vector<std::string>>("motor_labels", {"base", "shoulder", "elbow"});
        declare_parameter<std::vector<int64_t>>("node_ids", {0, 1, 2});
        declare_parameter<std::vector<double>>("gear_ratio", {8.0, 64.0, 8.0});
        declare_parameter<std::vector<bool>>("invert_direction", {true, false, true});
        declare_parameter<std::vector<double>>("torque_sign", {-1.0, 1.0, -1.0});
        declare_parameter<std::vector<double>>("torque_gear_ratio", {1.0, 8.0, 1.0});
        declare_parameter<std::vector<double>>("torque_limit", {5.0, 40.0, 5.0});
        declare_parameter<std::vector<double>>("joint_lower", {-0.7121, -0.2366, -0.2336});
        declare_parameter<std::vector<double>>("joint_upper", {1.0226, 1.3435, 1.7762});
        declare_parameter<std::vector<double>>("pid_pos_gain", {4.0, 0.5, 4.0});
        declare_parameter<std::vector<double>>("pid_vel_gain", {1.2, 0.1, 1.5});
        declare_parameter<std::vector<double>>("pid_vel_integrator_gain", {0.1, 0.0, 0.1});
        declare_parameter<std::vector<double>>("vel_limit", {4.0, 4.0, 4.0});
        declare_parameter<std::vector<double>>("accel_limit", {0.0, 0.0, 0.0});
        declare_parameter<std::vector<double>>("decel_limit", {0.0, 0.0, 0.0});
        declare_parameter<double>("control_rate_hz", 100.0);
        declare_parameter<double>("encoder_timeout", 0.2);   // s without CAN reply -> fault
        declare_parameter<double>("startup_timeout", 10.0);  // s to wait for encoders before giving up
        declare_parameter<std::string>("params_file", "/home/tue/gim_control/config/motor_params.yaml");
        declare_parameter<std::string>("trajectory_file", "/home/tue/gim_control/src/robot_trajectory/gim_arm_circle_traj.csv");
        declare_parameter<std::string>(
            "log_dir", "/home/tue/gim_control/config/calibration_logs/position_trajectory_logs");
        declare_parameter<std::string>("plot_script", "/home/tue/gim_control/scripts/plot_position_trajectory.py");
        declare_parameter<bool>("auto_plot", true);
        declare_parameter<bool>("loop_trajectory", false);
        declare_parameter<bool>("enable_terminal_cli", true);

        const auto names = get_parameter("joint_names").as_string_array();
        const auto labels = get_parameter("motor_labels").as_string_array();
        const auto ids = get_parameter("node_ids").as_integer_array();
        const auto invert = get_parameter("invert_direction").as_bool_array();
        if (names.size() != kN || labels.size() != kN || ids.size() != kN || invert.size() != kN) {
            throw std::runtime_error("joint_names / motor_labels / node_ids / invert_direction must have 3 elements");
        }
        for (size_t i = 0; i < kN; ++i) {
            joint_names_[i] = names[i];
            labels_[i] = labels[i];
            node_ids_[i] = static_cast<int>(ids[i]);
            direction_[i] = invert[i] ? -1.0 : 1.0;
        }
        load_vec3("gear_ratio", gear_ratio_);
        load_vec3("torque_sign", torque_sign_);
        load_vec3("torque_gear_ratio", torque_gear_ratio_);
        load_vec3("torque_limit", torque_limit_);
        load_vec3("joint_lower", lower_);
        load_vec3("joint_upper", upper_);
        load_vec3("pid_pos_gain", pid_pos_);
        load_vec3("pid_vel_gain", pid_vel_);
        load_vec3("pid_vel_integrator_gain", pid_vel_int_);
        load_vec3("vel_limit", vel_limit_);
        load_vec3("accel_limit", accel_limit_);
        load_vec3("decel_limit", decel_limit_);

        dt_ = 1.0 / std::max(10.0, get_parameter("control_rate_hz").as_double());
        encoder_timeout_ = get_parameter("encoder_timeout").as_double();
        startup_timeout_ = get_parameter("startup_timeout").as_double();
        params_file_ = get_parameter("params_file").as_string();
        trajectory_file_ = get_parameter("trajectory_file").as_string();
        log_dir_ = get_parameter("log_dir").as_string();
        plot_script_ = get_parameter("plot_script").as_string();
        auto_plot_ = get_parameter("auto_plot").as_bool();
        loop_trajectory_ = get_parameter("loop_trajectory").as_bool();
        enable_terminal_cli_ = get_parameter("enable_terminal_cli").as_bool();

        // 1) Load parameters from YAML file (source of truth)
        load_params_file(params_file_);

        // 2) Load multi-joint trajectory CSV
        load_trajectory_file(trajectory_file_);

        // ---------------- Hardware Interface ----------------
        hw_ = std::make_shared<ros2_gim_control::GimControlInterface>();
        if (!rclcpp::ok()) {
            throw std::runtime_error("Cannot open CAN interface can0 (is it up? `sudo ip link set can0 up type can bitrate 1000000`)");
        }
        // Safety: ensure all motors start unenergised
        for (size_t i = 0; i < kN; ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }

        // ---------------- ROS Publishers & Subscribers ----------------
        auto latched = rclcpp::QoS(1).transient_local().reliable();
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/gim/joint_states", 10);
        motor_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/motor_state", 50);
        status_pub_ = create_publisher<std_msgs::msg::String>("/gim/status", latched);
        traj_status_pub_ = create_publisher<std_msgs::msg::String>("/gim/trajectory_status", latched);
        traj_progress_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/trajectory_progress", 50);
        params_pub_ = create_publisher<std_msgs::msg::String>("/gim/motor_params", latched);
        params_data_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/motor_params_data", latched);

        // Control topics
        enable_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/enable", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("enable"); });
        disable_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/disable", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("disable"); });
        safety_stop_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/safety_stop", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("safety_stop"); });
        emergency_stop_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/emergency_stop", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("safety_stop"); });
        reset_fault_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/reset_fault", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("reset"); });

        // Param echoing and reloading topics
        echo_params_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/echo_params", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("params"); });
        reload_params_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/reload_params", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("reload"); });

        // Trajectory execution topics
        start_traj_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/start_trajectory", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("start"); });
        pause_traj_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/pause_trajectory", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("pause"); });
        stop_traj_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/stop_trajectory", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("stop"); });
        plot_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/plot", 10, [this](const std_msgs::msg::Empty::SharedPtr) { enqueue("plot"); });
        load_traj_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/load_trajectory", 10, [this](const std_msgs::msg::String::SharedPtr m) {
                enqueue("load " + m->data);
            });
        set_loop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/gim/set_loop", 10, [this](const std_msgs::msg::Bool::SharedPtr m) {
                enqueue(std::string("loop ") + (m->data ? "on" : "off"));
            });

        // Live gain / limit updates & generic command
        pid_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/pid_gains", 10, std::bind(&PositionTrajectoryNode::pid_gains_callback, this, std::placeholders::_1));
        limits_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/position_limits", 10, std::bind(&PositionTrajectoryNode::position_limits_callback, this, std::placeholders::_1));
        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/command", 10, [this](const std_msgs::msg::String::SharedPtr m) { enqueue(m->data); });

        // ---------------- Timers ----------------
        control_timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                                           std::bind(&PositionTrajectoryNode::control_loop, this));
        status_timer_ = create_wall_timer(1s, std::bind(&PositionTrajectoryNode::publish_status_timer, this));
        cmd_timer_ = create_wall_timer(20ms, std::bind(&PositionTrajectoryNode::process_commands, this));

        // Startup worker thread
        startup_thread_ = std::thread(&PositionTrajectoryNode::startup_worker, this);

        // Terminal CLI thread
        if (enable_terminal_cli_) {
            terminal_thread_ = std::thread(&PositionTrajectoryNode::terminal_worker, this);
        }

        RCLCPP_INFO(get_logger(), "position_trajectory_node initialized. Motors start DISABLED (idle).");
    }

    ~PositionTrajectoryNode() override {
        shutdown_hardware();
    }

    void shutdown_hardware() {
        if (shut_down_) return;
        shut_down_ = true;
        cancel_ = true;
        running_cli_ = false;
        motors_enabled_ = false;
        if (startup_thread_.joinable()) startup_thread_.join();
        if (switch_thread_.joinable()) switch_thread_.join();
        if (terminal_thread_.joinable()) terminal_thread_.detach();
        if (hw_) {
            for (size_t i = 0; i < kN; ++i) {
                hw_->DisableMotor(node_ids_[i]);
            }
        }
        RCLCPP_INFO(get_logger(), "All motors disabled. Shutdown complete.");
    }

private:
    // ---------------- Helpers & Parameter Handling ----------------
    void load_vec3(const std::string& name, Vec3& out) {
        const auto v = get_parameter(name).as_double_array();
        if (v.size() != kN) throw std::runtime_error("parameter '" + name + "' must have 3 elements");
        for (size_t i = 0; i < kN; ++i) out[i] = v[i];
    }

    double joint_per_turn(size_t i) const {
        return 2.0 * M_PI / gear_ratio_[i];
    }

    double turns_for_joint(size_t i, double q) const {
        return zero_turns_[i] + direction_[i] * q / joint_per_turn(i);
    }

    void load_params_file(const std::string& path) {
        std::ifstream f(path);
        if (!f.good()) {
            RCLCPP_WARN(get_logger(), "No parameter file '%s' found - using built-in defaults.", path.c_str());
            return;
        }
        try {
            const YAML::Node root = YAML::LoadFile(path);
            const YAML::Node motors = root["motors"];
            if (!motors) {
                RCLCPP_WARN(get_logger(), "Parameter file '%s' has no 'motors' section - ignored.", path.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < kN; ++i) {
                const YAML::Node m = motors[labels_[i]];
                if (!m) continue;
                if (m["node_id"]) node_ids_[i] = m["node_id"].as<int>();
                if (m["gear_ratio"]) gear_ratio_[i] = m["gear_ratio"].as<double>();
                if (m["torque_sign"]) torque_sign_[i] = m["torque_sign"].as<double>();
                if (m["torque_gear_ratio"]) torque_gear_ratio_[i] = m["torque_gear_ratio"].as<double>();
                if (m["torque_limit"]) torque_limit_[i] = m["torque_limit"].as<double>();
                if (m["joint_lower"]) lower_[i] = m["joint_lower"].as<double>();
                if (m["joint_upper"]) upper_[i] = m["joint_upper"].as<double>();
                if (m["invert_direction"]) direction_[i] = m["invert_direction"].as<bool>() ? -1.0 : 1.0;
                if (m["pid"]) {
                    if (m["pid"]["pos_gain"]) pid_pos_[i] = m["pid"]["pos_gain"].as<double>();
                    if (m["pid"]["vel_gain"]) pid_vel_[i] = m["pid"]["vel_gain"].as<double>();
                    if (m["pid"]["vel_integrator_gain"]) pid_vel_int_[i] = m["pid"]["vel_integrator_gain"].as<double>();
                }
                if (m["position"]) {
                    const YAML::Node p = m["position"];
                    if (p["vel_limit"]) vel_limit_[i] = p["vel_limit"].as<double>();
                    if (p["accel_limit"]) accel_limit_[i] = p["accel_limit"].as<double>();
                    if (p["decel_limit"]) decel_limit_[i] = p["decel_limit"].as<double>();
                }
            }
            const YAML::Node g = root["global"];
            if (g) {
                if (g["control_rate_hz"]) {
                    dt_ = 1.0 / std::max(10.0, g["control_rate_hz"].as<double>());
                }
                if (g["encoder_timeout"]) encoder_timeout_ = g["encoder_timeout"].as<double>();
            }
            params_file_ = path;
            RCLCPP_INFO(get_logger(), "Successfully loaded motor parameters from %s", path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to parse '%s' (%s) - using previous values.", path.c_str(), e.what());
        }
    }

    std::string build_params_yaml() const {
        std::ostringstream os;
        os << "motors:\n";
        for (size_t i = 0; i < kN; ++i) {
            os << "  " << labels_[i] << ":\n";
            os << "    node_id: " << node_ids_[i] << "\n";
            os << "    joint_name: " << joint_names_[i] << "\n";
            os << "    gear_ratio: " << gear_ratio_[i] << "\n";
            os << "    invert_direction: " << (direction_[i] < 0.0 ? "true" : "false") << "\n";
            os << "    torque_sign: " << torque_sign_[i] << "\n";
            os << "    torque_gear_ratio: " << torque_gear_ratio_[i] << "\n";
            os << "    torque_limit: " << torque_limit_[i] << "\n";
            os << "    joint_lower: " << lower_[i] << "\n";
            os << "    joint_upper: " << upper_[i] << "\n";
            os << "    pid:\n";
            os << "      pos_gain: " << pid_pos_[i] << "\n";
            os << "      vel_gain: " << pid_vel_[i] << "\n";
            os << "      vel_integrator_gain: " << pid_vel_int_[i] << "\n";
            os << "    position:\n";
            os << "      vel_limit: " << vel_limit_[i] << "\n";
            os << "      accel_limit: " << accel_limit_[i] << "\n";
            os << "      decel_limit: " << decel_limit_[i] << "\n";
        }
        return os.str();
    }

    void publish_params_locked() {
        std_msgs::msg::String y;
        y.data = build_params_yaml();
        params_pub_->publish(y);

        std_msgs::msg::Float64MultiArray d;
        d.data.reserve(kN * 14);
        for (size_t i = 0; i < kN; ++i) {
            d.data.push_back(static_cast<double>(node_ids_[i]));
            d.data.push_back(gear_ratio_[i]);
            d.data.push_back(direction_[i]);
            d.data.push_back(torque_sign_[i]);
            d.data.push_back(torque_gear_ratio_[i]);
            d.data.push_back(torque_limit_[i]);
            d.data.push_back(lower_[i]);
            d.data.push_back(upper_[i]);
            d.data.push_back(vel_limit_[i]);
            d.data.push_back(accel_limit_[i]);
            d.data.push_back(decel_limit_[i]);
            d.data.push_back(pid_pos_[i]);
            d.data.push_back(pid_vel_[i]);
            d.data.push_back(pid_vel_int_[i]);
        }
        params_data_pub_->publish(d);
    }

    void apply_motor_params_locked() {
        for (size_t i = 0; i < kN; ++i) {
            hw_->SetPIDGains(node_ids_[i], static_cast<float>(pid_pos_[i]),
                             static_cast<float>(pid_vel_[i]), static_cast<float>(pid_vel_int_[i]));
            hw_->SetTrajectoryVelLimit(node_ids_[i], static_cast<float>(vel_limit_[i]));
            if (accel_limit_[i] > 0.0 || decel_limit_[i] > 0.0) {
                hw_->SetTrajectoryAccelLimit(node_ids_[i], static_cast<float>(accel_limit_[i]),
                                             static_cast<float>(decel_limit_[i]));
            }
        }
    }

    void echo_params() {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_params_locked();

        std::ostringstream os;
        os << "\n==================== gim_control Position Parameters ====================\n"
           << "Source: " << params_file_ << "\n"
           << std::left << std::setw(10) << "Motor"
           << std::setw(6)  << "Node"
           << std::setw(8)  << "Ratio"
           << std::setw(6)  << "Dir"
           << std::setw(8)  << "PosGain"
           << std::setw(8)  << "VelGain"
           << std::setw(8)  << "IntGain"
           << std::setw(9)  << "VelLim"
           << "Joint Limits [rad]\n"
           << "-------------------------------------------------------------------------\n";
        for (size_t i = 0; i < kN; ++i) {
            os << std::left << std::setw(10) << labels_[i]
               << std::setw(6)  << node_ids_[i]
               << std::setw(8)  << gear_ratio_[i]
               << std::setw(6)  << (direction_[i] < 0 ? -1 : 1)
               << std::setw(8)  << pid_pos_[i]
               << std::setw(8)  << pid_vel_[i]
               << std::setw(8)  << pid_vel_int_[i]
               << std::setw(9)  << vel_limit_[i]
               << "[" << std::fixed << std::setprecision(4) << lower_[i] << ", " << upper_[i] << "]\n";
        }
        os << "=========================================================================\n";
        RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
        std::cout << os.str() << std::flush;
    }

    // ---------------- Trajectory Loading ----------------
    bool load_trajectory_file(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open trajectory file '%s'", path.c_str());
            return false;
        }

        std::string header_line;
        if (!std::getline(file, header_line)) {
            RCLCPP_ERROR(get_logger(), "Trajectory file '%s' is empty", path.c_str());
            return false;
        }

        std::stringstream hs(header_line);
        std::string col;
        std::vector<std::string> cols;
        while (std::getline(hs, col, ',')) {
            col.erase(col.begin(), std::find_if(col.begin(), col.end(), [](unsigned char ch) { return !std::isspace(ch); }));
            col.erase(std::find_if(col.rbegin(), col.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), col.end());
            cols.push_back(col);
        }

        int t_col = -1;
        std::array<int, kN> q_cols = {-1, -1, -1};
        std::array<int, kN> qd_cols = {-1, -1, -1};

        for (size_t c = 0; c < cols.size(); ++c) {
            if (cols[c] == "t" || cols[c] == "time") t_col = static_cast<int>(c);
            else if (cols[c] == "q_base") q_cols[0] = static_cast<int>(c);
            else if (cols[c] == "q_shoulder") q_cols[1] = static_cast<int>(c);
            else if (cols[c] == "q_elbow") q_cols[2] = static_cast<int>(c);
            else if (cols[c] == "qd_base") qd_cols[0] = static_cast<int>(c);
            else if (cols[c] == "qd_shoulder") qd_cols[1] = static_cast<int>(c);
            else if (cols[c] == "qd_elbow") qd_cols[2] = static_cast<int>(c);
        }

        // Fallbacks
        if (t_col < 0) t_col = 0;
        if (q_cols[0] < 0) { q_cols[0] = 1; q_cols[1] = 2; q_cols[2] = 3; }
        if (qd_cols[0] < 0 && cols.size() >= 7) { qd_cols[0] = 4; qd_cols[1] = 5; qd_cols[2] = 6; }

        std::vector<TrajectoryPoint> pts;
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string val;
            std::vector<double> row_vals;
            while (std::getline(ss, val, ',')) {
                try {
                    row_vals.push_back(std::stod(val));
                } catch (...) {
                    break;
                }
            }
            if (row_vals.size() <= static_cast<size_t>(std::max({t_col, q_cols[0], q_cols[1], q_cols[2]}))) {
                continue;
            }

            TrajectoryPoint pt;
            pt.t = row_vals[t_col];
            for (size_t i = 0; i < kN; ++i) {
                pt.q[i] = row_vals[q_cols[i]];
                if (qd_cols[i] >= 0 && static_cast<size_t>(qd_cols[i]) < row_vals.size()) {
                    pt.qd[i] = row_vals[qd_cols[i]];
                } else {
                    pt.qd[i] = 0.0;
                }
            }
            pts.push_back(pt);
        }

        if (pts.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid trajectory points found in '%s'", path.c_str());
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_data_ = std::move(pts);
        trajectory_total_time_ = trajectory_data_.back().t;
        trajectory_file_ = path;
        RCLCPP_INFO(get_logger(), "Loaded trajectory '%s': %zu points, total time: %.2f s",
                    path.c_str(), trajectory_data_.size(), trajectory_total_time_);
        return true;
    }

    // ---------------- Publishers ----------------
    void publish_status() {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_status_locked();
    }

    void publish_status_timer() {
        publish_status();
    }

    void publish_status_locked() {
        std::string s;
        if (faulted_.load()) {
            s = "FAULT: " + fault_ + " (all motors DISABLED)";
        } else if (!ready_.load()) {
            s = "STARTING (waiting for encoder feedback)";
        } else if (switching_.load()) {
            s = "SWITCHING";
        } else if (!motors_enabled_.load()) {
            s = "DISABLED (motors idle, send /gim/enable to enable)";
        } else {
            s = "ENABLED [POSITION ALL] | Trajectory: " + std::string(to_string(traj_state_));
            if (traj_state_ == TrajectoryState::RUNNING) {
                const double t = traj_current_t_.load();
                std::ostringstream ss;
                ss << " (" << std::fixed << std::setprecision(1) << t << " / " << trajectory_total_time_ << "s)";
                s += ss.str();
            } else if (traj_state_ == TrajectoryState::COMPLETED) {
                s += " (holding final pose)";
            }
        }
        std_msgs::msg::String msg;
        msg.data = s;
        status_pub_->publish(msg);
    }

    void publish_trajectory_status(const std::string& status_str) {
        std_msgs::msg::String msg;
        msg.data = status_str;
        traj_status_pub_->publish(msg);
    }

    void publish_joint_states() {
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name.assign(joint_names_.begin(), joint_names_.end());
        js.position.assign(q_.begin(), q_.end());
        js.velocity.assign(qd_.begin(), qd_.end());
        js.effort.assign(tau_meas_.begin(), tau_meas_.end());
        joint_state_pub_->publish(js);
    }

    // Realtime motor state: [q0..2, qd0..2, q_ref0..2, qd_ref0..2, q_cmd0..2, tau_meas0..2]
    void publish_motor_state() {
        std_msgs::msg::Float64MultiArray m;
        m.data.reserve(kN * 6);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(q_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(qd_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(q_ref_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(qd_ref_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(q_cmd_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(tau_meas_[i]);
        motor_state_pub_->publish(m);
    }

    void publish_trajectory_progress(double t_cur, double t_total) {
        std_msgs::msg::Float64MultiArray p;
        p.data = {t_cur, t_total, t_total > 1e-6 ? std::clamp(t_cur / t_total, 0.0, 1.0) : 0.0};
        traj_progress_pub_->publish(p);
    }

    // ---------------- Workers ----------------
    void startup_worker() {
        RCLCPP_INFO(get_logger(), "Waiting for CAN feedback from all motors...");
        const auto t0 = std::chrono::steady_clock::now();
        auto last_warn = t0;

        while (rclcpp::ok() && !cancel_.load()) {
            std::string missing;
            for (size_t i = 0; i < kN; ++i) {
                if (hw_->has_encoder_data(node_ids_[i])) continue;
                if (!missing.empty()) missing += ", ";
                missing += labels_[i] + " (node " + std::to_string(node_ids_[i]) + ")";
            }
            if (missing.empty()) break;

            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_warn).count() > 3.0) {
                last_warn = now;
                RCLCPP_WARN(get_logger(), "Waiting for encoder feedback from: %s...", missing.c_str());
            }
            if (startup_timeout_ > 0.0 && std::chrono::duration<double>(now - t0).count() > startup_timeout_) {
                std::lock_guard<std::mutex> lock(mutex_);
                faulted_ = true;
                fault_ = "Startup timeout waiting for: " + missing;
                RCLCPP_ERROR(get_logger(), "%s. Motors remain DISABLED.", fault_.c_str());
                publish_status_locked();
                return;
            }
            std::this_thread::sleep_for(50ms);
        }
        if (cancel_.load()) return;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < kN; ++i) {
                float p = 0.f, v = 0.f;
                hw_->get_encoder_data(node_ids_[i], p, v);
                zero_turns_[i] = p;  // capture current resting pose as joint 0
                q_[i] = 0.0;
                qd_[i] = 0.0;
                q_ref_[i] = 0.0;
                qd_ref_[i] = 0.0;
                q_cmd_[i] = 0.0;
            }
        }

        // Leave motors idle until enable command is sent
        for (size_t i = 0; i < kN && !cancel_.load(); ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }
        motors_enabled_ = false;

        if (cancel_.load()) return;
        ready_ = true;
        publish_params_locked();
        publish_status();
        publish_trajectory_status("IDLE");
        RCLCPP_INFO(get_logger(), "Motors ready and verified. Start DISABLED (idle). Send /gim/enable to enable.");
    }

    void enable_worker() {
        RCLCPP_INFO(get_logger(), "Enabling all motors in POSITION mode ...");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Capture resting pose as joint 0 so no sudden position jump occurs
            for (size_t i = 0; i < kN; ++i) {
                float p = 0.f, v = 0.f;
                hw_->get_encoder_data(node_ids_[i], p, v);
                zero_turns_[i] = p;
                q_[i] = 0.0;
                qd_[i] = 0.0;
                q_ref_[i] = 0.0;
                qd_ref_[i] = 0.0;
            }
            for (size_t i = 0; i < kN; ++i) {
                if (cancel_.load()) break;
                hw_->InitPositionMode(node_ids_[i]);
                apply_motor_params_locked();
                hw_->SendPositionCommand(node_ids_[i], static_cast<float>(turns_for_joint(i, 0.0)));
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!cancel_.load()) {
                const auto now_t = std::chrono::steady_clock::now();
                for (size_t i = 0; i < kN; ++i) {
                    last_count_[i] = hw_->encoder_sample_count(node_ids_[i]);
                    last_change_[i] = now_t;
                }
                motors_enabled_ = true;
                switching_ = false;
                traj_state_ = TrajectoryState::IDLE;
                publish_status_locked();
                publish_trajectory_status("IDLE");
                RCLCPP_INFO(get_logger(), "All motors ENABLED in position mode (holding joint 0).");
            } else {
                switching_ = false;
                motors_enabled_ = false;
            }
        }
    }

    void disable_worker() {
        RCLCPP_INFO(get_logger(), "Disabling all motors (idle)...");
        for (size_t i = 0; i < kN; ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motors_enabled_ = false;
            traj_state_ = TrajectoryState::IDLE;
            recording_ = false;
            q_cmd_ = {0.0, 0.0, 0.0};
            switching_ = false;
            publish_status_locked();
            publish_trajectory_status("IDLE");
        }
        RCLCPP_INFO(get_logger(), "All motors DISABLED (idle).");
    }

    // ---------------- Actions ----------------
    void do_enable() {
        if (!ready_.load()) {
            RCLCPP_WARN(get_logger(), "Motors not ready yet, please wait.");
            return;
        }
        if (faulted_.load()) {
            RCLCPP_WARN(get_logger(), "Cannot enable while in FAULT state. Reset fault first.");
            return;
        }
        if (switching_.load()) {
            RCLCPP_WARN(get_logger(), "Mode switch in progress, please wait.");
            return;
        }
        if (motors_enabled_.load()) {
            RCLCPP_INFO(get_logger(), "Motors are already enabled.");
            return;
        }
        switching_ = true;
        if (switch_thread_.joinable()) switch_thread_.join();
        switch_thread_ = std::thread(&PositionTrajectoryNode::enable_worker, this);
    }

    void do_disable() {
        if (switching_.load()) {
            RCLCPP_WARN(get_logger(), "Mode switch in progress, please wait.");
            return;
        }
        if (!motors_enabled_.load()) {
            RCLCPP_INFO(get_logger(), "Motors are already disabled.");
            return;
        }
        switching_ = true;
        if (switch_thread_.joinable()) switch_thread_.join();
        switch_thread_ = std::thread(&PositionTrajectoryNode::disable_worker, this);
    }

    void do_safety_stop() {
        // Immediate non-blocking hardware disable
        for (size_t i = 0; i < kN; ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        motors_enabled_ = false;
        recording_ = false;
        faulted_ = true;
        fault_ = "Safety stop triggered";
        traj_state_ = TrajectoryState::ABORTED;
        q_cmd_ = {0.0, 0.0, 0.0};
        publish_status_locked();
        publish_trajectory_status("ABORTED");
        RCLCPP_ERROR(get_logger(), "SAFETY STOP ACTIVATED: All motors disabled!");
        std::cout << "\n[SAFETY STOP] All motors disabled (AXIS_STATE_IDLE)!\n" << std::flush;
    }

    void do_reset_fault() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!faulted_.load()) {
            RCLCPP_INFO(get_logger(), "No fault currently active.");
            return;
        }
        faulted_ = false;
        fault_.clear();
        motors_enabled_ = false;
        recording_ = false;
        traj_state_ = TrajectoryState::IDLE;
        q_cmd_ = {0.0, 0.0, 0.0};
        publish_status_locked();
        publish_trajectory_status("IDLE");
        RCLCPP_INFO(get_logger(), "Fault cleared. Motors are DISABLED. Send /gim/enable to enable.");
    }

    void do_start_trajectory() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ready_.load()) {
            RCLCPP_WARN(get_logger(), "Cannot start trajectory: motors not ready yet.");
            return;
        }
        if (faulted_.load()) {
            RCLCPP_WARN(get_logger(), "Cannot start trajectory: fault is active.");
            return;
        }
        if (!motors_enabled_.load()) {
            RCLCPP_WARN(get_logger(), "Cannot start trajectory: motors are disabled. Enable motors first with 'enable'.");
            return;
        }
        if (trajectory_data_.empty()) {
            RCLCPP_WARN(get_logger(), "Cannot start trajectory: no trajectory loaded.");
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto& start_pt = trajectory_data_.front();

        // Check if current position is close to trajectory starting position
        double max_err = 0.0;
        for (size_t i = 0; i < kN; ++i) {
            max_err = std::max(max_err, std::abs(q_[i] - start_pt.q[i]));
        }

        // Reset sample buffer for a new trajectory recording run
        samples_.clear();
        samples_.reserve(std::max<size_t>(3000, trajectory_data_.size() + 100));
        recording_ = true;

        if (max_err > 0.05) {
            // Smooth S-curve approach to trajectory start to avoid any sudden jerk
            approach_start_time_ = now;
            approach_duration_ = std::max(1.5, max_err / 0.3);  // Safe gentle approach speed
            approach_q_start_ = q_;
            approach_target_ = start_pt.q;
            traj_state_ = TrajectoryState::APPROACHING;
            publish_trajectory_status("APPROACHING");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Approaching trajectory start pose (max err: %.3f rad) over %.2f s...",
                        max_err, approach_duration_);
        } else {
            traj_start_time_ = now;
            traj_state_ = TrajectoryState::RUNNING;
            publish_trajectory_status("RUNNING");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Trajectory started (duration: %.2f s). System info recording active.",
                        trajectory_total_time_);
        }
    }

    void do_pause_trajectory() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (traj_state_ == TrajectoryState::RUNNING || traj_state_ == TrajectoryState::APPROACHING) {
            traj_state_ = TrajectoryState::PAUSED;
            pause_time_ = std::chrono::steady_clock::now();
            qd_ref_ = {0.0, 0.0, 0.0};
            publish_trajectory_status("PAUSED");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Trajectory PAUSED (holding current pose).");
        } else if (traj_state_ == TrajectoryState::PAUSED) {
            const auto now = std::chrono::steady_clock::now();
            const auto paused_duration = now - pause_time_;
            traj_start_time_ += paused_duration;
            approach_start_time_ += paused_duration;
            traj_state_ = TrajectoryState::RUNNING;
            publish_trajectory_status("RUNNING");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Trajectory RESUMED.");
        } else {
            RCLCPP_WARN(get_logger(), "Cannot pause: no trajectory is running.");
        }
    }

    void do_stop_trajectory() {
        std::vector<TrajectorySample> samples_to_save;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (traj_state_ == TrajectoryState::IDLE) {
                RCLCPP_INFO(get_logger(), "Trajectory already IDLE.");
                return;
            }
            traj_state_ = TrajectoryState::IDLE;
            // Hold the current pose (no sudden move back to 0).
            q_ref_ = q_;
            qd_ref_ = {0.0, 0.0, 0.0};
            if (recording_ && !samples_.empty()) {
                samples_to_save = samples_;
            }
            recording_ = false;
            publish_trajectory_status("IDLE");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Trajectory STOPPED. All motors holding the current pose.");
        }
        if (!samples_to_save.empty()) {
            std::thread([this, samples = std::move(samples_to_save)]() {
                save_and_plot_log(samples);
            }).detach();
        }
    }

    // ---------------- Logging & Plotting ----------------
    void save_and_plot_log(const std::vector<TrajectorySample>& samples) {
        if (samples.empty()) {
            RCLCPP_WARN(get_logger(), "No samples recorded to save.");
            return;
        }

        std::error_code ec;
        std::filesystem::create_directories(log_dir_, ec);
        if (ec) {
            RCLCPP_ERROR(get_logger(), "Failed to create log directory '%s': %s",
                         log_dir_.c_str(), ec.message().c_str());
            return;
        }

        const std::string stamp = stamp_now();
        const std::string csv_path = log_dir_ + "/pos_traj_" + stamp + ".csv";
        const std::string png_path = log_dir_ + "/pos_traj_" + stamp + ".png";

        std::ofstream f(csv_path);
        if (!f.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open CSV file for writing: %s", csv_path.c_str());
            return;
        }

        // Header with complete system info across all 3 joints
        f << "timestamp,"
          << "q_base_act,q_base_ref,qd_base_act,qd_base_ref,e_p_base,e_v_base,tau_meas_base,"
          << "q_shoulder_act,q_shoulder_ref,qd_shoulder_act,qd_shoulder_ref,e_p_shoulder,e_v_shoulder,tau_meas_shoulder,"
          << "q_elbow_act,q_elbow_ref,qd_elbow_act,qd_elbow_ref,e_p_elbow,e_v_elbow,tau_meas_elbow\n";

        f << std::fixed << std::setprecision(6);
        for (const auto& s : samples) {
            f << s.t;
            for (size_t i = 0; i < kN; ++i) {
                f << ',' << s.q_act[i] << ',' << s.q_ref[i]
                  << ',' << s.qd_act[i] << ',' << s.qd_ref[i]
                  << ',' << s.e_p[i] << ',' << s.e_v[i]
                  << ',' << s.tau_meas[i];
            }
            f << '\n';
        }
        f.close();

        RCLCPP_INFO(get_logger(), "Trajectory system info saved to %s (%zu samples)", csv_path.c_str(), samples.size());
        std::cout << "\n[LOGGER] Trajectory log saved to: " << csv_path << " (" << samples.size() << " samples)\n" << std::flush;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_csv_ = csv_path;
            last_png_ = png_path;
        }

        if (auto_plot_) {
            make_figure(csv_path, png_path);
        }
    }

    bool make_figure(const std::string& csv, const std::string& png) {
        if (!std::filesystem::exists(plot_script_)) {
            RCLCPP_WARN(get_logger(), "Plot script not found: %s. CSV is saved.", plot_script_.c_str());
            return false;
        }
        std::ostringstream cmd;
        cmd << "python3 \"" << plot_script_ << "\" \"" << csv << "\" \"" << png << "\"";
        const int rc = std::system(cmd.str().c_str());
        if (rc != 0) {
            RCLCPP_WARN(get_logger(), "Plotting failed with return code %d. CSV remains at %s", rc, csv.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "Generated plot figure at: %s", png.c_str());
        std::cout << "[LOGGER] Generated plot figure at: " << png << "\n" << std::flush;
        return true;
    }

    // ---------------- Topic Callbacks ----------------
    void pid_gains_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(get_logger(), "pid_gains expects [joint_index, pos_gain, vel_gain, vel_integrator_gain].");
            return;
        }
        const int idx = static_cast<int>(std::lround(msg->data[0]));
        if (idx < 1 || idx > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid joint index %d (valid: 1..3).", idx);
            return;
        }
        const double p = msg->data[1], v = msg->data[2], i = msg->data[3];
        if (!std::isfinite(p) || !std::isfinite(v) || !std::isfinite(i) || p < 0.0 || v < 0.0 || i < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid PID gains (must be finite and >= 0).");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pid_pos_[idx - 1] = p;
            pid_vel_[idx - 1] = v;
            pid_vel_int_[idx - 1] = i;
            hw_->SetPIDGains(node_ids_[idx - 1], static_cast<float>(p), static_cast<float>(v), static_cast<float>(i));
            publish_params_locked();
        }
        RCLCPP_INFO(get_logger(), "%s PID gains updated: pos=%.3f, vel=%.3f, vel_int=%.3f",
                    labels_[idx - 1].c_str(), p, v, i);
    }

    void position_limits_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3 || msg->data.size() > 4) {
            RCLCPP_WARN(get_logger(), "position_limits expects [joint_index, vel_limit, accel_limit, (decel_limit)].");
            return;
        }
        const int idx = static_cast<int>(std::lround(msg->data[0]));
        if (idx < 1 || idx > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid joint index %d (valid: 1..3).", idx);
            return;
        }
        const double vl = msg->data[1];
        const double al = msg->data[2];
        const double dl = (msg->data.size() == 4) ? msg->data[3] : al;
        if (!std::isfinite(vl) || !std::isfinite(al) || !std::isfinite(dl) || vl <= 0.0 || al < 0.0 || dl < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid position limits (vel_limit > 0, accel/decel >= 0).");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vel_limit_[idx - 1] = vl;
            accel_limit_[idx - 1] = al;
            decel_limit_[idx - 1] = dl;
            hw_->SetTrajectoryVelLimit(node_ids_[idx - 1], static_cast<float>(vl));
            if (al > 0.0 || dl > 0.0) {
                hw_->SetTrajectoryAccelLimit(node_ids_[idx - 1], static_cast<float>(al), static_cast<float>(dl));
            }
            publish_params_locked();
        }
        RCLCPP_INFO(get_logger(), "%s position limits updated: vel=%.3f, accel=%.3f, decel=%.3f",
                    labels_[idx - 1].c_str(), vl, al, dl);
    }

    // ---------------- Realtime Control Loop (100 Hz) ----------------
    void control_loop() {
        // 1) Read sensor feedback over CAN
        Vec3 raw_pos{}, raw_vel{}, motor_tau{};
        for (size_t i = 0; i < kN; ++i) {
            float pos = 0.f, vel = 0.f, tau = 0.f;
            hw_->get_encoder_data(node_ids_[i], pos, vel);
            hw_->get_torque_data(node_ids_[i], tau);
            raw_pos[i] = pos;
            raw_vel[i] = vel;
            motor_tau[i] = tau;
            hw_->ReadEncoder(node_ids_[i]);
            hw_->ReadTorque(node_ids_[i]);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < kN; ++i) {
            q_[i] = direction_[i] * (raw_pos[i] - zero_turns_[i]) * joint_per_turn(i);
            qd_[i] = direction_[i] * raw_vel[i] * joint_per_turn(i);
            tau_meas_[i] = torque_sign_[i] * motor_tau[i] * torque_gear_ratio_[i];
        }

        publish_joint_states();
        publish_motor_state();

        if (faulted_.load() || !ready_.load() || switching_.load() || !motors_enabled_.load()) {
            return;
        }

        // 2) Safety: encoder watchdog, joint limits, torque limit
        const auto now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < kN; ++i) {
            const uint64_t cnt = hw_->encoder_sample_count(node_ids_[i]);
            if (cnt != last_count_[i]) {
                last_count_[i] = cnt;
                last_change_[i] = now;
            } else if (std::chrono::duration<double>(now - last_change_[i]).count() > encoder_timeout_) {
                fault_locked("Encoder timeout on " + labels_[i] + " (node " + std::to_string(node_ids_[i]) + ")");
                return;
            }
            if (q_[i] < lower_[i] - 0.25 || q_[i] > upper_[i] + 0.25) {
                fault_locked("Joint limit exceeded on " + labels_[i] + " (" + std::to_string(q_[i]) + " rad)");
                return;
            }
            if (std::fabs(tau_meas_[i]) > torque_limit_[i]) {
                fault_locked("Torque limit exceeded on " + labels_[i] + " (" + std::to_string(tau_meas_[i]) + " Nm)");
                return;
            }
        }

        // 3) Update reference from trajectory or approach phase
        bool trajectory_just_finished = false;
        double current_traj_t = 0.0;
        update_trajectory_reference_locked(now, trajectory_just_finished, current_traj_t);

        // 4) Record system info sample if recording is active
        if (recording_ && (traj_state_ == TrajectoryState::RUNNING || trajectory_just_finished)) {
            TrajectorySample s;
            s.t = current_traj_t;
            s.q_act = q_;
            s.q_ref = q_ref_;
            s.qd_act = qd_;
            s.qd_ref = qd_ref_;
            for (size_t i = 0; i < kN; ++i) {
                s.e_p[i] = q_ref_[i] - q_[i];
                s.e_v[i] = qd_ref_[i] - qd_[i];
            }
            s.tau_meas = tau_meas_;
            samples_.push_back(s);
        }

        // 5) Plain position control: send the reference as the position setpoint.
        for (size_t i = 0; i < kN; ++i) {
            const double q_cmd = std::clamp(q_ref_[i], lower_[i], upper_[i]);
            q_cmd_[i] = q_cmd;
            hw_->SendPositionCommand(node_ids_[i], static_cast<float>(turns_for_joint(i, q_cmd)));
        }

        // 6) If trajectory just finished, launch save & plot in background thread
        if (trajectory_just_finished) {
            auto samples_copy = samples_;
            std::thread([this, samples = std::move(samples_copy)]() {
                save_and_plot_log(samples);
            }).detach();
        }
    }

    void update_trajectory_reference_locked(std::chrono::steady_clock::time_point now,
                                            bool& just_finished, double& current_t) {
        just_finished = false;
        if (traj_state_ == TrajectoryState::APPROACHING) {
            const double elapsed = std::chrono::duration<double>(now - approach_start_time_).count();
            const double u = approach_duration_ > 1e-9 ? elapsed / approach_duration_ : 1.0;
            if (u >= 1.0) {
                q_ref_ = approach_target_;
                qd_ref_ = {0.0, 0.0, 0.0};
                traj_state_ = TrajectoryState::RUNNING;
                traj_start_time_ = now;
                samples_.clear();
                recording_ = true;
                publish_trajectory_status("RUNNING");
                publish_status_locked();
                RCLCPP_INFO(get_logger(), "Approach complete. Starting main trajectory and recording.");
            } else {
                const double s = s_curve(u);
                const double s_dot = s_curve_dot(u) / approach_duration_;
                for (size_t i = 0; i < kN; ++i) {
                    q_ref_[i] = approach_q_start_[i] + (approach_target_[i] - approach_q_start_[i]) * s;
                    qd_ref_[i] = (approach_target_[i] - approach_q_start_[i]) * s_dot;
                }
            }
        } else if (traj_state_ == TrajectoryState::RUNNING) {
            if (trajectory_data_.empty()) {
                traj_state_ = TrajectoryState::IDLE;
                return;
            }

            double t = std::chrono::duration<double>(now - traj_start_time_).count();
            current_t = t;
            traj_current_t_.store(t);
            publish_trajectory_progress(t, trajectory_total_time_);

            if (t >= trajectory_total_time_) {
                if (loop_trajectory_) {
                    traj_start_time_ = now;
                    t = 0.0;
                    current_t = 0.0;
                } else {
                    traj_state_ = TrajectoryState::COMPLETED;
                    // Hold the final trajectory point (the shipped circle trajectory ends at 0).
                    q_ref_ = trajectory_data_.back().q;
                    qd_ref_ = {0.0, 0.0, 0.0};
                    recording_ = false;
                    just_finished = true;
                    publish_trajectory_status("COMPLETED");
                    publish_status_locked();
                    RCLCPP_INFO(get_logger(), "Trajectory execution COMPLETED (run once). Holding final pose.");
                    return;
                }
            }

            // Interpolate trajectory point
            auto it = std::lower_bound(trajectory_data_.begin(), trajectory_data_.end(), t,
                [](const TrajectoryPoint& pt, double time) { return pt.t < time; });

            if (it == trajectory_data_.begin()) {
                q_ref_ = it->q;
                qd_ref_ = it->qd;
            } else if (it != trajectory_data_.end()) {
                auto prev = std::prev(it);
                const double dt_span = it->t - prev->t;
                const double ratio = (dt_span > 1e-9) ? (t - prev->t) / dt_span : 0.0;
                for (size_t i = 0; i < kN; ++i) {
                    q_ref_[i] = prev->q[i] + ratio * (it->q[i] - prev->q[i]);
                    qd_ref_[i] = prev->qd[i] + ratio * (it->qd[i] - prev->qd[i]);
                }
            } else {
                q_ref_ = trajectory_data_.back().q;
                qd_ref_ = trajectory_data_.back().qd;
            }
        }
    }

    void fault_locked(const std::string& reason) {
        fault_ = reason;
        faulted_ = true;
        motors_enabled_ = false;
        recording_ = false;
        for (size_t i = 0; i < kN; ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }
        traj_state_ = TrajectoryState::ABORTED;
        q_cmd_ = {0.0, 0.0, 0.0};
        publish_trajectory_status("ABORTED");
        publish_status_locked();
        RCLCPP_ERROR(get_logger(), "FAULT: %s -> all motors DISABLED!", reason.c_str());
    }

    // ---------------- Command Queue & Terminal UI ----------------
    void enqueue(const std::string& line) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_queue_.push(line);
    }

    void process_commands() {
        std::queue<std::string> local;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            std::swap(local, cmd_queue_);
        }
        while (!local.empty()) {
            execute_command(local.front());
            local.pop();
        }
    }

    void execute_command(const std::string& line) {
        std::istringstream ss(line);
        std::string cmd;
        ss >> cmd;
        cmd = lower(cmd);
        if (cmd.empty()) return;

        if (cmd == "help" || cmd == "?") {
            print_help();
        } else if (cmd == "enable" || cmd == "on" || cmd == "1") {
            do_enable();
        } else if (cmd == "disable" || cmd == "off" || cmd == "0") {
            do_disable();
        } else if (cmd == "safety_stop" || cmd == "estop" || cmd == "stop_safety") {
            do_safety_stop();
        } else if (cmd == "reset" || cmd == "clear_fault") {
            do_reset_fault();
        } else if (cmd == "start" || cmd == "run" || cmd == "2") {
            do_start_trajectory();
        } else if (cmd == "pause" || cmd == "resume") {
            do_pause_trajectory();
        } else if (cmd == "stop") {
            do_stop_trajectory();
        } else if (cmd == "plot") {
            std::string csv, png;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                csv = last_csv_;
                png = last_png_;
            }
            if (csv.empty()) {
                std::cout << "No previous trajectory log found to plot.\n";
            } else {
                std::cout << "Re-generating plot for " << csv << "...\n";
                std::thread([this, csv, png]() {
                    make_figure(csv, png);
                }).detach();
            }
        } else if (cmd == "params" || cmd == "echo" || cmd == "read") {
            echo_params();
        } else if (cmd == "reload") {
            std::string path;
            ss >> path;
            if (path.empty()) path = params_file_;
            load_params_file(path);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (motors_enabled_.load()) apply_motor_params_locked();
            }
            echo_params();
        } else if (cmd == "load") {
            std::string path;
            ss >> path;
            if (path.empty()) {
                std::cout << "Usage: load <path_to_csv>\n";
            } else {
                load_trajectory_file(path);
            }
        } else if (cmd == "loop") {
            std::string opt;
            ss >> opt;
            opt = lower(opt);
            if (opt == "on" || opt == "true" || opt == "1") {
                loop_trajectory_ = true;
                std::cout << "Looping enabled.\n";
            } else if (opt == "off" || opt == "false" || opt == "0") {
                loop_trajectory_ = false;
                std::cout << "Looping disabled (runs once).\n";
            } else {
                loop_trajectory_ = !loop_trajectory_;
                std::cout << "Looping: " << (loop_trajectory_ ? "ON" : "OFF (runs once)") << "\n";
            }
        } else if (cmd == "status") {
            print_status();
        } else if (cmd == "quit" || cmd == "exit" || cmd == "q") {
            running_cli_ = false;
            shutdown_hardware();
            rclcpp::shutdown();
        } else {
            std::cout << "Unknown command '" << cmd << "'. Type 'help' for command list.\n";
        }
    }

    void print_help() const {
        std::cout <<
            "\n================ gim_control Position Trajectory Node ================\n"
            "enable / on               Enable all motors in position mode (hold joint 0)\n"
            "disable / off             Disable all motors (idle / unpowered)\n"
            "start / run               Start executing trajectory once (records system info)\n"
            "pause / resume            Pause / resume running trajectory\n"
            "stop                      Stop trajectory (arm holds the current pose)\n"
            "plot                      Re-generate plot figure for the latest trajectory run\n"
            "safety_stop / estop       EMERGENCY STOP (immediate motor cut)\n"
            "reset                     Clear fault latch after safety stop\n"
            "params / echo             Echo / print motor parameters\n"
            "reload [path]             Reload motor parameters from YAML file\n"
            "load <csv_path>           Load new trajectory CSV file\n"
            "loop [on|off]             Toggle trajectory looping (default: off, runs once)\n"
            "status                    Print current status, log path, and joint positions\n"
            "help                      Show this help message\n"
            "quit                      Disable motors and exit\n"
            "======================================================================\n\n";
    }

    void print_status() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "\n---------------- Robot Status ----------------\n"
                  << "Motors enabled: " << (motors_enabled_.load() ? "YES" : "NO") << "\n"
                  << "Fault active:   " << (faulted_.load() ? ("YES (" + fault_ + ")") : "NO") << "\n"
                  << "Trajectory:     " << to_string(traj_state_) << "\n"
                  << "Loop mode:      " << (loop_trajectory_ ? "ON" : "OFF (runs once)") << "\n"
                  << "Trajectory file:" << trajectory_file_ << " (" << trajectory_data_.size() << " pts, "
                  << std::fixed << std::setprecision(2) << trajectory_total_time_ << "s)\n"
                  << "Log directory:  " << log_dir_ << "\n"
                  << "Latest log CSV: " << (last_csv_.empty() ? "none" : last_csv_) << "\n"
                  << "Latest log PNG: " << (last_png_.empty() ? "none" : last_png_) << "\n";
        if (traj_state_ == TrajectoryState::RUNNING) {
            std::cout << "Progress:       " << traj_current_t_.load() << "s / " << trajectory_total_time_ << "s\n";
        }
        std::cout << "\nJoints:          q_act [rad]   q_ref [rad]   tau_meas [Nm]\n";
        for (size_t i = 0; i < kN; ++i) {
            std::cout << std::left << std::setw(15) << labels_[i] << "  "
                      << std::fixed << std::setprecision(4)
                      << std::setw(12) << q_[i] << "  "
                      << std::setw(12) << q_ref_[i] << "  "
                      << std::setw(12) << tau_meas_[i] << "\n";
        }
        std::cout << "----------------------------------------------\n\n";
    }

    void terminal_worker() {
        // Wait a moment for startup messages to settle
        std::this_thread::sleep_for(500ms);
        print_help();
        std::string line;
        while (running_cli_.load() && rclcpp::ok()) {
            std::cout << "gim_pos_traj> " << std::flush;
            if (!std::getline(std::cin, line)) break;
            enqueue(line);
        }
    }

    // ---------------- Members ----------------
    std::shared_ptr<ros2_gim_control::GimControlInterface> hw_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traj_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr params_data_pub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr disable_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr safety_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_fault_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr echo_params_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reload_params_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plot_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_loop_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_gains_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr limits_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::mutex mutex_;
    std::mutex cmd_mutex_;
    std::queue<std::string> cmd_queue_;

    std::thread startup_thread_;
    std::thread switch_thread_;
    std::thread terminal_thread_;

    std::atomic<bool> ready_{false};
    std::atomic<bool> switching_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> cancel_{false};
    std::atomic<bool> motors_enabled_{false};
    std::atomic<bool> running_cli_{true};
    std::atomic<double> traj_current_t_{0.0};
    bool shut_down_ = false;
    std::string fault_;

    std::array<std::string, kN> joint_names_;
    std::array<std::string, kN> labels_;
    std::array<int, kN> node_ids_{};
    Vec3 direction_{}, gear_ratio_{}, torque_sign_{}, torque_gear_ratio_{}, torque_limit_{};
    Vec3 lower_{}, upper_{}, pid_pos_{}, pid_vel_{}, pid_vel_int_{};
    Vec3 vel_limit_{}, accel_limit_{}, decel_limit_{};
    Vec3 zero_turns_{0.0, 0.0, 0.0};

    double dt_ = 0.01;
    double encoder_timeout_ = 0.2;
    double startup_timeout_ = 10.0;
    std::string params_file_;
    std::string trajectory_file_;
    std::string log_dir_;
    std::string plot_script_;
    bool auto_plot_ = true;
    bool loop_trajectory_ = false;
    bool enable_terminal_cli_ = true;

    Vec3 q_{}, qd_{}, q_ref_{}, qd_ref_{}, q_cmd_{}, tau_meas_{};
    std::array<uint64_t, kN> last_count_{};
    std::array<std::chrono::steady_clock::time_point, kN> last_change_{};

    // Trajectory state
    std::vector<TrajectoryPoint> trajectory_data_;
    double trajectory_total_time_ = 0.0;
    TrajectoryState traj_state_ = TrajectoryState::IDLE;
    std::chrono::steady_clock::time_point traj_start_time_;
    std::chrono::steady_clock::time_point pause_time_;

    // Approach phase
    std::chrono::steady_clock::time_point approach_start_time_;
    double approach_duration_ = 2.0;
    Vec3 approach_q_start_{};
    Vec3 approach_target_{};

    // Recording & Logging
    bool recording_ = false;
    std::vector<TrajectorySample> samples_;
    std::string last_csv_;
    std::string last_png_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<PositionTrajectoryNode>();
        rclcpp::spin(node);
        node->shutdown_hardware();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("position_trajectory_node"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
