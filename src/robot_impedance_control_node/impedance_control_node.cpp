// Real-hardware control node for the 3-DOF GIM6010 arm (CAN bus, via GimControlInterface).
//
//   base_joint    -> CAN node id 1
//   shoulder_joint-> CAN node id 2
//   elbow_joint   -> CAN node id 3
//
// Behaviour
//   * At start every motor is enabled in POSITION mode and commanded to hold joint 0
//     (joint 0 = the pose captured at start-up, i.e. where the arm was when the node came up).
//   * /gim/select_motor picks ONE joint to run in IMPEDANCE mode for calibration. The other
//     two stay in POSITION mode holding 0. Only one joint can be impedance-controlled at a
//     time; selecting another joint puts the previous one back to POSITION @ 0.
//     Impedance law (joint space, Nm):  tau = kp * (q_ref - q) + kd * (qd_ref - qd)
//   * /gim/calibration_trajectory runs a simple point-to-point profile on the selected joint:
//       0 -> q_target over t_move, hold q_target for t_hold, then q_target -> 0 over t_return.
//     While it runs, /gim/calibration_data streams one sample per control tick:
//       [t, joint_index, q_actual, q_target, qd_actual, qd_target, torque, pos_error, vel_error]
//   * On shutdown (Ctrl-C / SIGTERM) every motor is disabled (AXIS_STATE_IDLE).
//
// Topics
//   sub  /gim/select_motor           std_msgs/Int32               0=none 1=base 2=shoulder 3=elbow
//   sub  /gim/impedance_gains        std_msgs/Float64MultiArray   [joint_index, kp, kd]
//   sub  /gim/pid_gains              std_msgs/Float64MultiArray   [joint_index, pos_gain, vel_gain, vel_int_gain]
//   sub  /gim/calibration_trajectory std_msgs/Float64MultiArray   [q_target, t_move, t_hold, (t_return)]
//   sub  /gim/stop_calibration       std_msgs/Empty               abort the running trajectory
//   pub  /gim/joint_states           sensor_msgs/JointState       measured pos/vel + commanded torque
//   pub  /gim/motor_state            std_msgs/Float64MultiArray   realtime [q0..q2, qd0..qd2, tau_cmd0..2, tau_meas0..2]
//   pub  /gim/status                 std_msgs/String              latched node state
//   pub  /gim/calibration_status     std_msgs/String              latched IDLE|RUNNING|DONE|ABORTED
//   pub  /gim/calibration_data       std_msgs/Float64MultiArray   calibration samples (see above)
//   pub  /gim/motor_params           std_msgs/String              latched parameter dump (YAML)
//   pub  /gim/motor_params_data      std_msgs/Float64MultiArray   latched parameter dump (numeric)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>

#include "gim_control_interface.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
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

struct CalibrationTrajectory {
    bool active = false;
    int joint = -1;          // 0..2
    double q_target = 0.0;   // rad
    double t_move = 1.0;     // s
    double t_hold = 1.0;     // s
    double t_return = 1.0;   // s
    std::chrono::steady_clock::time_point start;
};

}  // namespace

class ImpedanceControlNode : public rclcpp::Node {
public:
    ImpedanceControlNode() : Node("impedance_control_node") {
        // ---------------- parameters ----------------
        declare_parameter<std::vector<std::string>>("joint_names", {"base_joint", "shoulder_joint", "elbow_joint"});
        declare_parameter<std::vector<std::string>>("motor_labels", {"base", "shoulder", "elbow"});
        // CAN node ids (see urdf/gim_arm.urdf): base=0, shoulder=1, elbow=2
        declare_parameter<std::vector<int64_t>>("node_ids", {0, 1, 2});
        // joint-side gear ratio (rotor turns per joint turn); URDF: base 8, shoulder 64, elbow 8
        declare_parameter<std::vector<double>>("gear_ratio", {8.0, 64.0, 8.0});
        declare_parameter<std::vector<bool>>("invert_direction", {true, false, true});
        declare_parameter<std::vector<double>>("torque_sign", {-1.0, 1.0, -1.0});
        declare_parameter<std::vector<double>>("torque_gear_ratio", {1.0, 8.0, 1.0});
        declare_parameter<std::vector<double>>("torque_limit", {5.0, 40.0, 5.0});
        declare_parameter<std::vector<double>>("joint_lower", {-0.7121, -0.2366, -0.2336});
        declare_parameter<std::vector<double>>("joint_upper", {1.0226, 1.3435, 1.7762});
        declare_parameter<std::vector<double>>("kp", {5.0, 10.0, 5.0});
        declare_parameter<std::vector<double>>("kd", {0.2, 0.4, 0.2});
        declare_parameter<std::vector<double>>("pid_pos_gain", {20.0, 20.0, 20.0});
        declare_parameter<std::vector<double>>("pid_vel_gain", {2.0, 2.0, 2.0});
        declare_parameter<std::vector<double>>("pid_vel_integrator_gain", {0.2, 0.2, 0.2});
        declare_parameter<double>("control_rate_hz", 100.0);
        declare_parameter<double>("encoder_timeout", 0.2);   // s without CAN reply -> fault + disable all
        declare_parameter<double>("hold_vel_limit", 4.0);    // POSITION-mode trajectory vel limit (turns/s)
        declare_parameter<std::string>("params_file", "/home/tue/gim_control/config/motor_params.yaml");
        declare_parameter<bool>("load_params_on_start", true);
        declare_parameter<bool>("auto_save_params", true);
        declare_parameter<double>("startup_timeout", 10.0);  // s to wait for all encoders before giving up

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
        load_vec3("kp", kp_);
        load_vec3("kd", kd_);
        load_vec3("pid_pos_gain", pid_pos_);
        load_vec3("pid_vel_gain", pid_vel_);
        load_vec3("pid_vel_integrator_gain", pid_vel_int_);
        for (size_t i = 0; i < kN; ++i) {
            if (gear_ratio_[i] <= 0.0 || torque_gear_ratio_[i] <= 0.0) {
                throw std::runtime_error("gear_ratio / torque_gear_ratio must be > 0");
            }
        }
        dt_ = 1.0 / std::max(10.0, get_parameter("control_rate_hz").as_double());
        encoder_timeout_ = get_parameter("encoder_timeout").as_double();
        hold_vel_limit_ = static_cast<float>(get_parameter("hold_vel_limit").as_double());
        params_file_ = get_parameter("params_file").as_string();
        auto_save_params_ = get_parameter("auto_save_params").as_bool();
        startup_timeout_ = get_parameter("startup_timeout").as_double();

        // The YAML file is the source of truth: its values override the built-in defaults above.
        if (get_parameter("load_params_on_start").as_bool()) load_params_file(params_file_);
        for (size_t i = 0; i < kN; ++i) {
            if (gear_ratio_[i] <= 0.0 || torque_gear_ratio_[i] <= 0.0) {
                throw std::runtime_error("gear_ratio / torque_gear_ratio must be > 0 (after loading the params file)");
            }
        }

        // ---------------- hardware ----------------
        hw_ = std::make_shared<ros2_gim_control::GimControlInterface>();
        if (!rclcpp::ok()) {
            throw std::runtime_error("Cannot open CAN interface can0 (is it up? `sudo ip link set can0 up type can bitrate ...`)");
        }
        // Never leave anything energised from a previous run.
        for (size_t i = 0; i < kN; ++i) hw_->DisableMotor(node_ids_[i]);

        // ---------------- ROS interfaces ----------------
        auto latched = rclcpp::QoS(1).transient_local().reliable();
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/gim/joint_states", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/gim/status", latched);
        calib_status_pub_ = create_publisher<std_msgs::msg::String>("/gim/calibration_status", latched);
        calib_data_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/calibration_data", 50);
        params_pub_ = create_publisher<std_msgs::msg::String>("/gim/motor_params", latched);
        params_data_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/motor_params_data", latched);
        motor_state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/motor_state", 50);

        select_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/gim/select_motor", 10, std::bind(&ImpedanceControlNode::select_callback, this, std::placeholders::_1));
        imp_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/impedance_gains", 10, std::bind(&ImpedanceControlNode::impedance_gains_callback, this, std::placeholders::_1));
        pid_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/pid_gains", 10, std::bind(&ImpedanceControlNode::pid_gains_callback, this, std::placeholders::_1));
        traj_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/calibration_trajectory", 10, std::bind(&ImpedanceControlNode::calibration_callback, this, std::placeholders::_1));
        stop_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/stop_calibration", 10, std::bind(&ImpedanceControlNode::stop_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&ImpedanceControlNode::control_loop, this));
        status_timer_ = create_wall_timer(1s, std::bind(&ImpedanceControlNode::publish_status, this));

        // Enable everything in POSITION @ 0 on a worker thread (Init*Mode blocks for seconds).
        startup_thread_ = std::thread(&ImpedanceControlNode::startup_worker, this);

        RCLCPP_INFO(get_logger(), "impedance_control_node starting: motors start DISABLED (idle) ...");
    }

    ~ImpedanceControlNode() override { shutdown_hardware(); }

    // Called from main() after spin() returns, and from the destructor: always leave motors idle.
    void shutdown_hardware() {
        if (shut_down_) return;
        shut_down_ = true;
        cancel_ = true;
        motors_enabled_ = false;
        if (startup_thread_.joinable()) startup_thread_.join();
        if (switch_thread_.joinable()) switch_thread_.join();
        if (hw_) {
            for (size_t i = 0; i < kN; ++i) hw_->DisableMotor(node_ids_[i]);
        }
        RCLCPP_INFO(get_logger(), "All motors disabled. Bye.");
    }

private:
    // ---------------- helpers ----------------
    void load_vec3(const std::string& name, Vec3& out) {
        const auto v = get_parameter(name).as_double_array();
        if (v.size() != kN) throw std::runtime_error("parameter '" + name + "' must have 3 elements");
        for (size_t i = 0; i < kN; ++i) out[i] = v[i];
    }

    static bool all_finite(const std::vector<double>& v) {
        return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
    }

    double joint_per_turn(size_t i) const { return 2.0 * M_PI / gear_ratio_[i]; }

    // Motor-rotor turns that correspond to joint angle `q` (rad) for joint i.
    double turns_for_joint(size_t i, double q) const {
        return zero_turns_[i] + direction_[i] * q / joint_per_turn(i);
    }

    void load_params_file(const std::string& path) {
        std::ifstream f(path);
        if (!f.good()) {
            RCLCPP_INFO(get_logger(), "No parameter file '%s' found - using built-in defaults.", path.c_str());
            return;
        }
        try {
            const YAML::Node root = YAML::LoadFile(path);
            const YAML::Node motors = root["motors"];
            if (!motors) {
                RCLCPP_WARN(get_logger(), "Parameter file '%s' has no 'motors' section - ignored.", path.c_str());
                return;
            }
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
                if (m["impedance"]) {
                    if (m["impedance"]["kp"]) kp_[i] = m["impedance"]["kp"].as<double>();
                    if (m["impedance"]["kd"]) kd_[i] = m["impedance"]["kd"].as<double>();
                }
                if (m["pid"]) {
                    if (m["pid"]["pos_gain"]) pid_pos_[i] = m["pid"]["pos_gain"].as<double>();
                    if (m["pid"]["vel_gain"]) pid_vel_[i] = m["pid"]["vel_gain"].as<double>();
                    if (m["pid"]["vel_integrator_gain"]) pid_vel_int_[i] = m["pid"]["vel_integrator_gain"].as<double>();
                }
            }
            RCLCPP_INFO(get_logger(), "Loaded motor parameters from %s", path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to parse '%s' (%s) - using built-in defaults.", path.c_str(), e.what());
        }
    }

    // ---------------- publishers ----------------
    void publish_status() {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_status_locked();
    }

    // mutex_ must be held.
    void publish_status_locked() {
        std::string s;
        if (faulted_.load()) {
            s = "FAULT (all motors DISABLED)";
        } else if (!ready_.load()) {
            s = "STARTING";
        } else if (switching_.load()) {
            s = "SWITCHING";
        } else if (!motors_enabled_.load()) {
            s = "DISABLED (motors idle, select a motor to enable)";
        } else if (active_joint_ < 0) {
            s = "HOLD (all POSITION @ 0)";
        } else {
            s = std::string("IMPEDANCE ") + labels_[active_joint_];
            if (traj_.active) s += " (calibrating)";
        }
        if (!fault_.empty()) s += " | last fault: " + fault_;
        std_msgs::msg::String msg;
        msg.data = s;
        status_pub_->publish(msg);
    }

    void publish_calibration_status(const std::string& s) {
        std_msgs::msg::String msg;
        msg.data = s;
        calib_status_pub_->publish(msg);
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
            os << "    impedance:\n";
            os << "      kp: " << kp_[i] << "\n";
            os << "      kd: " << kd_[i] << "\n";
        }
        return os.str();
    }

    void publish_params() {
        std::lock_guard<std::mutex> lock(mutex_);
        std_msgs::msg::String y;
        y.data = build_params_yaml();
        params_pub_->publish(y);

        std_msgs::msg::Float64MultiArray d;
        d.data.reserve(kN * 11);
        for (size_t i = 0; i < kN; ++i) {
            d.data.push_back(static_cast<double>(node_ids_[i]));
            d.data.push_back(gear_ratio_[i]);
            d.data.push_back(direction_[i]);
            d.data.push_back(torque_sign_[i]);
            d.data.push_back(torque_gear_ratio_[i]);
            d.data.push_back(torque_limit_[i]);
            d.data.push_back(kp_[i]);
            d.data.push_back(kd_[i]);
            d.data.push_back(pid_pos_[i]);
            d.data.push_back(pid_vel_[i]);
            d.data.push_back(pid_vel_int_[i]);
        }
        params_data_pub_->publish(d);
    }

    // Writes the full parameter set to the YAML file. mutex_ must be held.
    bool write_params_file_locked() {
        std::ofstream f(params_file_);
        if (!f) {
            RCLCPP_WARN(get_logger(), "Cannot write parameter file '%s'", params_file_.c_str());
            return false;
        }
        f << "# gim_control motor parameters - source of truth for impedance_control_node.\n"
          << "# The node loads this file at start-up and re-writes it whenever a parameter changes\n"
          << "# (auto_save_params). Hand edits are kept while the node is stopped.\n"
          << build_params_yaml();
        return true;
    }

    bool save_params_file() {
        std::lock_guard<std::mutex> lock(mutex_);
        return write_params_file_locked();
    }

    void publish_joint_states() {
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name.assign(joint_names_.begin(), joint_names_.end());
        js.position.assign(q_.begin(), q_.end());
        js.velocity.assign(qd_.begin(), qd_.end());
        js.effort.assign(tau_cmd_.begin(), tau_cmd_.end());
        state_pub_->publish(js);
    }

    // Realtime state of all three motors: position, velocity, commanded and measured torque.
    // Layout: [q0 q1 q2 | qd0 qd1 qd2 | tau_cmd0 tau_cmd1 tau_cmd2 | tau_meas0 tau_meas1 tau_meas2]
    void publish_motor_state() {
        std_msgs::msg::Float64MultiArray m;
        m.data.reserve(kN * 4);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(q_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(qd_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(tau_cmd_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(tau_meas_[i]);
        motor_state_pub_->publish(m);
    }

    void publish_calibration_sample(int joint, double t, double q_ref, double qd_ref) {
        std_msgs::msg::Float64MultiArray m;
        m.data = {t,
                  static_cast<double>(joint + 1),
                  q_[joint],
                  q_ref,
                  qd_[joint],
                  qd_ref,
                  tau_cmd_[joint],
                  q_ref - q_[joint],
                  qd_ref - qd_[joint]};
        calib_data_pub_->publish(m);
    }

    // ---------------- workers ----------------
    // Waits until every motor answers on CAN, then enables all of them in POSITION mode holding
    // joint 0. Blocking; runs on its own thread. It never energises a motor it cannot read.
    void startup_worker() {
        RCLCPP_INFO(get_logger(), "Waiting for encoder feedback from all motors before enabling ...");
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
            const double waited = std::chrono::duration<double>(now - t0).count();
            if (std::chrono::duration<double>(now - last_warn).count() > 3.0) {
                last_warn = now;
                RCLCPP_WARN(get_logger(),
                            "No encoder reply from: %s - check CAN wiring/power and the node_id values in %s ...",
                            missing.c_str(), params_file_.c_str());
            }
            if (startup_timeout_ > 0.0 && waited > startup_timeout_) {
                std::lock_guard<std::mutex> lock(mutex_);
                faulted_ = true;
                fault_ = "no encoder reply from: " + missing;
                RCLCPP_ERROR(get_logger(),
                             "Giving up after %.0f s: no encoder reply from %s. Motors stay DISABLED. "
                             "Check CAN wiring/power and the node_id values in %s, then restart.",
                             startup_timeout_, missing.c_str(), params_file_.c_str());
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
                zero_turns_[i] = p;  // define joint 0 as the pose at start-up
            }
        }

        // Keep all motors disabled at start (idle/unpowered) until a command is received
        for (size_t i = 0; i < kN && !cancel_.load(); ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }
        motors_enabled_ = false;
        active_joint_ = -1;

        if (cancel_.load()) return;
        ready_ = true;
        publish_params();
        if (auto_save_params_ && save_params_file()) {
            RCLCPP_INFO(get_logger(), "Motor parameters written to %s", params_file_.c_str());
        }
        publish_status();
        publish_calibration_status("IDLE");
        RCLCPP_INFO(get_logger(), "Ready. Motors start DISABLED (idle). Select a motor with /gim/select_motor to enable.");
    }

    // Moves the requested joint into IMPEDANCE mode, all others into POSITION @ 0.
    // If target < 0 (sel == 0), disables all motors (AXIS_STATE_IDLE). Blocking.
    void select_worker(int target) {
        int prev = -1;
        bool was_enabled = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            prev = active_joint_;
            was_enabled = motors_enabled_.load();
        }

        if (target < 0) {
            RCLCPP_INFO(get_logger(), "Disabling all motors (idle) ...");
            for (size_t i = 0; i < kN; ++i) {
                hw_->DisableMotor(node_ids_[i]);
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const bool was_running = traj_.active;
                active_joint_ = -1;
                motors_enabled_ = false;
                traj_ = CalibrationTrajectory{};
                tau_cmd_ = {0.0, 0.0, 0.0};
                if (was_running) publish_calibration_status("ABORTED");

                const auto now_t = std::chrono::steady_clock::now();
                for (size_t i = 0; i < kN; ++i) {
                    last_count_[i] = hw_->encoder_sample_count(node_ids_[i]);
                    last_change_[i] = now_t;
                }
            }
            switching_ = false;
            publish_status();
            RCLCPP_INFO(get_logger(), "All motors DISABLED (idle).");
            return;
        }

        RCLCPP_INFO(get_logger(), "Enabling motors: %s in IMPEDANCE (other joints hold POSITION @ 0) ...",
                    labels_[target].c_str());

        // If motors were previously disabled, capture current resting position as joint 0
        // so enabling does not cause any sudden jump!
        if (!was_enabled) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < kN; ++i) {
                float p = 0.f, v = 0.f;
                hw_->get_encoder_data(node_ids_[i], p, v);
                zero_turns_[i] = p;
                q_[i] = 0.0;
                qd_[i] = 0.0;
            }
        }

        for (size_t i = 0; i < kN; ++i) {
            if (cancel_.load()) break;
            const int idx = static_cast<int>(i);
            const bool want_impedance = (idx == target);
            const bool is_impedance = was_enabled && (idx == prev);
            const bool is_position = was_enabled && !is_impedance;

            if (want_impedance && !is_impedance) {
                hw_->InitTorqueMode(node_ids_[i]);
                hw_->SendTorqueCommand(node_ids_[i], 0.0f);
            } else if (!want_impedance && !is_position) {
                hw_->InitPositionMode(node_ids_[i]);
                hw_->SetTrajectoryVelLimit(node_ids_[i], hold_vel_limit_);
                hw_->SetPosGain(node_ids_[i], static_cast<float>(pid_pos_[i]));
                hw_->SetVelGains(node_ids_[i], static_cast<float>(pid_vel_[i]), static_cast<float>(pid_vel_int_[i]));
                hw_->SendPositionCommand(node_ids_[i], static_cast<float>(turns_for_joint(i, 0.0)));
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            const bool was_running = traj_.active;
            active_joint_ = cancel_.load() ? -1 : target;
            motors_enabled_ = (active_joint_ >= 0);
            traj_ = CalibrationTrajectory{};
            if (active_joint_ >= 0) {
                // Impedance starts where the joint is -> no jump.
                q_ref_[active_joint_] = q_[active_joint_];
                qd_ref_[active_joint_] = qd_[active_joint_];
            }
            if (was_running) publish_calibration_status("ABORTED");

            const auto now_t = std::chrono::steady_clock::now();
            for (size_t i = 0; i < kN; ++i) {
                last_count_[i] = hw_->encoder_sample_count(node_ids_[i]);
                last_change_[i] = now_t;
            }
        }
        switching_ = false;
        publish_status();
        RCLCPP_INFO(get_logger(), "Ready: %s in IMPEDANCE mode (other joints hold POSITION @ 0)",
                    labels_[target].c_str());
    }

    // ---------------- callbacks ----------------
    void select_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        const int sel = msg->data;
        if (sel < 0 || sel > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid motor selection %d. Use 0 (disable) or 1..3.", sel);
            return;
        }
        if (!ready_.load()) {
            RCLCPP_WARN(get_logger(), "Motors not ready yet, please wait.");
            return;
        }
        if (switching_.load()) {
            RCLCPP_WARN(get_logger(), "Still switching mode, please wait.");
            return;
        }
        const int target = (sel == 0) ? -1 : sel - 1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (target == active_joint_ && motors_enabled_.load()) return;
            if (target < 0 && !motors_enabled_.load()) return;
        }
        switching_ = true;
        if (switch_thread_.joinable()) switch_thread_.join();
        switch_thread_ = std::thread(&ImpedanceControlNode::select_worker, this, target);
    }

    void impedance_gains_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 3) {
            RCLCPP_WARN(get_logger(), "impedance_gains expects [joint_index, kp, kd].");
            return;
        }
        const int idx = static_cast<int>(std::lround(msg->data[0]));
        if (idx < 1 || idx > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid joint index %d (valid 1..3).", idx);
            return;
        }
        const double kp = msg->data[1], kd = msg->data[2];
        if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0 || kd < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid impedance gains (must be finite and >= 0).");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            kp_[idx - 1] = kp;
            kd_[idx - 1] = kd;
        }
        RCLCPP_INFO(get_logger(), "%s impedance gains: kp=%.4f kd=%.4f", labels_[idx - 1].c_str(), kp, kd);
        publish_params();
        if (auto_save_params_) save_params_file();
    }

    void pid_gains_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(get_logger(), "pid_gains expects [joint_index, pos_gain, vel_gain, vel_integrator_gain].");
            return;
        }
        const int idx = static_cast<int>(std::lround(msg->data[0]));
        if (idx < 1 || idx > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid joint index %d (valid 1..3).", idx);
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
        }
        // Push to the ODrive right away (no sleeps: this runs on the executor thread).
        hw_->SetPosGain(node_ids_[idx - 1], static_cast<float>(p));
        hw_->SetVelGains(node_ids_[idx - 1], static_cast<float>(v), static_cast<float>(i));
        RCLCPP_INFO(get_logger(), "%s PID gains: pos=%.3f vel=%.3f vel_int=%.3f", labels_[idx - 1].c_str(), p, v, i);
        publish_params();
        if (auto_save_params_) save_params_file();
    }

    void calibration_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3 || msg->data.size() > 4) {
            RCLCPP_WARN(get_logger(), "calibration_trajectory expects [q_target, t_move, t_hold, (t_return)].");
            return;
        }
        if (!all_finite(msg->data)) {
            RCLCPP_WARN(get_logger(), "calibration_trajectory values must be finite.");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ready_.load() || switching_.load() || !motors_enabled_.load()) {
            RCLCPP_WARN(get_logger(), "Motors not ready or disabled - cannot run a calibration trajectory.");
            return;
        }
        if (active_joint_ < 0) {
            RCLCPP_WARN(get_logger(), "No motor selected - select one with /gim/select_motor first.");
            return;
        }
        const int j = active_joint_;
        CalibrationTrajectory traj;
        traj.joint = j;
        traj.q_target = std::clamp(msg->data[0], lower_[j], upper_[j]);
        traj.t_move = msg->data[1];
        traj.t_hold = msg->data[2];
        traj.t_return = (msg->data.size() == 4) ? msg->data[3] : msg->data[1];
        if (traj.t_move <= 0.0 || traj.t_hold < 0.0 || traj.t_return <= 0.0) {
            RCLCPP_WARN(get_logger(), "Trajectory times must be > 0 (t_hold may be 0).");
            return;
        }
        traj.active = true;
        traj.start = std::chrono::steady_clock::now();
        traj_ = traj;
        q_ref_[j] = 0.0;   // profile always starts from 0
        qd_ref_[j] = 0.0;

        RCLCPP_INFO(get_logger(), "Calibration started on %s: 0 -> %.4f rad (move %.2fs, hold %.2fs, return %.2fs)",
                    labels_[j].c_str(), traj.q_target, traj.t_move, traj.t_hold, traj.t_return);
        publish_calibration_status("RUNNING");
        publish_status_locked();
    }

    void stop_callback(const std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!traj_.active) return;
        traj_ = CalibrationTrajectory{};
        if (active_joint_ >= 0) {
            q_ref_[active_joint_] = 0.0;
            qd_ref_[active_joint_] = 0.0;
        }
        RCLCPP_WARN(get_logger(), "Calibration aborted by request.");
        publish_calibration_status("ABORTED");
    }

    // ---------------- control loop ----------------
    void control_loop() {
        // 1) feedback: read the last reply, then request a fresh one
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
            // motor-side measured torque -> joint side (inverse of the command conversion)
            tau_meas_[i] = torque_sign_[i] * motor_tau[i] * torque_gear_ratio_[i];
        }
        publish_joint_states();
        publish_motor_state();

        if (faulted_.load() || !ready_.load() || switching_.load() || !motors_enabled_.load()) return;

        // 2) safety: encoder watchdog + joint limits
        const auto now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < kN; ++i) {
            const uint64_t cnt = hw_->encoder_sample_count(node_ids_[i]);
            if (cnt != last_count_[i]) {
                last_count_[i] = cnt;
                last_change_[i] = now;
            } else if (std::chrono::duration<double>(now - last_change_[i]).count() > encoder_timeout_) {
                fault_locked("no encoder reply from node " + std::to_string(node_ids_[i]));
                return;
            }
            if (q_[i] < lower_[i] - 0.3 || q_[i] > upper_[i] + 0.3) {
                fault_locked("joint " + joint_names_[i] + " far outside limits (" + std::to_string(q_[i]) + " rad)");
                return;
            }
        }

        // 3) reference for the selected joint (trajectory, otherwise hold)
        const int act = active_joint_;
        bool sampling = false;
        double sample_t = 0.0;
        bool just_finished = false;
        if (act >= 0) update_reference_locked(now, sampling, sample_t, just_finished);

        // 4) command every motor: impedance on the selected joint, position @ 0 on the rest
        for (size_t i = 0; i < kN; ++i) {
            if (static_cast<int>(i) == act) {
                double tau = kp_[i] * (q_ref_[i] - q_[i]) + kd_[i] * (qd_ref_[i] - qd_[i]);
                if (!std::isfinite(tau)) tau = 0.0;
                tau = std::clamp(tau, -torque_limit_[i], torque_limit_[i]);
                tau_cmd_[i] = tau;
                hw_->SendTorqueCommand(node_ids_[i], static_cast<float>(torque_sign_[i] * tau / torque_gear_ratio_[i]));
            } else {
                tau_cmd_[i] = 0.0;
                hw_->SendPositionCommand(node_ids_[i], static_cast<float>(turns_for_joint(i, 0.0)));
            }
        }

        // 5) stream calibration samples
        if (sampling) publish_calibration_sample(act, sample_t, q_ref_[act], qd_ref_[act]);
        if (just_finished) {
            publish_calibration_status("DONE");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Calibration finished on %s.", labels_[act].c_str());
        }
    }

    // mutex_ must be held. Updates q_ref_/qd_ref_ for the selected joint.
    void update_reference_locked(std::chrono::steady_clock::time_point now, bool& sampling,
                                 double& sample_t, bool& just_finished) {
        const int j = active_joint_;
        if (!traj_.active || traj_.joint != j) return;  // hold the previous reference

        const double t = std::chrono::duration<double>(now - traj_.start).count();
        const double tm = traj_.t_move;
        const double th = traj_.t_hold;
        const double tr = traj_.t_return;
        const double qt = traj_.q_target;

        double qr = 0.0, qdr = 0.0;
        if (t < tm) {
            const double u = tm > 1e-9 ? t / tm : 1.0;
            qr = qt * s_curve(u);
            qdr = qt * s_curve_dot(u) / std::max(tm, 1e-9);
        } else if (t < tm + th) {
            qr = qt;
            qdr = 0.0;
        } else if (t < tm + th + tr) {
            const double u = tr > 1e-9 ? (t - tm - th) / tr : 1.0;
            qr = qt * (1.0 - s_curve(u));
            qdr = -qt * s_curve_dot(u) / std::max(tr, 1e-9);
        } else {
            qr = 0.0;
            qdr = 0.0;
            traj_ = CalibrationTrajectory{};
            just_finished = true;
        }

        q_ref_[j] = qr;
        qd_ref_[j] = qdr;
        sampling = true;
        sample_t = t;
    }

    // mutex_ must be held
    void fault_locked(const std::string& why) {
        fault_ = why;
        faulted_ = true;  // latch: stop the control loop so the fault fires only once
        motors_enabled_ = false;
        for (size_t i = 0; i < kN; ++i) hw_->DisableMotor(node_ids_[i]);
        active_joint_ = -1;
        traj_ = CalibrationTrajectory{};
        tau_cmd_ = {0.0, 0.0, 0.0};
        RCLCPP_ERROR(get_logger(), "FAULT: %s -> all motors DISABLED", why.c_str());
        publish_calibration_status("ABORTED");
        publish_status_locked();
    }

    // ---------------- members ----------------
    std::shared_ptr<ros2_gim_control::GimControlInterface> hw_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr calib_data_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr params_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_state_pub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr select_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imp_gains_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_gains_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::mutex mutex_;
    std::thread startup_thread_;
    std::thread switch_thread_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> switching_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> cancel_{false};
    std::atomic<bool> motors_enabled_{false};
    bool shut_down_ = false;
    std::string fault_;

    std::array<std::string, kN> joint_names_;
    std::array<std::string, kN> labels_;
    std::array<int, kN> node_ids_{};
    Vec3 direction_{}, gear_ratio_{}, torque_sign_{}, torque_gear_ratio_{}, torque_limit_{};
    Vec3 lower_{}, upper_{}, kp_{}, kd_{}, pid_pos_{}, pid_vel_{}, pid_vel_int_{};
    Vec3 zero_turns_{0.0, 0.0, 0.0};
    double dt_ = 0.01;
    double encoder_timeout_ = 0.2;
    float hold_vel_limit_ = 4.0f;
    std::string params_file_;
    bool auto_save_params_ = true;
    double startup_timeout_ = 10.0;

    int active_joint_ = -1;   // guarded by mutex_
    Vec3 q_{}, qd_{}, q_ref_{}, qd_ref_{}, tau_cmd_{}, tau_meas_{};
    CalibrationTrajectory traj_;

    std::array<uint64_t, kN> last_count_{};
    std::array<std::chrono::steady_clock::time_point, kN> last_change_{};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<ImpedanceControlNode>();
        rclcpp::spin(node);
        // Ctrl-C / SIGTERM: disable every motor before leaving.
        node->shutdown_hardware();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("impedance_control_node"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
