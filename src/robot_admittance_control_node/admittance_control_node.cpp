// Real-hardware admittance control node for the 3-DOF GIM6010 arm (CAN bus, via GimControlInterface).
//
//   base_joint     -> CAN node id 0
//   shoulder_joint -> CAN node id 1
//   elbow_joint    -> CAN node id 2
//
// Unlike the impedance node (torque mode on one joint), this node keeps EVERY joint in POSITION
// mode and runs a virtual mass-damper-spring ADMITTANCE law on each active joint:
//
//     M * ddelta_qdd + D * ddelta_qd + K * delta_q = tau_ext
//     q_cmd = q_ref + delta_q                       (fed to the ODrive position controller)
//
// The motor only measures the total active torque, so the external torque is isolated with
//   tau_meas -> tare subtraction -> 1st-order IIR low-pass -> deadband.
// Setting K = 0 gives transparent lead-through (free-floating) mode.
//
// Behaviour
//   * At start every motor is put to AXIS_STATE_IDLE (unpowered). The current pose is captured as
//     joint 0 once all encoders answer.
//   * /gim/select_motor picks the joint(s) that run in ADMITTANCE mode. The others hold POSITION @ 0.
//     All joints are in POSITION mode; selecting another joint does not need a mode change.
//   * /gim/calibration_trajectory runs a point-to-point profile on the active joint(s) as the
//     nominal reference q_ref: 0 -> q_target over t_move, hold for t_hold, then back to 0.
//   * On shutdown (Ctrl-C / SIGTERM) every motor is disabled (AXIS_STATE_IDLE).
//
// Topics
//   sub  /gim/select_motor           std_msgs/Int32               0=none 1=base 2=shoulder 3=elbow -1=all
//   sub  /gim/admittance_gains       std_msgs/Float64MultiArray   [joint_index, M, D, K]
//   sub  /gim/pid_gains              std_msgs/Float64MultiArray   [joint_index, pos_gain, vel_gain, vel_int_gain]
//   sub  /gim/admittance_deadband    std_msgs/Float64MultiArray   [joint_index, deadband_torque]
//   sub  /gim/tare_torque            std_msgs/Empty               sample resting torque as the zero baseline
//   sub  /gim/calibration_trajectory std_msgs/Float64MultiArray   [q_target, t_move, t_hold, (t_return)]
//   sub  /gim/stop_calibration       std_msgs/Empty               abort the running trajectory
//   pub  /gim/joint_states           sensor_msgs/JointState       measured pos/vel + measured torque
//   pub  /gim/motor_state            std_msgs/Float64MultiArray   realtime [q0..q2, qd0..qd2, q_cmd0..2, tau_meas0..2, tau_ext0..2]
//   pub  /gim/status                 std_msgs/String              latched node state
//   pub  /gim/calibration_status     std_msgs/String              latched IDLE|RUNNING|DONE|ABORTED
//   pub  /gim/calibration_data       std_msgs/Float64MultiArray   calibration samples (see below)
//   pub  /gim/motor_params           std_msgs/String              latched parameter dump (YAML)
//   pub  /gim/motor_params_data      std_msgs/Float64MultiArray   latched numeric parameter dump
//
// A calibration sample is: [t, joint_index, q_meas, q_ref, q_cmd, qd_meas, tau_ext, tau_meas, delta_q]

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

// Per-joint virtual admittance state (see TODO 3.1).
struct AdmittanceState {
    double delta_q = 0.0;       // Virtual deflection (rad)
    double delta_qd = 0.0;      // Virtual velocity (rad/s)
    double delta_qdd = 0.0;     // Virtual acceleration (rad/s^2)
    double tau_tare = 0.0;      // Zero-force baseline torque (Nm)
    double tau_filtered = 0.0;  // IIR low-pass filtered torque (Nm)
};

struct CalibrationTrajectory {
    bool active = false;
    double q_target = 0.0;   // rad
    double t_move = 1.0;     // s
    double t_hold = 1.0;     // s
    double t_return = 1.0;   // s
    std::chrono::steady_clock::time_point start;
};

}  // namespace

class AdmittanceControlNode : public rclcpp::Node {
public:
    AdmittanceControlNode() : Node("admittance_control_node") {
        // ---------------- parameters ----------------
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
        // Virtual admittance gains.
        declare_parameter<std::vector<double>>("mass", {0.15, 0.15, 0.15});
        declare_parameter<std::vector<double>>("damping", {4.0, 4.0, 4.0});
        declare_parameter<std::vector<double>>("stiffness", {25.0, 25.0, 25.0});
        declare_parameter<std::vector<double>>("max_delta_q", {0.5, 0.5, 0.5});
        declare_parameter<std::vector<double>>("max_admit_vel", {2.0, 2.0, 2.0});
        declare_parameter<std::vector<double>>("deadband_torque", {0.05, 0.05, 0.05});
        declare_parameter<std::vector<double>>("pid_pos_gain", {4.0, 0.5, 4.0});
        declare_parameter<std::vector<double>>("pid_vel_gain", {1.2, 0.1, 1.5});
        declare_parameter<std::vector<double>>("pid_vel_integrator_gain", {0.1, 0.0, 0.1});
        declare_parameter<double>("control_rate_hz", 100.0);
        declare_parameter<double>("encoder_timeout", 0.2);   // s without CAN reply -> fault + disable all
        declare_parameter<int64_t>("tare_samples", 50);      // ticks sampled at rest for the tare baseline
        declare_parameter<double>("torque_filter_hz", 20.0); // 1st-order IIR low-pass cut-off
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
        load_vec3("mass", mass_);
        load_vec3("damping", damping_);
        load_vec3("stiffness", stiffness_);
        load_vec3("max_delta_q", max_delta_q_);
        load_vec3("max_admit_vel", max_admit_vel_);
        load_vec3("deadband_torque", deadband_);
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
        tare_samples_ = std::max<int64_t>(1, get_parameter("tare_samples").as_int());
        filter_hz_ = std::max(0.1, get_parameter("torque_filter_hz").as_double());
        alpha_ = dt_ / (1.0 / (2.0 * M_PI * filter_hz_) + dt_);
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
            "/gim/select_motor", 10, std::bind(&AdmittanceControlNode::select_callback, this, std::placeholders::_1));
        gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/admittance_gains", 10, std::bind(&AdmittanceControlNode::gains_callback, this, std::placeholders::_1));
        pid_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/pid_gains", 10, std::bind(&AdmittanceControlNode::pid_gains_callback, this, std::placeholders::_1));
        deadband_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/admittance_deadband", 10, std::bind(&AdmittanceControlNode::deadband_callback, this, std::placeholders::_1));
        tare_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/tare_torque", 10, std::bind(&AdmittanceControlNode::tare_callback, this, std::placeholders::_1));
        traj_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/calibration_trajectory", 10, std::bind(&AdmittanceControlNode::calibration_callback, this, std::placeholders::_1));
        stop_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/stop_calibration", 10, std::bind(&AdmittanceControlNode::stop_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&AdmittanceControlNode::control_loop, this));
        status_timer_ = create_wall_timer(1s, std::bind(&AdmittanceControlNode::publish_status, this));

        // Wait for the encoders on a worker thread (Init*Mode blocks for seconds).
        startup_thread_ = std::thread(&AdmittanceControlNode::startup_worker, this);

        RCLCPP_INFO(get_logger(), "admittance_control_node starting: motors start DISABLED (idle) ...");
    }

    ~AdmittanceControlNode() override { shutdown_hardware(); }

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

    bool any_active() const {
        return admit_active_[0] || admit_active_[1] || admit_active_[2];
    }

    std::string active_label() const {
        std::string s;
        for (size_t i = 0; i < kN; ++i) {
            if (!admit_active_[i]) continue;
            if (!s.empty()) s += "+";
            s += labels_[i];
        }
        return s.empty() ? "none" : s;
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
            if (motors) {
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
                    if (m["deadband_torque"]) deadband_[i] = m["deadband_torque"].as<double>();
                    if (m["pid"]) {
                        if (m["pid"]["pos_gain"]) pid_pos_[i] = m["pid"]["pos_gain"].as<double>();
                        if (m["pid"]["vel_gain"]) pid_vel_[i] = m["pid"]["vel_gain"].as<double>();
                        if (m["pid"]["vel_integrator_gain"]) pid_vel_int_[i] = m["pid"]["vel_integrator_gain"].as<double>();
                    }
                    if (m["admittance"]) {
                        const YAML::Node a = m["admittance"];
                        if (a["mass"]) mass_[i] = a["mass"].as<double>();
                        if (a["damping"]) damping_[i] = a["damping"].as<double>();
                        if (a["stiffness"]) stiffness_[i] = a["stiffness"].as<double>();
                        if (a["max_delta_q"]) max_delta_q_[i] = a["max_delta_q"].as<double>();
                        if (a["max_admit_vel"]) max_admit_vel_[i] = a["max_admit_vel"].as<double>();
                    }
                }
            }
            const YAML::Node g = root["global"];
            if (g) {
                if (g["control_rate_hz"]) {
                    dt_ = 1.0 / std::max(10.0, g["control_rate_hz"].as<double>());
                }
                if (g["encoder_timeout"]) encoder_timeout_ = g["encoder_timeout"].as<double>();
                if (g["tare_samples"]) tare_samples_ = std::max<int64_t>(1, g["tare_samples"].as<int64_t>());
                if (g["torque_filter_hz"]) {
                    filter_hz_ = std::max(0.1, g["torque_filter_hz"].as<double>());
                }
                alpha_ = dt_ / (1.0 / (2.0 * M_PI * filter_hz_) + dt_);
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
        } else if (!any_active()) {
            s = "HOLD (all POSITION @ 0)";
        } else {
            s = "ADMITTANCE " + active_label();
            if (taring_) s += " (taring)";
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

    // Full parameter document. Reloads the current file so other sections (e.g. `impedance:`)
    // and hand edits are preserved, then overwrites every key this node owns.
    YAML::Node build_params_node() const {
        YAML::Node root;
        try {
            std::ifstream f(params_file_);
            if (f.good()) root = YAML::LoadFile(params_file_);
        } catch (const std::exception&) {
            root = YAML::Node(YAML::NodeType::Map);
        }
        if (!root.IsMap()) root = YAML::Node(YAML::NodeType::Map);

        YAML::Node motors = root["motors"];
        if (!motors.IsMap()) motors = YAML::Node(YAML::NodeType::Map);
        for (size_t i = 0; i < kN; ++i) {
            YAML::Node m = motors[labels_[i]];
            if (!m.IsMap()) m = YAML::Node(YAML::NodeType::Map);
            m["node_id"] = node_ids_[i];
            m["joint_name"] = joint_names_[i];
            m["gear_ratio"] = gear_ratio_[i];
            m["invert_direction"] = (direction_[i] < 0.0);
            m["torque_sign"] = torque_sign_[i];
            m["torque_gear_ratio"] = torque_gear_ratio_[i];
            m["torque_limit"] = torque_limit_[i];
            m["joint_lower"] = lower_[i];
            m["joint_upper"] = upper_[i];
            m["deadband_torque"] = deadband_[i];
            YAML::Node pid = m["pid"];
            if (!pid.IsMap()) pid = YAML::Node(YAML::NodeType::Map);
            pid["pos_gain"] = pid_pos_[i];
            pid["vel_gain"] = pid_vel_[i];
            pid["vel_integrator_gain"] = pid_vel_int_[i];
            m["pid"] = pid;
            YAML::Node adm = m["admittance"];
            if (!adm.IsMap()) adm = YAML::Node(YAML::NodeType::Map);
            adm["mass"] = mass_[i];
            adm["damping"] = damping_[i];
            adm["stiffness"] = stiffness_[i];
            adm["max_delta_q"] = max_delta_q_[i];
            adm["max_admit_vel"] = max_admit_vel_[i];
            m["admittance"] = adm;
            motors[labels_[i]] = m;
        }
        root["motors"] = motors;

        YAML::Node g = root["global"];
        if (!g.IsMap()) g = YAML::Node(YAML::NodeType::Map);
        g["control_rate_hz"] = 1.0 / dt_;
        g["encoder_timeout"] = encoder_timeout_;
        g["tare_samples"] = static_cast<int64_t>(tare_samples_);
        g["torque_filter_hz"] = filter_hz_;
        root["global"] = g;
        return root;
    }

    void publish_params() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream os;
        os << build_params_node();
        std_msgs::msg::String y;
        y.data = os.str();
        params_pub_->publish(y);

        std_msgs::msg::Float64MultiArray d;
        d.data.reserve(kN * 17);
        for (size_t i = 0; i < kN; ++i) {
            d.data.push_back(static_cast<double>(node_ids_[i]));
            d.data.push_back(gear_ratio_[i]);
            d.data.push_back(direction_[i]);
            d.data.push_back(torque_sign_[i]);
            d.data.push_back(torque_gear_ratio_[i]);
            d.data.push_back(torque_limit_[i]);
            d.data.push_back(lower_[i]);
            d.data.push_back(upper_[i]);
            d.data.push_back(mass_[i]);
            d.data.push_back(damping_[i]);
            d.data.push_back(stiffness_[i]);
            d.data.push_back(max_delta_q_[i]);
            d.data.push_back(max_admit_vel_[i]);
            d.data.push_back(deadband_[i]);
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
        f << "# gim_control motor parameters - source of truth for the control nodes.\n"
          << "# `motors:` holds shared motor/PID config plus the `admittance:` block; `global:` holds\n"
          << "# the loop settings. The node loads this file at start-up and re-writes it whenever an\n"
          << "# admittance parameter changes (auto_save_params).\n"
          << build_params_node();
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
        js.effort.assign(tau_meas_.begin(), tau_meas_.end());
        state_pub_->publish(js);
    }

    // Realtime state of all three motors.
    // Layout: [q | qd | q_cmd | tau_meas | tau_ext]
    void publish_motor_state() {
        std_msgs::msg::Float64MultiArray m;
        m.data.reserve(kN * 5);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(q_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(qd_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(q_cmd_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(tau_meas_[i]);
        for (size_t i = 0; i < kN; ++i) m.data.push_back(tau_ext_[i]);
        motor_state_pub_->publish(m);
    }

    void publish_calibration_samples(double t) {
        for (size_t i = 0; i < kN; ++i) {
            if (!admit_active_[i]) continue;
            std_msgs::msg::Float64MultiArray m;
            m.data = {t,
                      static_cast<double>(i + 1),
                      q_[i],
                      q_ref_[i],
                      q_cmd_[i],
                      qd_[i],
                      tau_ext_[i],
                      tau_meas_[i],
                      st_[i].delta_q};
            calib_data_pub_->publish(m);
        }
    }

    // ---------------- workers ----------------
    // Waits until every motor answers on CAN. Never energises a motor it cannot read.
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

        for (size_t i = 0; i < kN && !cancel_.load(); ++i) {
            hw_->DisableMotor(node_ids_[i]);
        }
        motors_enabled_ = false;
        admit_active_ = {false, false, false};

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

    // Enables the requested joint(s) in ADMITTANCE mode; all joints are POSITION mode.
    // sel == 0 disables all motors, sel == -1 selects all three. Blocking.
    void select_worker(int sel) {
        bool was_enabled = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            was_enabled = motors_enabled_.load();
        }

        if (sel == 0) {
            RCLCPP_INFO(get_logger(), "Disabling all motors (idle) ...");
            for (size_t i = 0; i < kN; ++i) hw_->DisableMotor(node_ids_[i]);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const bool was_running = traj_.active;
                admit_active_ = {false, false, false};
                motors_enabled_ = false;
                traj_ = CalibrationTrajectory{};
                taring_ = false;
                q_cmd_ = {0.0, 0.0, 0.0};
                for (size_t i = 0; i < kN; ++i) st_[i] = AdmittanceState{};
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

        std::array<bool, kN> want{};
        if (sel == -1) {
            want = {true, true, true};
        } else {
            want[sel - 1] = true;
        }

        // If motors were disabled, capture the current resting pose as joint 0 and put every
        // axis into POSITION mode with the tuned PID so enabling does not cause a jump.
        if (!was_enabled) {
            RCLCPP_INFO(get_logger(), "Enabling all motors in POSITION mode ...");
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (size_t i = 0; i < kN; ++i) {
                    float p = 0.f, v = 0.f;
                    hw_->get_encoder_data(node_ids_[i], p, v);
                    zero_turns_[i] = p;
                    q_[i] = 0.0;
                    qd_[i] = 0.0;
                    q_ref_[i] = 0.0;
                    qd_ref_[i] = 0.0;
                    st_[i] = AdmittanceState{};
                }
            }
            for (size_t i = 0; i < kN; ++i) {
                if (cancel_.load()) break;
                hw_->InitPositionMode(node_ids_[i]);
                hw_->SetTrajectoryVelLimit(node_ids_[i], hold_vel_limit_);
                hw_->SetPosGain(node_ids_[i], static_cast<float>(pid_pos_[i]));
                hw_->SetVelGains(node_ids_[i], static_cast<float>(pid_vel_[i]), static_cast<float>(pid_vel_int_[i]));
                hw_->SendPositionCommand(node_ids_[i], static_cast<float>(turns_for_joint(i, 0.0)));
            }
        }

        std::string active;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const bool was_running = traj_.active;
            admit_active_ = cancel_.load() ? std::array<bool, kN>{false, false, false} : want;
            motors_enabled_ = !cancel_.load();
            traj_ = CalibrationTrajectory{};
            taring_ = false;
            for (size_t i = 0; i < kN; ++i) {
                if (!admit_active_[i]) {
                    q_ref_[i] = 0.0;
                    qd_ref_[i] = 0.0;
                    st_[i].delta_q = 0.0;
                    st_[i].delta_qd = 0.0;
                    st_[i].delta_qdd = 0.0;
                }
            }
            if (was_running) publish_calibration_status("ABORTED");
            active = active_label();

            const auto now_t = std::chrono::steady_clock::now();
            for (size_t i = 0; i < kN; ++i) {
                last_count_[i] = hw_->encoder_sample_count(node_ids_[i]);
                last_change_[i] = now_t;
            }
        }
        switching_ = false;
        publish_status();
        RCLCPP_INFO(get_logger(), "Ready: ADMITTANCE on %s (other joints hold POSITION @ 0)", active.c_str());
    }

    // ---------------- callbacks ----------------
    void select_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        const int sel = msg->data;
        if (sel < -1 || sel > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid motor selection %d. Use 0 (disable), 1..3 or -1 (all).", sel);
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
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (sel == 0 && !motors_enabled_.load()) return;
            if (sel != 0 && motors_enabled_.load()) {
                std::array<bool, kN> want{};
                if (sel == -1) {
                    want = {true, true, true};
                } else {
                    want[sel - 1] = true;
                }
                if (want == admit_active_) return;
            }
        }
        switching_ = true;
        publish_status();
        if (switch_thread_.joinable()) switch_thread_.join();
        switch_thread_ = std::thread(&AdmittanceControlNode::select_worker, this, sel);
    }

    void gains_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(get_logger(), "admittance_gains expects [joint_index, M, D, K].");
            return;
        }
        const int idx = static_cast<int>(std::lround(msg->data[0]));
        if (idx < 1 || idx > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid joint index %d (valid 1..3).", idx);
            return;
        }
        const double m = msg->data[1], d = msg->data[2], k = msg->data[3];
        if (!std::isfinite(m) || !std::isfinite(d) || !std::isfinite(k) || m <= 0.0 || d < 0.0 || k < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid admittance gains (M > 0, D >= 0, K >= 0).");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            mass_[idx - 1] = m;
            damping_[idx - 1] = d;
            stiffness_[idx - 1] = k;
        }
        RCLCPP_INFO(get_logger(), "%s admittance gains: M=%.4f D=%.4f K=%.4f", labels_[idx - 1].c_str(), m, d, k);
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
        hw_->SetPosGain(node_ids_[idx - 1], static_cast<float>(p));
        hw_->SetVelGains(node_ids_[idx - 1], static_cast<float>(v), static_cast<float>(i));
        RCLCPP_INFO(get_logger(), "%s PID gains: pos=%.3f vel=%.3f vel_int=%.3f", labels_[idx - 1].c_str(), p, v, i);
        publish_params();
        if (auto_save_params_) save_params_file();
    }

    void deadband_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 2) {
            RCLCPP_WARN(get_logger(), "admittance_deadband expects [joint_index, deadband_torque].");
            return;
        }
        const int idx = static_cast<int>(std::lround(msg->data[0]));
        if (idx < 1 || idx > static_cast<int>(kN)) {
            RCLCPP_WARN(get_logger(), "Invalid joint index %d (valid 1..3).", idx);
            return;
        }
        const double db = msg->data[1];
        if (!std::isfinite(db) || db < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid deadband (must be finite and >= 0).");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            deadband_[idx - 1] = db;
        }
        RCLCPP_INFO(get_logger(), "%s deadband: %.4f Nm", labels_[idx - 1].c_str(), db);
        publish_params();
        if (auto_save_params_) save_params_file();
    }

    void tare_callback(const std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ready_.load() || switching_.load()) {
            RCLCPP_WARN(get_logger(), "Cannot tare now - motors are not ready.");
            return;
        }
        if (!motors_enabled_.load()) {
            RCLCPP_WARN(get_logger(), "Tare while motors are DISABLED reads ~0 torque. Enable a motor first.");
        }
        taring_ = true;
        tare_count_ = 0;
        tare_accum_ = {0.0, 0.0, 0.0};
        RCLCPP_INFO(get_logger(), "Taring: hold the arm still, sampling %ld ticks (%.2f s) ...",
                    static_cast<long>(tare_samples_), tare_samples_ * dt_);
        publish_status_locked();
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
        if (!any_active()) {
            RCLCPP_WARN(get_logger(), "No motor selected - select one with /gim/select_motor first.");
            return;
        }
        CalibrationTrajectory traj;
        traj.q_target = msg->data[0];
        traj.t_move = msg->data[1];
        traj.t_hold = msg->data[2];
        traj.t_return = (msg->data.size() == 4) ? msg->data[3] : msg->data[1];
        if (traj.t_move <= 0.0 || traj.t_hold < 0.0 || traj.t_return <= 0.0) {
            RCLCPP_WARN(get_logger(), "Trajectory times must be > 0 (t_hold may be 0).");
            return;
        }
        // Clamp the target into the limits of every active joint.
        double qt = traj.q_target;
        for (size_t i = 0; i < kN; ++i) {
            if (admit_active_[i]) qt = std::clamp(qt, lower_[i], upper_[i]);
        }
        traj.q_target = qt;
        traj.active = true;
        traj.start = std::chrono::steady_clock::now();
        traj_ = traj;
        for (size_t i = 0; i < kN; ++i) {
            if (admit_active_[i]) {
                q_ref_[i] = 0.0;   // profile always starts from 0
                qd_ref_[i] = 0.0;
            }
        }
        taring_ = false;

        RCLCPP_INFO(get_logger(), "Calibration started on %s: 0 -> %.4f rad (move %.2fs, hold %.2fs, return %.2fs)",
                    active_label().c_str(), traj.q_target, traj.t_move, traj.t_hold, traj.t_return);
        publish_calibration_status("RUNNING");
        publish_status_locked();
    }

    void stop_callback(const std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!traj_.active) return;
        traj_ = CalibrationTrajectory{};
        for (size_t i = 0; i < kN; ++i) {
            if (admit_active_[i]) {
                q_ref_[i] = 0.0;
                qd_ref_[i] = 0.0;
            }
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

        // 2) safety: encoder watchdog, joint limits, excessive interaction torque
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
            if (std::fabs(tau_meas_[i]) > torque_limit_[i]) {
                fault_locked("joint " + joint_names_[i] + " torque " + std::to_string(tau_meas_[i]) +
                             " Nm exceeds limit " + std::to_string(torque_limit_[i]));
                return;
            }
        }

        // 3) tare: sample the resting torque as the zero baseline
        if (taring_) {
            for (size_t i = 0; i < kN; ++i) tare_accum_[i] += tau_meas_[i];
            ++tare_count_;
            if (tare_count_ >= static_cast<int>(tare_samples_)) {
                for (size_t i = 0; i < kN; ++i) {
                    st_[i].tau_tare = tare_accum_[i] / static_cast<double>(tare_count_);
                    st_[i].tau_filtered = 0.0;
                }
                taring_ = false;
                RCLCPP_INFO(get_logger(), "Tare done: [%.4f %.4f %.4f] Nm",
                            st_[0].tau_tare, st_[1].tau_tare, st_[2].tau_tare);
                publish_status_locked();
            }
        }

        // 4) nominal reference q_ref for the active joint(s)
        bool sampling = false;
        double sample_t = 0.0;
        bool just_finished = false;
        if (!taring_) update_reference_locked(now, sampling, sample_t, just_finished);

        // 5) command every motor
        for (size_t i = 0; i < kN; ++i) {
            // torque conditioning: tare -> IIR low-pass -> deadband
            const double raw = tau_meas_[i] - st_[i].tau_tare;
            st_[i].tau_filtered = (1.0 - alpha_) * st_[i].tau_filtered + alpha_ * raw;
            double tau_ext = st_[i].tau_filtered;
            if (std::fabs(tau_ext) <= deadband_[i]) {
                tau_ext = 0.0;
            } else {
                tau_ext -= std::copysign(deadband_[i], tau_ext);
            }
            if (!std::isfinite(tau_ext)) tau_ext = 0.0;
            tau_ext_[i] = tau_ext;

            double q_cmd;
            if (admit_active_[i] && !taring_) {
                const double M = std::max(mass_[i], 1e-4);
                // virtual acceleration (semi-implicit Euler)
                st_[i].delta_qdd = (tau_ext - damping_[i] * st_[i].delta_qd - stiffness_[i] * st_[i].delta_q) / M;
                if (!std::isfinite(st_[i].delta_qdd)) st_[i].delta_qdd = 0.0;
                st_[i].delta_qd += st_[i].delta_qdd * dt_;
                st_[i].delta_qd = std::clamp(st_[i].delta_qd, -max_admit_vel_[i], max_admit_vel_[i]);
                st_[i].delta_q += st_[i].delta_qd * dt_;
                st_[i].delta_q = std::clamp(st_[i].delta_q, -max_delta_q_[i], max_delta_q_[i]);
                q_cmd = std::clamp(q_ref_[i] + st_[i].delta_q, lower_[i], upper_[i]);
            } else {
                st_[i].delta_q = 0.0;
                st_[i].delta_qd = 0.0;
                st_[i].delta_qdd = 0.0;
                q_cmd = std::clamp(q_ref_[i], lower_[i], upper_[i]);
            }
            q_cmd_[i] = q_cmd;
            hw_->SendPositionCommand(node_ids_[i], static_cast<float>(turns_for_joint(i, q_cmd)));
        }

        // 6) stream calibration samples
        if (sampling) publish_calibration_samples(sample_t);
        if (just_finished) {
            publish_calibration_status("DONE");
            publish_status_locked();
            RCLCPP_INFO(get_logger(), "Calibration finished on %s.", active_label().c_str());
        }
    }

    // mutex_ must be held. Updates q_ref_/qd_ref_ for the active joints.
    void update_reference_locked(std::chrono::steady_clock::time_point now, bool& sampling,
                                 double& sample_t, bool& just_finished) {
        if (!traj_.active) return;  // hold the previous reference

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

        for (size_t i = 0; i < kN; ++i) {
            if (!admit_active_[i]) continue;
            q_ref_[i] = qr;
            qd_ref_[i] = qdr;
        }
        sampling = true;
        sample_t = t;
    }

    // mutex_ must be held
    void fault_locked(const std::string& why) {
        fault_ = why;
        faulted_ = true;  // latch: stop the control loop so the fault fires only once
        motors_enabled_ = false;
        for (size_t i = 0; i < kN; ++i) hw_->DisableMotor(node_ids_[i]);
        admit_active_ = {false, false, false};
        traj_ = CalibrationTrajectory{};
        taring_ = false;
        q_cmd_ = {0.0, 0.0, 0.0};
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gains_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_gains_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr deadband_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tare_sub_;
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
    Vec3 lower_{}, upper_{}, mass_{}, damping_{}, stiffness_{}, max_delta_q_{}, max_admit_vel_{}, deadband_{};
    Vec3 pid_pos_{}, pid_vel_{}, pid_vel_int_{};
    Vec3 zero_turns_{0.0, 0.0, 0.0};
    double dt_ = 0.01;
    double encoder_timeout_ = 0.2;
    int64_t tare_samples_ = 50;
    double filter_hz_ = 20.0;
    double alpha_ = 0.5;
    float hold_vel_limit_ = 4.0f;
    std::string params_file_;
    bool auto_save_params_ = true;
    double startup_timeout_ = 10.0;

    std::array<bool, kN> admit_active_{false, false, false};  // guarded by mutex_
    bool taring_ = false;
    int tare_count_ = 0;
    Vec3 tare_accum_{};
    Vec3 q_{}, qd_{}, q_ref_{}, qd_ref_{}, q_cmd_{}, tau_meas_{}, tau_ext_{};
    std::array<AdmittanceState, kN> st_{};
    CalibrationTrajectory traj_;

    std::array<uint64_t, kN> last_count_{};
    std::array<std::chrono::steady_clock::time_point, kN> last_change_{};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<AdmittanceControlNode>();
        rclcpp::spin(node);
        // Ctrl-C / SIGTERM: disable every motor before leaving.
        node->shutdown_hardware();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("admittance_control_node"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
