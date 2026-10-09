// Impedance controller for the REAL 3-DOF GIM arm (CAN bus, via GimControlInterface).
//
//   tau = Kp * (q_ref - q) + Kd * (qd_ref - qd)        (joint space, Nm)
//
// The node always starts with the motors DISABLED (AXIS_STATE_IDLE). It only publishes the
// measured state. Motors are enabled / disabled with the /gim/mode topic:
//
//   ros2 topic pub --once /gim/mode std_msgs/msg/String "{data: gui}"         # follow joint_state_publisher_gui
//   ros2 topic pub --once /gim/mode std_msgs/msg/String "{data: trajectory}"  # follow the CSV trajectory
//   ros2 topic pub --once /gim/mode std_msgs/msg/String "{data: disable}"     # disable all motors
//
// Topics
//   sub  /gim/mode            std_msgs/String            disable | gui | trajectory
//   sub  /impedance_gains     std_msgs/Float64MultiArray [kp0 kp1 kp2 kd0 kd1 kd2]
//   sub  /gim/torque_limit    std_msgs/Float64MultiArray [t0 t1 t2]   joint torque limit (Nm)
//   sub  /gim/max_speed       std_msgs/Float64           reference slew speed (rad/s)
//   sub  /gim/set_zero        std_msgs/Empty             take the current pose as joint zero (disabled only)
//   sub  <gui_topic>          sensor_msgs/JointState     target from joint_state_publisher_gui ("/joint_states")
//   pub  /gim/joint_states    sensor_msgs/JointState     measured joint angle (rad), velocity, commanded torque
//   pub  /gim/motor_raw       std_msgs/Float64MultiArray [turns0..2, turns/s0..2] raw encoder values
//   pub  /gim/status          std_msgs/String            DISABLED | ENABLING | GUI | TRAJECTORY (+ info)
//
// Every parameter below can also be changed with `ros2 param set` / a launch file.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

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

constexpr size_t kNumJoints = 3;
using Vec3 = std::array<double, kNumJoints>;

struct TrajectoryPoint {
    double t;
    Vec3 q;
    Vec3 qd;
};

enum class State { DISABLED, ENABLING, GUI, TRAJECTORY };
enum class TrajPhase { APPROACH, PLAYING, FINISHED };

const char* state_name(State s) {
    switch (s) {
        case State::DISABLED: return "DISABLED";
        case State::ENABLING: return "ENABLING";
        case State::GUI: return "GUI";
        case State::TRAJECTORY: return "TRAJECTORY";
    }
    return "UNKNOWN";
}

}  // namespace

class ImpedanceNode : public rclcpp::Node {
public:
    ImpedanceNode() : Node("impedance_simulator_node") {
        // ---------------- parameters ----------------
        declare_parameter<std::vector<std::string>>("joint_names", {"base_joint", "shoulder_joint", "elbow_joint"});
        declare_parameter<std::vector<int64_t>>("node_ids", {01, 02, 03});
        // joint-side gear ratio (rotor turns per joint turn); URDF: base 8, shoulder 64, elbow 8
        declare_parameter<std::vector<double>>("gear_ratio", {8.0, 64.0, 8.0});
        // invert_direction per joint (URDF: base true, shoulder false, elbow true)
        declare_parameter<std::vector<bool>>("invert_direction", {true, false, true});
        declare_parameter<std::vector<double>>("torque_sign", {-1.0, 1.0, -1.0});
        declare_parameter<std::vector<double>>("torque_gear_ratio", {1.0, 8.0, 1.0});
        declare_parameter<std::vector<double>>("torque_limit", {5.0, 40.0, 5.0});
        declare_parameter<std::vector<double>>("joint_lower", {-0.7121, -0.2366, -0.2336});
        declare_parameter<std::vector<double>>("joint_upper", {1.0226, 1.3435, 1.7762});
        declare_parameter<std::vector<double>>("kp", {5.0, 10.0, 5.0});
        declare_parameter<std::vector<double>>("kd", {0.2, 0.4, 0.2});
        declare_parameter<double>("max_speed", 0.3);  // rad/s, slew limit of the reference
        declare_parameter<double>("control_rate_hz", 100.0);
        declare_parameter<double>("encoder_timeout", 0.2);  // s without CAN reply -> fault + disable
        declare_parameter<bool>("loop_trajectory", false);
        declare_parameter<std::string>("gui_topic", "/joint_states");
        declare_parameter<std::string>("csv_file_path",
                                       "/home/tue/gim_control/src/robot_trajectory/gim_arm_circle_traj.csv");

        const auto names = get_parameter("joint_names").as_string_array();
        const auto ids = get_parameter("node_ids").as_integer_array();
        const auto invert = get_parameter("invert_direction").as_bool_array();
        if (names.size() != kNumJoints || ids.size() != kNumJoints || invert.size() != kNumJoints) {
            throw std::runtime_error("joint_names / node_ids / invert_direction must have 3 elements");
        }
        for (size_t i = 0; i < kNumJoints; ++i) {
            joint_names_[i] = names[i];
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
        max_speed_ = std::max(0.0, get_parameter("max_speed").as_double());
        loop_traj_ = get_parameter("loop_trajectory").as_bool();
        encoder_timeout_ = get_parameter("encoder_timeout").as_double();
        dt_ = 1.0 / std::max(10.0, get_parameter("control_rate_hz").as_double());
        for (size_t i = 0; i < kNumJoints; ++i) {
            if (gear_ratio_[i] <= 0.0 || torque_gear_ratio_[i] <= 0.0) {
                throw std::runtime_error("gear_ratio / torque_gear_ratio must be > 0");
            }
        }

        load_trajectory(get_parameter("csv_file_path").as_string());

        // ---------------- hardware ----------------
        hw_ = std::make_shared<ros2_gim_control::GimControlInterface>();
        if (!rclcpp::ok()) {
            throw std::runtime_error("Cannot open CAN interface can0 (is it up? `sudo ip link set can0 up type can bitrate ...`)");
        }
        // Make sure nothing is left energised from a previous run.
        for (size_t i = 0; i < kNumJoints; ++i) hw_->DisableMotor(node_ids_[i]);

        // ---------------- ROS interfaces ----------------
        auto latched = rclcpp::QoS(1).transient_local().reliable();
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/gim/joint_states", 10);
        raw_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/motor_raw", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/gim/status", latched);

        mode_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/mode", 10, std::bind(&ImpedanceNode::mode_callback, this, std::placeholders::_1));
        gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/impedance_gains", 10, std::bind(&ImpedanceNode::gains_callback, this, std::placeholders::_1));
        limit_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/torque_limit", 10, std::bind(&ImpedanceNode::limit_callback, this, std::placeholders::_1));
        speed_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/gim/max_speed", 10, std::bind(&ImpedanceNode::speed_callback, this, std::placeholders::_1));
        zero_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/gim/set_zero", 10, std::bind(&ImpedanceNode::zero_callback, this, std::placeholders::_1));
        gui_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("gui_topic").as_string(), 10,
            std::bind(&ImpedanceNode::gui_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&ImpedanceNode::control_loop, this));
        status_timer_ = create_wall_timer(1s, std::bind(&ImpedanceNode::publish_status, this));

        publish_status();
        RCLCPP_INFO(get_logger(),
                    "Impedance node started with motors DISABLED. Enable with: "
                    "ros2 topic pub --once /gim/mode std_msgs/msg/String \"{data: gui}\"  (or trajectory)");
    }

    ~ImpedanceNode() override { shutdown_hardware(); }

    // Called from main() after spin() returns (and from the destructor): always leave motors idle.
    void shutdown_hardware() {
        if (shut_down_) return;
        shut_down_ = true;
        cancel_enable_ = true;
        state_ = State::DISABLED;
        if (worker_.joinable()) worker_.join();
        if (hw_) {
            for (size_t i = 0; i < kNumJoints; ++i) hw_->DisableMotor(node_ids_[i]);
        }
        RCLCPP_INFO(get_logger(), "All motors disabled. Bye.");
    }

private:
    // ---------------- helpers ----------------
    void load_vec3(const std::string& name, Vec3& out) {
        const auto v = get_parameter(name).as_double_array();
        if (v.size() != kNumJoints) throw std::runtime_error("parameter '" + name + "' must have 3 elements");
        for (size_t i = 0; i < kNumJoints; ++i) out[i] = v[i];
    }

    static bool all_finite(const std::vector<double>& v) {
        return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
    }

    double joint_per_turn(size_t i) const { return 2.0 * M_PI / gear_ratio_[i]; }

    void publish_status() {
        std_msgs::msg::String msg;
        msg.data = state_name(state_.load());
        if (!fault_.empty()) msg.data += " (last fault: " + fault_ + ")";
        status_pub_->publish(msg);
    }

    void load_trajectory(const std::string& filename) {
        trajectory_.clear();
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_WARN(get_logger(), "Cannot open trajectory CSV '%s' - trajectory mode unavailable", filename.c_str());
            return;
        }
        std::string line;
        std::getline(file, line);  // header
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string cell;
            TrajectoryPoint pt;
            try {
                std::getline(ss, cell, ','); pt.t = std::stod(cell);
                for (size_t i = 0; i < kNumJoints; ++i) { std::getline(ss, cell, ','); pt.q[i] = std::stod(cell); }
                for (size_t i = 0; i < kNumJoints; ++i) { std::getline(ss, cell, ','); pt.qd[i] = std::stod(cell); }
                trajectory_.push_back(pt);
            } catch (...) { continue; }
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu trajectory points from %s", trajectory_.size(), filename.c_str());
    }

    TrajectoryPoint sample_trajectory(double t) const {
        if (t <= trajectory_.front().t) return trajectory_.front();
        if (t >= trajectory_.back().t) return trajectory_.back();
        auto it = std::lower_bound(trajectory_.begin(), trajectory_.end(), t,
                                   [](const TrajectoryPoint& p, double time) { return p.t < time; });
        auto prev = std::prev(it);
        const double span = it->t - prev->t;
        const double r = span > 1e-9 ? (t - prev->t) / span : 0.0;
        TrajectoryPoint pt;
        pt.t = t;
        for (size_t i = 0; i < kNumJoints; ++i) {
            pt.q[i] = prev->q[i] + r * (it->q[i] - prev->q[i]);
            pt.qd[i] = prev->qd[i] + r * (it->qd[i] - prev->qd[i]);
        }
        return pt;
    }

    // ---------------- callbacks ----------------
    void gains_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 6 || !all_finite(msg->data) ||
            std::any_of(msg->data.begin(), msg->data.end(), [](double x) { return x < 0.0; })) {
            RCLCPP_WARN(get_logger(), "Invalid gains. Expected 6 finite non-negative values [kp0 kp1 kp2 kd0 kd1 kd2].");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < kNumJoints; ++i) {
            kp_[i] = msg->data[i];
            kd_[i] = msg->data[i + 3];
        }
        RCLCPP_INFO(get_logger(), "Gains - Kp: [%.2f, %.2f, %.2f], Kd: [%.3f, %.3f, %.3f]",
                    kp_[0], kp_[1], kp_[2], kd_[0], kd_[1], kd_[2]);
    }

    void limit_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 3 || !all_finite(msg->data) ||
            std::any_of(msg->data.begin(), msg->data.end(), [](double x) { return x < 0.0; })) {
            RCLCPP_WARN(get_logger(), "Invalid torque limit. Expected 3 finite non-negative values (Nm).");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < kNumJoints; ++i) torque_limit_[i] = msg->data[i];
        RCLCPP_INFO(get_logger(), "Torque limit [Nm]: [%.2f, %.2f, %.2f]", torque_limit_[0], torque_limit_[1], torque_limit_[2]);
    }

    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data) || msg->data < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid max_speed.");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        max_speed_ = msg->data;
        RCLCPP_INFO(get_logger(), "max_speed = %.3f rad/s", max_speed_);
    }

    void zero_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (state_ != State::DISABLED) {
            RCLCPP_WARN(get_logger(), "set_zero is only allowed while motors are DISABLED.");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < kNumJoints; ++i) {
            float pos = 0.f, vel = 0.f;
            hw_->get_encoder_data(node_ids_[i], pos, vel);
            zero_turns_[i] = pos;
        }
        RCLCPP_INFO(get_logger(), "Joint zero set to current pose (raw turns: %.4f %.4f %.4f)",
                    zero_turns_[0], zero_turns_[1], zero_turns_[2]);
    }

    void gui_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < kNumJoints; ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
            if (it == msg->name.end()) continue;
            const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
            if (idx < msg->position.size() && std::isfinite(msg->position[idx])) {
                gui_target_[i] = msg->position[idx];
                gui_received_[i] = true;
            }
        }
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), [](unsigned char c) { return std::tolower(c); });

        if (cmd == "disable" || cmd == "off" || cmd == "stop") {
            disable_all("requested by user");
            return;
        }

        State target;
        if (cmd == "gui") target = State::GUI;
        else if (cmd == "trajectory" || cmd == "traj") target = State::TRAJECTORY;
        else {
            RCLCPP_WARN(get_logger(), "Unknown mode '%s'. Use: disable | gui | trajectory", msg->data.c_str());
            return;
        }

        if (target == State::TRAJECTORY && trajectory_.empty()) {
            RCLCPP_ERROR(get_logger(), "No trajectory loaded - cannot start trajectory mode.");
            return;
        }

        const State current = state_.load();
        if (current == State::ENABLING) {
            RCLCPP_WARN(get_logger(), "Motors are still being enabled, please wait.");
            return;
        }
        if (current == target) return;

        if (current != State::DISABLED) {
            // Hot switch between GUI <-> TRAJECTORY: keep the reference, no motor re-init.
            std::lock_guard<std::mutex> lock(mutex_);
            begin_mode(target);
            RCLCPP_INFO(get_logger(), "Switched to %s mode", state_name(target));
            publish_status();
            return;
        }

        for (size_t i = 0; i < kNumJoints; ++i) {
            if (!hw_->has_encoder_data(node_ids_[i])) {
                RCLCPP_ERROR(get_logger(), "No encoder data from node %d - refusing to enable (CAN up? motor powered?)",
                             node_ids_[i]);
                return;
            }
        }

        fault_.clear();
        if (worker_.joinable()) worker_.join();
        cancel_enable_ = false;
        state_ = State::ENABLING;
        publish_status();
        worker_ = std::thread(&ImpedanceNode::enable_worker, this, target);
    }

    // InitTorqueMode() blocks for several seconds per motor, so it runs off the executor thread.
    void enable_worker(State target) {
        RCLCPP_INFO(get_logger(), "Enabling motors (torque mode)...");
        for (size_t i = 0; i < kNumJoints && !cancel_enable_; ++i) {
            hw_->InitTorqueMode(node_ids_[i]);
            hw_->SendTorqueCommand(node_ids_[i], 0.0f);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (cancel_enable_) {
                for (size_t i = 0; i < kNumJoints; ++i) hw_->DisableMotor(node_ids_[i]);
                state_ = State::DISABLED;
                RCLCPP_WARN(get_logger(), "Enabling cancelled, motors disabled.");
                publish_status();
                return;
            }
            for (size_t i = 0; i < kNumJoints; ++i) {
                last_count_[i] = hw_->encoder_sample_count(node_ids_[i]);
                last_change_[i] = std::chrono::steady_clock::now();
                q_ref_[i] = q_[i];  // start exactly where the arm is -> no jump
            }
            begin_mode(target);
        }
        publish_status();
        RCLCPP_INFO(get_logger(), "Motors ENABLED in %s mode", state_name(target));
    }

    // mutex_ must be held. Resets the reference generator for the given mode.
    void begin_mode(State target) {
        if (target == State::TRAJECTORY) traj_phase_ = TrajPhase::APPROACH;
        qd_ref_ = {0.0, 0.0, 0.0};
        state_ = target;
    }

    void disable_all(const std::string& reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::ENABLING) {
            // The worker thread notices the flag between motors and disables everything.
            cancel_enable_ = true;
            RCLCPP_WARN(get_logger(), "Cancel requested while enabling (%s) - will disable after the current motor init.",
                        reason.c_str());
            return;
        }
        const State before = state_.exchange(State::DISABLED);
        for (size_t i = 0; i < kNumJoints; ++i) hw_->DisableMotor(node_ids_[i]);
        tau_cmd_ = {0.0, 0.0, 0.0};
        if (before != State::DISABLED) RCLCPP_WARN(get_logger(), "Motors DISABLED (%s)", reason.c_str());
        publish_status();
    }

    // ---------------- control loop ----------------
    void control_loop() {
        // 1) read the latest feedback (reply to the previous request) and ask for a new one
        Vec3 raw_pos{}, raw_vel{};
        for (size_t i = 0; i < kNumJoints; ++i) {
            float pos = 0.f, vel = 0.f;
            hw_->get_encoder_data(node_ids_[i], pos, vel);
            raw_pos[i] = pos;
            raw_vel[i] = vel;
            hw_->ReadEncoder(node_ids_[i]);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < kNumJoints; ++i) {
            q_[i] = direction_[i] * (raw_pos[i] - zero_turns_[i]) * joint_per_turn(i);
            qd_[i] = direction_[i] * raw_vel[i] * joint_per_turn(i);
        }
        publish_feedback(raw_pos, raw_vel);

        const State st = state_.load();
        if (st != State::GUI && st != State::TRAJECTORY) return;

        // 2) safety: encoder watchdog and joint limits
        const auto now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < kNumJoints; ++i) {
            const uint64_t cnt = hw_->encoder_sample_count(node_ids_[i]);
            if (cnt != last_count_[i]) {
                last_count_[i] = cnt;
                last_change_[i] = now;
            } else if (std::chrono::duration<double>(now - last_change_[i]).count() > encoder_timeout_) {
                fault("no encoder reply from node " + std::to_string(node_ids_[i]));
                return;
            }
            if (q_[i] < lower_[i] - 0.3 || q_[i] > upper_[i] + 0.3) {
                fault("joint " + joint_names_[i] + " far outside limits (" + std::to_string(q_[i]) + " rad)");
                return;
            }
        }

        // 3) reference generation
        Vec3 target{};
        Vec3 target_qd{};
        bool use_ff = false;
        if (st == State::GUI) {
            const bool have_gui = gui_received_[0] && gui_received_[1] && gui_received_[2];
            target = have_gui ? gui_target_ : q_ref_;  // no GUI message yet: hold
        } else {
            if (traj_phase_ == TrajPhase::APPROACH) {
                target = trajectory_.front().q;
            } else {
                double t = std::chrono::duration<double>(now - traj_start_).count();
                const double T = trajectory_.back().t;
                if (t > T) {
                    if (loop_traj_) {
                        traj_start_ = now;
                        t = 0.0;
                    } else {
                        traj_phase_ = TrajPhase::FINISHED;
                    }
                }
                if (traj_phase_ == TrajPhase::FINISHED) {
                    target = trajectory_.back().q;
                } else {
                    const TrajectoryPoint pt = sample_trajectory(t);
                    target = pt.q;
                    target_qd = pt.qd;
                    use_ff = true;
                }
            }
        }

        for (size_t i = 0; i < kNumJoints; ++i) target[i] = std::clamp(target[i], lower_[i], upper_[i]);

        if (use_ff) {
            // follow the trajectory exactly (it is already smooth)
            q_ref_ = target;
            qd_ref_ = target_qd;
        } else {
            // slew-limited approach: never jump
            const double step = max_speed_ * dt_;
            bool arrived = true;
            for (size_t i = 0; i < kNumJoints; ++i) {
                const double err = target[i] - q_ref_[i];
                const double d = std::clamp(err, -step, step);
                q_ref_[i] += d;
                qd_ref_[i] = 0.0;
                if (std::fabs(target[i] - q_ref_[i]) > 1e-4) arrived = false;
            }
            if (st == State::TRAJECTORY && traj_phase_ == TrajPhase::APPROACH && arrived) {
                traj_phase_ = TrajPhase::PLAYING;
                traj_start_ = now;
                RCLCPP_INFO(get_logger(), "Reached trajectory start, playing trajectory");
            }
        }

        // 4) impedance law + saturation, then send to the motors
        for (size_t i = 0; i < kNumJoints; ++i) {
            double tau = kp_[i] * (q_ref_[i] - q_[i]) + kd_[i] * (qd_ref_[i] - qd_[i]);
            if (!std::isfinite(tau)) tau = 0.0;
            tau = std::clamp(tau, -torque_limit_[i], torque_limit_[i]);
            tau_cmd_[i] = tau;
            const double motor_tau = torque_sign_[i] * tau / torque_gear_ratio_[i];
            hw_->SendTorqueCommand(node_ids_[i], static_cast<float>(motor_tau));
        }
    }

    // mutex_ must be held
    void fault(const std::string& why) {
        fault_ = why;
        for (size_t i = 0; i < kNumJoints; ++i) hw_->DisableMotor(node_ids_[i]);
        state_ = State::DISABLED;
        tau_cmd_ = {0.0, 0.0, 0.0};
        RCLCPP_ERROR(get_logger(), "FAULT: %s -> all motors DISABLED", why.c_str());
        publish_status();
    }

    void publish_feedback(const Vec3& raw_pos, const Vec3& raw_vel) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name.assign(joint_names_.begin(), joint_names_.end());
        js.position.assign(q_.begin(), q_.end());
        js.velocity.assign(qd_.begin(), qd_.end());
        js.effort.assign(tau_cmd_.begin(), tau_cmd_.end());
        state_pub_->publish(js);

        std_msgs::msg::Float64MultiArray raw;
        raw.data.assign(raw_pos.begin(), raw_pos.end());
        raw.data.insert(raw.data.end(), raw_vel.begin(), raw_vel.end());
        raw_pub_->publish(raw);
    }

    // ---------------- members ----------------
    std::shared_ptr<ros2_gim_control::GimControlInterface> hw_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gains_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr limit_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr zero_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gui_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::mutex mutex_;
    std::thread worker_;
    std::atomic<State> state_{State::DISABLED};
    std::atomic<bool> cancel_enable_{false};
    bool shut_down_ = false;
    std::string fault_;

    std::array<std::string, kNumJoints> joint_names_;
    std::array<int, kNumJoints> node_ids_{};
    Vec3 direction_{}, gear_ratio_{}, torque_sign_{}, torque_gear_ratio_{}, torque_limit_{};
    Vec3 lower_{}, upper_{}, kp_{}, kd_{};
    Vec3 zero_turns_{0.0, 0.0, 0.0};
    double max_speed_ = 0.3;
    double dt_ = 0.01;
    double encoder_timeout_ = 0.2;
    bool loop_traj_ = false;

    Vec3 q_{}, qd_{}, q_ref_{}, qd_ref_{}, tau_cmd_{};
    Vec3 gui_target_{};
    std::array<bool, kNumJoints> gui_received_{false, false, false};

    std::vector<TrajectoryPoint> trajectory_;
    TrajPhase traj_phase_ = TrajPhase::APPROACH;
    std::chrono::steady_clock::time_point traj_start_;

    std::array<uint64_t, kNumJoints> last_count_{};
    std::array<std::chrono::steady_clock::time_point, kNumJoints> last_change_{};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<ImpedanceNode>();
        rclcpp::spin(node);
        // Ctrl-C / SIGTERM: disable every motor before leaving.
        node->shutdown_hardware();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("impedance_simulator_node"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
