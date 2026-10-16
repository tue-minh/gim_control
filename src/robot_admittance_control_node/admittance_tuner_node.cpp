// Interactive tuner / calibration front-end for the 3-DOF GIM arm in ADMITTANCE mode.
//
// This node does NOT touch the CAN bus. It talks to `admittance_control_node` over topics and
// gives you a continuous command line in the terminal to tune the virtual admittance live:
//
//   gim_admit> select shoulder        # run one joint in ADMITTANCE mode (others hold POSITION @ 0)
//   gim_admit> admit shoulder 0.15 4 25   # set virtual gains [M D K]
//   gim_admit> mass shoulder 0.15     # change one virtual gain
//   gim_admit> tare                   # capture the resting torque as the zero baseline
//   gim_admit> deadband shoulder 0.05 # sensor deadband [Nm]
//   gim_admit> pid shoulder 0.5 0.1 0 # set ODrive PID gains [pos_gain vel_gain vel_integrator_gain]
//   gim_admit> traj 0.6 2 1 2         # 0 -> 0.6 rad in 2 s, hold 1 s, back to 0 in 2 s
//   gim_admit> read                   # print the live motor parameters
//   gim_admit> save                   # write parameters to config/motor_params.yaml
//   gim_admit> plot                   # re-plot the last calibration run
//   gim_admit> status                 # show node/selection state
//   gim_admit> quit
//
// After every calibration trajectory finishes, the recorded run is written to
//   config/calibration_logs/admit_<motor>_<timestamp>.csv   and the matching .png figure is generated.
//
// The same functions are also exposed as topics so another node / a script can drive the tuner:
//   sub  /tuner/select_motor            std_msgs/Int32
//   sub  /tuner/admittance_gains        std_msgs/Float64MultiArray [joint_index, M, D, K]
//   sub  /tuner/pid_gains               std_msgs/Float64MultiArray [joint_index, p, v, i]
//   sub  /tuner/deadband                std_msgs/Float64MultiArray [joint_index, deadband]
//   sub  /tuner/tare                    std_msgs/Empty
//   sub  /tuner/calibration_trajectory  std_msgs/Float64MultiArray [q_target, t_move, t_hold, (t_return)]
//   sub  /tuner/read_params             std_msgs/Empty
//   sub  /tuner/save_params             std_msgs/String  (filename, empty = motor_params.yaml)
//   sub  /tuner/plot                    std_msgs/Empty
//   sub  /tuner/stop                    std_msgs/Empty
//   sub  /tuner/command                 std_msgs/String  (a command line, e.g. "select elbow")
//   pub  /tuner/motor_params            std_msgs/String  (latched copy of the parameters)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
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

namespace {

constexpr size_t kN = 3;

std::string stamp_now() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

struct Sample {
    double t = 0.0;
    double q_meas = 0.0;
    double q_ref = 0.0;
    double q_cmd = 0.0;
    double qd_meas = 0.0;
    double tau_ext = 0.0;
    double tau_meas = 0.0;
    double delta_q = 0.0;
};

}  // namespace

class AdmittanceTunerNode : public rclcpp::Node {
public:
    AdmittanceTunerNode() : Node("admittance_tuner_node") {
        declare_parameter<std::vector<std::string>>("motor_labels", {"base", "shoulder", "elbow"});
        declare_parameter<std::string>("config_dir", "/home/tue/gim_control/config");
        declare_parameter<std::string>("log_dir", "");   // empty -> <config_dir>/calibration_logs
        declare_parameter<std::string>("params_filename", "motor_params.yaml");
        declare_parameter<std::string>("plot_script", "/home/tue/gim_control/scripts/plot_admittance.py");

        const auto labels = get_parameter("motor_labels").as_string_array();
        if (labels.size() != kN) throw std::runtime_error("motor_labels must have 3 elements");
        for (size_t i = 0; i < kN; ++i) labels_[i] = labels[i];
        config_dir_ = get_parameter("config_dir").as_string();
        log_dir_ = get_parameter("log_dir").as_string();
        if (log_dir_.empty()) log_dir_ = config_dir_ + "/calibration_logs";
        params_filename_ = get_parameter("params_filename").as_string();
        plot_script_ = get_parameter("plot_script").as_string();

        // ---- to the hardware node ----
        select_pub_ = create_publisher<std_msgs::msg::Int32>("/gim/select_motor", 10);
        gains_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/admittance_gains", 10);
        pid_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/pid_gains", 10);
        deadband_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/admittance_deadband", 10);
        tare_pub_ = create_publisher<std_msgs::msg::Empty>("/gim/tare_torque", 10);
        traj_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/calibration_trajectory", 10);
        stop_pub_ = create_publisher<std_msgs::msg::Empty>("/gim/stop_calibration", 10);

        // ---- from the hardware node ----
        auto latched = rclcpp::QoS(1).transient_local().reliable();
        params_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/motor_params", latched, std::bind(&AdmittanceTunerNode::params_callback, this, std::placeholders::_1));
        params_data_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/motor_params_data", latched,
            std::bind(&AdmittanceTunerNode::params_data_callback, this, std::placeholders::_1));
        arm_status_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/status", latched, std::bind(&AdmittanceTunerNode::arm_status_callback, this, std::placeholders::_1));
        calib_status_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/calibration_status", latched,
            std::bind(&AdmittanceTunerNode::calib_status_callback, this, std::placeholders::_1));
        calib_data_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/calibration_data", 100,
            std::bind(&AdmittanceTunerNode::calib_data_callback, this, std::placeholders::_1));

        // ---- tuner topics (drive the tuner from other nodes/scripts) ----
        select_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/tuner/select_motor", 10, [this](const std_msgs::msg::Int32::SharedPtr m) { do_select(m->data); });
        gains_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuner/admittance_gains", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                if (m->data.size() == 4)
                    do_set_gains(static_cast<int>(std::lround(m->data[0])), m->data[1], m->data[2], m->data[3]);
            });
        pid_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuner/pid_gains", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                if (m->data.size() == 4)
                    do_set_pid(static_cast<int>(std::lround(m->data[0])), m->data[1], m->data[2], m->data[3]);
            });
        deadband_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuner/deadband", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                if (m->data.size() == 2) do_set_deadband(static_cast<int>(std::lround(m->data[0])), m->data[1]);
            });
        tare_cmd_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/tuner/tare", 10, [this](const std_msgs::msg::Empty::SharedPtr) { do_tare(); });
        traj_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuner/calibration_trajectory", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                if (m->data.size() >= 3) do_trajectory(m->data[0], m->data[1], m->data[2],
                                                       m->data.size() >= 4 ? m->data[3] : m->data[1]);
            });
        read_cmd_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/tuner/read_params", 10, [this](const std_msgs::msg::Empty::SharedPtr) { do_read(); });
        save_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/tuner/save_params", 10, [this](const std_msgs::msg::String::SharedPtr m) { do_save(m->data); });
        plot_cmd_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/tuner/plot", 10, [this](const std_msgs::msg::Empty::SharedPtr) { do_plot(""); });
        stop_cmd_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/tuner/stop", 10, [this](const std_msgs::msg::Empty::SharedPtr) { do_stop(); });
        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/tuner/command", 10, [this](const std_msgs::msg::String::SharedPtr m) { enqueue(m->data); });

        tuner_params_pub_ = create_publisher<std_msgs::msg::String>("/tuner/motor_params", latched);
    }

    void run() {
        print_banner();
        running_ = true;
        input_thread_ = std::thread(&AdmittanceTunerNode::input_loop, this);
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(this->shared_from_this());
        while (rclcpp::ok() && running_.load()) {
            exec.spin_some(std::chrono::milliseconds(20));
            process_commands();
        }
        running_ = false;
        if (input_thread_.joinable()) input_thread_.detach();
        std::cout << "[tuner] bye.\n";
    }

private:
    // ---------------- input / command plumbing ----------------
    void input_loop() {
        std::string line;
        while (running_.load()) {
            std::cout << "gim_admit> " << std::flush;
            if (!std::getline(std::cin, line)) break;
            enqueue(line);
        }
    }

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

    void print_banner() const {
        std::cout << "\n================ gim_control admittance tuner ================\n"
                  << "3 motors: 1=base  2=shoulder  3=elbow\n"
                  << "type 'help' for the command list.\n"
                  << "==============================================================\n\n";
    }

    void print_help() const {
        std::cout <<
            "select <motor>              run motor in ADMITTANCE mode, others hold POSITION @ 0\n"
            "                            (0=disable, 1..3 or base|shoulder|elbow, -1 or all)\n"
            "disable / off               disable all motors (AXIS_STATE_IDLE / unpowered)\n"
            "admit <motor> <M> <D> <K>   set virtual inertia, damping, stiffness\n"
            "mass <motor> <value>        set virtual inertia only\n"
            "damp <motor> <value>        set virtual damping only\n"
            "stiff <motor> <value>       set virtual stiffness only (0 = lead-through)\n"
            "tare                        capture resting torque as the zero baseline (hold still)\n"
            "deadband <motor> <Nm>       set the torque deadband\n"
            "pid <motor> <p> <v> <i>     set ODrive PID gains (pos_gain, vel_gain, vel_integrator_gain)\n"
            "traj <q> <t_move> <t_hold> [t_return]   run 0 -> q -> 0 calibration trajectory\n"
            "stop                        abort the running trajectory\n"
            "read                        print the live motor parameters\n"
            "save [filename]             write parameters to config/ (default: motor_params.yaml)\n"
            "plot                        re-plot the last calibration run\n"
            "status                      show connection / selection state\n"
            "help                        this text\n"
            "quit                        exit\n";
    }

    void execute_command(const std::string& line) {
        std::istringstream ss(line);
        std::string cmd;
        ss >> cmd;
        cmd = lower(cmd);
        if (cmd.empty()) return;

        if (cmd == "help" || cmd == "?") {
            print_help();
        } else if (cmd == "quit" || cmd == "exit" || cmd == "q") {
            running_ = false;
        } else if (cmd == "disable" || cmd == "off" || cmd == "idle" || cmd == "0") {
            do_select(0);
        } else if (cmd == "select") {
            std::string motor;
            ss >> motor;
            do_select(parse_motor(motor));
        } else if (cmd == "read") {
            do_read();
        } else if (cmd == "admit" || cmd == "admittance" || cmd == "gain") {
            std::string motor; double m = 0, d = 0, k = 0;
            if (!(ss >> motor >> m >> d >> k)) { std::cout << "[tuner] usage: admit <motor> <M> <D> <K>\n"; return; }
            do_set_gains(parse_motor(motor), m, d, k);
        } else if (cmd == "mass" || cmd == "m") {
            std::string motor; double v = 0;
            if (!(ss >> motor >> v)) { std::cout << "[tuner] usage: mass <motor> <value>\n"; return; }
            const int idx = parse_motor(motor);
            if (idx >= 1) do_set_gains(idx, v, damping_[idx - 1], stiffness_[idx - 1]);
        } else if (cmd == "damp" || cmd == "damping" || cmd == "d") {
            std::string motor; double v = 0;
            if (!(ss >> motor >> v)) { std::cout << "[tuner] usage: damp <motor> <value>\n"; return; }
            const int idx = parse_motor(motor);
            if (idx >= 1) do_set_gains(idx, mass_[idx - 1], v, stiffness_[idx - 1]);
        } else if (cmd == "stiff" || cmd == "stiffness" || cmd == "k") {
            std::string motor; double v = 0;
            if (!(ss >> motor >> v)) { std::cout << "[tuner] usage: stiff <motor> <value>\n"; return; }
            const int idx = parse_motor(motor);
            if (idx >= 1) do_set_gains(idx, mass_[idx - 1], damping_[idx - 1], v);
        } else if (cmd == "tare" || cmd == "calibrate_torque") {
            do_tare();
        } else if (cmd == "deadband" || cmd == "db") {
            std::string motor; double v = 0;
            if (!(ss >> motor >> v)) { std::cout << "[tuner] usage: deadband <motor> <Nm>\n"; return; }
            do_set_deadband(parse_motor(motor), v);
        } else if (cmd == "pid") {
            std::string motor; double p = 0, v = 0, i = 0;
            if (!(ss >> motor >> p >> v >> i)) { std::cout << "[tuner] usage: pid <motor> <pos> <vel> <vel_int>\n"; return; }
            do_set_pid(parse_motor(motor), p, v, i);
        } else if (cmd == "traj" || cmd == "trajectory") {
            double q = 0, tm = 0, th = 0, tr = -1;
            if (!(ss >> q >> tm >> th)) { std::cout << "[tuner] usage: traj <q> <t_move> <t_hold> [t_return]\n"; return; }
            if (!(ss >> tr)) tr = tm;
            do_trajectory(q, tm, th, tr);
        } else if (cmd == "stop") {
            do_stop();
        } else if (cmd == "save") {
            std::string file;
            ss >> file;
            do_save(file);
        } else if (cmd == "plot") {
            do_plot("");
        } else if (cmd == "status") {
            print_status();
        } else if (cmd == "select_motor") {
            int s = 0; ss >> s; do_select(s);
        } else {
            std::cout << "[tuner] unknown command '" << cmd << "' (type 'help')\n";
        }
    }

    // Returns: 0 = none/disable, 1..3 = joint, -1 = all, -2 = invalid.
    int parse_motor(const std::string& token) const {
        const std::string t = lower(token);
        if (t == "none" || t == "off" || t == "disable") return 0;
        if (t == "all") return -1;
        for (size_t i = 0; i < kN; ++i) {
            if (t == labels_[i]) return static_cast<int>(i) + 1;
        }
        try {
            const int v = std::stoi(t);
            if (v == -1 || (v >= 0 && v <= static_cast<int>(kN))) return v;
        } catch (...) {
        }
        return -2;
    }

    std::string label_of(int idx1) const {
        if (idx1 == -1) return "all";
        if (idx1 >= 1 && idx1 <= static_cast<int>(kN)) return labels_[idx1 - 1];
        return "?";
    }

    // ---------------- operations ----------------
    void do_select(int sel) {
        if (sel < -1 || sel > static_cast<int>(kN)) {
            std::cout << "[tuner] select expects 0 (none), 1..3, or -1/all\n";
            return;
        }
        std_msgs::msg::Int32 m;
        m.data = sel;
        select_pub_->publish(m);
        selected_ = sel;
        if (sel == 0) {
            std::cout << "[tuner] request: DISABLE all motors (idle / unpowered)\n";
        } else {
            std::cout << "[tuner] request: enable " << label_of(sel) << " in ADMITTANCE mode (others hold POSITION @ 0)\n";
            std::cout << "[tuner] note: mode switch takes ~0.5s. Wait until status shows 'ADMITTANCE "
                      << label_of(sel) << "' before running 'traj'.\n";
        }
    }

    void do_set_gains(int idx1, double m, double d, double k) {
        if (idx1 < 1 || idx1 > static_cast<int>(kN)) { std::cout << "[tuner] invalid motor.\n"; return; }
        if (!std::isfinite(m) || !std::isfinite(d) || !std::isfinite(k) || m <= 0 || d < 0 || k < 0) {
            std::cout << "[tuner] gains need M > 0 and D, K >= 0.\n";
            return;
        }
        mass_[idx1 - 1] = m;
        damping_[idx1 - 1] = d;
        stiffness_[idx1 - 1] = k;
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {static_cast<double>(idx1), m, d, k};
        gains_pub_->publish(msg);
        std::cout << "[tuner] " << label_of(idx1) << " admittance gains -> M=" << m << " D=" << d << " K=" << k << "\n";
    }

    void do_set_pid(int idx1, double p, double v, double i) {
        if (idx1 < 1 || idx1 > static_cast<int>(kN)) { std::cout << "[tuner] invalid motor.\n"; return; }
        if (!std::isfinite(p) || !std::isfinite(v) || !std::isfinite(i) || p < 0 || v < 0 || i < 0) {
            std::cout << "[tuner] PID gains must be finite and >= 0.\n";
            return;
        }
        pid_pos_[idx1 - 1] = p;
        pid_vel_[idx1 - 1] = v;
        pid_vel_int_[idx1 - 1] = i;
        std_msgs::msg::Float64MultiArray m;
        m.data = {static_cast<double>(idx1), p, v, i};
        pid_pub_->publish(m);
        std::cout << "[tuner] " << label_of(idx1) << " PID -> pos=" << p << " vel=" << v << " vel_int=" << i << "\n";
    }

    void do_set_deadband(int idx1, double db) {
        if (idx1 < 1 || idx1 > static_cast<int>(kN)) { std::cout << "[tuner] invalid motor.\n"; return; }
        if (!std::isfinite(db) || db < 0) { std::cout << "[tuner] deadband must be finite and >= 0.\n"; return; }
        deadband_[idx1 - 1] = db;
        std_msgs::msg::Float64MultiArray m;
        m.data = {static_cast<double>(idx1), db};
        deadband_pub_->publish(m);
        std::cout << "[tuner] " << label_of(idx1) << " deadband -> " << db << " Nm\n";
    }

    void do_tare() {
        if (arm_status_.find("DISABLED") != std::string::npos) {
            std::cout << "[tuner] motors are DISABLED - tare would read ~0. Run 'select <motor>' first.\n";
            return;
        }
        tare_pub_->publish(std_msgs::msg::Empty());
        std::cout << "[tuner] tare requested - hold the arm still for ~0.5 s.\n";
    }

    void do_trajectory(double q, double t_move, double t_hold, double t_return) {
        if (selected_ == 0) { std::cout << "[tuner] select a motor first (select <motor>).\n"; return; }
        if (arm_status_.find("DISABLED") != std::string::npos) {
            std::cout << "[tuner] motors are DISABLED. Run 'select <motor>' to enable and choose a motor first!\n";
            return;
        }
        if (arm_status_.find("SWITCHING") != std::string::npos) {
            std::cout << "[tuner] cannot run trajectory: motors are still SWITCHING mode. Please wait for 'ADMITTANCE "
                      << label_of(selected_) << "'!\n";
            return;
        }
        if (arm_status_.find("STARTING") != std::string::npos) {
            std::cout << "[tuner] cannot run trajectory: motors are still STARTING, please wait.\n";
            return;
        }
        if (arm_status_.find("FAULT") != std::string::npos) {
            std::cout << "[tuner] cannot run trajectory: arm is in " << arm_status_ << "!\n";
            return;
        }
        if (selected_ >= 1 && selected_ <= static_cast<int>(kN) && mass_[selected_ - 1] <= 1e-6) {
            std::cout << "[tuner] WARNING: virtual mass for " << label_of(selected_)
                      << " is 0! Set gains with 'admit " << label_of(selected_) << " <M> <D> <K>' first!\n";
            return;
        }
        if (!std::isfinite(q) || !std::isfinite(t_move) || !std::isfinite(t_hold) || !std::isfinite(t_return) ||
            t_move <= 0 || t_hold < 0 || t_return <= 0) {
            std::cout << "[tuner] traj needs finite values, t_move>0, t_hold>=0, t_return>0.\n";
            return;
        }
        std_msgs::msg::Float64MultiArray m;
        m.data = {q, t_move, t_hold, t_return};
        traj_pub_->publish(m);
        std::cout << "[tuner] calibration on " << label_of(selected_) << ": 0 -> " << q << " rad in " << t_move
                  << " s, hold " << t_hold << " s, back in " << t_return << " s\n";
    }

    void do_stop() {
        stop_pub_->publish(std_msgs::msg::Empty());
        std::cout << "[tuner] stop requested\n";
    }

    void do_read() {
        if (params_yaml_.empty()) {
            std::cout << "[tuner] no parameters received yet (is admittance_control_node running?)\n";
            return;
        }
        std::cout << "---- motor parameters ----\n" << params_yaml_ << "--------------------------\n";
        std_msgs::msg::String m;
        m.data = params_yaml_;
        tuner_params_pub_->publish(m);
    }

    void do_save(const std::string& filename) {
        if (params_yaml_.empty()) {
            std::cout << "[tuner] nothing to save (no parameters received yet)\n";
            return;
        }
        std::error_code ec;
        std::filesystem::create_directories(config_dir_, ec);
        const std::string name = filename.empty() ? params_filename_ : filename;
        const std::string path = name.front() == '/' ? name : config_dir_ + "/" + name;
        std::ofstream f(path);
        if (!f) {
            std::cout << "[tuner] cannot write " << path << "\n";
            return;
        }
        f << "# gim_control motor parameters - saved by admittance_tuner_node " << stamp_now() << "\n"
          << params_yaml_;
        std::cout << "[tuner] parameters saved -> " << path << "\n";
    }

    void do_plot(const std::string& csv_override) {
        std::string csv = csv_override.empty() ? last_csv_ : csv_override;
        if (csv.empty()) {
            csv = latest_csv();
            if (csv.empty()) { std::cout << "[tuner] no calibration CSV found in " << log_dir_ << "\n"; return; }
        }
        const std::string png = csv.substr(0, csv.size() - 4) + ".png";
        if (make_figure(csv, png)) std::cout << "[tuner] figure -> " << png << "\n";
    }

    void print_status() const {
        std::cout << "[tuner] selected=" << (selected_ == 0 ? "none" : label_of(selected_))
                  << "  arm_status='" << arm_status_ << "'"
                  << "  calibration='" << calib_status_ << "'"
                  << "  samples=" << samples_.size() << "\n";
    }

    // ---------------- subscriptions ----------------
    void params_callback(const std_msgs::msg::String::SharedPtr msg) { params_yaml_ = msg->data; }

    void params_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < kN * 17) return;
        for (size_t i = 0; i < kN; ++i) {
            const size_t b = i * 17;
            mass_[i] = msg->data[b + 8];
            damping_[i] = msg->data[b + 9];
            stiffness_[i] = msg->data[b + 10];
            deadband_[i] = msg->data[b + 13];
            pid_pos_[i] = msg->data[b + 14];
            pid_vel_[i] = msg->data[b + 15];
            pid_vel_int_[i] = msg->data[b + 16];
        }
    }

    void arm_status_callback(const std_msgs::msg::String::SharedPtr msg) { arm_status_ = msg->data; }

    void calib_status_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string s = msg->data;
        if (s == "RUNNING") {
            if (!recording_) {
                samples_.clear();
                rec_joint_ = (selected_ >= 1 && selected_ <= static_cast<int>(kN)) ? selected_ : 0;
                recording_ = true;
                std::cout << "[tuner] recording ...\n";
            }
        } else if (s == "DONE" && recording_) {
            recording_ = false;
            finish_run();
        } else if (s == "ABORTED") {
            recording_ = false;
            samples_.clear();
        }
        calib_status_ = s;
    }

    void calib_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!recording_ || msg->data.size() < 9) return;
        const int j = static_cast<int>(std::lround(msg->data[1]));
        if (rec_joint_ == 0 && j >= 1 && j <= static_cast<int>(kN)) rec_joint_ = j;  // first joint seen
        if (j != rec_joint_) return;  // ignore other joints when several are active
        Sample s;
        s.t = msg->data[0];
        s.q_meas = msg->data[2];
        s.q_ref = msg->data[3];
        s.q_cmd = msg->data[4];
        s.qd_meas = msg->data[5];
        s.tau_ext = msg->data[6];
        s.tau_meas = msg->data[7];
        s.delta_q = msg->data[8];
        samples_.push_back(s);
    }

    // ---------------- logging ----------------
    void finish_run() {
        if (samples_.empty()) { std::cout << "[tuner] no samples recorded, nothing to save.\n"; return; }
        std::error_code ec;
        std::filesystem::create_directories(log_dir_, ec);
        const int j = rec_joint_ > 0 ? rec_joint_ : 1;
        const std::string base = "admit_" + label_of(j) + "_" + stamp_now();
        const std::string csv = log_dir_ + "/" + base + ".csv";
        const std::string png = log_dir_ + "/" + base + ".png";
        write_csv(csv);
        double max_dq = 0.0, max_tau = 0.0;
        for (const auto& s : samples_) {
            max_dq = std::max(max_dq, std::fabs(s.delta_q));
            max_tau = std::max(max_tau, std::fabs(s.tau_ext));
        }
        std::cout << "[tuner] run finished: " << samples_.size() << " samples, duration " << samples_.back().t
                  << " s, max|delta_q|=" << max_dq << " rad, max|tau_ext|=" << max_tau << " Nm\n"
                  << "[tuner] csv -> " << csv << "\n";
        last_csv_ = csv;
        if (make_figure(csv, png)) std::cout << "[tuner] figure -> " << png << "\n";
    }

    void write_csv(const std::string& path) const {
        std::ofstream f(path);
        if (!f) { std::cout << "[tuner] cannot write " << path << "\n"; return; }
        f << "timestamp,joint,q_meas,q_ref,q_cmd,qd_meas,tau_ext,tau_meas,delta_q\n";
        f << std::setprecision(9);
        for (const auto& s : samples_) {
            f << s.t << ',' << rec_joint_ << ',' << s.q_meas << ',' << s.q_ref << ',' << s.q_cmd << ',' << s.qd_meas
              << ',' << s.tau_ext << ',' << s.tau_meas << ',' << s.delta_q << '\n';
        }
    }

    bool make_figure(const std::string& csv, const std::string& png) const {
        if (!std::filesystem::exists(plot_script_)) {
            std::cout << "[tuner] plot script not found: " << plot_script_ << " (csv is still saved)\n";
            return false;
        }
        std::ostringstream cmd;
        cmd << "python3 \"" << plot_script_ << "\" \"" << csv << "\" \"" << png << "\"";
        const int rc = std::system(cmd.str().c_str());
        if (rc != 0) {
            std::cout << "[tuner] plotting failed (matplotlib installed?). csv kept at " << csv << "\n";
            return false;
        }
        return true;
    }

    std::string latest_csv() const {
        std::error_code ec;
        if (!std::filesystem::exists(log_dir_, ec)) return "";
        std::string best;
        std::filesystem::file_time_type best_t{};
        for (const auto& entry : std::filesystem::directory_iterator(log_dir_, ec)) {
            if (entry.path().extension() != ".csv") continue;
            const auto t = entry.last_write_time(ec);
            if (best.empty() || t > best_t) { best = entry.path().string(); best_t = t; }
        }
        return best;
    }

    // ---------------- members ----------------
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr select_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gains_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr deadband_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr tare_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tuner_params_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr params_data_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calib_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calib_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr select_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gains_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr deadband_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tare_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traj_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr read_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plot_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

    std::array<std::string, kN> labels_;
    std::string config_dir_, log_dir_, params_filename_, plot_script_;
    std::string params_yaml_, arm_status_ = "unknown", calib_status_ = "IDLE";
    std::string last_csv_;
    int selected_ = 0;
    int rec_joint_ = 0;
    bool recording_ = false;
    std::vector<Sample> samples_;
    std::array<double, kN> mass_{0.15, 0.15, 0.15}, damping_{4.0, 4.0, 4.0}, stiffness_{25.0, 25.0, 25.0};
    std::array<double, kN> deadband_{0.05, 0.05, 0.05};
    std::array<double, kN> pid_pos_{4.0, 0.5, 4.0}, pid_vel_{1.2, 0.1, 1.5}, pid_vel_int_{0.1, 0.0, 0.1};

    std::atomic<bool> running_{false};
    std::thread input_thread_;
    std::mutex cmd_mutex_;
    std::queue<std::string> cmd_queue_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<AdmittanceTunerNode>();
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("admittance_tuner_node"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
