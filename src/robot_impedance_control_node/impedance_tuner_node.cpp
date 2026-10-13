// Interactive tuner / calibration front-end for the 3-DOF GIM arm.
//
// This node does NOT touch the CAN bus. It talks to `impedance_control_node` over topics and
// gives you a continuous command line in the terminal to tune the motors live:
//
//   gim> select shoulder          # put one motor in IMPEDANCE mode (others hold POSITION @ 0)
//   gim> imp shoulder 8 0.5       # set impedance gains   [kp kd]
//   gim> kp shoulder 8            # change one impedance gain
//   gim> pid shoulder 20 2 0.2    # set ODrive PID gains  [pos_gain vel_gain vel_integrator_gain]
//   gim> traj 0.6 2 1 2           # 0 -> 0.6 rad in 2 s, hold 1 s, back to 0 in 2 s
//   gim> read                     # print the live motor parameters
//   gim> save                     # write parameters to config/motor_params.yaml
//   gim> plot                     # re-plot the last calibration run
//   gim> status                   # show node/selection state
//   gim> quit
//
// After every calibration trajectory finishes, the recorded run is written to
//   config/calibration_logs/calib_<motor>_<timestamp>.csv   (timestamp, actual/target position and
//   velocity, torque, position/velocity error)  and the matching .png figure is generated.
//
// The same functions are also exposed as topics so another node / a script can drive the tuner:
//   sub  /tuner/select_motor            std_msgs/Int32
//   sub  /tuner/impedance_gains         std_msgs/Float64MultiArray [joint_index, kp, kd]
//   sub  /tuner/pid_gains               std_msgs/Float64MultiArray [joint_index, p, v, i]
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
    double q_act = 0.0;
    double q_tgt = 0.0;
    double qd_act = 0.0;
    double qd_tgt = 0.0;
    double tau = 0.0;
    double e_p = 0.0;
    double e_v = 0.0;
};

}  // namespace

class ImpedanceTunerNode : public rclcpp::Node {
public:
    ImpedanceTunerNode() : Node("impedance_tuner_node") {
        declare_parameter<std::vector<std::string>>("motor_labels", {"base", "shoulder", "elbow"});
        declare_parameter<std::string>("config_dir", "/home/tue/gim_control/config");
        declare_parameter<std::string>("log_dir", "");   // empty -> <config_dir>/calibration_logs
        declare_parameter<std::string>("params_filename", "motor_params.yaml");
        declare_parameter<std::string>("plot_script", "/home/tue/gim_control/scripts/plot_calibration.py");

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
        imp_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/impedance_gains", 10);
        pid_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/pid_gains", 10);
        traj_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gim/calibration_trajectory", 10);
        stop_pub_ = create_publisher<std_msgs::msg::Empty>("/gim/stop_calibration", 10);

        // ---- from the hardware node ----
        auto latched = rclcpp::QoS(1).transient_local().reliable();
        params_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/motor_params", latched, std::bind(&ImpedanceTunerNode::params_callback, this, std::placeholders::_1));
        params_data_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/motor_params_data", latched,
            std::bind(&ImpedanceTunerNode::params_data_callback, this, std::placeholders::_1));
        arm_status_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/status", latched, std::bind(&ImpedanceTunerNode::arm_status_callback, this, std::placeholders::_1));
        calib_status_sub_ = create_subscription<std_msgs::msg::String>(
            "/gim/calibration_status", latched,
            std::bind(&ImpedanceTunerNode::calib_status_callback, this, std::placeholders::_1));
        calib_data_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gim/calibration_data", 100,
            std::bind(&ImpedanceTunerNode::calib_data_callback, this, std::placeholders::_1));

        // ---- tuner topics (drive the tuner from other nodes/scripts) ----
        select_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/tuner/select_motor", 10, [this](const std_msgs::msg::Int32::SharedPtr m) { do_select(m->data); });
        imp_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuner/impedance_gains", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                if (m->data.size() == 3) do_set_impedance(static_cast<int>(std::lround(m->data[0])), m->data[1], m->data[2]);
            });
        pid_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tuner/pid_gains", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
                if (m->data.size() == 4)
                    do_set_pid(static_cast<int>(std::lround(m->data[0])), m->data[1], m->data[2], m->data[3]);
            });
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
        input_thread_ = std::thread(&ImpedanceTunerNode::input_loop, this);
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
            std::cout << "gim> " << std::flush;
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
        std::cout << "\n================ gim_control impedance tuner ================\n"
                  << "3 motors: 1=base  2=shoulder  3=elbow\n"
                  << "type 'help' for the command list.\n"
                  << "============================================================\n\n";
    }

    void print_help() const {
        std::cout <<
            "select <motor>            enable & put motor in IMPEDANCE mode, others hold POSITION @ 0 (0=disable)\n"
            "disable / off             disable all motors (AXIS_STATE_IDLE / unpowered)\n"
            "imp <motor> <kp> <kd>     set impedance gains for a motor\n"
            "kp <motor> <value>        set impedance stiffness gain only\n"
            "kd <motor> <value>        set impedance damping gain only\n"
            "pid <motor> <p> <v> <i>   set ODrive PID gains (pos_gain, vel_gain, vel_integrator_gain)\n"
            "traj <q> <t_move> <t_hold> [t_return]   run 0 -> q -> 0 calibration trajectory\n"
            "stop                      abort the running trajectory\n"
            "read                      print the live motor parameters\n"
            "save [filename]           write parameters to config/ (default: motor_params.yaml)\n"
            "plot                      re-plot the last calibration run\n"
            "status                    show connection / selection state\n"
            "help                      this text\n"
            "quit                      exit\n";
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
        } else if (cmd == "disable" || cmd == "off" || cmd == "idle") {
            do_select(0);
        } else if (cmd == "select") {
            std::string motor;
            ss >> motor;
            do_select(parse_motor(motor));
        } else if (cmd == "read") {
            do_read();
        } else if (cmd == "imp" || cmd == "impedance" || cmd == "gain") {
            std::string motor; double kp = 0, kd = 0;
            if (!(ss >> motor >> kp >> kd)) { std::cout << "[tuner] usage: imp <motor> <kp> <kd>\n"; return; }
            do_set_impedance(parse_motor(motor), kp, kd);
        } else if (cmd == "kp") {
            std::string motor; double v = 0;
            if (!(ss >> motor >> v)) { std::cout << "[tuner] usage: kp <motor> <value>\n"; return; }
            const int idx = parse_motor(motor);
            if (idx >= 1) do_set_impedance(idx, v, kd_[idx - 1]);
        } else if (cmd == "kd") {
            std::string motor; double v = 0;
            if (!(ss >> motor >> v)) { std::cout << "[tuner] usage: kd <motor> <value>\n"; return; }
            const int idx = parse_motor(motor);
            if (idx >= 1) do_set_impedance(idx, kp_[idx - 1], v);
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

    int parse_motor(const std::string& token) const {
        const std::string t = lower(token);
        if (t == "none" || t == "off" || t == "all") return 0;
        for (size_t i = 0; i < kN; ++i) {
            if (t == labels_[i]) return static_cast<int>(i) + 1;
        }
        try {
            const int v = std::stoi(t);
            if (v >= 0 && v <= static_cast<int>(kN)) return v;
        } catch (...) {
        }
        return -1;
    }

    std::string label_of(int idx1) const {
        if (idx1 >= 1 && idx1 <= static_cast<int>(kN)) return labels_[idx1 - 1];
        return "?";
    }

    // ---------------- operations ----------------
    void do_select(int sel) {
        if (sel < 0 || sel > static_cast<int>(kN)) {
            std::cout << "[tuner] select expects 0 (none) .. 3\n";
            return;
        }
        std_msgs::msg::Int32 m;
        m.data = sel;
        select_pub_->publish(m);
        selected_ = sel;
        if (sel == 0) {
            std::cout << "[tuner] request: DISABLE all motors (idle / unpowered)\n";
        } else {
            std::cout << "[tuner] request: enable & IMPEDANCE on " << label_of(sel) << " (others hold POSITION @ 0)\n";
            std::cout << "[tuner] note: mode switch takes ~0.5s. Wait until status is 'IMPEDANCE "
                      << label_of(sel) << "' before running 'traj'.\n";
        }
    }

    void do_set_impedance(int idx1, double kp, double kd) {
        if (idx1 < 1 || idx1 > static_cast<int>(kN)) { std::cout << "[tuner] invalid motor.\n"; return; }
        if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0 || kd < 0) {
            std::cout << "[tuner] gains must be finite and >= 0.\n";
            return;
        }
        kp_[idx1 - 1] = kp;
        kd_[idx1 - 1] = kd;
        std_msgs::msg::Float64MultiArray m;
        m.data = {static_cast<double>(idx1), kp, kd};
        imp_pub_->publish(m);
        std::cout << "[tuner] " << label_of(idx1) << " impedance gains -> kp=" << kp << " kd=" << kd << "\n";
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

    void do_trajectory(double q, double t_move, double t_hold, double t_return) {
        if (selected_ == 0) { std::cout << "[tuner] select a motor first (select <motor>).\n"; return; }
        if (arm_status_.find("DISABLED") != std::string::npos) {
            std::cout << "[tuner] motors are DISABLED. Run 'select <motor>' to enable and choose a motor first!\n";
            return;
        }
        if (arm_status_.find("SWITCHING") != std::string::npos) {
            std::cout << "[tuner] cannot run trajectory: motors are still SWITCHING mode. Please wait for 'IMPEDANCE "
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
        if (selected_ >= 1 && selected_ <= static_cast<int>(kN) && kp_[selected_ - 1] <= 1e-6) {
            std::cout << "[tuner] WARNING: kp for " << label_of(selected_)
                      << " is 0! The motor will not generate torque. Set gains with 'imp "
                      << label_of(selected_) << " <kp> <kd>' first!\n";
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
            std::cout << "[tuner] no parameters received yet (is impedance_control_node running?)\n";
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
        f << "# gim_control motor parameters - saved by impedance_tuner_node " << stamp_now() << "\n"
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
        if (msg->data.size() < kN * 11) return;
        for (size_t i = 0; i < kN; ++i) {
            const size_t b = i * 11;
            kp_[i] = msg->data[b + 6];
            kd_[i] = msg->data[b + 7];
            pid_pos_[i] = msg->data[b + 8];
            pid_vel_[i] = msg->data[b + 9];
            pid_vel_int_[i] = msg->data[b + 10];
        }
    }

    void arm_status_callback(const std_msgs::msg::String::SharedPtr msg) { arm_status_ = msg->data; }

    void calib_status_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string s = msg->data;
        if (s == "RUNNING") {
            if (!recording_) {
                samples_.clear();
                rec_joint_ = selected_ > 0 ? selected_ : 1;
                recording_ = true;
                std::cout << "[tuner] recording " << label_of(rec_joint_) << " ...\n";
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
        if (j >= 1 && j <= static_cast<int>(kN)) rec_joint_ = j;  // authoritative joint index
        Sample s;
        s.t = msg->data[0];
        s.q_act = msg->data[2];
        s.q_tgt = msg->data[3];
        s.qd_act = msg->data[4];
        s.qd_tgt = msg->data[5];
        s.tau = msg->data[6];
        s.e_p = msg->data[7];
        s.e_v = msg->data[8];
        samples_.push_back(s);
    }

    // ---------------- logging ----------------
    void finish_run() {
        if (samples_.empty()) { std::cout << "[tuner] no samples recorded, nothing to save.\n"; return; }
        std::error_code ec;
        std::filesystem::create_directories(log_dir_, ec);
        const std::string base = "calib_" + label_of(rec_joint_) + "_" + stamp_now();
        const std::string csv = log_dir_ + "/" + base + ".csv";
        const std::string png = log_dir_ + "/" + base + ".png";
        write_csv(csv);
        double max_e = 0.0, max_tau = 0.0;
        for (const auto& s : samples_) {
            max_e = std::max(max_e, std::fabs(s.e_p));
            max_tau = std::max(max_tau, std::fabs(s.tau));
        }
        std::cout << "[tuner] run finished: " << samples_.size() << " samples, duration " << samples_.back().t
                  << " s, max|pos_err|=" << max_e << " rad, max|torque|=" << max_tau << " Nm\n"
                  << "[tuner] csv -> " << csv << "\n";
        last_csv_ = csv;
        if (make_figure(csv, png)) std::cout << "[tuner] figure -> " << png << "\n";
    }

    void write_csv(const std::string& path) const {
        std::ofstream f(path);
        if (!f) { std::cout << "[tuner] cannot write " << path << "\n"; return; }
        f << "timestamp,actual_position,target_position,actual_velocity,target_velocity,torque,position_error,"
             "velocity_error\n";
        f << std::setprecision(9);
        for (const auto& s : samples_) {
            f << s.t << ',' << s.q_act << ',' << s.q_tgt << ',' << s.qd_act << ',' << s.qd_tgt << ',' << s.tau << ','
              << s.e_p << ',' << s.e_v << '\n';
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tuner_params_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr params_data_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calib_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr calib_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr select_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imp_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_cmd_sub_;
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
    int rec_joint_ = 1;
    bool recording_ = false;
    std::vector<Sample> samples_;
    std::array<double, kN> kp_{5.0, 10.0, 5.0}, kd_{0.2, 0.4, 0.2};
    std::array<double, kN> pid_pos_{20.0, 20.0, 20.0}, pid_vel_{2.0, 2.0, 2.0}, pid_vel_int_{0.2, 0.2, 0.2};

    std::atomic<bool> running_{false};
    std::thread input_thread_;
    std::mutex cmd_mutex_;
    std::queue<std::string> cmd_queue_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<ImpedanceTunerNode>();
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("impedance_tuner_node"), "%s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
