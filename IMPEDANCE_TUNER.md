# Impedance Tuner — Introduction & User Guide

The **impedance tuner** is a two-node tool for bringing up and calibrating the three
GIM6010 motors of the upper-arm rehabilitation robot in **impedance mode**.

| Node | Executable | Role |
|---|---|---|
| Hardware node | `ros2 run gim_control impedance_control_node` | Real-time CAN control. Holds all motors in **POSITION @ 0** on start and moves **one** selected motor to **IMPEDANCE** mode for calibration. |
| Tuner node | `ros2 run gim_control impedance_tuner_node` | Interactive terminal front-end. Selects a motor, tunes PID / impedance gains live, runs the calibration trajectory, saves parameters, and writes the CSV + figure. |

The tuner never touches CAN directly; it talks to the hardware node over topics. That keeps
the real-time loop isolated from terminal input and file I/O.

```
                 /gim/select_motor, /gim/impedance_gains, /gim/pid_gains,
                 /gim/calibration_trajectory, /gim/stop_calibration
 impedance_tuner_node ────────────────────────────────────────────────► impedance_control_node ──► can0
        ▲        ◄────────────────────────────────────────────────────────────────────────────────────
        │        /gim/joint_states, /gim/status, /gim/calibration_status, /gim/calibration_data,
        │        /gim/motor_params, /gim/motor_params_data
        │
   terminal (stdin)                 + /tuner/* command topics           config/motor_params.yaml
                                                                        config/calibration_logs/*.csv|*.png
```

---

## 1. Quick start

```bash
# 0) workspace + CAN up (once)
cd /home/tue/gim_control
colcon build --packages-select gim_control
source install/setup.bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 1) terminal 1 - real hardware (keep it running)
ros2 run gim_control impedance_control_node

# 2) terminal 2 - tuner (interactive)
ros2 run gim_control impedance_tuner_node
```

On start the hardware node:

1. reads the encoders and takes the **current pose as joint 0**,
2. enables **all three motors in POSITION mode** holding `0`,
3. waits for a selection.

> [!WARNING]
> Place the arm in a safe, supported pose before starting. There is **no gravity
> compensation** — a joint put into impedance mode becomes free and can sag. Keep a hand
> on the e-stop and start with small gains and small targets.

---

## 2. Terminal commands

Once the tuner is running you get a `gim>` prompt. All commands are live.

| Command | Description |
|---|---|
| `select <motor>` | Put a motor in **IMPEDANCE** mode; the other two hold **POSITION @ 0**. `<motor>` = `0`\|`1`\|`2`\|`3` or `none`\|`base`\|`shoulder`\|`elbow`. Only one motor is ever in impedance mode. |
| `imp <motor> <kp> <kd>` | Set the impedance gains for a motor (Nm/rad, Nm·s/rad). |
| `kp <motor> <value>` | Change only the impedance stiffness gain. |
| `kd <motor> <value>` | Change only the impedance damping gain. |
| `pid <motor> <p> <v> <i>` | Set the ODrive PID gains: `pos_gain`, `vel_gain`, `vel_integrator_gain`. Pushed to the motor immediately. |
| `traj <q> <t_move> <t_hold> [t_return]` | Run the calibration profile on the selected motor: `0 → q` over `t_move` s, hold `q` for `t_hold` s, `q → 0` over `t_return` s (defaults to `t_move`). |
| `stop` | Abort the running trajectory; the motor returns to holding `0`. |
| `read` | Print the live parameters of all three motors. |
| `save [filename]` | Write the parameters to `config/` (default `motor_params.yaml`). |
| `plot` | Re-generate the figure for the last calibration run. |
| `status` | Show selection + node state. |
| `help` | Command list. |
| `quit` | Exit. |

Example session:

```text
gim> select shoulder
gim> pid shoulder 20 2 0.2
gim> imp shoulder 8 0.5
gim> traj 0.6 2 1 2
gim> read
gim> save
gim> quit
```

## 3. Calibration workflow

1. **Select** the motor → it goes to impedance mode, the other two stay at position 0.
2. **Tune** `kp` / `kd` (and `pid` if you want to change how the held joints settle).
3. **Run** `traj <q> <t_move> <t_hold> [t_return]`. The hardware node streams one sample per
   control tick on `/gim/calibration_data` while the profile runs.
4. When the profile finishes the tuner writes:
   * `config/calibration_logs/calib_<motor>_<timestamp>.csv`
   * `config/calibration_logs/calib_<motor>_<timestamp>.png`
5. Tuned parameters are **persisted automatically** to `config/motor_params.yaml` (every `kp`/`kd`/
   `pid` change re-writes the file). `save` in the tuner does the same thing on demand, and the
   next start-up loads the file.

The CSV columns are:

```text
timestamp, actual_position, target_position, actual_velocity,
target_velocity, torque, position_error, velocity_error
```

`timestamp` is seconds from the start of the profile. The figure has four stacked panels
(position, velocity, torque, error) sharing the time axis.

---

## 4. Topics

### Hardware node (`impedance_control_node`)

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/gim/select_motor` | `std_msgs/Int32` | sub | `0`=none, `1`=base, `2`=shoulder, `3`=elbow. Starts an impedance switch (takes a few seconds — the ODrive mode change blocks). |
| `/gim/impedance_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, kp, kd]` |
| `/gim/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, pos_gain, vel_gain, vel_integrator_gain]` |
| `/gim/calibration_trajectory` | `std_msgs/Float64MultiArray` | sub | `[q_target, t_move, t_hold, (t_return)]` |
| `/gim/stop_calibration` | `std_msgs/Empty` | sub | Abort the running profile. |
| `/gim/joint_states` | `sensor_msgs/JointState` | pub | Measured position/velocity (rad, rad/s) + commanded torque (Nm). |
| `/gim/motor_state` | `std_msgs/Float64MultiArray` | pub | Realtime state at the control rate: `[q0 q1 q2 \| qd0 qd1 qd2 \| tau_cmd0 tau_cmd1 tau_cmd2 \| tau_meas0 tau_meas1 tau_meas2]` (rad, rad/s, Nm). `tau_meas` is the actual motor torque read back over CAN. |
| `/gim/status` | `std_msgs/String` | pub (latched) | `STARTING` \| `HOLD (all POSITION @ 0)` \| `SWITCHING` \| `IMPEDANCE <motor>`. |
| `/gim/calibration_status` | `std_msgs/String` | pub (latched) | `IDLE` \| `RUNNING` \| `DONE` \| `ABORTED`. |
| `/gim/calibration_data` | `std_msgs/Float64MultiArray` | pub | `[t, joint_index, q_actual, q_target, qd_actual, qd_target, torque, pos_error, vel_error]` (only while running). |
| `/gim/motor_params` | `std_msgs/String` | pub (latched) | Human-readable parameter dump (YAML). |
| `/gim/motor_params_data` | `std_msgs/Float64MultiArray` | pub (latched) | Numeric dump: 11 values per motor `[node_id, gear_ratio, invert, torque_sign, torque_gear_ratio, torque_limit, kp, kd, pos_gain, vel_gain, vel_integrator_gain]`. |

Command-line equivalents:

```bash
ros2 topic pub --once /gim/select_motor      std_msgs/msg/Int32             "{data: 2}"
ros2 topic pub --once /gim/impedance_gains   std_msgs/msg/Float64MultiArray "{data: [2, 8.0, 0.5]}"
ros2 topic pub --once /gim/pid_gains         std_msgs/msg/Float64MultiArray "{data: [2, 20.0, 2.0, 0.2]}"
ros2 topic pub --once /gim/calibration_trajectory std_msgs/msg/Float64MultiArray "{data: [0.6, 2.0, 1.0, 2.0]}"
ros2 topic echo /gim/joint_states
ros2 topic echo /gim/motor_state    # 12 values: pos, vel, commanded torque, measured torque
```

### Tuner node (`impedance_tuner_node`)

The same actions are exposed as topics so a script or another node can drive the tuner:

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/tuner/select_motor` | `std_msgs/Int32` | sub | Same as the `select` command. |
| `/tuner/impedance_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, kp, kd]` |
| `/tuner/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, p, v, i]` |
| `/tuner/calibration_trajectory` | `std_msgs/Float64MultiArray` | sub | `[q, t_move, t_hold, (t_return)]` |
| `/tuner/read_params` | `std_msgs/Empty` | sub | Print the parameters. |
| `/tuner/save_params` | `std_msgs/String` | sub | Save to `config/<data>` (empty = `motor_params.yaml`). |
| `/tuner/plot` | `std_msgs/Empty` | sub | Re-plot the last run. |
| `/tuner/stop` | `std_msgs/Empty` | sub | Abort the profile. |
| `/tuner/command` | `std_msgs/String` | sub | Any terminal command line, e.g. `"{data: 'select elbow'}"`. |
| `/tuner/motor_params` | `std_msgs/String` | pub (latched) | Last parameters read. |

---

## 5. Parameters

### `impedance_control_node`

| Parameter | Default | Description |
|---|---|---|
| `node_ids` | `[0, 1, 2]` | CAN node id per joint (base, shoulder, elbow), matching `urdf/gim_arm.urdf`. |
| `motor_labels` | `[base, shoulder, elbow]` | Names used on topics and log files. |
| `gear_ratio` | `[8, 64, 8]` | Rotor turns per joint turn. |
| `invert_direction` | `[true, false, true]` | Flips the sign of position/velocity. |
| `torque_sign` | `[-1, 1, -1]` | Sign applied to the motor torque command. |
| `torque_gear_ratio` | `[1, 8, 1]` | Motor torque = joint torque / ratio. |
| `torque_limit` | `[5, 40, 5]` | Joint torque saturation (Nm). |
| `kp` / `kd` | `[5,10,5]` / `[0.2,0.4,0.2]` | Impedance gains. |
| `pid_pos_gain` / `pid_vel_gain` / `pid_vel_integrator_gain` | `[20,20,20]` / `[2,2,2]` / `[0.2,0.2,0.2]` | ODrive PID gains for POSITION mode. |
| `control_rate_hz` | `100` | Control loop rate. |
| `encoder_timeout` | `0.2` | Encoder watchdog (s); a fault disables all motors. |
| `hold_vel_limit` | `4.0` | POSITION-mode trajectory velocity limit (turns/s). |
| `params_file` | `.../config/motor_params.yaml` | The parameter file: loaded at start, written on change. |
| `load_params_on_start` | `true` | Set `false` to ignore the file and use built-in defaults. |
| `auto_save_params` | `true` | Re-write the parameter file whenever a parameter is tuned. |
| `startup_timeout` | `10.0` | Seconds to wait for **all** encoders before giving up (0 = wait forever). |

### `impedance_tuner_node`

| Parameter | Default | Description |
|---|---|---|
| `config_dir` | `/home/tue/gim_control/config` | Where `save` writes the YAML. |
| `log_dir` | `<config_dir>/calibration_logs` | Where CSV + PNG are written. |
| `params_filename` | `motor_params.yaml` | Default `save` filename. |
| `plot_script` | `.../scripts/plot_calibration.py` | Matplotlib helper used for the figure. |
| `motor_labels` | `[base, shoulder, elbow]` | Must match the hardware node. |

---

## 6. Parameter file (source of truth)

`config/motor_params.yaml` holds **all** motor parameters. The hardware node:

* **reads** it at start-up (`load_params_on_start`, `params_file`) — the values in the file
  override the built-in defaults; and
* **writes** it whenever a parameter is tuned live (`auto_save_params`), and whenever the tuner
  runs `save`.

That means the node always runs from the file, and every tuned value is stored there — nothing
lives only in the running process. The shipped file (see `config/motor_params.yaml`) looks like:

```yaml
motors:
  base:
    node_id: 0              # CAN node id (base=0, shoulder=1, elbow=2)
    joint_name: base_joint
    gear_ratio: 8           # rotor turns per joint turn
    invert_direction: true  # flips position/velocity sign
    torque_sign: -1         # sign of the motor torque command
    torque_gear_ratio: 1    # motor torque = joint torque / this
    torque_limit: 5         # joint torque saturation [Nm]
    joint_lower: -0.7121    # joint limits [rad]
    joint_upper: 1.0226
    pid:                    # ODrive POSITION-mode gains
      pos_gain: 20
      vel_gain: 2
      vel_integrator_gain: 0.2
    impedance:              # impedance gains
      kp: 5                 # [Nm/rad]
      kd: 0.2               # [Nm*s/rad]
  shoulder: { ... }
  elbow:    { ... }
```

> [!NOTE]
> `auto_save_params` re-writes the file (and drops any comments you added) every time a parameter
> changes. Edit the file while the node is **stopped**, or set `auto_save_params:=false` to keep
> your comments and save manually from the tuner.

---

## 7. Safety behaviour

* **Start-up waits for every motor:** the node does not energise anything until all three motors
  answer on CAN. If a motor is silent it reports which one (e.g. `elbow (node 2)`) and, after
  `startup_timeout`, leaves every motor DISABLED instead of enabling and faulting.
* **Start-up:** all motors are first put to `AXIS_STATE_IDLE`, then enabled in POSITION @ 0.
* **One impedance motor at a time:** selecting a new motor puts the previous one back to
  POSITION @ 0 before the new one is enabled. During the mode change (a few seconds at 100 %
  busy) no commands are sent; the joints keep holding their last position command.
* **Watchdog:** if a motor stops answering on CAN for `encoder_timeout` seconds, or a joint
  runs far outside its limits, **all motors are disabled** and `/gim/status` reports the fault.
* **Shutdown:** `Ctrl-C` / `SIGTERM` disables every motor before the process exits.
* The calibration reference always starts at 0 and the target is clamped to the URDF joint
  limits.

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No parameter file ... found` | Normal on the first run — built-in defaults are used. |
| `Cannot open CAN interface can0` | Bring `can0` up (`sudo ip link set can0 up type can ...`). |
| `Error sending ...` lines with no motors connected | Expected: the CAN bus never gets an ACK. Ignore during bench testing. |
| `FAULT: no encoder reply from node N` (or `No encoder reply from: <motor> (node N)`) | Motor unpowered / not on the bus, or its **CAN node id differs** from `node_id` in `config/motor_params.yaml`. Find the real ids with `candump can0` while sending requests. |
| Startup reports one motor missing but the others are fine | That motor's CAN id is different from `node_id` in `config/motor_params.yaml`. Find the real ids with `candump can0`, then either fix `node_id` or re-assign the motor's CAN id. |
| Stuck in `SWITCHING` | Each ODrive mode change blocks for a few seconds; wait. |
| Figure not produced | `matplotlib` missing (`pip install matplotlib`) — the CSV is still saved. |
| Parameters not loaded at start | Check `params_file` path (`load_params_on_start` is on by default). |
| My comments in `motor_params.yaml` disappeared | `auto_save_params` rewrote the file; stop the node to edit, or set `auto_save_params:=false`. |
