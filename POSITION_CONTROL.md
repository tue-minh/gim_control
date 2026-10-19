# Position Control — Introduction & User Guide

The **position control** package brings up and calibrates the three GIM6010 motors of the
upper-arm rehabilitation robot in plain **POSITION** mode. It runs the ODrive's built-in
position PID — there is no admittance or impedance law and no torque command:

$$q_{cmd} = q_{ref}(t) \qquad \text{ODrive position PID} \;\longrightarrow\; \tau_{motor}$$

| Node | Executable | Role |
|---|---|---|
| Hardware node | `ros2 run gim_control position_control_node` | Real-time CAN control. Keeps **every** joint in **POSITION** mode, holds `q_ref = 0`, and runs the point-to-point **calibration trajectory** on the selected joint(s). |
| Calibration node | `ros2 run gim_control position_tuner_node` | Interactive terminal front-end. Selects a motor, tunes the ODrive PID / velocity limits live, runs the calibration trajectory, saves parameters, and writes the CSV + figure. |
| Run node | `ros2 run gim_control position_trajectory_node` | Loads and executes 3-DOF trajectories from `src/robot_trajectory/*.csv` (e.g. `gim_arm_circle_traj.csv`) across all joints, records the run, and writes the CSV + figure. |

The tuner never touches CAN directly; it talks to the hardware node over topics. That keeps
the real-time loop isolated from terminal input and file I/O.

```
                 /gim/select_motor, /gim/pid_gains, /gim/position_limits,
                 /gim/calibration_trajectory, /gim/stop_calibration
position_tuner_node ──────────────────────────────────────────────────► position_control_node ──► can0
        ▲        ◄──────────────────────────────────────────────────────────────────────────────────
        │        /gim/joint_states, /gim/status, /gim/calibration_status, /gim/calibration_data,
        │        /gim/motor_params, /gim/motor_params_data
        │
   terminal (stdin)                 + /tuner/* command topics           config/motor_params.yaml
                                                                        config/calibration_logs/*.csv|*.png

                 /gim/enable, /gim/start_trajectory, /gim/stop_trajectory, ...
position_trajectory_node ──────────────────────────────────────────────► position_trajectory_node ──► can0
                                                                        config/calibration_logs/position_trajectory_logs/
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
ros2 run gim_control position_control_node

# 2) terminal 2 - calibration front-end (interactive)
ros2 run gim_control position_tuner_node

# --- or run a stored 3-DOF trajectory instead (terminal 1) ---
ros2 run gim_control position_trajectory_node
```

On start the hardware node:

1. reads the encoders and takes the **current pose as joint 0**,
2. leaves every motor in `AXIS_STATE_IDLE` (unpowered) until a selection arrives,
3. waits for a selection.

> [!WARNING]
> Place the arm in a safe, supported pose before starting. Joint 0 is whatever pose the arm
> has when the node starts. Keep a hand on the e-stop and use small targets while tuning.

---

## 2. Terminal commands (`position_tuner_node`)

Once the tuner is running you get a `gim_pos>` prompt. All commands are live.

| Command | Description |
|---|---|
| `select <motor>` | Make a motor **active** (follow the calibration reference); the others hold **POSITION @ 0**. `<motor>` = `0`\|`1`\|`2`\|`3` or `none`\|`base`\|`shoulder`\|`elbow`, or `-1`/`all` for all three. |
| `disable` / `off` | Disable all motors (`AXIS_STATE_IDLE` / unpowered). |
| `pid <motor> <p> <v> <i>` | Set the ODrive PID gains: `pos_gain`, `vel_gain`, `vel_integrator_gain`. Pushed to the motor immediately. |
| `limit <motor> <v> <a> [d]` | Set the trajectory velocity / acceleration / deceleration limits (motor-rotor turns/s, turns/s²). `d` defaults to `a`. |
| `traj <q> <t_move> <t_hold> [t_return]` | Run the calibration profile on the active joint(s): `0 → q` over `t_move` s, hold `q` for `t_hold` s, `q → 0` over `t_return` s (defaults to `t_move`). |
| `stop` | Abort the running trajectory; the joint returns to holding `0`. |
| `read` | Print the live parameters of all three motors. |
| `save [filename]` | Write the parameters to `config/` (default `motor_params.yaml`). |
| `plot` | Re-generate the figure for the last calibration run. |
| `status` | Show selection + node state. |
| `help` | Command list. |
| `quit` | Exit. |

Example session:

```text
gim_pos> select shoulder
gim_pos> pid shoulder 0.5 0.1 0
gim_pos> limit shoulder 4 20 20
gim_pos> traj 0.6 2 1 2
gim_pos> read
gim_pos> save
gim_pos> quit
```

## 3. Calibration workflow

1. **Select** the motor(s) → they follow the calibration reference, the others stay at position 0.
2. **Tune** the ODrive PID and, if needed, the velocity/acceleration limits.
3. **Run** `traj <q> <t_move> <t_hold> [t_return]`. The hardware node streams one sample per
   control tick on `/gim/calibration_data` while the profile runs.
4. When the profile finishes the tuner writes:
   * `config/calibration_logs/pos_<motor>_<timestamp>.csv`
   * `config/calibration_logs/pos_<motor>_<timestamp>.png`
5. Tuned parameters are **persisted automatically** to `config/motor_params.yaml` (every
   `pid`/`limit` change re-writes the file). `save` in the tuner does the same thing on demand,
   and the next start-up loads the file.

The CSV columns are:

```text
timestamp, actual_position, target_position, actual_velocity, target_velocity,
torque, position_error, velocity_error
```

`timestamp` is seconds from the start of the profile. The figure has four stacked panels
(position, velocity, torque, error) sharing the time axis (`scripts/plot_calibration.py`).

---

## 4. Running a 3-DOF trajectory (`position_trajectory_node`)

The run node loads `src/robot_trajectory/gim_arm_circle_traj.csv` by default. It expects the
columns `t`, `q_base`, `q_shoulder`, `q_elbow` and optionally `qd_base`, `qd_shoulder`,
`qd_elbow`.

Terminal commands:

| Command | Description |
|---|---|
| `enable` / `on` | Enable all motors in position mode (hold joint 0). |
| `disable` / `off` | Disable all motors (idle / unpowered). |
| `start` / `run` | Start the loaded trajectory once and record the run. |
| `pause` / `resume` | Pause / resume the trajectory (holds the current pose). |
| `stop` | Stop the trajectory; the arm holds its current pose and the run is saved. |
| `plot` | Re-generate the figure for the latest run. |
| `safety_stop` / `estop` | Emergency stop (immediate motor cut). |
| `reset` | Clear the fault latch after a safety stop. |
| `params` / `echo` | Print motor parameters. |
| `reload [path]` | Reload `config/motor_params.yaml` live. |
| `load <csv_path>` | Load a different trajectory CSV. |
| `loop [on\|off]` | Toggle looping (default: off, runs once). |
| `status` | Print status, trajectory file, log paths and joint positions. |
| `quit` | Disable motors and exit. |

If the current pose differs from the first trajectory point by more than `0.05 rad`, the node
first runs a smooth S-curve **approach** to the start pose. On completion it holds the last
trajectory point (the shipped circle trajectory ends at `0`).

Recorded output:

* `config/calibration_logs/position_trajectory_logs/pos_traj_<timestamp>.csv`
* `config/calibration_logs/position_trajectory_logs/pos_traj_<timestamp>.png`

The CSV contains, per joint, `q_act`, `q_ref`, `qd_act`, `qd_ref`, `e_p`, `e_v` and
`tau_meas`. The figure (`scripts/plot_position_trajectory.py`) shows positions, velocities,
position/velocity errors, measured torque and per-joint position RMSE.

---

## 5. Topics

### Hardware node (`position_control_node`)

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/gim/select_motor` | `std_msgs/Int32` | sub | `0`=none, `1`=base, `2`=shoulder, `3`=elbow, `-1`=all. |
| `/gim/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, pos_gain, vel_gain, vel_integrator_gain]` |
| `/gim/position_limits` | `std_msgs/Float64MultiArray` | sub | `[joint_index, vel_limit, accel_limit, (decel_limit)]` (motor turns/s). |
| `/gim/calibration_trajectory` | `std_msgs/Float64MultiArray` | sub | `[q_target, t_move, t_hold, (t_return)]` |
| `/gim/stop_calibration` | `std_msgs/Empty` | sub | Abort the running profile. |
| `/gim/reload_params` | `std_msgs/Empty` | sub | Reload `config/motor_params.yaml`. |
| `/gim/echo_params` | `std_msgs/Empty` | sub | Republish the parameter dump. |
| `/gim/joint_states` | `sensor_msgs/JointState` | pub | Measured position/velocity (rad, rad/s) + measured torque (Nm). |
| `/gim/motor_state` | `std_msgs/Float64MultiArray` | pub | Realtime `[q0..2 \| qd0..2 \| q_cmd0..2 \| qd_ref0..2 \| tau_meas0..2]` (15 values). |
| `/gim/status` | `std_msgs/String` | pub (latched) | `STARTING` \| `HOLD (all POSITION @ 0)` \| `SWITCHING` \| `DISABLED` \| `POSITION <motor>`. |
| `/gim/calibration_status` | `std_msgs/String` | pub (latched) | `IDLE` \| `RUNNING` \| `DONE` \| `ABORTED`. |
| `/gim/calibration_data` | `std_msgs/Float64MultiArray` | pub | `[t, joint_index, q_meas, q_ref, q_cmd, qd_meas, qd_ref, tau_meas]` (only while running). |
| `/gim/motor_params` | `std_msgs/String` | pub (latched) | Parameter dump (YAML). |
| `/gim/motor_params_data` | `std_msgs/Float64MultiArray` | pub (latched) | Numeric dump: 14 values per motor (see §6). |

Command-line equivalents:

```bash
ros2 topic pub --once /gim/select_motor      std_msgs/msg/Int32             "{data: 2}"
ros2 topic pub --once /gim/pid_gains         std_msgs/msg/Float64MultiArray "{data: [2, 0.5, 0.1, 0.0]}"
ros2 topic pub --once /gim/position_limits   std_msgs/msg/Float64MultiArray "{data: [2, 4.0, 20.0, 20.0]}"
ros2 topic pub --once /gim/calibration_trajectory std_msgs/msg/Float64MultiArray "{data: [0.6, 2.0, 1.0, 2.0]}"
ros2 topic echo /gim/joint_states
ros2 topic echo /gim/motor_state   # 15 values: pos, vel, cmd position, ref velocity, measured torque
```

### Calibration node (`position_tuner_node`)

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/tuner/select_motor` | `std_msgs/Int32` | sub | Same as the `select` command. |
| `/tuner/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, p, v, i]` |
| `/tuner/position_limits` | `std_msgs/Float64MultiArray` | sub | `[joint_index, vel_limit, accel_limit, (decel_limit)]` |
| `/tuner/calibration_trajectory` | `std_msgs/Float64MultiArray` | sub | `[q, t_move, t_hold, (t_return)]` |
| `/tuner/read_params` | `std_msgs/Empty` | sub | Print the parameters. |
| `/tuner/save_params` | `std_msgs/String` | sub | Save to `config/<data>` (empty = `motor_params.yaml`). |
| `/tuner/plot` | `std_msgs/Empty` | sub | Re-plot the last run. |
| `/tuner/stop` | `std_msgs/Empty` | sub | Abort the profile. |
| `/tuner/command` | `std_msgs/String` | sub | Any terminal command line, e.g. `"{data: 'select elbow'}"`. |
| `/tuner/motor_params` | `std_msgs/String` | pub (latched) | Last parameters read. |

### Run node (`position_trajectory_node`)

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/gim/enable` | `std_msgs/Empty` | sub | Enable all motors in position mode. |
| `/gim/disable` | `std_msgs/Empty` | sub | Disable all motors. |
| `/gim/safety_stop`, `/gim/emergency_stop` | `std_msgs/Empty` | sub | Emergency stop. |
| `/gim/reset_fault` | `std_msgs/Empty` | sub | Clear the fault latch. |
| `/gim/start_trajectory` | `std_msgs/Empty` | sub | Start the loaded trajectory once. |
| `/gim/pause_trajectory` | `std_msgs/Empty` | sub | Pause / resume. |
| `/gim/stop_trajectory` | `std_msgs/Empty` | sub | Stop and hold the current pose. |
| `/gim/load_trajectory` | `std_msgs/String` | sub | Load a CSV path. |
| `/gim/set_loop` | `std_msgs/Bool` | sub | Enable / disable looping. |
| `/gim/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, p, v, i]` live update. |
| `/gim/position_limits` | `std_msgs/Float64MultiArray` | sub | `[joint_index, vel_limit, accel_limit, (decel_limit)]` live update. |
| `/gim/echo_params`, `/gim/reload_params`, `/gim/plot`, `/gim/command` | `Empty`/`String` | sub | Same actions as the terminal commands. |
| `/gim/trajectory_status` | `std_msgs/String` | pub (latched) | `IDLE` \| `APPROACHING` \| `RUNNING` \| `PAUSED` \| `COMPLETED` \| `ABORTED`. |
| `/gim/trajectory_progress` | `std_msgs/Float64MultiArray` | pub | `[t_cur, t_total, progress_ratio]`. |
| `/gim/motor_state` | `std_msgs/Float64MultiArray` | pub | `[q0..2 \| qd0..2 \| q_ref0..2 \| qd_ref0..2 \| q_cmd0..2 \| tau_meas0..2]` (18 values). |

---

## 6. Parameters

### `position_control_node`

| Parameter | Default | Description |
|---|---|---|
| `node_ids` | `[0, 1, 2]` | CAN node id per joint (base, shoulder, elbow). |
| `motor_labels` | `[base, shoulder, elbow]` | Names used on topics and log files. |
| `gear_ratio` | `[8, 64, 8]` | Rotor turns per joint turn. |
| `invert_direction` | `[true, false, true]` | Flips the sign of position/velocity. |
| `torque_sign` / `torque_gear_ratio` | `[-1,1,-1]` / `[1,8,1]` | Convert the measured motor torque to joint torque. |
| `torque_limit` | `[5, 40, 5]` | Joint torque limit (Nm), also the safety threshold. |
| `joint_lower` / `joint_upper` | URDF limits | Joint position limits (rad) used for clamping + the fault check. |
| `pid_pos_gain` / `pid_vel_gain` / `pid_vel_integrator_gain` | per motor | ODrive position PID gains. |
| `vel_limit` / `accel_limit` / `decel_limit` | `[4,4,4]` / `[0,0,0]` | ODrive trajectory limits (motor turns/s, turns/s²). `0` accel/decel = leave the ODrive default. |
| `control_rate_hz` | `100` | Control loop rate. |
| `encoder_timeout` | `0.2` | Encoder watchdog (s); a fault disables all motors. |
| `params_file` | `.../config/motor_params.yaml` | The parameter file: loaded at start, written on change. |
| `load_params_on_start` | `true` | Set `false` to ignore the file and use built-in defaults. |
| `auto_save_params` | `true` | Re-write the parameter file whenever a parameter is tuned. |
| `startup_timeout` | `10.0` | Seconds to wait for **all** encoders before giving up (0 = wait forever). |

### `position_tuner_node`

| Parameter | Default | Description |
|---|---|---|
| `config_dir` | `/home/tue/gim_control/config` | Where `save` writes the YAML. |
| `log_dir` | `<config_dir>/calibration_logs` | Where CSV + PNG are written. |
| `params_filename` | `motor_params.yaml` | Default `save` filename. |
| `plot_script` | `.../scripts/plot_calibration.py` | Matplotlib helper used for the figure. |
| `motor_labels` | `[base, shoulder, elbow]` | Must match the hardware node. |

### `position_trajectory_node`

| Parameter | Default | Description |
|---|---|---|
| `trajectory_file` | `.../src/robot_trajectory/gim_arm_circle_traj.csv` | Default trajectory CSV. |
| `log_dir` | `.../config/calibration_logs/position_trajectory_logs` | Where run CSV + PNG are written. |
| `plot_script` | `.../scripts/plot_position_trajectory.py` | Matplotlib helper used for the figure. |
| `auto_plot` | `true` | Generate the PNG automatically after each run. |
| `loop_trajectory` | `false` | Repeat the trajectory (default: run once). |
| `enable_terminal_cli` | `true` | Start the interactive `gim_pos_traj>` prompt. |

The hardware parameters (`node_ids`, `gear_ratio`, `invert_direction`, `torque_*`,
`joint_lower/upper`, PID gains, `position:` limits) are shared with `position_control_node`
and are read from the same `config/motor_params.yaml`.

### `/gim/motor_params_data` layout (14 values per motor)

```text
[node_id, gear_ratio, invert, torque_sign, torque_gear_ratio, torque_limit,
 joint_lower, joint_upper, vel_limit, accel_limit, decel_limit,
 pos_gain, vel_gain, vel_integrator_gain]
```

---

## 7. Parameter file (source of truth)

`config/motor_params.yaml` holds **all** motor parameters. The position nodes:

* **read** it at start-up (`load_params_on_start`, `params_file`) — the values in the file
  override the built-in defaults; and
* **write** it whenever a parameter is tuned live (`auto_save_params`), and whenever the tuner
  runs `save`. Writes re-load the current file first, so other sections (e.g. `admittance:`,
  `impedance:`) are preserved.

```yaml
motors:
  base:
    node_id: 0              # CAN node id (base=0, shoulder=1, elbow=2)
    joint_name: base_joint
    gear_ratio: 8           # rotor turns per joint turn
    invert_direction: true  # flips position/velocity sign
    torque_sign: -1         # sign of the measured torque
    torque_gear_ratio: 1    # joint torque = motor torque * this
    torque_limit: 5         # joint torque limit [Nm]
    joint_lower: -0.7121    # joint limits [rad]
    joint_upper: 1.0226
    pid:                    # ODrive POSITION-mode PID
      pos_gain: 4
      vel_gain: 1.2
      vel_integrator_gain: 0.1
    position:               # ODrive trajectory limits (motor turns/s, turns/s^2)
      vel_limit: 4
      accel_limit: 0
      decel_limit: 0
    admittance: { ... }     # used by admittance_control_node (left untouched here)
    impedance:  { ... }     # used by impedance_control_node (left untouched here)
global:
  control_rate_hz: 100
  encoder_timeout: 0.2
```

> [!NOTE]
> `auto_save_params` re-writes the file (and drops any comments you added) every time a
> parameter changes. Edit the file while the node is **stopped**, or set
> `auto_save_params:=false` to keep your comments and save manually from the tuner.
>
> Only run **one** control node at a time — they all share `can0`.

---

## 8. Safety behaviour

* **Start-up waits for every motor:** the node does not energise anything until all three motors
  answer on CAN. If a motor is silent it reports which one (e.g. `elbow (node 2)`) and, after
  `startup_timeout`, leaves every motor DISABLED instead of enabling and faulting.
* **Start-up:** all motors are first put to `AXIS_STATE_IDLE`; the resting pose is captured as
  joint 0 so enabling does not jump.
* **All joints in POSITION mode:** no torque command is ever sent. The ODrive position PID and
  the `vel_limit` bound the motion; the node additionally clamps `q_cmd` to the joint limits.
* **Watchdog:** if a motor stops answering on CAN for `encoder_timeout` s, a joint runs far
  outside its limits, or the measured torque exceeds `torque_limit`, **all motors are disabled**
  and `/gim/status` reports the fault.
* **Shutdown:** `Ctrl-C` / `SIGTERM` disables every motor before the process exits.
* The calibration reference always starts at 0 and the target is clamped to the URDF joint
  limits. The run node approaches a distant start pose with a smooth S-curve.

---

## 9. Tuning guide

1. **Start with the position loop.** With the arm supported, `select base` and run small
   `traj` steps. Raise `pid <motor> <p> <v> <i>` until the joint tracks the step quickly
   without oscillation; lower `pos_gain` or raise `vel_gain` if it buzzes.
2. **Set the velocity limit.** `vel_limit` is in **motor-rotor turns/s**. Joint speed is
   `vel_limit * 2π / gear_ratio`; e.g. base (`gear_ratio 8`) at `4` turns/s ≈ `3.14 rad/s`.
3. **Use the acceleration limits** (`limit <motor> <v> <a> [d]`) if the ODrive's trapezoidal
   ramp is enabled, or leave them at `0` for pass-through reference following.
4. **Verify with the calibration run:** `traj 0.3 2 1 2`, then `plot`. The position and
   velocity panels should settle without overshoot; the error panel should return to ~0 during
   the hold.
5. **Run the full trajectory** with `position_trajectory_node` and check the RMSE bar chart
   after the run.

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No parameter file ... found` | Normal on the first run — built-in defaults are used. |
| `Cannot open CAN interface can0` | Bring `can0` up (`sudo ip link set can0 up type can ...`). |
| `Error sending ...` lines with no motors connected | Expected: the CAN bus never gets an ACK. Ignore during bench testing. |
| `FAULT: no encoder reply from node N` | Motor unpowered / not on the bus, or its **CAN node id differs** from `node_id` in `config/motor_params.yaml`. Find the real ids with `candump can0`. |
| Joint moves the wrong way | `invert_direction` is wrong in `config/motor_params.yaml`. |
| Joint oscillates / buzzes | Lower `pos_gain`, raise `vel_gain` (damping), or reduce `vel_limit`. |
| `FAULT: joint <j> torque ... exceeds limit` | The measured torque passed `torque_limit`. Raise it or reduce acceleration. |
| `Cannot start trajectory: motors are disabled` | Send `/gim/enable` (or type `enable` in the run node). |
| Figure not produced | `matplotlib` missing (`pip install matplotlib`) — the CSV is still saved. |
| Trajectory runs but errors are large | PID gains too low, velocity limit too low, or the arm is loaded beyond the motor's torque. |
| `position:` section vanished from `motor_params.yaml` | An older node (admittance/impedance) rewrote the file. Re-run `position_control_node` (it re-adds the section) or add it manually. |
