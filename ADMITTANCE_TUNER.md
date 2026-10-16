# Admittance Tuner — Introduction & User Guide

The **admittance tuner** is a two-node tool for bringing up and calibrating the three
GIM6010 motors of the upper-arm rehabilitation robot in **admittance mode**.

| Node | Executable | Role |
|---|---|---|
| Hardware node | `ros2 run gim_control admittance_control_node` | Real-time CAN control. Keeps **every** joint in **POSITION** mode and runs a virtual mass-damper-spring admittance law on the selected joint(s). |
| Tuner node | `ros2 run gim_control admittance_tuner_node` | Interactive terminal front-end. Selects a motor, tunes the virtual gains / deadband / PID live, tares the torque sensor, runs the calibration trajectory, saves parameters, and writes the CSV + figure. |

The tuner never touches CAN directly; it talks to the hardware node over topics. That keeps
the real-time loop isolated from terminal input and file I/O.

```
                 /gim/select_motor, /gim/admittance_gains, /gim/pid_gains, /gim/admittance_deadband,
                 /gim/tare_torque, /gim/calibration_trajectory, /gim/stop_calibration
admittance_tuner_node ─────────────────────────────────────────────────────────► admittance_control_node ──► can0
        ▲        ◄────────────────────────────────────────────────────────────────────────────────────────────
        │        /gim/joint_states, /gim/status, /gim/calibration_status, /gim/calibration_data,
        │        /gim/motor_params, /gim/motor_params_data
        │
   terminal (stdin)                 + /tuner/* command topics           config/motor_params.yaml
                                                                        config/calibration_logs/*.csv|*.png
```

---

## 1. How it works

Unlike the impedance node (which drives **one** joint in **torque** mode), the admittance node
measures the interaction torque and turns it into a compliant **position** command. Every joint
stays in **POSITION** mode; each selected joint obeys the virtual admittance

$$M_a \Delta\ddot{q} + D_a \Delta\dot{q} + K_a \Delta q = \tau_{ext}
\qquad\Longrightarrow\qquad q_{cmd} = q_{ref} + \Delta q$$

* $q_{ref}$ — nominal equilibrium trajectory (0, or the running calibration profile),
* $\Delta q$ — virtual compliance deflection,
* $q_{cmd}$ — position fed to the ODrive position controller,
* $\tau_{ext}$ — external torque, isolated from the measured motor torque.

The external torque is conditioned from the raw motor torque feedback
(`Get_Iq`, 0x14) as:

$$\tau_{meas} \;\rightarrow\; \underbrace{\text{tare}}_{\tau_{tare}} \;\rightarrow\; \underbrace{\text{1st-order IIR low-pass}}_{\approx 20\,\text{Hz}} \;\rightarrow\; \underbrace{\text{deadband}}_{\text{soft threshold}} \;\rightarrow\; \tau_{ext}$$

At each 100 Hz tick the node integrates the virtual dynamics with semi-implicit Euler,
clamping $\Delta\dot q$ (`max_admit_vel`), $\Delta q$ (`max_delta_q`) and the total command
(the URDF joint limits), then sends the result as motor-rotor turns.

> [!NOTE]
> Setting $K_a = 0$ disables the virtual spring: the joint becomes **transparent / free-floating**
> (lead-through teaching mode, damped by $D_a$ only).

---

## 2. Quick start

```bash
# 0) workspace + CAN up (once)
cd /home/tue/gim_control
colcon build --packages-select gim_control
source install/setup.bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 1) terminal 1 - real hardware (keep it running)
ros2 run gim_control admittance_control_node

# 2) terminal 2 - tuner (interactive)
ros2 run gim_control admittance_tuner_node
```

On start the hardware node:

1. reads the encoders and takes the **current pose as joint 0**,
2. leaves every motor in `AXIS_STATE_IDLE` (unpowered) until a selection arrives,
3. waits for a selection.

> [!WARNING]
> Place the arm in a safe, supported pose before starting. There is **no gravity
> compensation** beyond the tare baseline — a joint put into admittance mode with low stiffness
> becomes free and can sag. Keep a hand on the e-stop and start with small gains and small targets.

---

## 3. Terminal commands

Once the tuner is running you get a `gim_admit>` prompt. All commands are live.

| Command | Description |
|---|---|
| `select <motor>` | Run a motor in **ADMITTANCE** mode; the others hold **POSITION @ 0**. `<motor>` = `0`\|`1`\|`2`\|`3` or `none`\|`base`\|`shoulder`\|`elbow`, or `-1`/`all` for all three. |
| `disable` / `off` | Disable all motors (`AXIS_STATE_IDLE` / unpowered). |
| `admit <motor> <M> <D> <K>` | Set the virtual inertia, damping and stiffness (kg·m², Nm·s/rad, Nm/rad). |
| `mass <motor> <value>` | Change only the virtual inertia $M$. |
| `damp <motor> <value>` | Change only the virtual damping $D$. |
| `stiff <motor> <value>` | Change only the virtual stiffness $K$ (`0` = lead-through). |
| `tare` | Sample the resting torque for `tare_samples` ticks and store it as the zero baseline. **Hold the arm still.** |
| `deadband <motor> <Nm>` | Set the torque deadband (sensor-noise / cogging rejection). |
| `pid <motor> <p> <v> <i>` | Set the ODrive PID gains: `pos_gain`, `vel_gain`, `vel_integrator_gain`. Pushed to the motor immediately. |
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
gim_admit> select shoulder
gim_admit> pid shoulder 0.5 0.1 0
gim_admit> admit shoulder 0.15 4 25
gim_admit> tare
gim_admit> traj 0.6 2 1 2
gim_admit> read
gim_admit> save
gim_admit> quit
```

## 4. Calibration workflow

1. **Select** the motor(s) → they go to admittance mode, the others stay at position 0.
2. **Tare** the torque sensor while the arm rests in the calibration pose.
3. **Tune** `M` / `D` / `K` (and `pid` if you want to change how the held joints settle).
4. **Run** `traj <q> <t_move> <t_hold> [t_return]`. The hardware node streams one sample per
   control tick on `/gim/calibration_data` while the profile runs.
5. When the profile finishes the tuner writes:
   * `config/calibration_logs/admit_<motor>_<timestamp>.csv`
   * `config/calibration_logs/admit_<motor>_<timestamp>.png`
6. Tuned parameters are **persisted automatically** to `config/motor_params.yaml` (every `admit`/
   `deadband`/`pid` change re-writes the file). `save` in the tuner does the same thing on demand,
   and the next start-up loads the file.

The CSV columns are:

```text
timestamp, joint, q_meas, q_ref, q_cmd, qd_meas, tau_ext, tau_meas, delta_q
```

`timestamp` is seconds from the start of the profile. The figure has three panels:

1. **position** — `q_meas` vs `q_ref` vs `q_cmd` with the compliance deflection `Δq` on a secondary axis;
2. **torque** — external torque `τ_ext` and measured torque `τ_meas`;
3. **phase plot** — `τ_ext` vs `Δq`; its slope is the empirical stiffness $K_{eff}$, printed in the title.

---

## 5. Topics

### Hardware node (`admittance_control_node`)

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/gim/select_motor` | `std_msgs/Int32` | sub | `0`=none, `1`=base, `2`=shoulder, `3`=elbow, `-1`=all. Starts an admittance switch. |
| `/gim/admittance_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, M, D, K]` |
| `/gim/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, pos_gain, vel_gain, vel_integrator_gain]` |
| `/gim/admittance_deadband` | `std_msgs/Float64MultiArray` | sub | `[joint_index, deadband_torque_Nm]` |
| `/gim/tare_torque` | `std_msgs/Empty` | sub | Sample the resting torque as the zero baseline. |
| `/gim/calibration_trajectory` | `std_msgs/Float64MultiArray` | sub | `[q_target, t_move, t_hold, (t_return)]` |
| `/gim/stop_calibration` | `std_msgs/Empty` | sub | Abort the running profile. |
| `/gim/joint_states` | `sensor_msgs/JointState` | pub | Measured position/velocity (rad, rad/s) + measured torque (Nm). |
| `/gim/motor_state` | `std_msgs/Float64MultiArray` | pub | Realtime state at the control rate: `[q0 q1 q2 \| qd0 qd1 qd2 \| q_cmd0 q_cmd1 q_cmd2 \| tau_meas0..2 \| tau_ext0..2]` (rad, rad/s, Nm). |
| `/gim/status` | `std_msgs/String` | pub (latched) | `STARTING` \| `HOLD (all POSITION @ 0)` \| `SWITCHING` \| `DISABLED` \| `ADMITTANCE <motor>`. |
| `/gim/calibration_status` | `std_msgs/String` | pub (latched) | `IDLE` \| `RUNNING` \| `DONE` \| `ABORTED`. |
| `/gim/calibration_data` | `std_msgs/Float64MultiArray` | pub | `[t, joint_index, q_meas, q_ref, q_cmd, qd_meas, tau_ext, tau_meas, delta_q]` (only while running). |
| `/gim/motor_params` | `std_msgs/String` | pub (latched) | Human-readable parameter dump (YAML). |
| `/gim/motor_params_data` | `std_msgs/Float64MultiArray` | pub (latched) | Numeric dump: 17 values per motor (see §6). |

Command-line equivalents:

```bash
ros2 topic pub --once /gim/select_motor        std_msgs/msg/Int32             "{data: 2}"
ros2 topic pub --once /gim/admittance_gains    std_msgs/msg/Float64MultiArray "{data: [2, 0.15, 4.0, 25.0]}"
ros2 topic pub --once /gim/pid_gains           std_msgs/msg/Float64MultiArray "{data: [2, 0.5, 0.1, 0.0]}"
ros2 topic pub --once /gim/admittance_deadband std_msgs/msg/Float64MultiArray "{data: [2, 0.05]}"
ros2 topic pub --once /gim/tare_torque         std_msgs/msg/Empty             "{}"
ros2 topic pub --once /gim/calibration_trajectory std_msgs/msg/Float64MultiArray "{data: [0.6, 2.0, 1.0, 2.0]}"
ros2 topic echo /gim/joint_states
ros2 topic echo /gim/motor_state   # 15 values: pos, vel, cmd position, measured torque, external torque
```

### Tuner node (`admittance_tuner_node`)

The same actions are exposed as topics so a script or another node can drive the tuner:

| Topic | Type | Dir | Description |
|---|---|---|---|
| `/tuner/select_motor` | `std_msgs/Int32` | sub | Same as the `select` command. |
| `/tuner/admittance_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, M, D, K]` |
| `/tuner/pid_gains` | `std_msgs/Float64MultiArray` | sub | `[joint_index, p, v, i]` |
| `/tuner/deadband` | `std_msgs/Float64MultiArray` | sub | `[joint_index, deadband]` |
| `/tuner/tare` | `std_msgs/Empty` | sub | Tare the torque sensor. |
| `/tuner/calibration_trajectory` | `std_msgs/Float64MultiArray` | sub | `[q, t_move, t_hold, (t_return)]` |
| `/tuner/read_params` | `std_msgs/Empty` | sub | Print the parameters. |
| `/tuner/save_params` | `std_msgs/String` | sub | Save to `config/<data>` (empty = `motor_params.yaml`). |
| `/tuner/plot` | `std_msgs/Empty` | sub | Re-plot the last run. |
| `/tuner/stop` | `std_msgs/Empty` | sub | Abort the profile. |
| `/tuner/command` | `std_msgs/String` | sub | Any terminal command line, e.g. `"{data: 'select elbow'}"`. |
| `/tuner/motor_params` | `std_msgs/String` | pub (latched) | Last parameters read. |

---

## 6. Parameters

### `admittance_control_node`

| Parameter | Default | Description |
|---|---|---|
| `node_ids` | `[0, 1, 2]` | CAN node id per joint (base, shoulder, elbow), matching `urdf/gim_arm.urdf`. |
| `motor_labels` | `[base, shoulder, elbow]` | Names used on topics and log files. |
| `gear_ratio` | `[8, 64, 8]` | Rotor turns per joint turn. |
| `invert_direction` | `[true, false, true]` | Flips the sign of position/velocity. |
| `torque_sign` | `[-1, 1, -1]` | Sign applied to the measured torque. |
| `torque_gear_ratio` | `[1, 8, 1]` | Joint torque = motor torque × this. |
| `torque_limit` | `[5, 40, 5]` | Joint torque limit (Nm). Also the excessive-torque fault threshold. |
| `joint_lower` / `joint_upper` | URDF limits | Joint position limits (rad) used for clamping + the fault check. |
| `mass` ($M_a$) | `[0.15, 0.15, 0.15]` | Virtual inertia (kg·m²). Must be `> 0`. |
| `damping` ($D_a$) | `[4, 4, 4]` | Virtual damping (Nm·s/rad). |
| `stiffness` ($K_a$) | `[25, 25, 25]` | Virtual stiffness (Nm/rad). `0` = lead-through. |
| `max_delta_q` | `[0.5, 0.5, 0.5]` | Deflection saturation (rad). |
| `max_admit_vel` | `[2, 2, 2]` | Virtual velocity saturation (rad/s). |
| `deadband_torque` | `[0.05, 0.05, 0.05]` | Torque deadband (Nm). |
| `pid_pos_gain` / `pid_vel_gain` / `pid_vel_integrator_gain` | per motor | ODrive PID gains for POSITION mode. |
| `control_rate_hz` | `100` | Control loop rate. |
| `encoder_timeout` | `0.2` | Encoder watchdog (s); a fault disables all motors. |
| `tare_samples` | `50` | Ticks sampled at rest for the tare baseline (0.5 s at 100 Hz). |
| `torque_filter_hz` | `20` | Low-pass cut-off for the external torque. |
| `hold_vel_limit` | `4.0` | POSITION-mode trajectory velocity limit (turns/s). |
| `params_file` | `.../config/motor_params.yaml` | The parameter file: loaded at start, written on change. |
| `load_params_on_start` | `true` | Set `false` to ignore the file and use built-in defaults. |
| `auto_save_params` | `true` | Re-write the parameter file whenever a parameter is tuned. |
| `startup_timeout` | `10.0` | Seconds to wait for **all** encoders before giving up (0 = wait forever). |

### `admittance_tuner_node`

| Parameter | Default | Description |
|---|---|---|
| `config_dir` | `/home/tue/gim_control/config` | Where `save` writes the YAML. |
| `log_dir` | `<config_dir>/calibration_logs` | Where CSV + PNG are written. |
| `params_filename` | `motor_params.yaml` | Default `save` filename. |
| `plot_script` | `.../scripts/plot_admittance.py` | Matplotlib helper used for the figure. |
| `motor_labels` | `[base, shoulder, elbow]` | Must match the hardware node. |

### `/gim/motor_params_data` layout (17 values per motor)

```text
[node_id, gear_ratio, invert, torque_sign, torque_gear_ratio, torque_limit,
 joint_lower, joint_upper, mass, damping, stiffness, max_delta_q, max_admit_vel,
 deadband_torque, pos_gain, vel_gain, vel_integrator_gain]
```

---

## 7. Parameter file (source of truth)

`config/motor_params.yaml` holds **all** motor parameters. The admittance node:

* **reads** it at start-up (`load_params_on_start`, `params_file`) — the values in the file
  override the built-in defaults; and
* **writes** it whenever a parameter is tuned live (`auto_save_params`), and whenever the tuner
  runs `save`. Writes re-load the current file first, so other sections (e.g. `impedance:`) are
  preserved.

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
    deadband_torque: 0.05   # torque deadband [Nm]
    pid:                    # ODrive POSITION-mode gains
      pos_gain: 4
      vel_gain: 1.2
      vel_integrator_gain: 0.1
    admittance:             # virtual admittance gains
      mass: 0.15            # [kg*m^2]
      damping: 4            # [Nm*s/rad]
      stiffness: 25         # [Nm/rad]  (0 = lead-through)
      max_delta_q: 0.5      # [rad]
      max_admit_vel: 2      # [rad/s]
    impedance:              # used by impedance_control_node (left untouched here)
      kp: 0
      kd: 0
global:
  control_rate_hz: 100
  encoder_timeout: 0.2
  tare_samples: 50
  torque_filter_hz: 20
```

> [!NOTE]
> `auto_save_params` re-writes the file (and drops any comments you added) every time a parameter
> changes. Edit the file while the node is **stopped**, or set `auto_save_params:=false` to keep
> your comments and save manually from the tuner.
>
> Running the **impedance** node also (re)writes this file and does not know about the `admittance:`/
> `global:` sections, so it will drop them. Only run one control node at a time (they share `can0`).

---

## 8. Safety behaviour

* **Start-up waits for every motor:** the node does not energise anything until all three motors
  answer on CAN. If a motor is silent it reports which one (e.g. `elbow (node 2)`) and, after
  `startup_timeout`, leaves every motor DISABLED instead of enabling and faulting.
* **Start-up:** all motors are first put to `AXIS_STATE_IDLE`; the resting pose is captured as joint 0
  so enabling does not jump.
* **All joints in POSITION mode:** switching the active joint needs **no** mode change, so it is
  fast. The inactive joints keep holding `q_ref = 0`.
* **Watchdog:** if a motor stops answering on CAN for `encoder_timeout` s, a joint runs far outside
  its limits, or the measured torque exceeds `torque_limit`, **all motors are disabled** and
  `/gim/status` reports the fault.
* **Output clamps:** `Δq`, `Δq̇` and the final `q_cmd` are all saturated, so a bad gain cannot demand
  an unbounded position.
* **Shutdown:** `Ctrl-C` / `SIGTERM` disables every motor before the process exits.

---

## 9. Tuning guide

Work through this in order — stiffness first, then damping, then inertia.

### 9.1 Position loop first
Before enabling admittance, tune the ODrive position loop (`pid <motor> <p> <v> <i>`) so the axis
tracks a step **stiffly and without oscillation**. A sluggish or ringing position loop adds an
extra lag/limit cycle on top of the virtual dynamics.

### 9.2 Tare
Run `tare` with the arm at rest in the pose you will use for the experiment. This records the
resting bias (gravity on the shoulder/elbow links is included) so it is not mistaken for user
force. Re-tare whenever the arm's resting pose changes.

### 9.3 Virtual mass-damper-spring
Start conservative and push the joint by hand:

| Step | $M_a$ (kg·m²) | $D_a$ (Nm·s/rad) | $K_a$ (Nm/rad) | Effect |
|---|---|---|---|---|
| 1 | 0.15 | 4 | 25 | Noticeably compliant, strong return-to-centre |
| 2 | 0.15 | 4 | 10 | Softer spring, easier to push |
| 3 | 0.15 | 2 | 0 | Transparent lead-through (damped free float) |
| 4 | 0.05 | 2 | 0 | Lighter/faster lead-through (watch for oscillation) |

For each setting:

1. push the joint by hand and verify it **returns to centre** when released (for $K_a > 0$);
2. decrease $K_a$ toward zero to reach the teaching mode;
3. keep the system **critically damped or slightly over-damped**:

$$D_a \;\ge\; 2\sqrt{M_a K_a}$$

| $M_a$ | $K_a$ | minimum $D_a$ |
|---|---|---|
| 0.15 | 25 | 3.87 |
| 0.15 | 10 | 2.45 |
| 0.15 | 0 | 0 (damping from $D_a$ only) |

Too little damping → oscillation / buzzing; too much → sluggish and heavy feel. Increase the
deadband (`deadband <motor> <Nm>`) if the joint drifts from sensor noise or cogging torque.

### 9.4 Verify with the trajectory
Run `traj <q> <t_move> <t_hold> [t_return]` and push against the moving nominal reference. Then run
`plot` and read the **phase plot**: the slope `K_eff` should match the configured $K_a$ when the
motion is slow enough that the $M_a$ / $D_a$ terms are negligible.

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No parameter file ... found` | Normal on the first run — built-in defaults are used. |
| `Cannot open CAN interface can0` | Bring `can0` up (`sudo ip link set can0 up type can ...`). |
| `Error sending ...` lines with no motors connected | Expected: the CAN bus never gets an ACK. Ignore during bench testing. |
| `FAULT: no encoder reply from node N` (or `No encoder reply from: <motor> (node N)`) | Motor unpowered / not on the bus, or its **CAN node id differs** from `node_id` in `config/motor_params.yaml`. Find the real ids with `candump can0`. |
| `FAULT: joint <j> torque ... exceeds limit` | The measured torque passed `torque_limit`. Raise it, or re-tare if the resting pose changed. |
| Joint drifts on its own | Tare with the arm at rest; increase `deadband_torque`; check `torque_sign` / `invert_direction`. |
| Joint oscillates / buzzes | Too little damping for the stiffness — increase $D_a$ (keep `D >= 2*sqrt(M*K)`); reduce $K_a$; check the position loop PID. |
| Joint feels heavy / sluggish | Too much damping or mass — lower $D_a$ / $M_a$. |
| Pushes the wrong way | `torque_sign` (or `invert_direction`) is wrong in `config/motor_params.yaml`. |
| Stuck in `SWITCHING` | Each ODrive mode change blocks for a few seconds; wait. |
| Figure not produced | `matplotlib` missing (`pip install matplotlib`) — the CSV is still saved. |
| `admittance:` section vanished from `motor_params.yaml` | The **impedance** node (or an old build) rewrote the file. Run only the admittance node, then re-save. |
