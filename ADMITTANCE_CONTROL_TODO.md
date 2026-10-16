# TODO: Robot Admittance Control Node Design & Implementation Roadmap

> **Based on**: Architecture and implementation of [`src/robot_impedance_control_node`](file:///home/tue/gim_control/src/robot_impedance_control_node)  
> **Target Robot**: 3-DOF Rehabilitation Arm (GIM6010-8 Actuators via CAN Bus / ODrive protocol)  
> **Package**: `gim_control`  

---

## 1. Architectural Comparison: Impedance vs. Admittance

| Feature | Impedance Control (`robot_impedance_control_node`) | Admittance Control (`robot_admittance_control_node`) |
| :--- | :--- | :--- |
| **Causality** | Motion in $\to$ Force/Torque out | Force/Torque in $\to$ Compliant Motion/Position out |
| **Governing Law** | $\tau_{cmd} = K_p (q_{ref} - q) + K_d (\dot{q}_{ref} - \dot{q})$ | $M_a \Delta \ddot{q} + D_a \Delta \dot{q} + K_a \Delta q = \tau_{ext}$ |
| **Motor Mode** | **Torque Mode** (`InitTorqueMode` / `0x0E`) on active joint; **Position Mode** on others | **Position Mode** (`InitPositionMode` / `0x0C`) on **all** joints |
| **Hardware Output** | `hw_->SendTorqueCommand(node_id, tau)` | `hw_->SendPositionCommand(node_id, turns_cmd, vel_ff)` |
| **Feedback Needed** | Encoders (actual $q, \dot{q}$) | Encoders ($q, \dot{q}$) **AND** Torque/Current feedback ($\tau_{meas}$ via `Get_Iq`) |
| **Tuning Parameters** | $K_p$ (stiffness, Nm/rad), $K_d$ (damping, Nm·s/rad) | $M_a$ (virtual inertia, kg·m²), $D_a$ (damping, Nm·s/rad), $K_a$ (stiffness, Nm/rad) |
| **Key Advantage** | Natural backdrivability, fast compliance | Stiff trajectory tracking, zero-chatter steady state, user-guided transparent lead-through ($K_a \to 0$) |

---

## 2. Proposed Directory & File Layout

Following the exact structure established by `robot_impedance_control_node`:

```text
/home/tue/gim_control/
├── config/
│   ├── motor_params.yaml                # Default & saved M, D, K, deadband, PID gains
│   └── calibration_logs/                # Logs from admittance step/trajectory runs
├── scripts/
│   ├── plot_admittance.py               # Visualizer for admittance response (F vs Δq, step response)
├── src/
│   ├── robot_admittance_control_node/
│   │   ├── admittance_control_node.cpp  # Real-time CAN position-based admittance loop (100 Hz)
│   │   └── admittance_tuner_node.cpp    # Interactive terminal CLI (gim_admit> prompt)
│   └── robot_impedance_control_node/    # (Existing reference implementation)
│       ├── impedance_control_node.cpp
│       └── impedance_tuner_node.cpp
```

---

## 3. Mathematical Model & Discrete Formulation

### 3.1 Virtual Admittance Dynamics
For each joint $i \in \{1, 2, 3\}$:
$$M_{a,i} \Delta \ddot{q}_i + D_{a,i} \Delta \dot{q}_i + K_{a,i} \Delta q_i = \tau_{ext,i}$$

Where:
- $q_{ref,i}(t)$: Nominal equilibrium trajectory/position (rad)
- $\Delta q_i(t) = q_{cmd,i}(t) - q_{ref,i}(t)$: Virtual compliance deformation (rad)
- $q_{cmd,i}(t)$: Target position command fed into the ODrive position controller
- $\tau_{ext,i}$: External interaction torque exerted on joint $i$ (Nm)
- $M_{a,i} > 0$: Virtual inertia / apparent mass (kg·m² or Nm·s²/rad)
- $D_{a,i} > 0$: Virtual damping (Nm·s/rad)
- $K_{a,i} \ge 0$: Virtual spring stiffness (Nm/rad). Note: Setting $K_{a,i} = 0$ yields **free-floating / lead-through** mode.

### 3.2 External Torque Acquisition & Filtering
Because the GIM6010 motor measures total active torque $\tau_{meas}$ (via $I_q$ current feedback), external torque is isolated via:
1. **Torque Reading & Scaling**:
   $$\tau_{meas,i} = \text{sign}_i \cdot \tau_{motor,i} \cdot \text{gear\_ratio}_i$$
2. **Tare / Baseline Offset Removal**:
   $$\tau_{raw\_ext,i} = \tau_{meas,i} - \tau_{tare,i} - \tau_{gravity,i}(q)$$
3. **Low-Pass Filter (1st order IIR, cut-off $\approx 15-20\text{ Hz}$)**:
   $$\tau_{filt,i}[k] = (1 - \alpha) \tau_{filt,i}[k-1] + \alpha \, \tau_{raw\_ext,i}[k]$$
4. **Deadband Threshold (prevents drift from sensor noise / cogging torque)**:
   $$\tau_{ext,i} = \begin{cases} 
   0, & |\tau_{filt,i}| \le \tau_{deadband,i} \\
   \tau_{filt,i} - \text{sgn}(\tau_{filt,i}) \tau_{deadband,i}, & |\tau_{filt,i}| > \tau_{deadband,i}
   \end{cases}$$

### 3.3 Numerical Integration (Tustin / Bilinear or Forward-Backward Euler)
At each control tick $\Delta t$ ($100\text{ Hz} \implies \Delta t = 0.01\text{ s}$):
1. **Virtual Acceleration**:
   $$\Delta \ddot{q}_i[k] = \frac{1}{M_{a,i}} \left( \tau_{ext,i}[k] - D_{a,i} \Delta \dot{q}_i[k-1] - K_{a,i} \Delta q_i[k-1] \right)$$
2. **Virtual Velocity**:
   $$\Delta \dot{q}_i[k] = \Delta \dot{q}_i[k-1] + \Delta \ddot{q}_i[k] \Delta t$$
   *Apply velocity saturation*: $\Delta \dot{q}_i[k] \leftarrow \text{clamp}(\Delta \dot{q}_i[k], -\dot{q}_{max,i}, \dot{q}_{max,i})$
3. **Virtual Displacement**:
   $$\Delta q_i[k] = \Delta q_i[k-1] + \Delta \dot{q}_i[k] \Delta t$$
   *Apply deformation limit*: $\Delta q_i[k] \leftarrow \text{clamp}(\Delta q_i[k], -\Delta q_{limit,i}, \Delta q_{limit,i})$
4. **Final Motor Position Command**:
   $$q_{cmd,i}[k] = q_{ref,i}[k] + \Delta q_i[k]$$
   *Apply joint safety limits*: $q_{cmd,i}[k] \leftarrow \text{clamp}(q_{cmd,i}[k], q_{lower,i}, q_{upper,i})$
5. **Convert to Motor Rotor Turns**:
   $$\theta_{motor,i}[k] = \theta_{zero,i} + \text{dir}_i \cdot \frac{q_{cmd,i}[k]}{2\pi / \text{gear\_ratio}_i}$$
   `hw_->SendPositionCommand(node_ids[i], theta_motor_i, vel_ff);`

---

## 4. ROS 2 Topic & Interface Specification

### 4.1 Subscribed Topics (`admittance_control_node`)

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/gim/select_motor` | `std_msgs/msg/Int32` | `0`: Disable all (idle)<br>`1..3`: Enable selected motor in ADMITTANCE mode (others hold POSITION @ 0)<br>`-1` (optional): Enable **all 3 motors** in Admittance mode |
| `/gim/admittance_gains` | `std_msgs/msg/Float64MultiArray` | `[joint_index, M, D, K]` (set virtual inertia, damping, stiffness) |
| `/gim/pid_gains` | `std_msgs/msg/Float64MultiArray` | `[joint_index, pos_gain, vel_gain, vel_integrator_gain]` (set onboard ODrive PID) |
| `/gim/admittance_deadband` | `std_msgs/msg/Float64MultiArray` | `[joint_index, deadband_torque_Nm]` |
| `/gim/tare_torque` | `std_msgs/msg/Empty` | Triggers a baseline torque calibration: samples rest torque for 0.5 s and stores as $\tau_{tare}$ |
| `/gim/calibration_trajectory`| `std_msgs/msg/Float64MultiArray` | `[q_target, t_move, t_hold, (t_return)]` (nominal reference trajectory $q_{ref}(t)$) |
| `/gim/stop_calibration` | `std_msgs/msg/Empty` | Aborts running trajectory, resets $q_{ref}$ to 0 |

### 4.2 Published Topics (`admittance_control_node`)

| Topic Name | Message Type | QoS | Description |
| :--- | :--- | :--- | :--- |
| `/gim/joint_states` | `sensor_msgs/msg/JointState` | Volatile, 10 | Measured $q, \dot{q}$ and measured torque $\tau_{meas}$ |
| `/gim/motor_state` | `std_msgs/msg/Float64MultiArray` | Volatile, 50 | Real-time array `[q[3], qd[3], q_cmd[3], tau_meas[3], tau_ext[3]]` |
| `/gim/status` | `std_msgs/msg/String` | Latched, 1 | Node state: `DISABLED`, `STARTING`, `HOLD`, `ADMITTANCE <joint>`, `FAULT` |
| `/gim/calibration_status` | `std_msgs/msg/String` | Latched, 1 | `IDLE`, `RUNNING`, `DONE`, `ABORTED` |
| `/gim/calibration_data` | `std_msgs/msg/Float64MultiArray` | Volatile, 50 | `[t, joint, q_meas, q_ref, q_cmd, qd_meas, tau_ext, delta_q]` |
| `/gim/motor_params` | `std_msgs/msg/String` | Latched, 1 | Active parameter YAML string dump |
| `/gim/motor_params_data` | `std_msgs/msg/Float64MultiArray` | Latched, 1 | Numeric parameter dump (node IDs, gear ratios, M, D, K, PID, deadbands) |

---

## 5. Detailed Step-by-Step Implementation Checklist

### Phase 1: Parameter Definition & Configuration File Setup
- [ ] **TODO 1.1**: Define parameter schema in `config/motor_params.yaml`:
  - Per-motor fields: `node_id`, `gear_ratio`, `torque_sign`, `torque_gear_ratio`, `torque_limit`, `joint_lower`, `joint_upper`, `deadband_torque`.
  - Admittance gains per joint: `mass` ($M$), `damping` ($D$), `stiffness` ($K$), `max_delta_q` ($\Delta q_{max}$), `max_admit_vel` ($\dot{q}_{max}$).
  - Motor ODrive PID gains: `pos_gain`, `vel_gain`, `vel_integrator_gain`.
  - Global settings: `control_rate_hz` (100 Hz), `encoder_timeout` (0.2 s), `tare_samples` (50 ticks).
- [ ] **TODO 1.2**: Add YAML serialization & deserialization methods (`load_params_file` and `save_params_file`).

### Phase 2: Hardware Control & Mode Handling
- [ ] **TODO 2.1**: Audit `GimControlInterface` compatibility with pure Position Mode:
  - Verify that `InitPositionMode(node_id)` is applied to all active joints.
  - Verify that `SendPositionCommand(node_id, turns, vel_ff)` transmits smoothly at 100 Hz without CAN bus congestion.
  - Verify that `ReadEncoder(node_id)` and `ReadTorque(node_id)` execute cleanly every 10 ms.
- [ ] **TODO 2.2**: Implement smooth startup & transition worker thread (`startup_worker` and `select_worker`):
  - Encoders verified first before enabling any motor.
  - Zero reference turns $\theta_{zero}$ captured at rest to prevent jumping.
  - On switch to `ADMITTANCE`: motor is in `InitPositionMode` with tuned PID, and admittance state $(\Delta q, \Delta \dot{q})$ initializes to zero.
  - Safe shutdown (`shutdown_hardware`): sends `DisableMotor(node_id)` to put all axes into `AXIS_STATE_IDLE`.

### Phase 3: Core Admittance Algorithm & State Machine (`admittance_control_node.cpp`)
- [ ] **TODO 3.1**: Create data structures:
  ```cpp
  struct AdmittanceState {
      double delta_q = 0.0;       // Virtual deflection (rad)
      double delta_qd = 0.0;      // Virtual velocity (rad/s)
      double delta_qdd = 0.0;     // Virtual acceleration (rad/s²)
      double tau_tare = 0.0;      // Zero-force baseline torque (Nm)
      double tau_filtered = 0.0;  // IIR low-pass filtered torque (Nm)
  };
  ```
- [ ] **TODO 3.2**: Implement torque conditioning pipeline:
  - Reading `tau_meas` $\to$ tare subtraction $\to$ 1st-order IIR low-pass filter $\to$ deadband non-linearity.
- [ ] **TODO 3.3**: Implement the 100 Hz numerical admittance integrator:
  - Compute $\Delta \ddot{q} = \frac{1}{M} (\tau_{ext} - D \Delta \dot{q} - K \Delta q)$.
  - Semi-implicit Euler integration: $\Delta \dot{q} \mathrel{+}= \Delta \ddot{q} \Delta t$, $\Delta q \mathrel{+}= \Delta \dot{q} \Delta t$.
  - Apply clamping to $\Delta \dot{q}$, $\Delta q$, and total position command $q_{cmd} = q_{ref} + \Delta q$.
- [ ] **TODO 3.4**: Integrate nominal trajectory generator:
  - Retain S-curve trajectory generator (`s_curve`, `s_curve_dot`) to provide $q_{ref}(t)$ during calibration or autonomous motion.
- [ ] **TODO 3.5**: Implement safety watchdog & fault latch:
  - CAN encoder loss watchdog (`> encoder_timeout` s).
  - Out-of-bounds joint limit check ($q < q_{lower} - 0.3$ or $q > q_{upper} + 0.3$).
  - Excessive interaction torque limit ($|\tau_{meas}| > \tau_{limit}$).
  - Immediate `DisableMotor()` on all nodes upon any fault.

### Phase 4: Interactive Tuner Front-End (`admittance_tuner_node.cpp`)
- [ ] **TODO 4.1**: Create interactive terminal loop with prompt `gim_admit>`:
  - `select <base|shoulder|elbow|all|none>`: Selects active admittance joint(s).
  - `admit <motor> <M> <D> <K>`: Live updates virtual inertia, damping, and stiffness.
  - `tare`: Runs zero-torque baseline calibration while holding position.
  - `deadband <motor> <torque_Nm>`: Adjusts sensor deadband.
  - `pid <motor> <P> <V> <I>`: Tunes underlying ODrive position loop gains.
  - `traj <q_target> <t_move> <t_hold> [t_return]`: Runs trajectory to test compliance against moving nominal path.
  - `save [file]` & `read`: Persists and displays YAML settings.
  - `plot`: Triggers visualization script on last recorded log.
- [ ] **TODO 4.2**: Calibration logging:
  - Stream `[t, joint, q_meas, q_ref, q_cmd, qd_meas, tau_ext, delta_q]` into timestamped CSV `config/calibration_logs/admit_<motor>_<timestamp>.csv`.

### Phase 5: Python Visualization & Data Analysis (`scripts/plot_admittance.py`)
- [ ] **TODO 5.1**: Build matplotlib script accepting calibration CSV:
  - Subplot 1: Position tracking ($q_{meas}$ vs $q_{ref}$ vs $q_{cmd}$) showing compliance deflection $\Delta q$.
  - Subplot 2: External torque $\tau_{ext}(t)$ and measured torque $\tau_{meas}(t)$.
  - Subplot 3: Admittance phase plot ($\tau_{ext}$ vs $\Delta q$) demonstrating empirical stiffness $K_{eff} \approx \Delta \tau / \Delta q$.

### Phase 6: Build System & Package Integration
- [ ] **TODO 6.1**: Create subdirectory:
  ```bash
  mkdir -p src/robot_admittance_control_node
  ```
- [ ] **TODO 6.2**: Update [CMakeLists.txt](file:///home/tue/gim_control/CMakeLists.txt):
  - Add `add_executable(admittance_control_node src/robot_admittance_control_node/admittance_control_node.cpp)`.
  - Add `add_executable(admittance_tuner_node src/robot_admittance_control_node/admittance_tuner_node.cpp)`.
  - Link dependencies: `rclcpp`, `std_msgs`, `sensor_msgs`, `gim_control_interface`, `yaml-cpp`.
  - Add executables to `install(TARGETS ...)`.
  - Install `scripts/plot_admittance.py`.
- [ ] **TODO 6.3**: Verify build with `colcon build --packages-select gim_control`.

### Phase 7: Hardware Verification & Tuning Guide
- [ ] **TODO 7.1**: Initial position loop tuning:
  - Tune ODrive position gains ($P, V, I$) for stiff and responsive tracking without oscillations before engaging admittance.
- [ ] **TODO 7.2**: Tare torque calibration:
  - Run `tare` to record resting bias (especially for shoulder/elbow affected by arm link mass).
- [ ] **TODO 7.3**: Virtual mass-damper-spring tuning protocol:
  - Start with high stiffness $K$ (e.g. $20-30\text{ Nm/rad}$), moderate damping $D$ ($2-5\text{ Nm}\cdot\text{s/rad}$), and conservative inertia $M$ ($0.1-0.2\text{ kg}\cdot\text{m}^2$).
  - Push joint by hand to verify compliance direction and return-to-center.
  - Decrease $K$ towards zero to test transparent lead-through / zero-force teaching mode.
  - Ensure system passes stability check (critically damped or slightly over-damped: $D \ge 2\sqrt{M K}$).
