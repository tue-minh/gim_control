# Hướng Dẫn Hiệu Chỉnh Điều Khiển Admittance (Admittance Tuning Guide)

Tài liệu này hướng dẫn chi tiết nguyên lý, cấu trúc hệ thống và quy trình từng bước để hiệu chỉnh (tune) bộ điều khiển **Admittance (Tuân thủ nghịch đảo / Thừa nhận lực)** cho cánh tay robot phục hồi chức năng 3 bậc tự do (GIM6010-8) sử dụng mạng CAN bus và ROS 2.

---

## 1. Tổng quan & Nguyên lý hoạt động

### 1.1. Khác biệt giữa Admittance Control và Impedance Control

| Đặc tính | Impedance Control (Trở kháng) | Admittance Control (Tuân thủ) |
|---|---|---|
| **Chế độ điều khiển động cơ** | Động cơ chạy ở **TORQUE (Mô-men)** mode. | Động cơ chạy ở **POSITION (Vị trí)** mode. |
| **Đầu vào / Đầu ra thuật toán** | Nhận sai lệch vị trí $\Delta q$ $\rightarrow$ Tính toán và xuất mô-men $\tau$. | Đo mô-men ngoại lực tác động $\tau_{ext}$ $\rightarrow$ Tính toán độ lệch vị trí tuân thủ $\Delta q$. |
| **Độ an toàn phần cứng** | Khi mất kiểm soát hoặc mất tải dễ vọt tốc độ cao nếu không giới hạn tốt. | Cực kỳ an toàn và ổn định vì bộ điều khiển vị trí ODrive luôn giám sát vị trí chặt chẽ. |
| **Khớp không được chọn** | Phải chuyển đổi chế độ giữa vị trí và mô-men phức tạp. | Tất cả các khớp luôn ở POSITION mode; các khớp không tune giữ vị trí $0$ rad cố định. |

---

### 1.2. Mô hình động học ảo (Virtual Admittance Model)

Bộ điều khiển mô phỏng khớp quay như một hệ cơ học ảo gồm **Khối lượng (Quán tính $M_a$) – Giảm chấn ($D_a$) – Lò xo (Độ cứng $K_a$)**:

$$M_a \Delta\ddot{q} + D_a \Delta\dot{q} + K_a \Delta q = \tau_{ext}$$

Từ đó, gia tốc tuân thủ ảo được tính toán tại mỗi chu kỳ điều khiển:

$$\Delta\ddot{q} = \frac{\tau_{ext} - D_a \Delta\dot{q} - K_a \Delta q}{M_a}$$

Lệnh vị trí cuối cùng gửi xuống driver ODrive là:

$$q_{cmd} = q_{ref} + \Delta q$$

Trong đó:
* $q_{ref}$: Vị trí danh định (thường là $0$ rad ở trạng thái giữ tĩnh, hoặc quỹ đạo chuyển động chuẩn khi chạy hiệu chuẩn).
* $\Delta q$: Độ lệch vị trí ảo do ngoại lực sinh ra.
* $\tau_{ext}$: Ngoại lực thực tế người dùng tác động lên khớp (đã loại bỏ trọng lực tĩnh, lọc nhiễu và triệt tiêu vùng chết).
* $M_a$ (kg·m²): Quán tính ảo. Đại diện cho độ "nặng" khi bắt đầu đẩy hoặc đổi chiều chuyển động.
* $D_a$ (Nm·s/rad): Hệ số cản/giảm chấn ảo. Dập tắt dao động, mang lại cảm giác đầm tay và êm ái.
* $K_a$ (Nm/rad): Độ cứng lò xo ảo. Kéo khớp quay về vị trí danh định $q_{ref}$ khi buông tay. Khi đặt **$K_a = 0$**, hệ thống chuyển sang chế độ **Dắt tay tự do (Lead-through / Teaching mode)**.

---

### 1.3. Chu trình xử lý tín hiệu ngoại lực ($\tau_{meas} \rightarrow \tau_{ext}$)

Mô-men đo được từ dòng điện động cơ ODrive phản ánh toàn bộ nội lực, ma sát, trọng lực và ngoại lực. Để tách riêng ngoại lực tương tác của con người ($\tau_{ext}$), hệ thống thực hiện chuỗi xử lý:

```
τ_meas (Mô-men đo)
   │
   ▼
[ 1. Trừ Tare (τ_tare) ] ──► Loại bỏ trọng lực tĩnh và ma sát tĩnh ban đầu
   │
   ▼
[ 2. Lọc IIR bậc 1 (20 Hz) ] ──► Loại bỏ nhiễu răng cưa cao tần từ dòng điện
   │
   ▼
[ 3. Vùng chết Deadband ] ──► Triệt tiêu nhiễu dao động nhỏ và cogging torque
   │
   ▼
τ_ext (Ngoại lực tương tác đưa vào mô hình ảo)
```

1. **Tare ($\tau_{tare}$)**: Đo trung bình mẫu mô-men tĩnh khi cánh tay đứng yên trong $N$ chu kỳ (`tare_samples = 50` chu kỳ $\approx 0.5$ s):
   $$\tau_{raw} = \tau_{meas} - \tau_{tare}$$
2. **Bộ lọc thông thấp IIR bậc 1 (Cutoff $f_c = 20\text{ Hz}$)**:
   $$\tau_{filtered, k} = (1 - \alpha)\tau_{filtered, k-1} + \alpha \tau_{raw, k}, \quad \text{với } \alpha = \frac{dt}{\frac{1}{2\pi f_c} + dt}$$
3. **Bộ ngưỡng mềm vùng chết (Soft Deadband)**:
   $$\tau_{ext} = \begin{cases} 0, & \text{nếu } |\tau_{filtered}| \le \text{deadband} \\ \tau_{filtered} - \text{sign}(\tau_{filtered}) \cdot \text{deadband}, & \text{nếu } |\tau_{filtered}| > \text{deadband} \end{cases}$$

---

### 1.4. Tích phân số học & Giới hạn an toàn (Clamping)

Tại mỗi chu kỳ 100 Hz ($dt = 0.01$ s), thuật toán tích phân Euler bán ẩn (Semi-implicit Euler):
1. Tính gia tốc ảo: $\Delta\ddot{q} = (\tau_{ext} - D_a \Delta\dot{q} - K_a \Delta q) / M_a$
2. Tích phân vận tốc & Bão hòa vận tốc:
   $$\Delta\dot{q} \leftarrow \text{clamp}(\Delta\dot{q} + \Delta\ddot{q} \cdot dt, \; -\text{max\_admit\_vel}, \; \text{max\_admit\_vel})$$
3. Tích phân góc lệch & Bão hòa góc lệch:
   $$\Delta q \leftarrow \text{clamp}(\Delta q + \Delta\dot{q} \cdot dt, \; -\text{max\_delta\_q}, \; \text{max\_delta\_q})$$
4. Giới hạn hành trình vật lý khớp:
   $$q_{cmd} = \text{clamp}(q_{ref} + \Delta q, \; q_{lower}, \; q_{upper})$$

---

## 2. Chuẩn bị & Khởi động hệ thống

### 2.1. Cấu hình mạng CAN và Build mã nguồn

Mở Terminal trên máy tính điều khiển:

```bash
# 1. Di chuyển vào thư mục workspace
cd /home/tue/gim_control

# 2. Build package gim_control
colcon build --packages-select gim_control
source install/setup.bash

# 3. Kích hoạt giao tiếp CAN bus (tốc độ 1 Mbps)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 2.2. Khởi chạy 2 Node

Hệ thống được thiết kế thành 2 node độc lập nhằm cách ly vòng điều khiển thời gian thực và giao diện người dùng:

* **Terminal 1 — Hardware Node (`admittance_control_node`)**:
  Chịu trách nhiệm giao tiếp trực tiếp với CAN bus, chạy vòng lặp điều khiển thời gian thực 100 Hz, đảm bảo an toàn tuyệt đối.
  ```bash
  source /home/tue/gim_control/install/setup.bash
  ros2 run gim_control admittance_control_node
  ```
  *(Khi khởi động, node sẽ đọc encoder và lấy vị trí hiện tại làm vị trí $0$, các động cơ ở trạng thái chưa cấp điện IDLE cho đến khi được chọn).*

* **Terminal 2 — Interactive Tuner Node (`admittance_tuner_node`)**:
  Giao diện dòng lệnh tương tác trực tiếp với người dùng, gửi lệnh qua ROS 2 Topics, vẽ đồ thị và ghi log CSV.
  ```bash
  source /home/tue/gim_control/install/setup.bash
  ros2 run gim_control admittance_tuner_node
  ```

---

## 3. Bảng tra cứu lệnh CLI trong `admittance_tuner_node`

Sau khi chạy Tuner Node, bạn sẽ thấy dấu nhắc lệnh `gim_admit>`. Dưới đây là các lệnh chính:

| Lệnh | Cú pháp | Ý nghĩa & Mô tả |
|---|---|---|
| **Chọn khớp** | `select <motor>` | Kích hoạt Admittance trên khớp được chọn; các khớp còn lại giữ cứng vị trí 0.<br>`<motor>`: `base` (1), `shoulder` (2), `elbow` (3), hoặc `all` (-1). |
| **Ngắt nguồn** | `disable` hoặc `off` | Tắt nguồn tất cả các động cơ (`AXIS_STATE_IDLE`). |
| **Cài đặt M, D, K** | `admit <motor> <M> <D> <K>` | Cài đặt trọn bộ quán tính ảo $M$, giảm chấn $D$, độ cứng $K$. |
| **Chỉnh quán tính** | `mass <motor> <value>` | Chỉ thay đổi thông số quán tính ảo $M_a$ (kg·m²). |
| **Chỉnh giảm chấn** | `damp <motor> <value>` | Chỉ thay đổi thông số giảm chấn ảo $D_a$ (Nm·s/rad). |
| **Chỉnh độ cứng** | `stiff <motor> <value>` | Chỉ thay đổi độ cứng ảo $K_a$ (Nm/rad). Nhập `0` để vào chế độ dắt tay. |
| **Lấy mẫu Tare** | `tare` | Lấy mẫu mô-men tĩnh khi robot nghỉ. **Yêu cầu giữ yên robot**. |
| **Chỉnh Deadband** | `deadband <motor> <Nm>` | Cài đặt ngưỡng triệt tiêu nhiễu / cogging torque (thường 0.03 – 0.08 Nm). |
| **Chỉnh PID ODrive** | `pid <motor> <p> <v> <i>` | Cài đặt bộ PID vòng vị trí phần cứng ODrive (`pos_gain`, `vel_gain`, `vel_int`). |
| **Chạy quỹ đạo** | `traj <q> <t_move> <t_hold> [t_ret]` | Chạy quỹ đạo kiểm tra: từ $0 \rightarrow q$ trong $t_{move}$ s, giữ $t_{hold}$ s, quay về $0$. |
| **Dừng quỹ đạo** | `stop` | Dừng quỹ đạo đang chạy ngay lập tức. |
| **Đọc thông số** | `read` | Hiển thị toàn bộ thông số thực tế đang hoạt động của 3 động cơ. |
| **Lưu cấu hình** | `save [filename]` | Lưu thông số vào file YAML (`config/motor_params.yaml`). |
| **Vẽ đồ thị** | `plot` | Tạo lại hình vẽ đồ thị phân tích từ file log hiệu chuẩn gần nhất. |
| **Trạng thái** | `status` | Xem trạng thái hiện tại của node và khớp đang active. |
| **Thoát** | `quit` | Thoát chương trình tuner. |

---

## 4. Quy trình Tuning Admittance từng bước (Step-by-Step)

Quy tắc bất di bất dịch khi tune Admittance:
$$\text{Vòng PID vị trí ODrive} \;\longrightarrow\; \text{Tare bù điểm 0} \;\longrightarrow\; \text{Deadband} \;\longrightarrow\; \text{Độ cứng } K_a \;\longrightarrow\; \text{Giảm chấn } D_a \;\longrightarrow\; \text{Quán tính } M_a$$

```
  ┌────────────────────────────────────────────────────────┐
  │ Bước 1: Tune PID vòng vị trí ODrive (pos, vel, vel_int)│
  │ (Cần bám cứng vững, không rung chuông, không vọt lố)   │
  └──────────────────────────┬─────────────────────────────┘
                             │
                             ▼
  ┌────────────────────────────────────────────────────────┐
  │ Bước 2: Chọn khớp & Tare cân bằng tĩnh                 │
  │ (select <motor> -> Giữ yên tay robot -> tare)          │
  └──────────────────────────┬─────────────────────────────┘
                             │
                             ▼
  ┌────────────────────────────────────────────────────────┐
  │ Bước 3: Đặt Deadband chống trôi tự do                  │
  │ (deadband <motor> 0.05 -> Tăng nếu khớp tự bò trôi)    │
  └──────────────────────────┬─────────────────────────────┘
                             │
                             ▼
  ┌────────────────────────────────────────────────────────┐
  │ Bước 4: Hiệu chỉnh bộ ba Admittance (K_a, D_a, M_a)    │
  │ (Đảm bảo điều kiện giảm chấn tới hạn D >= 2*sqrt(M*K)) │
  └──────────────────────────┬─────────────────────────────┘
                             │
                             ▼
  ┌────────────────────────────────────────────────────────┐
  │ Bước 5: Chạy quỹ đạo Trajectory & Phân tích Phase Plot │
  │ (traj -> phân tích K_eff trên đồ thị PNG)              │
  └──────────────────────────┬─────────────────────────────┘
                             │
                             ▼
  ┌────────────────────────────────────────────────────────┐
  │ Bước 6: Lưu thông số (save motor_params.yaml)          │
  └────────────────────────────────────────────────────────┘
```

---

### Bước 1: Tối ưu hóa vòng vị trí ODrive (Position Loop PID)

Do bộ điều khiển Admittance xuất ra lệnh vị trí $q_{cmd}$, vòng điều khiển vị trí nội tại của ODrive đóng vai trò quyết định:
* Nếu vòng vị trí quá mềm (gains thấp): Robot phản hồi trễ, vị trí thực $q_{meas}$ không theo kịp $q_{cmd}$.
* Nếu vòng vị trí bị rung (ringing / chấn động do gains quá cao): Dao động tần số cao sẽ cộng hưởng với vòng lặp Admittance, gây mất ổn định nghiêm trọng.

**Thông số khởi điểm khuyên dùng cho phần cứng GIM6010-8**:
* Khớp **Base (Trục xoay chân đế, tỷ số truyền 8:1)**:
  ```text
  gim_admit> pid base 4.0 1.2 0.1
  ```
* Khớp **Shoulder (Khớp vai, tỷ số truyền lớn 64:1 - tải trọng lớn)**:
  ```text
  gim_admit> pid shoulder 0.5 0.1 0.0
  ```
* Khớp **Elbow (Khớp khuỷu, tỷ số truyền 8:1)**:
  ```text
  gim_admit> pid elbow 2.0 0.4 0.1
  ```

---

### Bước 2: Chọn khớp và lấy mẫu bù điểm 0 (Tare)

1. Chọn khớp muốn hiệu chỉnh:
   ```text
   gim_admit> select shoulder
   ```
   *(Khớp Shoulder sẽ vào chế độ Admittance; các khớp Base và Elbow sẽ giữ chặt vị trí ban đầu).*
2. Giữ nguyên tay robot ở tư thế cân bằng nghỉ dự kiến làm việc, sau đó gõ:
   ```text
   gim_admit> tare
   ```
   Hệ thống sẽ lấy mẫu liên tục trong 50 chu kỳ (0.5 giây) và ghi nhận giá trị $\tau_{tare}$.
   > [!IMPORTANT]
   > Hệ thống Admittance trong node này **không dùng mô hình bù trọng lực động học URDF tự động**, mà dùng giá trị baseline từ Tare. Do đó, **bất cứ khi nào thay đổi tư thế nghỉ ban đầu của cánh tay, bạn bắt buộc phải gõ lệnh `tare` lại**.

---

### Bước 3: Cài đặt Deadband chống trôi tự do (Drift)

Sau khi Tare, buông tay hoàn toàn khỏi robot:
* Nếu khớp đứng yên hoàn toàn: Deadband hiện tại đã phù hợp.
* Nếu khớp từ từ tự trôi (drift) về một phía: Do mô-men gợn (cogging torque) của động cơ hoặc nhiễu đo dòng điện lớn hơn ngưỡng hiện tại.
* Khắc phục bằng cách tăng nhẹ Deadband (bước tăng $0.01 \sim 0.02$ Nm):
  ```text
  gim_admit> deadband shoulder 0.08
  ```
  *(Giá trị chuẩn thường dao động từ $0.04$ đến $0.10$ Nm).*

---

### Bước 4: Hiệu chỉnh bộ thông số Admittance ($K_a, D_a, M_a$)

#### 4.1. Quy tắc giảm chấn tới hạn (Critical Damping)
Để khớp robot không bị vọt lố (overshoot) hoặc lắc lư bồng bềnh qua lại khi bị đẩy lệch rồi buông tay, hệ thống phải thỏa mãn điều kiện **tắt dần tới hạn hoặc hơi dập tắt quá mức (over-damped)**:

$$D_a \;\ge\; 2\sqrt{M_a \cdot K_a}$$

Bảng tra cứu giá trị giảm chấn tối thiểu $D_{min} = 2\sqrt{M_a \cdot K_a}$:

| Quán tính $M_a$ (kg·m²) | Độ cứng $K_a$ (Nm/rad) | $D_a$ tối thiểu ($D_{min}$) | Đề xuất chọn $D_a$ | Cảm giác tương tác khi chạm tay |
|:---:|:---:|:---:|:---:|---|
| **0.15** | **25.0** | $3.87$ | **4.0 ~ 5.0** | Cứng vững rõ rệt, đàn hồi về tâm rất nhanh. |
| **0.15** | **10.0** | $2.45$ | **2.5 ~ 3.5** | Mềm vừa phải, lực đẩy nhẹ tay, hồi tâm êm. |
| **0.10** | **5.0** | $1.41$ | **1.8 ~ 2.5** | Rất mềm, lực đẩy nhẹ nhàng, hồi tâm chậm. |
| **0.15** | **0.0** | $0$ | **2.0 ~ 4.0** | **Chế độ dắt tay (Free-float)**: Buông tay ở đâu đứng yên ở đó, cản êm tay. |
| **0.05** | **0.0** | $0$ | **1.5 ~ 2.5** | Dắt tay siêu nhẹ (quán tính thấp), chuyển động cực nhạy. |

#### 4.2. Trình tự thử nghiệm thực tế
1. **Thiết lập an toàn ban đầu (Stiff compliance)**:
   ```text
   gim_admit> admit shoulder 0.15 4.0 25.0
   ```
   Dùng tay đẩy thử khớp: Cảm nhận phản lực đẩy ngược lại rõ ràng, khi thả tay khớp hồi vị trí gốc dứt khoát không rung chuông.
2. **Làm mềm lò xo (Softer spring)**:
   Giảm dần độ cứng $K_a$, đồng thời giảm $D_a$ tương ứng theo công thức:
   ```text
   gim_admit> stiff shoulder 10.0
   gim_admit> damp shoulder 3.0
   ```
3. **Chuyển sang chế độ Dắt tay (Lead-through / Teaching mode)**:
   Khi muốn người bệnh hoặc người vận hành dẫn động cánh tay di chuyển tự do:
   ```text
   gim_admit> stiff shoulder 0
   gim_admit> damp shoulder 2.5
   ```
   * Thử đẩy khớp: Khớp sẽ trôi êm theo tay đẩy, khi dừng tay khớp dừng lại ngay mà không bị quán tính văng quá đà.
   * Nếu thấy cảm giác bị ì nặng tay: Giảm nhẹ $D_a$ (`damp shoulder 1.8`).
   * Nếu thấy khớp trôi tự do quá trớn hoặc rung lắc: Tăng $D_a$ (`damp shoulder 3.5`).
4. **Giảm quán tính ảo $M_a$ (Lighter feel)**:
   Nếu muốn phản ứng tăng tốc của khớp nhạy hơn khi người dùng bắt đầu chạm vào:
   ```text
   gim_admit> mass shoulder 0.08
   ```
   > [!WARNING]
   > Không giảm $M_a$ quá nhỏ ($M_a < 0.03\text{ kg}\cdot\text{m}^2$) vì trong tích phân rời rạc $\Delta\ddot{q} \propto 1/M_a$, $M_a$ quá nhỏ sẽ khuếch đại nhiễu làm động cơ bị rùng hoặc giật cục.

---

### Bước 5: Kiểm tra với quỹ đạo Trajectory & Phân tích Đồ thị

Để đánh giá định lượng độ cứng và khả năng tuân thủ của khớp, chạy một quỹ đạo kiểm tra tự động:

```text
# Cú pháp: traj <góc_target_rad> <thời_gian_đi_s> <thời_gian_giữ_s> [thời_gian_về_s]
gim_admit> traj 0.5 2.0 1.0 2.0
```
*(Khớp sẽ di chuyển mượt mà theo đường cong Cosine $S$-curve từ $0 \rightarrow 0.5$ rad trong 2 giây, giữ nguyên 1 giây, rồi quay về $0$ trong 2 giây).*

Trong lúc khớp đang chạy quỹ đạo:
1. Người kiểm tra có thể **dùng tay cản nhẹ hoặc đẩy thêm** vào cánh tay.
2. Quan sát cánh tay nhường nhịn (tuân thủ) theo lực tay của bạn.
3. Khi quỹ đạo kết thúc, Tuner Node sẽ tự động xuất dữ liệu ra 2 file:
   * File dữ liệu dạng bảng: `config/calibration_logs/admit_<motor>_<timestamp>.csv`
   * Hình ảnh đồ thị: `config/calibration_logs/admit_<motor>_<timestamp>.png`

#### Cách đọc biểu đồ phân tích (Hình ảnh 3 khung):

```
┌────────────────────────────────────────────────────────────┐
│ Khung 1: Position Tracking & Deflection (Vị trí & Độ lệch) │
│ - Đường nét đứt (q_ref): Vị trí danh định quỹ đạo          │
│ - Đường nét chấm (q_cmd): q_cmd = q_ref + delta_q          │
│ - Đường liền nét (q_meas): Vị trí góc quay thực tế đo được │
│ - Trục phải màu xanh (delta_q): Độ biến dạng tuân thủ ảo  │
├────────────────────────────────────────────────────────────┤
│ Khung 2: Torques (Mô-men tương tác)                        │
│ - Màu đỏ (tau_ext): Mô-men người dùng thực tế tác động     │
│ - Màu xám (tau_meas): Mô-men đo được từ dòng điện động cơ  │
├────────────────────────────────────────────────────────────┤
│ Khung 3: Phase Plot (Biểu đồ pha tau_ext vs delta_q)       │
│ - Độ dốc tuyến tính của đồ thị chính là:                   │
│          K_eff = Cov(tau_ext, delta_q) / Var(delta_q)     │
│ - So sánh: K_eff đo được trên hình phải xấp xỉ bằng K_a   │
│   đã cài đặt trong lệnh `stiff`.                           │
└────────────────────────────────────────────────────────────┘
```

Nếu muốn vẽ lại biểu đồ bất kỳ lúc nào, chỉ cần gõ lệnh:
```text
gim_admit> plot
```

---

### Bước 6: Lưu thông số vào file cấu hình

Khi đã đạt được thông số vận hành ưng ý:
```text
gim_admit> save
```
Lệnh này ghi đè các thông số chuẩn xác vào file cấu hình:
[`config/motor_params.yaml`](file:///home/tue/gim_control/config/motor_params.yaml)

Lần khởi động tiếp theo, `admittance_control_node` sẽ tự động tải các giá trị này mà không cần tune lại từ đầu.

---

## 5. Chẩn đoán & Xử lý sự cố thường gặp (Troubleshooting)

| Hiện tượng | Nguyên nhân cốt lõi | Cách xử lý |
|---|---|---|
| **Khớp bị rung giật cao tần (Buzzing / Chấn động)** | Hệ số giảm chấn quá nhỏ so với độ cứng ($D_a < 2\sqrt{M_a K_a}$), hoặc PID vòng vị trí ODrive quá căng. | 1. Tăng $D_a$ (hoặc giảm $K_a$).<br>2. Giảm bớt `pos_gain` và `vel_gain` trong ODrive PID (`pid <motor> ...`). |
| **Khớp tự bò / trôi từ từ khi không ai chạm vào (Drift)** | 1. Chưa Tare bù trọng lực ở tư thế hiện tại.<br>2. Vùng chết Deadband quá nhỏ so với mô-men gợn (cogging). | 1. Giữ yên tay robot và gõ lệnh `tare`.<br>2. Tăng giá trị Deadband lên (`deadband <motor> 0.08`). |
| **Cảm giác quá nặng, đẩy tốn sức (Sluggish)** | Quán tính ảo $M_a$ hoặc giảm chấn $D_a$ đặt quá lớn. | Giảm $D_a$ (ví dụ từ $4.0 \rightarrow 2.0$) và giảm $M_a$ (từ $0.15 \rightarrow 0.08$). |
| **Càng đẩy robot càng gồng đè ngược lại hoặc tự lao đi** | Chiều mô-men (`torque_sign`) hoặc chiều encoder (`invert_direction`) bị ngược dấu. | Kiểm tra dấu trong `config/motor_params.yaml`. Đảm bảo đẩy theo chiều dương thì $\tau_{ext}$ phải cùng dấu và sinh ra $\Delta q > 0$. |
| **Lỗi `FAULT: joint ... torque exceeds limit`** | Lực tác động vượt quá giới hạn an toàn `torque_limit`. | Không tì đè lực quá mạnh. Kiểm tra lại xem tư thế có bị gập kẹt cơ khí không. Nếu cần tải lớn hơn trong bài tập, tăng `torque_limit` trong YAML. |
| **Lỗi `FAULT: no encoder reply from node N`** | Đứt kết nối cáp CAN hoặc mất nguồn động cơ ODrive. | Kiểm tra đèn LED ODrive, kiểm tra giắc cắm CAN, chạy lệnh `candump can0` để soi gói tin CAN. |
| **Không xuất được file hình ảnh `.png`** | Máy tính chưa cài thư viện `matplotlib` cho Python 3. | Cài đặt bổ sung: `pip3 install matplotlib` hoặc `sudo apt install python3-matplotlib`. |
| **Thông số Admittance bị mất sau khi chạy node khác** | Node `impedance_control_node` phiên bản cũ có thể ghi đè file YAML mà không giữ trường `admittance:`. | Chỉ chạy một node điều khiển tại một thời điểm. Kiểm tra lại nội dung file `motor_params.yaml` trước và sau khi lưu. |

---

## 6. Khuyến nghị thông số mẫu cho Robot phục hồi chức năng 3-DOF

Dưới đây là bộ thông số thực nghiệm khuyến nghị cho cánh tay GIM6010-8:

### Chế độ Tập vận động có trợ lực (Assisted / Compliant Mode)
*Phù hợp cho bệnh nhân có cử động yếu, robot giữ đường dẫn và hỗ trợ đàn hồi:*
* **Base**: `admit base 0.15 4.0 20.0` | `deadband base 0.05`
* **Shoulder**: `admit shoulder 0.15 5.0 25.0` | `deadband shoulder 0.06`
* **Elbow**: `admit elbow 0.15 4.0 20.0` | `deadband elbow 0.05`

### Chế độ Dắt tay dạy học / Tập chủ động tự do (Free Lead-through / Active Mode)
*Phù hợp cho bài tập chủ động hoàn toàn, robot di chuyển nhẹ nhàng không cản trở:*
* **Base**: `admit base 0.10 2.5 0.0` | `deadband base 0.05`
* **Shoulder**: `admit shoulder 0.12 3.0 0.0` | `deadband shoulder 0.06`
* **Elbow**: `admit elbow 0.08 2.0 0.0` | `deadband elbow 0.05`
