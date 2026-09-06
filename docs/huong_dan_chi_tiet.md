# Hướng Dẫn Chi Tiết & Báo Cáo Kỹ Thuật Dự Án: Điều Khiển Cánh Tay Robot Interbotix RX150 Bằng Bộ Điều Khiển Fuzzy Logic & MoveIt 2

---

## MỤC LỤC BÀI VIẾT BLOG HƯỚNG DẪN

1. **Tổng Quan Kiến Trúc & Triết Lý Thiết Kế**
2. **Quy Trình 1: Sinh Mã C Từ FIS & Trực Quan Hóa Mặt Điều Khiển 3D (Code Generation)**
3. **Quy Trình 2: Tuning An Toàn Từng Khớp Đơn (Single-Joint Safety Tuning & Overlay)**
4. **Quy Trình 3: Chạy Fuzzy Controller Toàn Arm & Live Tuning (GUI / CLI)**
5. **Quy Trình 4: Điều Khiển Hoạch Định Quỹ Đạo Bằng MoveIt 2 & Trajectory Bridge (Fuzzy / FF)**
6. **Quy Trình 5: Giám Sát Realtime, Ghi Dữ Liệu & So Sánh A/B (PlotJuggler, ROS Bag, CSV)**
7. **Bảng Cheatsheet Tổng Hợp Lệnh Terminal**

---

## 1. TỔNG QUAN KIẾN TRÚC & TRIẾT LÝ THIẾT KẾ

### 1.1. Bối cảnh phần cứng & phần mềm
- **Robot**: Cánh tay robot 5 bậc tự do Interbotix RX150 dùng động cơ thông minh Dynamixel (XM540 / XM430, Protocol 2.0). Giao tiếp với PC thông qua adapter U2D2 (`/dev/ttyDXL`).
- **Môi trường**: ROS 2 Humble trên Ubuntu 22.04 LTS.
- **Triết lý không xâm nhập (Non-invasive design)**: Không sửa đổi mã nguồn gốc (upstream/vendor) của Interbotix. Mọi tính năng fuzzy và bridge được xây dựng biệt lập hoàn toàn trong gói ROS 2 package `rx150_fuzzy_controller`.

### 1.2. Tại sao lại dùng PWM mode thay vì Position mode mặc định?
- Mặc định, Dynamixel sử dụng bộ điều khiển PID vị trí **nằm sẵn trong firmware** của động cơ.
- Để bộ điều khiển **Fuzzy Logic (Mamdani Type-1)** do chúng ta lập trình trên máy chủ (host-side) trực tiếp điều khiển robot, ta chuyển các motor sang **`operating_mode: pwm`**. Lúc này, firmware chỉ đóng vai trò khuếch đại công suất, toàn bộ vòng kín vị trí do node ROS 2 C++ tính toán và gửi lệnh PWM raw (độ rộng xung).

### 1.3. Sơ đồ phân tầng hệ thống (3 Layer Architecture)
1. **Tầng Toán Fuzzy (C Pure)**: Mã nguồn `fuzzy_type1.c` / `fuzzy_type1.h` được sinh tự động từ file `.fis`. Không chứa mã ROS 2.
2. **Tầng Điều Khiển ROS 2 (C++ Node)**: Node `fuzzy_node` chạy timer (100 Hz hoặc 500 Hz), đọc sensor_msgs/JointState, gọi toán Fuzzy và publish `JointGroupCommand` (PWM).
3. **Tầng Giao Diện / Thượng Cấp**: MoveIt 2 (`move_group`), `fuzzy_trajectory_bridge` (Action Server), `fuzzy_gui` (Tkinter GUI), PlotJuggler.

---

## 2. QUY TRÌNH 1: SINH MÃ C TỪ FIS & TRỰC QUAN HÓA MẶT 3D

Mọi luật điều khiển Fuzzy bắt đầu từ file thiết kế `.fis` (MATLAB Fuzzy Inference System format).

### Bước 2.1: Chỉnh sửa thiết kế Fuzzy
File nguồn sự thật: `fuzzy_codegen/fuzzy_type1.fis` hoặc `src/rx150_fuzzy_controller/src/fuzzy/fuzzy_type1.fis`.
Định nghĩa:
- **Đầu vào 1 (`e`)**: Sai số vị trí $e \in [-1, 1]$ (đã chuẩn hóa qua $K_e$).
- **Đầu vào 2 (`ed`)**: Vận tốc sai số $\dot{e} \in [-1, 1]$ (đã chuẩn hóa qua $K_{ed}$).
- **Đầu ra (`u`)**: Lệnh điều khiển chuẩn hóa $u \in [-1, 1]$.
- **Phương pháp**: Mamdani (AND=min, OR=max, Implication=min, Defuzzification=centroid).

### Bước 2.2: Sinh mã C tự động bằng `fis2c.py`
Công cụ `fis2c.py` đọc `.fis` và tự động tạo mã C99 portable mà không cần sửa tay.

**Thực hiện trên Terminal:**
```bash
cd ~/interbotix_ws/fuzzy_codegen

# 1. Sinh file fuzzy_type1.h, fuzzy_type1.c và fuzzy_type1_demo.c
python3 fis2c.py fuzzy_type1.fis

# 2. Hoặc dùng script đồng bộ tự động vào ROS 2 package:
cd ~/interbotix_ws/src/rx150_fuzzy_controller/src/fuzzy
./regenerate.sh
```

### Bước 2.3: Tạo Web Artifact 3D xem mặt điều khiển (Control Surface)
Để kiểm tra tính liên tục và độ mượt của mặt điều khiển Fuzzy trước khi nạp lên robot real:

```bash
cd ~/interbotix_ws/fuzzy_codegen

# 1. Tính lưới 2D (e, ed) và xuất surface.json
python3 gen_surface.py

# 2. Đóng gói thành file HTML 3D tương tác standalone
python3 build_surface_html.py

# 3. Mở file HTML trên trình duyệt
xdg-open fuzzy_surface.html
```
*Kết quả*: Mở trang web hiển thị khối 3D mặt điều khiển Fuzzy tương tác (xoay, thu phóng, xem contour).

---

## 3. QUY TRÌNH 2: TUNING AN TOÀN TỪNG KHỚP ĐƠN (`test_joint5.py` & `plot_runs.py`)

Thách thức lớn nhất khi điều khiển PWM là: nếu chọn sai Gain ($K_e, K_{ed}, K_u$), robot có thể bị vọt ngưỡng hoặc rung lắc nguy hiểm. Do đó dự án cung cấp quy trình **Kiểm thử độc lập Khớp 5 (`wrist_rotate`)**.

### Bước 3.1: Tại sao lại là Khớp 5?
- Khớp 5 (`wrist_rotate`) quay quanh trục dọc, không gánh trọng lực trực tiếp.
- Các khớp 1..4 vẫn giữ nguyên ở **Position mode** (firmware tự giữ vị trí bằng mô-men xoắn), chỉ riêng khớp 5 chuyển sang **PWM mode**.

### Bước 3.2: Chạy bài test Step Response khớp 5
Kịch bản test:
1. **Pha 1 (0 – 3s)**: Giữ vị trí hiện tại (`HOLD`).
2. **Pha 2 (3 – 16s)**: Bước nhảy đặt vị trí đích lệch $+2.0\text{ rad}$ ($\approx 115^\circ$).
3. **An toàn (Cleanup)**: Tự động đưa PWM về 0, chuyển khớp 5 về Position mode tại vị trí hiện tại (tránh giật nẩy khi kết thúc).

**Thực hiện trên Terminal:**
```bash
# Terminal 1: Chạy driver xs_sdk
source ~/interbotix_ws/install/setup.bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx150 use_rviz:=false

# Terminal 2: Biên dịch thư viện C tạm và chạy script test khớp 5
cd ~/interbotix_ws
gcc -shared -fPIC -O2 -o /tmp/fuzzy_type1.so fuzzy_codegen/fuzzy_type1.c -lm
python3 test_joint5.py
```
*Kết quả*: Script xuất log real-time và tự động tạo 2 file kết quả trong thư mục `runs/`:
- `runs/wrist_rotate_Ke0.2_Ked0.0005_Ku700.0_step+2.0_<timestamp>.csv`
- `runs/wrist_rotate_Ke0.2_Ked0.0005_Ku700.0_step+2.0_<timestamp>.png`

### Bước 3.3: So sánh A/B Tuning nhiều lần chạy qua `plot_runs.py`
Mỗi lần thay đổi tham số $K_e, K_{ed}, K_u$ trong `test_joint5.py` và chạy lại, file `.csv` mới được lưu vào `runs/`. Dùng `plot_runs.py` để chồng các đồ thị lên nhau so sánh đáp ứng quá độ.

```bash
cd ~/interbotix_ws

# Vẽ overlay tất cả các lần chạy trong runs/
python3 plot_runs.py

# Hoặc chỉ định các file CSV cụ thể
python3 plot_runs.py runs/run1.csv runs/run2.csv
```
*Đồ thị tạo ra (`runs/overlay.png`)* gồm 3 biểu đồ xếp chồng:
1. **Vị trí (pos vs ref)** theo thời gian.
2. **Điện áp PWM (u)** theo thời gian.
3. **Vận tốc (vel)** theo thời gian.

---

## 4. QUY TRÌNH 3: CHẠY FUZZY CONTROLLER TOÀN ARM & LIVE TUNING

Sau khi đã verified luật fuzzy an toàn trên khớp đơn, tiến hành chạy bộ điều khiển Fuzzy cho cả 5 khớp arm.

### Bước 4.1: Launch toàn bộ hệ thống Fuzzy Controller
Sử dụng file launch chính:
```bash
source ~/interbotix_ws/install/setup.bash
ros2 launch rx150_fuzzy_controller fuzzy_control.launch.py
```
*Tự động thực thi:*
1. Khởi động `xs_sdk` driver với file cấu hình động cơ `rx150_fuzzy.yaml`.
2. Khởi động `fuzzy_node` trong namespace `/rx150`.
3. Node gọi service `set_operating_modes` đưa cả 5 khớp arm sang **PWM mode**.
4. Bộ điều khiển bám pose mặc định (`reference_pose: [0.0, -1.80, 1.55, 0.8, 0.0]`).

### Bước 4.2: Điều khiển Setpoint & Tune Tham Số Bằng GUI Tkinter (`fuzzy_gui`)
Dự án có sẵn ứng dụng GUI Tkinter giao tiếp trực tiếp qua ROS 2 Topic & Parameter Service.

**Chạy GUI trên Terminal:**
```bash
source ~/interbotix_ws/install/setup.bash
ros2 run rx150_fuzzy_controller fuzzy_gui
```

**Tính năng trên GUI:**
- **Tab "Setpoint"**: 5 thanh trượt Slider tương ứng với 5 khớp (`waist`, `shoulder`, `elbow`, `wrist_angle`, `wrist_rotate`). Kéo trượt để publish trực tiếp tới topic `/rx150/fuzzy/setpoint`.
- **Tab "Gains"**: Bảng nhập tham số $K_e, K_{ed}, K_u, u_{max}$ cho từng khớp. Nút **Áp dụng hết** gọi service `set_parameters` đè thông số tức thì vào vòng lặp điều khiển 100 Hz mà không cần khởi động lại Node. Đặc biệt có nút **Lưu vào yaml** giúp bạn lưu thẳng bộ gain tối ưu vào file `fuzzy_gains.yaml` để dùng cho lần khởi động tiếp theo.

### Bước 4.3: Tune tham số hoặc Đặt Setpoint qua ROS 2 CLI

**Đặt Setpoint trực tiếp bằng Command Line:**
```bash
# Gửi góc tham chiếu 5 khớp (đơn vị rad)
ros2 topic pub --once /rx150/fuzzy/setpoint std_msgs/msg/Float64MultiArray "{data: [0.0, -1.2, 1.0, 0.5, 0.0]}"
```

**Đổi Gain live qua ROS 2 Parameter Service:**
```bash
# Đổi Ke của 5 khớp
ros2 param set /rx150/fuzzy_node Ke "[2.0, 3.0, 3.0, 1.5, 2.0]"

# Đổi Ku (gain công suất PWM)
ros2 param set /rx150/fuzzy_node Ku "[600.0, 800.0, 700.0, 700.0, 700.0]"
```

---

## 5. QUY TRÌNH 4: ĐIỀU KHIỂN HOẠCH ĐỊNH QUỸ ĐẠO MOVEIT 2 & BRIDGE NODE

Để kết hợp khả năng né vật cản / quy hoạch động học ngược IK của **MoveIt 2** với bộ điều khiển **Fuzzy PWM**, dự án sử dụng node cầu nối `fuzzy_trajectory_bridge`.

### Bước 5.1: Nguyên lý hoạt động của Trajectory Bridge
- MoveIt 2 phát Action Goal dạng `control_msgs/action/FollowJointTrajectory`.
- `fuzzy_trajectory_bridge` lắng nghe Action Server `/rx150/arm_controller/follow_joint_trajectory`.
- Bridge chia nhỏ quỹ đạo TOTP (Time-Optimal Trajectory Parameterization) của MoveIt thành chuỗi các điểm vị trí theo mốc thời gian, sau đó **nội suy tuyến tính ở tần số 100 Hz** và đẩy vào topic `/rx150/fuzzy/setpoint`.
- `fuzzy_node` tiếp nhận setpoint liên tục và điều khiển động cơ bám theo bằng luồng PWM.

### Bước 5.2: Kiểm tra Độc Lập Trajectory Bridge (Test Script)
Trước khi chạy toàn bộ hệ thống MoveIt cồng kềnh, bạn nên kiểm tra xem Bridge có đang hoạt động tốt (nhận Goal và stream setpoint thành công) bằng script tự động.

**Chạy kiểm thử (2 Terminal):**
```bash
# Terminal 1: Chạy driver và bộ điều khiển Fuzzy
ros2 launch rx150_fuzzy_controller fuzzy_control.launch.py

# Terminal 2: Chạy node test bridge (giao tiếp qua action server)
ros2 run rx150_fuzzy_controller fuzzy_bridge_test

# (Tùy chọn) Chạy test với góc lệch tùy chỉnh ở khớp khác
ros2 run rx150_fuzzy_controller fuzzy_bridge_test -- 0.05 elbow
```
*Kết quả:* Script sẽ gửi liên tiếp 2 Goal (zero-motion và small-motion) để test tracking error. Nếu Terminal 2 báo `TẤT CẢ PASS ✓`, hệ thống Action Server của bridge đã sẵn sàng cho MoveIt. Đóng Terminal 1 trước khi sang Bước 5.3.

### Bước 5.3: Khởi động tích hợp MoveIt + Fuzzy Controller
Chạy file launch tích hợp duy nhất:

```bash
source ~/interbotix_ws/install/setup.bash

# Launch đầy đủ Driver + Fuzzy Node + Bridge + MoveGroup + RViz2
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py
```

### Bước 5.4: Khởi động tích hợp MoveIt + FF Controller (Bộ điều khiển thứ 2)
Dự án hỗ trợ **2 bộ điều khiển** có thể chạy thay thế nhau tùy mục đích:

| Bộ điều khiển | Package | Feedforward | Ưu điểm |
|---|---|---|---|
| **Fuzzy + Gravity Comp** | `rx150_fuzzy_controller` | Bù trọng lực Pinocchio (RNEA) | Giữ vị trí tĩnh tốt nhờ bù trọng lực model-based |
| **Fuzzy + FF vel/acc** | `rx150_ff_controller` | $K_v \cdot \dot{q}_{profile} + K_a \cdot \ddot{q}_{profile}$ | Không cần URDF/Pinocchio, bám quỹ đạo động tốt |

```bash
source ~/interbotix_ws/install/setup.bash

# Chạy với FF controller (thay thế rx150_fuzzy_controller)
ros2 launch rx150_ff_controller ff_moveit.launch.py
```

> **Lưu ý**: Chỉ chạy **1 trong 2** controller tại 1 thời điểm. Đóng terminal controller cũ trước khi launch controller mới.

### Bước 5.5: Thao tác điều khiển trên RViz 2
1. Khi RViz 2 mở ra, nhìn vào khung **MotionPlanning** ở góc trái.
2. Chọn tab **Planning**.
3. Kéo con trỏ End-Effector (cầu điều khiển màu cam/xanh) đến vị trí mong muốn trong không gian 3D, hoặc chọn sẵn pose ở menu `Goal State` (VD: `Home`, `Sleep`).
4. Nút **Plan**: MoveIt tính toán quỹ đạo không va chạm.
5. Nút **Execute** (hoặc **Plan & Execute**): Quỹ đạo được gửi xuống `fuzzy_trajectory_bridge` (hoặc `ff_trajectory_bridge`) → Robot thực hiện chuyển động mượt mà bám theo bằng PWM Fuzzy.

---

## 6. QUY TRÌNH 5: GIÁM SÁT REALTIME, GHI DỮ LIỆU & SO SÁNH A/B

Để phân tích chất lượng điều khiển (chỉ số overshoot, settling time, tracking error), dự án hỗ trợ nhiều cách thu thập dữ liệu phù hợp từng mục đích.

### 6.1. Tổng quan các phương pháp thu thập dữ liệu

| Phương pháp | Dữ liệu thu được | Tự động? | Dùng khi nào |
|---|---|---|---|
| **`fuzzy_record.sh`** → ros2 bag | 5 topic controller (reference, error, effort, edot, joint_states) @ 100 Hz | ✅ Hoàn toàn tự động | Ghi đầy đủ data controller để phân tích chuyên sâu |
| **`test_pick_place_*.py --save-data`** → CSV | Position, Velocity, Effort tất cả joints | ✅ Tự động (tùy chọn bật/tắt) | Ghi data thí nghiệm pick-place |
| **PlotJuggler** → Toolbox CSV Exporter | Bất kỳ series nào đang hiển thị | ❌ Thủ công qua GUI | Export dữ liệu đã chọn lọc cho paper/report |

### 6.2. Cách 1: Ghi ROS 2 Bag bằng `fuzzy_record.sh` (Khuyến nghị cho nghiên cứu)

Script ghi bag **tự thêm timestamp** vào tên file và lưu vào thư mục `data/` chuyên dụng để không ghi đè lần chạy trước.

```bash
# Terminal 1: Chạy hệ thống robot
ros2 launch rx150_fuzzy_controller fuzzy_control.launch.py

# Terminal 2: Record data (tên bag tự sinh: step_20260813_134100)
cd ~/interbotix_ws/src/rx150_fuzzy_controller
./config/fuzzy_record.sh step

# Ctrl+C để dừng ghi → bag lưu tại:
# ~/interbotix_ws/src/rx150_fuzzy_controller/data/step_20260813_134100/
```

**So sánh A/B giữa 2 chế độ (Step vs Profile):**
```bash
# Lần 1: enable_profile: false → nhảy step thô
./config/fuzzy_record.sh step

# Lần 2: enable_profile: true → qua Ruckig Profile
./config/fuzzy_record.sh profile
```

**So sánh A/B giữa 2 controller (Fuzzy vs FF):**
```bash
# Lần 1: Chạy rx150_fuzzy_controller + record
cd ~/interbotix_ws/src/rx150_fuzzy_controller
./config/fuzzy_record.sh fuzzy_gravity

# Lần 2: Đổi sang ff_controller (cần tạo ff_record.sh tương tự)
# Record các topic /rx150/ff/... thay vì /rx150/fuzzy/...
```

### 6.3. Cách 2: Ghi CSV trong script Pick-Place (cho thí nghiệm gắp-đặt)

Các script pick-place hỗ trợ ghi dữ liệu `joint_states` (Position, Velocity, Effort) ra file CSV với **timestamp tự động**.

```bash
# Chạy pick-place MoveIt VỚI ghi data (mặc định BẬT)
ros2 run rx150_perception test_pick_place_moveit.py
# → File CSV lưu tại: rx150_perception/data/pick_place_moveit_20260813_134500.csv

# Chạy pick-place A→B VỚI ghi data
ros2 run rx150_perception test_pick_place_A_to_B.py
# → File CSV lưu tại: rx150_perception/data/pick_place_A_to_B_20260813_134500.csv

# Chạy KHÔNG ghi data (tiết kiệm bộ nhớ khi demo/debug)
ros2 run rx150_perception test_pick_place_moveit.py --no-save
ros2 run rx150_perception test_pick_place_A_to_B.py --no-save
```

**Cấu trúc file CSV:**
```
Time, Pos_0, Pos_1, ..., Pos_N, Vel_0, ..., Vel_N, Eff_0, ..., Eff_N
0.001, 0.012, -1.799, ...,     0.005, ...,        12.3, ...
0.011, 0.012, -1.798, ...,     0.008, ...,        11.8, ...
```

### 6.4. Cách 3: PlotJuggler — Giám sát Realtime & So sánh A/B Offline

PlotJuggler cung cấp giao diện đồ hoạ cho phép **giám sát trực quan các đáp ứng thời gian thực (Realtime Live Streaming)** cũng như **nạp nhiều bản ghi ROS 2 Bag (`.db3`) cùng lúc để so sánh đáp ứng (A/B Testing)** giữa bộ điều khiển Mờ và bộ điều khiển PID truyền thống.

**Bước 1: Khởi động PlotJuggler với Layout XML sẵn**
```bash
# Terminal 1: Khởi chạy bộ điều khiển mờ và robot (hoặc mô phỏng)
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py

# Terminal 2: Mở PlotJuggler kèm theo Layout dùng chung
ros2 run plotjuggler plotjuggler -l ~/interbotix_ws/data_analysis/layouts/fuzzy_plotjuggler_layout.xml
```

**Bước 2: Kết nối dữ liệu trực tuyến (Live Streaming)**
1. Chọn panel **Streaming** (bên cột trái) $\rightarrow$ Dropdown chọn **ROS2 Topic Subscriber** $\rightarrow$ Bấm **Start** ($\blacktriangleright$).
2. Trong hộp thoại chọn topic, tích chọn các topic chính:
   - `/rx150/joint_states` (Vị trí $q$ và vận tốc thực $\dot{q}$ từ động cơ)
   - `/rx150/fuzzy/reference` (Góc vị trí đặt $q_{ref}$ và vận tốc đặt $\dot{q}_{ref}$)
   - `/rx150/fuzzy/error` (Sai số vị trí góc $e = q_{ref} - q$)
   - `/rx150/fuzzy/edot` (Đạo hàm sai số $\dot{e}$)
   - `/rx150/fuzzy/effort` (Tín hiệu điều khiển PWM $u$ và momen bù trọng lực)
3. Các dạng sóng thu được sẽ tự động phân bổ vào **4 Tab đồ thị đã cấu hình sẵn** (Position & Reference, Joint Velocity, Error & Edot, Control Effort PWM).

**Bước 3: Nạp bản ghi ROS 2 Bag để Phân tích & So sánh A/B Offline**
1. Chọn menu **Data** $\rightarrow$ **Load Data from File** $\rightarrow$ Chọn file bản ghi dạng `.db3` hoặc `.mcap` trong thư mục `src/rx150_fuzzy_controller/data/`.
2. **Kỹ thuật Chồng đồ thị (Overlay)**: Nạp tiếp tệp bản ghi của đợt thử nghiệm thứ 2 (ví dụ: đợt chạy bộ điều khiển PID hoặc đường chạy Step vs Profile).
3. Đặt **Prefix** phân biệt cho từng đợt chạy (VD: `Fuzzy_` và `PID_`). Kéo các biến tương ứng vào chung một khung đồ thị để đánh giá thời gian tăng ($t_r$), thời gian quá độ ($t_s$) và độ vọt lố (%OS).

**Bước 4: Xuất dữ liệu CSV từ GUI PlotJuggler**
1. Chọn menu **Toolbox** $\rightarrow$ **CSV Exporter**.
2. Chọn các chuỗi thời gian (time-series) muốn xuất $\rightarrow$ Chọn khung thời gian (Toàn bộ hoặc Zoom vùng quan tâm).
3. Bấm **Export to File** $\rightarrow$ Lưu file `.csv` phục vụ xử lý lại trong Python/MATLAB.

> **Lưu ý**: Khác với việc ghi CSV tự động thông qua `csv_logger.py`, tính năng Export CSV của PlotJuggler thực hiện trực tiếp trên giao diện GUI và rất phù hợp khi cần trích xuất nhanh một phân đoạn thí nghiệm cụ thể để minh hoạ trong báo cáo.


### 6.5. Thư mục lưu trữ dữ liệu

Sau khi chạy thí nghiệm, dữ liệu được tổ chức vào các thư mục `data/` chuyên dụng:

```
rx150_fuzzy_controller/
└── data/                           ← ros2 bag recordings
    ├── step_20260813_134100/        ← bag thí nghiệm step
    ├── profile_20260813_140200/     ← bag thí nghiệm profile
    └── fuzzy_gravity_20260813_142000/

rx150_perception/
└── data/                           ← CSV data files
    ├── pick_place_moveit_20260813_134500.csv
    ├── pick_place_moveit_20260813_150300.csv
    └── pick_place_A_to_B_20260813_151000.csv
```

---

## 7. BẢNG CHEATSHEET TỔNG HỢP LỆNH TERMINAL

### 7.1. Build & Code Generation

| Công việc | Dòng lệnh Terminal |
|---|---|
| **Build Fuzzy Package** | `cd ~/interbotix_ws && colcon build --packages-select rx150_fuzzy_controller && source install/setup.bash` |
| **Build FF Package** | `cd ~/interbotix_ws && colcon build --packages-select rx150_ff_controller && source install/setup.bash` |
| **Sinh lại code C từ .fis** | `cd ~/interbotix_ws/src/rx150_fuzzy_controller/src/fuzzy && ./regenerate.sh` |
| **Xem mặt 3D Web** | `cd ~/interbotix_ws/fuzzy_codegen && python3 gen_surface.py && python3 build_surface_html.py && xdg-open fuzzy_surface.html` |

### 7.2. Chạy hệ thống

| Công việc | Dòng lệnh Terminal |
|---|---|
| **Launch Fuzzy Controller** | `ros2 launch rx150_fuzzy_controller fuzzy_control.launch.py` |
| **Launch MoveIt + Fuzzy** | `ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py` |
| **Launch MoveIt + FF** | `ros2 launch rx150_ff_controller ff_moveit.launch.py` |
| **Mở GUI Tune Tkinter** | `ros2 run rx150_fuzzy_controller fuzzy_gui` |
| **Test Bridge Độc Lập** | `ros2 run rx150_fuzzy_controller fuzzy_bridge_test` |
| **Test an toàn Khớp 5** | `cd ~/interbotix_ws && gcc -shared -fPIC -O2 -o /tmp/fuzzy_type1.so fuzzy_codegen/fuzzy_type1.c -lm && python3 test_joint5.py` |
| **Vẽ đồ thị Overlay Runs** | `cd ~/interbotix_ws && python3 plot_runs.py` |

### 7.3. Thu thập dữ liệu

| Công việc | Dòng lệnh Terminal |
|---|---|
| **Ghi ROS2 Bag Fuzzy** | `cd ~/interbotix_ws/src/rx150_fuzzy_controller && ./config/fuzzy_record.sh my_experiment` |
| **Pick-Place MoveIt (có ghi CSV)** | `ros2 run rx150_perception test_pick_place_moveit.py` |
| **Pick-Place MoveIt (không ghi)** | `ros2 run rx150_perception test_pick_place_moveit.py --no-save` |
| **Pick-Place A→B (có ghi CSV)** | `ros2 run rx150_perception test_pick_place_A_to_B.py` |
| **Pick-Place A→B (không ghi)** | `ros2 run rx150_perception test_pick_place_A_to_B.py --no-save` |
| **Mở PlotJuggler Layout** | `ros2 run plotjuggler plotjuggler -l ~/interbotix_ws/data_analysis/layouts/fuzzy_plotjuggler_layout.xml` |

### 7.4. Tune & Monitor

| Công việc | Dòng lệnh Terminal |
|---|---|
| **Pub Setpoint Fuzzy** | `ros2 topic pub --once /rx150/fuzzy/setpoint std_msgs/msg/Float64MultiArray "{data: [0.0, -1.8, 1.55, 0.8, 0.0]}"` |
| **Pub Setpoint FF** | `ros2 topic pub --once /rx150/ff/setpoint std_msgs/msg/Float64MultiArray "{data: [0.0, -1.8, 1.55, 0.8, 0.0]}"` |
| **Set Ke (Fuzzy)** | `ros2 param set /rx150/fuzzy_node Ke "[2.0, 3.0, 3.0, 1.5, 2.0]"` |
| **Set Ku (Fuzzy)** | `ros2 param set /rx150/fuzzy_node Ku "[600.0, 800.0, 700.0, 700.0, 700.0]"` |
| **Set Kv (FF)** | `ros2 param set /rx150/ff_node Kv "[50.0, 50.0, 50.0, 50.0, 50.0]"` |
| **Set Ka (FF)** | `ros2 param set /rx150/ff_node Ka "[10.0, 10.0, 10.0, 10.0, 10.0]"` |

---
*Báo cáo được tổng hợp đầy đủ cho việc soạn thảo bài viết blog kỹ thuật chi tiết.*
