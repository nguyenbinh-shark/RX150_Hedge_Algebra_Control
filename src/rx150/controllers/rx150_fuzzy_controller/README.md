# Fuzzy Controller cho Interbotix RX150 (ROS 2 Humble)

Package này cung cấp hệ thống điều khiển mờ (Fuzzy Logic Controller - Type 1 Mamdani) điều khiển trực tiếp mức xung (PWM mode) cho cánh tay robot Interbotix RX150 trên nền tảng ROS 2 Humble. 
Đặc biệt, hệ thống được thiết kế theo triết lý **không xâm nhập (Non-invasive)**: giữ nguyên mã nguồn gốc của Interbotix, tích hợp trơn tru với MoveIt 2 và hệ thống Perception (Camera 3D).

## Vì sao PWM mode chứ không phải Position mode

Mặc định Dynamixel chạy **position mode**: vòng kín vị trí nằm sẵn trong firmware động
cơ — không quan sát được, không sửa được, không so sánh A/B được.

Chuyển motor sang `operating_mode: pwm` (khai trong `rx150_motor.yaml` của
`rx150_motion_common`) thì firmware chỉ còn đóng vai trò **khuếch đại công suất**: toàn
bộ vòng kín vị trí do node C++ này tính ở 100 Hz rồi gửi PWM thô qua `xs_sdk`. Đó là
điều kiện để ba bộ `fuzzy` / `ff` / `hac` thay nhau cùng một chỗ mà tầng ứng dụng không
phải sửa gì.

Đánh đổi: mất luôn phần giữ vị trí của firmware, nên **mất điện node là tay rơi** — mọi
quy trình bring-up đều phải theo thang bậc trong RUNBOOK.

## Tính năng chính

1. **Điều khiển PWM vòng kín (Closed-loop PWM Control)**:
   - Thay thế bộ PID vị trí mặc định trong firmware của động cơ Dynamixel bằng bộ điều khiển Fuzzy tính toán trên máy tính chủ (host-side) chạy ở tần số cao (100 Hz).
2. **Biên dịch FIS sang C nguyên thuỷ**:
   - Luật điều khiển (Mamdani) được thiết kế từ file `.fis` (MATLAB format), sau đó được tự động sinh thành mã C thuần (`fuzzy_type1.c`) giúp tối ưu tốc độ thực thi, không phụ thuộc vào thư viện bên ngoài.
3. **Tích hợp MoveIt 2 (Trajectory Bridge)**:
   - Node cầu nối `fuzzy_trajectory_bridge` nội suy quỹ đạo an toàn từ MoveIt 2 (TOTP) và truyền thành các setpoint liên tục cho bộ Fuzzy.
4. **Tích hợp Nhận thức không gian 3D (Perception & Obstacle Avoidance)**:
   - Sẵn sàng cấu hình Octomap (`sensors_3d.yaml` từ `rx150_motion_common`) để đọc luồng PointCloud từ Camera 3D (ví dụ: Intel RealSense). MoveIt có thể tự động lập quỹ đạo lách qua các vật cản động.
5. **Giao diện Tune tham số trực tiếp (GUI)**:
   - Ứng dụng `rx150_tuning_gui.py` (Tkinter, ở `rx150_motion_common`) cho phép tinh chỉnh Gains ($K_e, K_{ed}, K_u, u_{max}$) theo thời gian thực (Live-tuning) và lưu cấu hình trực tiếp vào YAML.
6. **Kiểm thử an toàn & Trực quan hoá**:
   - Thu dữ liệu ROS 2 bag để so sánh A/B (`scripts/rx150_fuzzy_record.sh`). Trực quan hoá realtime và offline qua module dùng chung [data_analysis/](../../../../data_analysis/).

## Cấu trúc thư mục (Packages)

- `config/`: Chứa file YAML cấu hình gain (`rx150_fuzzy_gains.yaml`). Cấu hình động cơ
  (`rx150_motor.yaml`) và Octomap (`sensors_3d.yaml`) nằm ở `rx150_motion_common/config/`.
- *Lưu ý:* phần nhận diện (YOLO/gesture/PCL), camera driver, hand-eye calibration đã được tách sang package `rx150_perception`.
- `launch/`: Các file khởi động tích hợp (chạy độc lập, chạy với MoveIt).
- `scripts/`: Chứa GUI cũ (`rx150_fuzzy_gui.py`), cầu nối gripper (`rx150_gripper_trajectory_bridge.py`) và kịch bản test (`rx150_fuzzy_bridge_test.py`). Cầu nối quỹ đạo TAY dùng chung ở `rx150_motion_common/scripts/rx150_trajectory_bridge.py`.
- `src/`: Mã nguồn C++ Node chính (`fuzzy_node.cpp`) và thư viện mã C sinh từ FIS (`src/fuzzy/`).

## Hướng dẫn sử dụng nhanh

### 1. Khởi động với MoveIt 2 (Khuyên dùng)

Lệnh này khởi động toàn bộ: Driver xs_sdk, Fuzzy Node, Trajectory Bridge, Gripper
Bridge, MoveGroup, Static TF cho Camera và RViz 2.

```bash
source ~/RX150_Hedge_Algebra_Control/source_all.sh
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py
```

Hoặc dùng wrapper ở gốc workspace, vốn đã đặt sẵn tổ hợp cờ đúng:

```bash
./rx150.sh t1     # rồi ./rx150.sh t2 cho perception + TF calib
```

> ⚠️ `fuzzy_node` bật torque + PWM **ngay khi launch** và lái tay về tư thế sleep
> `[0, −1.80, 1.55, 0.8, 0]` bằng một **bước nhảy** (launch ép `enable_profile: False`).
> Chạy `pkill -f xs_sdk` trước, dọn thoáng quanh robot.
> Quy trình bring-up đầy đủ: [`rx150_pick_place/docs/RUNBOOK.md`](../../apps/rx150_pick_place/docs/RUNBOOK.md).

*(Camera đã nằm trong launch này — `use_camera` mặc định `true`. Không chạy thêm
`realsense2_camera` ở terminal khác: hai driver cùng mở một thiết bị sẽ báo
"Device or resource busy".)*

### 2. Mở GUI tinh chỉnh thông số (Live Tuning)

```bash
ros2 run rx150_motion_common rx150_tuning_gui.py --ros-args -p target:=fuzzy
```

GUI dùng chung cho cả ba controller (`target` ∈ `fuzzy | hac | ff`). Bản cũ riêng
của package này còn ở `ros2 run rx150_fuzzy_controller rx150_fuzzy_gui.py`.

- Tab **Setpoint** để điều khiển robot thủ công.
- Tab **Gains** để chỉnh sửa và **Lưu vào yaml** các thông số Fuzzy.

### 3. Trực quan hoá & Giám sát dữ liệu (PlotJuggler)

Hệ thống cung cấp sẵn XML layout dùng chung và các script ghi dữ liệu trong module [data_analysis/](../../../../data_analysis/).

#### Mở PlotJuggler với Layout cấu hình sẵn:
```bash
ros2 run plotjuggler plotjuggler -l ~/RX150_Hedge_Algebra_Control/data_analysis/layouts/fuzzy_plotjuggler_layout.xml
```

#### Các Topic chính được theo dõi trong PlotJuggler:
| Tên Topic trong ROS 2 | Loại Message | Mục đích giám sát |
| :--- | :--- | :--- |
| `/rx150/fuzzy/reference` | `sensor_msgs/msg/JointState` | Vị trí góc đặt ($q_{ref}$) và vận tốc đặt ($\dot{q}_{ref}$) |
| `/rx150/joint_states` | `sensor_msgs/msg/JointState` | Vị trí góc thực tế ($q$) và vận tốc thực ($\dot{q}$) từ Encoder |
| `/rx150/fuzzy/error` | `sensor_msgs/msg/JointState` | Sai số vị trí ($e = q_{ref} - q$) từng khớp |
| `/rx150/fuzzy/edot` | `sensor_msgs/msg/JointState` | Đạo hàm sai số ($\dot{e} = \dot{q}_{ref} - \dot{q}$) |
| `/rx150/fuzzy/effort` | `sensor_msgs/msg/JointState` | Xung PWM điều khiển ($u$) và Momen bù trọng lực |

> Xem tài liệu chi tiết về quy trình Live Streaming, nạp ROS Bag offline, export CSV và vẽ đồ thị xuất bản tại [data_analysis/README.md](../../../../data_analysis/README.md).


## Đọc thêm

| Cần gì | Ở đâu |
| :--- | :--- |
| Thiết kế luật mờ, sinh mã C từ `.fis`, xem mặt 3D | [fuzzy_codegen/README.md](../../../../fuzzy_codegen/README.md) |
| Thang bậc bring-up B0→B6, bảng triệu chứng → nguyên nhân | [RUNBOOK.md](../../apps/rx150_pick_place/docs/RUNBOOK.md) |
| Sơ đồ điều khiển 5 tầng, 3 vòng kín | [docs/so_do_dieu_khien.md](../../../../docs/so_do_dieu_khien.md) |
| Ghi CSV, quy ước topic, vẽ đồ thị | [data_analysis/README.md](../../../../data_analysis/README.md) |
