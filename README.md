<div align="center">

# RX150 · Hedge Algebra Control

**So sánh ba bộ điều khiển trên cánh tay robot thật — Fuzzy Mamdani, Feedforward, và Đại số gia tử (HAC)**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Robot](https://img.shields.io/badge/Robot-Interbotix%20RX150-0A7E8C)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

</div>

> **Nhánh hiện tại: `feat/400hz-fast-sync-read`** — controller loop **400 Hz**, bus Dynamixel **4 Mbps + FastSyncRead (0x8A)**.
> Nhánh `main` chạy ở **100 Hz** với SyncRead tiêu chuẩn và baudrate 1 Mbps.

---

## Mục lục

1. [Ý tưởng](#1-ý-tưởng)
2. [Yêu cầu hệ thống](#2-yêu-cầu-hệ-thống)
3. [Cấu trúc dự án](#3-cấu-trúc-dự-án)
4. [Cài đặt](#4-cài-đặt)
5. [Cách chạy — chia theo tác vụ](#5-cách-chạy--chia-theo-tác-vụ)
   - [A. Kiểm tra offline (không cần robot)](#a-kiểm-tra-offline-không-cần-robot)
   - [B. Bring-up robot + camera](#b-bring-up-robot--camera)
   - [C. Hiệu chuẩn](#c-hiệu-chuẩn)
   - [D. Chạy ứng dụng](#d-chạy-ứng-dụng)
   - [E. Thu dữ liệu + vẽ đồ thị](#e-thu-dữ-liệu--vẽ-đồ-thị)
   - [F. So sánh A/B (HAC vs Fuzzy)](#f-so-sánh-ab-hac-vs-fuzzy)
   - [G. Kiểm thử + chẩn đoán](#g-kiểm-thử--chẩn-đoán)
6. [Kiến trúc điều khiển](#6-kiến-trúc-điều-khiển)
7. [Tài liệu chuyên sâu](#7-tài-liệu-chuyên-sâu)
8. [Giấy phép & trích dẫn](#8-giấy-phép--trích-dẫn)

---

## 1. Ý tưởng

Cánh tay **Interbotix ReactorX-150** mặc định đóng vòng vị trí **bên trong firmware** của
động cơ Dynamixel. Vòng đó không quan sát được, không sửa được, và vì thế không so sánh
được.

Dự án này **kéo vòng kín ra ngoài**: động cơ chuyển sang `operating_mode: pwm`, firmware
chỉ đóng vai trò khuếch đại công suất, còn toàn bộ vòng kín vị trí chạy trên host ở
**400 Hz** (nhánh này; `main` chạy 100 Hz). Luật điều khiển trở thành **tháo ra lắp vào được** — và đó là chỗ để đặt
câu hỏi.

Vào đúng vị trí đó, kho này cắm **ba luật khác nhau**:

| Bộ điều khiển | Luật | Feedforward |
| :--- | :--- | :--- |
| **Fuzzy** (`rx150_fuzzy_controller`) | Fuzzy Mamdani type-1, sinh từ `.fis` sang C thuần | Bù trọng lực `g(q)` (Pinocchio RNEA) |
| **FF** (`rx150_ff_controller`) | **Cùng** bảng luật mờ đó | `Kv·q̇ + Ka·q̈` — động học thuần |
| **HAC** (`rx150_hac_controller`) | Đại số gia tử, mặt tuyến tính ba hệ số `a, b, c` | Bù trọng lực `g(q)` |

**Câu hỏi trung tâm:** Bộ HAC thu gọn **toàn bộ** bảng luật Mamdani xuống còn **ba số
vô hướng** — nếu HAC bám ngang Fuzzy, thì bảng luật mờ đã không đóng góp gì ngoài một
mặt phi tuyến.

Ba bộ điều khiển **không chạy không tải** — bên trên chúng là ứng dụng hoàn chỉnh:
gắp ống nghiệm cắm lên giá bằng YOLO, tương tác người–máy bằng cử chỉ tay MediaPipe,
hiệu chuẩn hand-eye bằng AprilTag.

---

## 2. Yêu cầu hệ thống

### Phần cứng

| Thành phần | Ghi chú |
| :--- | :--- |
| Interbotix RX150 + U2D2 | Động cơ Dynamixel XL430/XM430 |
| Intel RealSense D435i | Chỉ cần khi chạy perception/hiệu chuẩn |
| AprilTag trên đế robot + trên tay gắp | Cho hiệu chuẩn hand-eye |

### Phần mềm

| Thành phần | Phiên bản |
| :--- | :--- |
| Ubuntu | 22.04 |
| ROS 2 | Humble (desktop-full) |
| Công cụ | `python3-vcstool`, `python3-colcon-common-extensions` |

### Overlay workspace (tùy chọn — chỉ cần khi chạy hiệu chuẩn)

| Workspace | Tác dụng |
| :--- | :--- |
| `~/apriltag_ws` | AprilTag detector cho ROS 2 |
| `~/easy_handeye2_ws` | Hiệu chuẩn hand-eye |

> Hai overlay này là **tuỳ chọn**: `source_all.sh` kiểm tra sự tồn tại trước khi source, thiếu thì bỏ qua.

---

## 3. Cấu trúc dự án

### 3.1. Cây thư mục tổng quan

```
RX150_Hedge_Algebra_Control/
│
├── rx150.sh                 ← ĐIỂM VÀO DUY NHẤT — mọi chế độ chạy đều qua đây
├── source_all.sh            ← Source 4 overlay workspace
├── rx150.repos              ← Pin phiên bản vendor (Interbotix upstream)
├── read_tf_offset.sh        ← Đo lệch TF camera vs encoder
│
├── src/                     ← MÃ NGUỒN CHÍNH
│   ├── vendor/              ← Interbotix upstream (KHÔNG commit, khôi phục bằng setup_vendor.sh)
│   └── rx150/               ← Code của dự án, phân theo 5 tầng IRROS
│       ├── rx150/                     Metapackage
│       ├── controllers/               ── Tầng Control ──
│       │   ├── rx150_fuzzy_controller   Fuzzy Mamdani type-1 + bù trọng lực
│       │   ├── rx150_ff_controller      Fuzzy PD + feedforward vận tốc/gia tốc
│       │   └── rx150_hac_controller     HAC tuyến tính + Ruckig + bù trọng lực
│       ├── rx150_toolbox/             ── Tầng Application Support ──
│       │   ├── rx150_modules            Thư viện dùng chung: IK, MoveIt executor, gripper, scene
│       │   ├── rx150_motion_common      Hạ tầng controller: trajectory bridge, tuning, gravity ID
│       │   └── rx150_perception         YOLO + cử chỉ tay + AprilTag hand-eye
│       └── apps/                      ── Tầng Application ──
│           ├── rx150_pick_place         Gắp ống nghiệm lên giá; gắp vật bằng cử chỉ
│           └── rx150_hri                Tương tác người–máy (task ↔ executor)
│
├── tools/                   ← CÔNG CỤ HỖ TRỢ
│   ├── build.sh               Build workspace (gỡ bẫy setuptools/packaging)
│   ├── setup_vendor.sh        Kéo vendor Interbotix + init submodule
│   ├── check_vendor_patches.sh  Kiểm patch đã apply chưa
│   ├── collect_diag.sh        Chụp trạng thái hệ thống → .tar.gz
│   ├── record_tube_run.py     Ghi dữ liệu 1 lần chạy gắp → tuning_runs/
│   ├── plot_tube_run.py       Vẽ đồ thị offline từ dữ liệu record
│   ├── dxl_baud.py            Đổi baudrate Dynamixel
│   ├── record_pickplace.sh    Ghi rosbag lúc chạy pick & place
│   └── ab_compare/            Pipeline so sánh A/B (HAC vs Fuzzy)
│       ├── ab_run.py            Ra lệnh + ghi dữ liệu (cần robot)
│       ├── ab_report.py         Chỉ số + thống kê + đồ thị (offline)
│       ├── bench_law.py         Đo chi phí tính toán luật (không cần robot)
│       └── tasks/               Chuỗi waypoint cho phiên A/B
│
├── docs/                    ← TÀI LIỆU CHUYÊN SÂU
│   ├── README.md              Mục lục toàn bộ tài liệu
│   ├── cai_dat.md             Cài đặt chi tiết, gỡ rối
│   ├── cau_truc_kho.md        Cây thư mục đầy đủ, 5 tầng IRROS
│   ├── so_do_dieu_khien.md    Sơ đồ node/topic/action, tần số
│   ├── lenh_chay_tube_rack.md Sổ lệnh dán thẳng vào terminal
│   ├── tuning/                Kết quả đo & quy trình hiệu chuẩn
│   └── lich_su/               Nhật ký đưa lên phần cứng (lịch sử, KHÔNG mô tả hệ hiện tại)
│
├── fuzzy_codegen/           ← SINH MÃ C TỪ FILE .fis
│   ├── fis2c.py               Trình dịch .fis → .c/.h
│   ├── fuzzy_type1.fis        Bản chép file .fis (đừng sửa ở đây)
│   ├── fuzzy_type1.c/.h       Mã C sinh ra (đừng sửa tay)
│   ├── gen_surface.py         Quét lưới → surface.json
│   └── build_surface_html.py  surface.json → fuzzy_surface.html (3D viewer)
│
├── data_analysis/           ← GHI CSV + VẼ ĐỒ THỊ ĐIỀU KHIỂN
│   ├── csv_logger.py          Node ROS 2: subscribe telemetry → CSV
│   ├── plot_control_csv.py    Vẽ đồ thị offline từ CSV
│   └── layouts/               Layout PlotJuggler dựng sẵn
│
├── module_tests/            ← SMOKE-TEST THEO TẦNG (không phụ thuộc colcon)
│   ├── run_test.py            Runner chạy test
│   ├── hardware/              Test camera, motor, gripper
│   ├── fuzzy_controller/      Test luật mờ, gains, trajectory bridge
│   ├── moveit/                Test planning, IK, thực thi trajectory
│   └── perception/            Test YOLO, TF, xử lý point cloud
│
├── patches/                 ← VÁ VENDOR (chỉ nhánh 400 Hz)
│   └── vendor/                FastSyncRead 0x8A + baudrate 4 Mbps cho Dynamixel
│
├── tuning_runs/             ← DỮ LIỆU TỪNG LẦN ĐO (CSV thô KHÔNG commit)
│
├── .github/                 ← Template issue/PR cho GitHub
├── CHANGELOG.md             ← Lịch sử thay đổi
├── CONTRIBUTING.md          ← Hướng dẫn đóng góp
├── CITATION.cff             ← Trích dẫn cho bài báo
└── LICENSE                  ← BSD 3-Clause
```

### 3.2. Hai nhánh — 100 Hz vs 400 Hz

| | `main` | `feat/400hz-fast-sync-read` ← **nhánh hiện tại** |
| :--- | :--- | :--- |
| Controller loop | 100 Hz | **400 Hz** |
| Baudrate bus Dynamixel | 1 Mbps | **4 Mbps** |
| Giao thức đọc encoder | SyncRead tiêu chuẩn | **FastSyncRead (0x8A)** — gom gói phản hồi, RTT < 1 ms |
| Patches vendor | Không | `patches/vendor/` — 2 bản vá cho `dynamixel_workbench_toolbox` + `interbotix_xs_driver` |
| Công cụ đổi baudrate | Không cần | `tools/dxl_baud.py` — đổi EEPROM motor sang 4 Mbps |
| So sánh A/B | Chưa có | `tools/ab_compare/` — pipeline đo + thống kê + đồ thị |

> ⚠️ **Nhánh 400 Hz yêu cầu motor đã được đổi baudrate sang 4 Mbps** bằng `tools/dxl_baud.py` trước khi chạy. Nhánh `main` không cần bước này.

### 3.3. Kiến trúc 5 tầng IRROS

```
Application         rx150_pick_place · rx150_hri           ← quyết định gắp GÌ, đặt Ở ĐÂU
Application Support rx150_modules · rx150_motion_common · rx150_perception
Control             rx150_fuzzy_controller │ rx150_ff_controller │ rx150_hac_controller
Driver              interbotix_xs_sdk · realsense2_camera · apriltag_ros
Hardware            RX150 (XL430/XM430) + U2D2 · RealSense D435i · AprilTag
```

**Tầng trên gọi tầng dưới, không bao giờ ngược lại.** Ba bộ điều khiển nằm cùng một chỗ
vì chúng là ba biến thể của một bài toán — chọn một trong ba, không phải chạy cả ba.

### 3.4. Mô tả chi tiết từng thành phần

#### Controllers (Tầng Control)

| Package | Mô tả | File chính |
| :--- | :--- | :--- |
| `rx150_fuzzy_controller` | Fuzzy Mamdani PD + bù trọng lực Pinocchio RNEA. Mã C sinh tự động từ `.fis` | `fuzzy_node.cpp`, `fuzzy_type1.c/.h` |
| `rx150_ff_controller` | Cùng luật mờ + feedforward vận tốc/gia tốc (không cần mô hình động lực học) | `ff_node.cpp`, `fuzzy_type1.c/.h` |
| `rx150_hac_controller` | HAC tuyến tính 3 hệ số `a,b,c` + Ruckig + bù trọng lực (model đã hiệu chuẩn) | `hac_node.cpp`, `hac.c/.h` |

#### Application Support (Tầng Hỗ trợ)

| Package | Mô tả |
| :--- | :--- |
| `rx150_modules` | IK giải tích 5-DoF, MoveIt executor, gripper xác nhận kẹp, planning scene, skill primitives |
| `rx150_motion_common` | Trajectory bridge (JTA), Ruckig trajectory, tuning GUI, gravity/friction identification, motor config |
| `rx150_perception` | YOLO tube detector, hand gesture (MediaPipe), point cloud, AprilTag hand-eye, rack calibration |

#### Applications (Tầng Ứng dụng)

| Package | Mô tả |
| :--- | :--- |
| `rx150_pick_place` | Gắp ống nghiệm cắm lên giá (YOLO); gắp vật được chỉ bằng cử chỉ tay. State machine hoàn chỉnh |
| `rx150_hri` | Tương tác người–máy: tách task (quyết định) ↔ executor (chấp hành) qua topic |

#### Công cụ ngoài ROS

| Thư mục | Mô tả |
| :--- | :--- |
| `fuzzy_codegen/` | Sinh `fuzzy_type1.c` từ file `.fis` (MATLAB FIS → C99). Xem mặt điều khiển 3D |
| `data_analysis/` | Ghi CSV telemetry + vẽ đồ thị offline. Layout PlotJuggler cho giám sát realtime |
| `module_tests/` | Smoke-test theo tầng, chạy bằng `run_test.py`, không phụ thuộc colcon |
| `tools/ab_compare/` | Pipeline so sánh A/B: chạy → ghi → báo cáo thống kê + đồ thị |

---

## 4. Cài đặt

### Bước 1: Clone repository

```bash
git clone https://github.com/nguyenbinh-shark/RX150_Hedge_Algebra_Control.git ~/RX150_Hedge_Algebra_Control
cd ~/RX150_Hedge_Algebra_Control
```

### Bước 2: Kéo vendor Interbotix (bắt buộc trước khi build)

```bash
./tools/setup_vendor.sh
```

> Script này làm hai việc:
> 1. `vcs import src/vendor < rx150.repos` — kéo mã nguồn Interbotix theo đúng phiên bản đã pin
> 2. `vcs custom ... submodule update --init --recursive` — init submodule lồng (thiếu thì build hỏng vì thiếu header)
>
> `src/vendor/` **không nằm trong git** — mọi máy khôi phục lại bằng lệnh này.

### Bước 3: Build workspace

```bash
./tools/build.sh
```

> **Vì sao phải dùng `tools/build.sh` chứ không gọi thẳng `colcon build`:**
> - **Bẫy 1 — setuptools vs packaging**: `build.sh` đặt `PYTHONNOUSERSITE=1` để tránh xung đột `setuptools 84` (trong `~/.local`) với `packaging 21.3` (từ apt).
> - **Bẫy 2 — overlay**: Script tự source đủ 4 overlay trước khi build.
>
> Build riêng một package:
> ```bash
> ./tools/build.sh --packages-select rx150_modules
> ```

### Bước 4: Source môi trường

```bash
source source_all.sh
```

> Thứ tự source: `/opt/ros/humble` → `~/apriltag_ws` → `~/easy_handeye2_ws` → workspace này.
>
> Script có cảnh báo nếu phát hiện `~/interbotix_ws` đang che (shadow) package đã vá trong repo.

---

## 5. Cách chạy — chia theo tác vụ

Mọi chế độ chạy đều qua điểm vào duy nhất [`rx150.sh`](rx150.sh). Gọi không tham số để xem danh sách đầy đủ.

> ⚠️ **Ở PWM mode, mất node là tay rơi** — firmware không còn giữ vị trí. Luôn tuân thủ thứ tự bring-up:
> `reach → t1 → t2 → [hiệu chuẩn] → check → dry-fake → tubes`

---

### A. Kiểm tra offline (không cần robot)

Chạy được **ngay sau khi build**, chưa cần cắm robot hay camera.

```bash
# Kiểm tầm với + kiểm config — tất cả điểm trong YAML có nằm trong workspace không
./rx150.sh reach
```
> Chạy `rx150_reach_check.py`: tính FK cho mọi điểm trong `tube_rack_params.yaml` và `pick_place_params.yaml`, kiểm tư thế home. **Exit 0 = mọi điểm OK.**

```bash
# Smoke-test offline: YOLO + hình học hiệu chuẩn giá
./rx150.sh test
```
> Chạy 14 test YOLO + test hình học rack. Không cần ROS graph đang chạy.

---

### B. Bring-up robot + camera

Thứ tự bring-up bắt buộc. **Hỏng ở bậc nào thì dừng ở bậc đó.**

#### B1. Terminal 1 — Robot + MoveIt + Camera

```bash
# Bộ Fuzzy (mặc định):
./rx150.sh t1
```
> Khởi động: `interbotix_xs_sdk` (driver) + `fuzzy_node` (controller 400 Hz) + `trajectory_bridge` + `move_group` (MoveIt) + `realsense2_camera`.
> Robot **BẬT TORQUE** và tự về HOME ngay khi launch.

```bash
# Bộ HAC (thay cho Fuzzy):
./rx150.sh t1-hac
```
> Cùng stack nhưng dùng `hac_node` + model trọng lực đã hiệu chuẩn trên phần cứng.

```bash
# Bộ Fuzzy + bật point cloud (chỉ khi cần PCL/OctoMap, tốn ~295 MB/s):
./rx150.sh t1-pcl
```

#### B2. Terminal 2 — Perception + TF hiệu chuẩn

```bash
./rx150.sh t2
```
> Nạp TF hiệu chuẩn camera (`static_transforms.yaml`) + mở RViz.
> **Không chạy YOLO** — YOLO được bật riêng trong launch task (ứng dụng).

#### B3. Smoke-test trên phần cứng

```bash
./rx150.sh check
```
> Ba bài test không phát lệnh tới robot:
> 1. `joint_states_test.py` — kiểm `joint_states` có publish ≥ 50 Hz
> 2. `action_servers_test.py` — kiểm action server MoveIt sống
> 3. `tf_and_detection_test.py` — kiểm TF chain + detector

---

### C. Hiệu chuẩn

#### C1. Hiệu chuẩn camera (hand-eye bằng ArmTag)

```bash
# Mở GUI Snap Pose — đưa AprilTag vào khung hình, chỉnh Snapshots=10, bấm Snap Pose
./rx150.sh calib
```
> Kết quả lưu vào `rx150_perception/config/static_transforms.yaml`. Xong thì Ctrl+C.

#### C2. Hiệu chuẩn vị trí giá ống nghiệm (Rack)

```bash
# Snap vị trí giá → rack_pose.yaml
./rx150.sh rack-calib
```
> Tag AprilTag trên giá phải nằm trong khung hình camera. Cần T1 + T2 đang chạy.

```bash
# Kiểm bằng mắt: RViz + ảnh chồng 4 lỗ lên hình camera (không ghi file)
./rx150.sh rack-gui
```
> Lục = đang đo. Cam = đã lưu. 4 vòng tròn phải nằm ĐÚNG trên 4 miệng lỗ thật.

```bash
# Theo dõi giá có bị xê dịch không (đo liên tục, KHÔNG ghi file)
./rx150.sh rack-watch
```

```bash
# Phát TF tĩnh tube_rack (không cần camera)
./rx150.sh rack-pub
```

#### C3. Hiệu chuẩn tag trên tay gắp

```bash
# Đo vị trí tag trên tay gắp (3 lượt × ~5 phút = ~17 phút)
# Cần: T1 + T2 + ./rx150.sh eetag đang chạy
./rx150.sh eetag-tagcal
```
> Quét `wrist_rotate ±1.2 rad`, 3 lượt khác động lực học, gộp kết quả tự cài vào `ee_tag_offset.yaml`.

#### C4. Hiệu chuẩn tổng hợp (tag + camera giải chung — khuyên dùng)

```bash
# 3 lượt tagcal + 1 lượt grid = ~22 phút
# Giải chung AX=ZB → cài cả ee_tag_offset.yaml lẫn static_transforms.yaml
./rx150.sh onecal
```
> Thay cho `tagcal` rồi `eetag-calib` riêng lẻ — giải chung thì tag và camera khớp nhau theo cùng một nghiệm.

#### C5. Các lệnh hiệu chuẩn phụ

```bash
# Bật detector AprilTag liên tục cho tag trên tay gắp (Terminal 3)
./rx150.sh eetag

# Hiệu chuẩn lại TF camera bằng tag tay gắp (39 tư thế, ~5 phút)
./rx150.sh eetag-calib

# Đo lệch TF: camera nhìn tag vs encoder URDF
./read_tf_offset.sh

# Xem TF perception + rack trong cùng RViz
./rx150.sh t2-rack
```

---

### D. Chạy ứng dụng

#### D1. Gắp ống nghiệm lên giá

```bash
# B4: Chạy thử KHÔNG cần camera — bơm 1 ống giả, chạy hết state machine
./rx150.sh dry-fake

# B5: Chạy thử — chỉ IK + log, không cử động
./rx150.sh dry

# B5: Gắp ống nghiệm THẬT (YOLO detector)
./rx150.sh tubes
```
> Cần T1 + T2 đang chạy. Detector YOLO tự bật trong launch task.

```bash
# Gắp ống nghiệm KHÔNG qua MoveIt (motion_backend=direct)
./rx150.sh tubes-direct
```
> Không có tránh vật cản — an toàn dựa vào waypoint. Nhanh hơn, không cần `move_group`.

#### D2. Gắp vật bằng cử chỉ tay

```bash
./rx150.sh gesture
```
> Chỉ tay vào vật cần gắp, dấu OK để xác nhận.

#### D3. Tất cả trong một terminal

```bash
./rx150.sh all
```
> Fuzzy + MoveIt + Perception + YOLO trong MỘT terminal. Task ở terminal khác **PHẢI** thêm `detector:=false`.

---

### E. Thu dữ liệu + vẽ đồ thị

#### E1. Ghi dữ liệu một lần chạy (song song với ứng dụng)

```bash
# Ghi CSV: quỹ đạo ee + nhận diện + toạ độ giá → tuning_runs/<nhãn>_<ts>/
./rx150.sh record
```
> Không ra lệnh cho robot — an toàn chạy song song với `tubes`. Muốn có đường tag (camera nhìn tay gắp) thì bật `./rx150.sh eetag` trước.

#### E2. Vẽ đồ thị offline

```bash
# Vẽ từ thư mục record
./rx150.sh plot tuning_runs/<thư_mục_record>
```
> Không cần ROS đang chạy.

#### E3. Giám sát realtime bằng PlotJuggler

```bash
# Cài PlotJuggler (1 lần)
sudo apt install ros-humble-plotjuggler-ros

# Mở với layout Fuzzy
ros2 run plotjuggler plotjuggler -l data_analysis/layouts/fuzzy_plotjuggler_layout.xml

# Mở với layout HAC
ros2 run plotjuggler plotjuggler -l data_analysis/layouts/hac_plotjuggler_layout.xml
```
> Sau khi mở: vào Streaming → ROS2 Topic Subscriber → Start → tick topic.

#### E4. Ghi CSV telemetry thủ công

```bash
cd data_analysis
python3 csv_logger.py --ros-args -p controller_prefix:=fuzzy -p robot_name:=rx150
# Ctrl+C để dừng → file: fuzzy_data_YYYYMMDD_HHMMSS.csv

# Vẽ đồ thị từ CSV
python3 plot_control_csv.py <file.csv>
python3 plot_control_csv.py <file.csv> --joints waist shoulder --save output.png
```

#### E5. Bench cam vs encoder (AprilTag trên tay gắp)

```bash
# So vị trí: lệnh vs encoder vs camera. Robot ĐI QUA bộ pose tĩnh
./rx150.sh eetag-hold

# Chỉ quan sát, không ra lệnh (an toàn khi task đang chạy)
./rx150.sh eetag-watch

# Test bám quỹ đạo mô phỏng gắp ống trên giá
./rx150.sh eetag-pick
```

---

### F. So sánh A/B (HAC vs Fuzzy)

Pipeline 3 bước: **chạy HAC → chạy Fuzzy → xử lý số liệu**. Chi tiết: [`tools/ab_compare/README.md`](tools/ab_compare/README.md).

#### F1. Kiểm tra task (không cần robot)

```bash
./rx150.sh ab-run --task pick_cycle --check
```
> Kiểm giới hạn khớp, độ cao EE, hold đủ dài.

#### F2. Chạy A/B trên robot

```bash
# ── Terminal 1: Robot + HAC (KHÔNG MoveIt, KHÔNG camera) ──
./rx150.sh ab-hac

# ── Terminal 2: Ra lệnh + ghi dữ liệu ──
./rx150.sh ab-run --ctrl hac --task pick_cycle --session pc1 --trials 5

# Ctrl+C terminal 1, rồi đổi sang Fuzzy:

# ── Terminal 1: Robot + Fuzzy ──
./rx150.sh ab-fuzzy

# ── Terminal 2: ──
./rx150.sh ab-run --ctrl fuzzy --task pick_cycle --session pc1 --trials 5
```
> Cùng `--session` = cùng thư mục. Chạy xen kẽ HAC 3 → Fuzzy 3 → HAC 3 → Fuzzy 3 để giảm sai lệch hệ thống.

#### F3. Xử lý số liệu + đồ thị

```bash
./rx150.sh ab-report tuning_runs/ab_pc1
./rx150.sh ab-report tuning_runs/ab_pc1 --drop-first 1 --band-deg 0.5
```
> Sinh `report/`: `report.md` (bảng + thống kê + Cohen d + p-value), `fig_tracking.png`, `fig_trials.png`, `fig_joints.png`, các file CSV.

#### F4. Đo chi phí tính toán (không cần robot)

```bash
./rx150.sh ab-bench
```
> Biên dịch `hac.c` vs `fuzzy_type1.c` vào khung đo, đo ns/lần gọi ở `-O0` và `-O2`.

---

### G. Kiểm thử + chẩn đoán

```bash
# Smoke-test theo tầng
./rx150.sh check

# Smoke-test offline (YOLO + hình học giá)
./rx150.sh test

# Chụp trạng thái hệ thống → diag_<ts>.tar.gz (gửi kèm báo lỗi)
./rx150.sh diag
```

#### Chạy test riêng lẻ

```bash
# Liệt kê tất cả test
python3 module_tests/run_test.py --list

# Chạy 1 test cụ thể
python3 module_tests/run_test.py hardware/joint_states_test.py

# Truyền tham số cho test
python3 module_tests/run_test.py hardware/joint_states_test.py -- --seconds 5 --min-hz 80

# Xem camera + YOLO trực tiếp
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py
```

---

## 6. Kiến trúc điều khiển

### Sơ đồ tổng quan

```
              MoveIt (move_group)
                     │ FollowJointTrajectory action
                     ▼
          trajectory_bridge (400 Hz)
            │ sinh q_ref, q̇_ref bằng Ruckig
            ▼
    ┌───────────────────────────────┐
    │  CHỌN MỘT TRONG BA:          │
    │  fuzzy_node  │ ff_node  │ hac │
    │     ↓             ↓        ↓  │
    │  e, ė → Mamdani → u        │  │
    │  e, ė → Mamdani + Kv·q̇    │  │
    │  e, ė → a·e + b·ė → u     │  │
    │  + bù trọng lực g(q)        │
    └──────────────┬────────────────┘
                   │ PWM command (−1023…1023)
                   ▼
            interbotix_xs_sdk
                   │
                   ▼
            Dynamixel XL430/XM430 (PWM mode)
```

### Tần số hoạt động

| Thành phần | Tần số |
| :--- | :--- |
| `joint_states` (encoder) | 400 Hz |
| Controller loop (fuzzy/ff/hac) | 400 Hz |
| Trajectory bridge (Ruckig) | 400 Hz |
| Debug publish (ref/err/effort) | 50 Hz |
| Camera color/depth | 30 Hz |
| YOLO detector | 4–5 Hz |

### Luật điều khiển HAC

$$u \;=\; \frac{2c}{3a}\,e \;+\; \frac{c}{3b}\,\dot{e}$$

Ba hệ số `a`, `b`, `c` chỉnh được trực tiếp khi robot đang chạy:

```bash
ros2 param set /rx150/hac_node a 0.3
ros2 param set /rx150/hac_node b 12.0
ros2 param set /rx150/hac_node c 1200.0
```

---

## 7. Tài liệu chuyên sâu

README này là bản tổng hợp. Mọi tài liệu chuyên sâu nằm ở [`docs/`](docs/README.md):

### Bắt đầu

| Tài liệu | Nội dung |
| :--- | :--- |
| [cai_dat.md](docs/cai_dat.md) | Cài đặt chi tiết, vendor, bẫy setuptools/packaging |
| [cau_truc_kho.md](docs/cau_truc_kho.md) | Cây thư mục đầy đủ, 5 tầng IRROS, luật phụ thuộc |
| [lenh_chay_tube_rack.md](docs/lenh_chay_tube_rack.md) | Sổ lệnh dán thẳng vào terminal |
| [RUNBOOK.md](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md) | Thang bậc bring-up B0→B6, tiêu chí GO/NO-GO |

### Điều khiển

| Tài liệu | Nội dung |
| :--- | :--- |
| [so_do_dieu_khien.md](docs/so_do_dieu_khien.md) | Sơ đồ node/topic/action, tần số, 3 vòng kín |
| [rx150_fuzzy_controller](src/rx150/controllers/rx150_fuzzy_controller/README.md) | Fuzzy Mamdani + bù trọng lực |
| [rx150_hac_controller](src/rx150/controllers/rx150_hac_controller/README.md) | HAC 3 hệ số, hiệu chuẩn trọng lực, số đo phần cứng |
| [rx150_ff_controller](src/rx150/controllers/rx150_ff_controller/README.md) | Fuzzy PD + feedforward động học |
| [fuzzy_codegen/](fuzzy_codegen/README.md) | Sinh mã C từ `.fis`, xem mặt 3D |

### Đo đạc & hiệu chuẩn

| Tài liệu | Nội dung |
| :--- | :--- |
| [tuning/](docs/tuning/README.md) | Kết quả đo, quy trình hiệu chuẩn A→B→C |
| [data_analysis/](data_analysis/README.md) | Quy ước topic/CSV, ghi và vẽ đồ thị |
| [tools/ab_compare/](tools/ab_compare/README.md) | Pipeline so sánh A/B: chạy, chỉ số, công bằng |

### Nhận diện & ứng dụng

| Tài liệu | Nội dung |
| :--- | :--- |
| [rx150_pick_place](src/rx150/apps/rx150_pick_place/README.md) | Gắp ống nghiệm; cử chỉ tay |
| [rx150_modules](src/rx150/rx150_toolbox/rx150_modules/README.md) | IK, MoveIt executor, gripper, scene |
| [rx150_perception](src/rx150/rx150_toolbox/rx150_perception/docs/PERCEPTION_GUIDE.md) | Hand-eye, PCL tuning, YOLO |

---

## 8. Giấy phép & trích dẫn

BSD 3-Clause — xem [LICENSE](LICENSE). `src/vendor/` giữ giấy phép gốc của Interbotix.
Nếu dùng kho này trong công bố khoa học, xem [CITATION.cff](CITATION.cff).
