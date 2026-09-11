# RX150 Workspace — điều khiển, nhận diện và ứng dụng cho cánh tay Interbotix RX150

Workspace ROS 2 Humble cho cánh tay **Interbotix ReactorX-150** (5 bậc tự do + gripper)
kèm camera **RealSense D435i**. Kho này chứa ba nhóm việc:

* **Bộ điều khiển khớp** — ba biến thể chạy song song để so sánh: fuzzy Mamdani có bù
  trọng lực, fuzzy PD + feedforward vận tốc/gia tốc, và HAC tuyến tính.
* **Nhận diện** — YOLO segmentation cho ống nghiệm, cử chỉ tay, lọc point cloud,
  hiệu chuẩn hand-eye bằng AprilTag.
* **Ứng dụng** — gắp–thả ống nghiệm lên giá, và tương tác người–máy bằng cử chỉ.

Kho được tổ chức theo chuẩn [IRROS của Interbotix](https://github.com/Interbotix/interbotix_ros_core)
để code của dự án ghép được vào hệ sinh thái upstream mà không phải sửa.

---

## Cấu trúc kho (Repo Structure)

```
interbotix_ws/
├── rx150.sh                   Điểm vào duy nhất — mọi chế độ chạy đều qua đây
├── rx150.repos                Pin phiên bản vendor Interbotix + third-party
├── source_all.sh              Source 4 overlay (ROS → apriltag → easy_handeye2 → ws này)
├── docs/                      Tài liệu dài; docs/tuning/ giữ kết quả đo & hiệu chuẩn
├── module_tests/              Smoke-test theo tầng, chạy độc lập với colcon
├── tools/                     setup_vendor.sh, build.sh, collect_diag.sh, record_pickplace.sh
├── data_analysis/             Ghi CSV + vẽ đồ thị (PlotJuggler layout)
├── fuzzy_codegen/             Sinh fuzzy_type1.c từ file .fis (MATLAB FIS → C)
└── src/
    ├── vendor/                KHÔNG commit — khôi phục bằng ./tools/setup_vendor.sh
    │   ├── interbotix_ros_core            Driver layer (xs_sdk, xs_driver, xs_msgs)
    │   ├── interbotix_ros_manipulators    Mô tả + MoveIt config cho X-Series
    │   ├── interbotix_ros_toolboxes       Module Python hỗ trợ của Interbotix
    │   └── moveit_visual_tools
    └── rx150/
        ├── rx150/                         Metapackage — depend toàn bộ bên dưới
        ├── controllers/                   ── Control layer ──
        │   ├── rx150_fuzzy_controller       Fuzzy Mamdani type-1 + bù trọng lực (Pinocchio)
        │   ├── rx150_ff_controller          Fuzzy PD + feedforward vận tốc/gia tốc (không bù trọng lực)
        │   └── rx150_hac_controller         HAC tuyến tính + Ruckig + bù trọng lực
        ├── rx150_toolbox/                 ── Application Support layer ──
        │   ├── rx150_modules                Thư viện dùng chung: IK giải tích, MoveIt executor,
        │   │                                gripper có xác nhận kẹp, planning scene, skills
        │   ├── rx150_motion_common          Hạ tầng chung cho controller: trajectory bridge,
        │   │                                tuning GUI, nhận dạng ma sát/trọng lực, cấu hình motor
        │   └── rx150_perception             YOLO + cử chỉ tay + PCL + AprilTag hand-eye
        └── apps/                          ── Application layer ──
            ├── rx150_pick_place             Gắp ống nghiệm lên giá; gắp vật chỉ bằng cử chỉ
            └── rx150_hri                    Tương tác người–máy (task ↔ executor qua topic)
```

Ba tầng như trên là bản áp dụng của cấu trúc ba tầng upstream: *landing page* → *thư mục
nhóm* → *package*.

## Cấu trúc mã nguồn (IRROS)

Năm tầng của **I**nterbotix **R**esearch **R**obotics **O**pen **S**tandard, điền cho RX150:

| Tầng | Nội dung trong kho này |
| :--- | :--- |
| **Hardware** | RX150 (Dynamixel XL430/XM430) qua U2D2; RealSense D435i; AprilTag gắn trên đế |
| **Driver** | `interbotix_xs_sdk`, `interbotix_xs_driver` (vendor); `realsense2_camera`, `apriltag_ros` (bên thứ ba) |
| **Control** | `rx150_fuzzy_controller`, `rx150_ff_controller`, `rx150_hac_controller` — mỗi package có `config/` (gain, giới hạn) + launch bring-up riêng |
| **Application Support** | `rx150_modules` (IK/motion/gripper/scene dùng chung), `rx150_motion_common` (hạ tầng controller), `rx150_perception` (nhận diện) |
| **Application** | `rx150_pick_place`, `rx150_hri` — code người dùng cuối, chỉ gọi xuống tầng dưới |

Quy tắc: **tầng trên gọi tầng dưới, không bao giờ ngược lại**. Ứng dụng mới không được tự
viết lại primitive MoveIt/gripper/scene — dùng `rx150_modules` (xem
[src/rx150/apps/README.md](src/rx150/apps/README.md)).

## Cài đặt

Yêu cầu: Ubuntu 22.04 + ROS 2 Humble, và hai overlay `~/apriltag_ws`, `~/easy_handeye2_ws`.

```bash
git clone <url> ~/interbotix_ws && cd ~/interbotix_ws
./tools/setup_vendor.sh          # vcs import + submodule lồng — BẮT BUỘC trước khi build
./tools/build.sh                 # colcon build --symlink-install (đã xử lý bẫy setuptools)
source install/setup.bash
```

> **Vì sao phải dùng `tools/build.sh` chứ không gọi thẳng `colcon build`:** máy này có
> `setuptools 84` trong `~/.local` nhưng `packaging 21.3` từ apt. `setuptools ≥ 71` không
> còn vendor `packaging` nữa nên gọi `canonicalize_version(strip_trailing_zero=…)` — kwarg
> mà 21.3 không có → **mọi** package dùng `ament_python_install_package` build hỏng
> (`interbotix_xs_msgs`, `interbotix_common_modules`, …). `tools/build.sh` đặt
> `PYTHONNOUSERSITE=1` để dùng cặp `setuptools 59.6.0` + `packaging 21.3` của apt vốn khớp nhau.

## Chạy

Mọi thứ đi qua [`rx150.sh`](rx150.sh); `./rx150.sh` không tham số sẽ in danh sách chế độ.

```bash
./rx150.sh reach      # B0  bảng tầm với + kiểm config — KHÔNG cần robot
./rx150.sh t1         # B1  robot + MoveIt + camera   (torque BẬT, tay tự về home)
./rx150.sh t2         # B2  perception + TF hiệu chuẩn
./rx150.sh check      # B3  smoke-test: joint_states + action server + TF/detection
./rx150.sh dry-fake   # B4  chạy hết state machine với ống giả — không cần camera
./rx150.sh tubes      # B5  gắp ống nghiệm thật
```

Thang bậc B0→B6: **hỏng ở bậc nào thì dừng ở bậc đó**. Chi tiết từng bậc và cách đọc lỗi
nằm trong [RUNBOOK](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md).

## Tài liệu

| Đường dẫn | Nội dung |
| :--- | :--- |
| [RUNBOOK.md](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md) | Thang bậc bring-up, bảng triệu chứng → nguyên nhân |
| [docs/lenh_chay_tube_rack.md](docs/lenh_chay_tube_rack.md) | Sổ lệnh dán thẳng vào terminal: bring-up, hiệu chuẩn, khôi phục motor |
| [PERCEPTION_GUIDE.md](src/rx150/rx150_toolbox/rx150_perception/docs/PERCEPTION_GUIDE.md) | Hiệu chuẩn hand-eye, tune PCL, tham số YOLO |
| [docs/so_do_dieu_khien.md](docs/so_do_dieu_khien.md) | Sơ đồ cấu trúc điều khiển (5 tầng, 3 vòng kín) + bảng chênh lệch so với baseline cũ `key_point/task4_full.py` |
| [docs/huong_dan_chi_tiet.md](docs/huong_dan_chi_tiet.md) | Hướng dẫn tổng thể, dạng bài viết |
| [docs/tuning/](docs/tuning/) | Kết quả đo: mặt điều khiển, đường quỹ đạo, ma sát/trọng lực |

## Đóng góp

Theo đúng yêu cầu của upstream: giữ nguyên cấu trúc thư mục, quy ước đặt tên, và **mỗi
package phải có README riêng**. Package mới đặt đúng tầng IRROS của nó; ứng dụng mới vào
`src/rx150/apps/`.

## Giấy phép

BSD 3-Clause — xem [LICENSE](LICENSE). Phần `src/vendor/` giữ giấy phép gốc của Interbotix
và các bên thứ ba.
