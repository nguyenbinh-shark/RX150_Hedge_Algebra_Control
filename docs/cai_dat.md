# Cài đặt

> Bản chi tiết của phần "Bắt đầu nhanh" trong [README](../README.md).

## 1. Yêu cầu

| Thành phần | Phiên bản / ghi chú |
| :--- | :--- |
| Hệ điều hành | Ubuntu 22.04 |
| ROS | ROS 2 Humble (desktop-full) |
| Công cụ | `python3-vcstool`, `python3-colcon-common-extensions` |
| Overlay ngoài | `~/apriltag_ws` và `~/easy_handeye2_ws` (dùng cho hiệu chuẩn hand-eye) |
| Phần cứng | Interbotix RX150 + U2D2; Intel RealSense D435i (chỉ cần khi chạy perception) |

Hai overlay ngoài là **tuỳ chọn** nếu chỉ chạy điều khiển: `source_all.sh` và
`tools/build.sh` đều kiểm tra sự tồn tại trước khi source, thiếu thì bỏ qua.

## 2. Clone và dựng vendor

```bash
git clone https://github.com/nguyenbinh-shark/RX150_Hedge_Algebra_Control.git ~/RX150_Hedge_Algebra_Control
cd ~/RX150_Hedge_Algebra_Control
./tools/setup_vendor.sh
```

`src/vendor/` **không nằm trong git**. Phiên bản upstream được pin bằng SHA trong
[`rx150.repos`](../rx150.repos), nên mọi máy dựng lại được đúng một trạng thái.

Script làm hai việc, và **bước thứ hai mới là lý do phải có script**:

1. `vcs import src/vendor < rx150.repos`
2. `vcs custom src/vendor --git --args submodule update --init --recursive`

`interbotix_ros_core` và `interbotix_ros_toolboxes` có **submodule lồng**
(`interbotix_xs_driver`, `dynamixel_workbench_toolbox`, `trossen_slate`,
`ModernRobotics`). Chỉ `vcs import` thôi thì các thư mục đó rỗng và
`interbotix_xs_sdk` build hỏng vì thiếu header.

## 3. Build

```bash
./tools/build.sh                                  # tất cả
./tools/build.sh --packages-select rx150_modules  # tham số chuyển thẳng cho colcon
```

### Vì sao phải dùng `tools/build.sh` chứ không gọi thẳng `colcon build`

**Bẫy 1 — `setuptools` vs `packaging`.** Máy dev này có `setuptools 84` trong `~/.local`
nhưng `packaging 21.3` từ apt. Từ `setuptools ≥ 71` nó không còn vendor `packaging` nữa
mà gọi thẳng `canonicalize_version(strip_trailing_zero=…)` — kwarg chỉ có từ
`packaging 24`. Kết quả: **mọi** package dùng `ament_python_install_package` build hỏng
(`interbotix_xs_msgs`, `interbotix_common_modules`, …).

`tools/build.sh` đặt `PYTHONNOUSERSITE=1` để Python bỏ qua `~/.local` và dùng cặp
`setuptools 59.6.0` + `packaging 21.3` của apt vốn khớp nhau. **Không phải cài hay gỡ gì
trên máy.**

**Bẫy 2 — overlay.** Phải source đủ 4 overlay *trước* khi build, nếu không `apriltag_ros`
và `easy_handeye2` không tìm thấy. Script đã làm sẵn.

## 4. Source môi trường

```bash
source ~/RX150_Hedge_Algebra_Control/source_all.sh
```

Thứ tự: `/opt/ros/humble` → `~/apriltag_ws` → `~/easy_handeye2_ws` → workspace này.

## 5. Kiểm tra không cần robot

```bash
./rx150.sh reach    # bảng tầm với + kiểm config
./rx150.sh test     # smoke-test theo tầng (module_tests/)
```

Hai lệnh này chạy được ngay sau khi build, chưa cần cắm robot hay camera. Chỉ khi cả hai
xanh mới sang thang bậc bring-up trên phần cứng trong
[RUNBOOK](../src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md).

## 6. Gỡ rối thường gặp

| Triệu chứng | Nguyên nhân |
| :--- | :--- |
| `Chưa có src/vendor/` khi build | Chưa chạy `./tools/setup_vendor.sh` |
| Thiếu header khi build `interbotix_xs_sdk` | `vcs import` chạy tay, quên bước submodule |
| `canonicalize_version() got an unexpected keyword argument` | Gọi thẳng `colcon build` thay vì `tools/build.sh` |
| `Device or resource busy` trên camera | Đã có `realsense2_camera` chạy — launch của controller đã bao sẵn camera (`use_camera:=true`) |
| `ros2 param set` báo OK mà không có tác dụng | Tham số chỉ đọc lúc khởi tạo (`enable_gravity_comp`, `gravity_sign`, `gravity_model_source`) — phải relaunch |

Bảng triệu chứng → nguyên nhân đầy đủ cho lúc chạy thật nằm trong
[RUNBOOK](../src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md).
