# rx150_motion_common — hạ tầng & công cụ dùng chung cho các bộ điều khiển

**Tầng IRROS:** Application Support · **Ngôn ngữ:** Python

Nơi chứa những thứ cả ba bộ điều khiển (`fuzzy`, `ff`, `hac`) đều cần, để chúng không phải
copy của nhau. **Package này cố tình không có thư viện C++**: mỗi controller tự biên dịch
engine riêng, chỉ dùng chung phần hạ tầng Python + cấu hình.

## Cấu trúc

| Đường dẫn | Nội dung |
| :--- | :--- |
| `scripts/rx150_trajectory_bridge.py` | `FollowJointTrajectory` action server → setpoint. Cầu nối MoveIt ↔ controller |
| `scripts/rx150_tuning_gui.py` | GUI Tkinter tune gain trực tiếp khi đang chạy, lưu thẳng vào YAML |
| `scripts/rx150_tuning_session.py` | Ghi một phiên tune: `data.csv` 46 cột + `plots/*.png` |
| `scripts/rx150_gravity_id.py` | Nhận dạng hệ số trọng lực (B4: `identify` → `validate`) |
| `scripts/rx150_friction_id.py` | Nhận dạng ma sát theo từng khớp và từng tốc độ |
| `scripts/compare_fuzzy_vs_hac.py` | Mặt điều khiển 3D + overlay quỹ đạo, xuất CSV so sánh |
| `scripts/plot_hac_velocity_analysis.py` | Phân tích 4 góc phần tư (e, ė) của mặt HAC |
| `scripts/rx150_run_compare.py` | Overlay nhiều lần chạy đã ghi |
| `scripts/tuning_lib.py` | Hằng số khớp + đọc/ghi CSV dùng chung cho các script trên |
| `config/rx150_motor.yaml` | Cấu hình động cơ Dynamixel — **một nguồn sự thật** cho mọi controller |
| `config/sensors_3d.yaml` | Cấu hình Octomap cho MoveIt (đọc point cloud từ D435i) |
| `env-hooks/geometric_shapes_shim.sh` | Shim cho overlay `~/ws_moveit` cũ (cần `libgeometric_shapes.so.2.3.2` trong khi Humble ship 2.3.4) |

## Dùng

```bash
ros2 run rx150_motion_common rx150_tuning_gui.py          # tune gain live
ros2 run rx150_motion_common rx150_gravity_id.py identify # nhận dạng trọng lực
ros2 run rx150_motion_common compare_fuzzy_vs_hac.py      # so sánh A/B
```

Các script vẽ đồ thị ghi kết quả vào **thư mục đang đứng** (đổi bằng `RX150_PLOT_DIR`),
không ghi vào `scripts/`. Kết quả tham chiếu: [docs/tuning/](../../../../docs/tuning/).

## Lưu ý khi sửa

`rx150_motor.yaml` được cả ba controller nạp. Đổi `operating_mode` hay giới hạn ở đây là
đổi cho **tất cả** — đó là chủ ý, đừng fork thành bản riêng cho từng controller.
