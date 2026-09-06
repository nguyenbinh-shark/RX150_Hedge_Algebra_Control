# rx150_hac_controller — HAC tuyến tính + Ruckig + bù trọng lực

**Tầng IRROS:** Control · **Ngôn ngữ:** C++ (`hac_node`)

Bộ điều khiển PWM vòng kín dùng mặt điều khiển **HAC tuyến tính** thay cho luật mờ, giữ
nguyên phần còn lại của kiến trúc (profile Ruckig + bù trọng lực Pinocchio) để so sánh
công bằng với [`rx150_fuzzy_controller`](../rx150_fuzzy_controller/README.md).

Mặt điều khiển có ba hệ số vô hướng `a`, `b`, `c` (live-tunable qua ROS parameter), thay
cho toàn bộ bảng luật Mamdani. Ý nghĩa của việc này: nếu HAC bám ngang fuzzy thì phần
"mờ" không đóng góp gì ngoài một mặt phi tuyến — đó chính là câu hỏi mà package này dựng
ra để trả lời.

## Cấu trúc

| Thư mục | Nội dung |
| :--- | :--- |
| `src/hac_node.cpp` | Node chính, 100 Hz: `joint_states` → HAC + g(q) → `commands/joint_group` |
| `src/hac/hac.c` | Mặt điều khiển HAC thuần C |
| `src/gravity_comp.cpp` | Bù trọng lực bằng Pinocchio RNEA từ URDF |
| `config/rx150_hac_gains.yaml` | `a/b/c`, `error_limit`, `error_dot_limit`, `u_max`, hệ số N·m→PWM |
| `config/rx150_hac_gains_safe.yaml` | Bộ gain rón rén, dùng khi thử trên máy thật lần đầu |
| `launch/hac_control.launch.py` | Chỉ controller (không MoveIt) |
| `launch/hac_moveit.launch.py` | Bring-up đầy đủ kèm MoveIt + camera |

`error_limit` và `error_dot_limit` được đặt bằng `1/Ke` và `1/Ked` của bộ fuzzy — đây là
điều kiện để hai bộ so sánh được với nhau, đổi một bên thì phải đổi bên kia.

## Chạy

```bash
# Lần đầu trên máy thật: dùng bộ gain an toàn
ros2 launch rx150_hac_controller hac_control.launch.py \
    gains_file:=$(ros2 pkg prefix rx150_hac_controller)/share/rx150_hac_controller/config/rx150_hac_gains_safe.yaml

# Đầy đủ với MoveIt
ros2 launch rx150_hac_controller hac_moveit.launch.py
```

> ⚠️ Bật torque và đưa tay về home ngay khi khởi động.

## Phân tích

```bash
ros2 run rx150_motion_common plot_hac_velocity_analysis.py   # mặt 3D 4 góc phần tư, đường cắt 2D
ros2 run rx150_motion_common compare_fuzzy_vs_hac.py         # overlay fuzzy vs HAC
```

Kết quả tham chiếu đã lưu trong [docs/tuning/](../../../../docs/tuning/); phần lý thuyết ma sát
xem [docs/tuning/friction_hac.md](../../../../docs/tuning/friction_hac.md).
