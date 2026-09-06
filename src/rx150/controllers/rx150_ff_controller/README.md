# rx150_ff_controller — Fuzzy PD + feedforward vận tốc/gia tốc

**Tầng IRROS:** Control · **Ngôn ngữ:** C++ (`ff_node`) + Python (bridge)

Bộ điều khiển PWM vòng kín chạy song song và **cùng cấu trúc** với
[`rx150_fuzzy_controller`](../rx150_fuzzy_controller/README.md), khác đúng một chỗ — cũng là
lý do package này tồn tại:

|                | `rx150_fuzzy_controller` | **`rx150_ff_controller`** |
| :--- | :--- | :--- |
| Vòng phản hồi  | Fuzzy PD (Mamdani type-1) | Fuzzy PD (**cùng** `fuzzy_type1.c`) |
| Feedforward    | Bù trọng lực `g(q)` qua Pinocchio RNEA | `PWM_ff = Kv·q̇_profile + Ka·q̈_profile` |
| Phụ thuộc nặng | Pinocchio + URDF | **không có** — FF là gain động học thuần |
| Khi đứng yên   | Vẫn giữ được tư thế (g(q) ≠ 0) | q̇=q̈=0 ⇒ FF=0 ⇒ **có droop tĩnh** |

Droop tĩnh **không phải bug** — đó chính là điểm so sánh A/B giữa hai chiến lược
feedforward: bù mô hình động lực học (fuzzy) và bù profile động học (package này).

## Cấu trúc

| Thư mục | Nội dung |
| :--- | :--- |
| `src/ff_node.cpp` | Node chính, 100 Hz: đọc `joint_states` → fuzzy PD + FF → `commands/joint_group` |
| `src/fuzzy/` | `fuzzy_type1.c` sinh từ `.fis` (xem [fuzzy_codegen/](../../../../fuzzy_codegen/)) |
| `config/rx150_ff_gains.yaml` | Gain `Ke/Ked/Ku/u_max` + `Kv/Ka` + giới hạn profile Ruckig |
| `config/rx150_ff.yaml` | Tham số bring-up |
| `launch/ff_moveit.launch.py` | Bring-up đầy đủ: xs_sdk + ff_node + bridge + MoveIt + camera |
| `scripts/ff_trajectory_bridge` | `FollowJointTrajectory` action server → setpoint cho `ff_node` |

Hạ tầng dùng chung (bridge quỹ đạo, tuning GUI, `rx150_motor.yaml`) nằm ở
[`rx150_motion_common`](../../rx150_toolbox/rx150_motion_common/README.md).

## Chạy

```bash
ros2 launch rx150_ff_controller ff_moveit.launch.py \
    use_camera_static_tf:=false use_moveit_rviz:=false
```

> ⚠️ Launch này **bật torque và đưa tay về home** ngay khi khởi động. Dọn chỗ quanh robot trước.

## Tham số cần tune

`Kv` và `Ka` mặc định **bằng 0** — tức là chạy thuần fuzzy PD. Tăng dần khi tay có chuyển
động (q̇, q̈ ≠ 0) để giảm sai số bám. Tune trực tiếp bằng GUI:

```bash
ros2 run rx150_motion_common rx150_tuning_gui.py
```

So sánh A/B với các bộ khác: `ros2 run rx150_motion_common compare_fuzzy_vs_hac.py`
(kết quả ghi vào thư mục đang đứng, xem [docs/tuning/](../../../../docs/tuning/)).
