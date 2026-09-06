# rx150_hri — chức năng HRI tự chứa (detect/lựa chọn → lệnh arm)

1 package = 1 chức năng: **toàn bộ logic đọc nhận diện + xử lý + lựa chọn + định trình
tự** nằm trong `hri_task_node`; đầu ra của chức năng = **lệnh vị trí arm cần đến +
kẹp/nhả** gửi qua topic cho executor `hri_motion_node` (MoveIt + IK-oracle + gripper).

```text
rx150_perception (giữ nguyên)                 rx150_hri
├─ yolo_detector ─► /yolo/detected_tubes ──► hri_task_node   (B2 trở đi)
└─ hand_gesture  ─► /hand_gesture/* (B3+)       │ logic chọn vật + định trình tự
                                                 ├─► /hri/cmd_pose     (PoseStamped: xyz + pitch)
                                                 ├─► /hri/cmd_gripper  (Bool: True = kẹp)
                                                 ├─► /hri/cmd_home     (Empty)
                                                 ├─► /hri/set_vel_scale(Float32: 0.05..1)
                                                 └─◄ /hri/status       ("POSE_DONE #12" …)
                                                 │
                                            hri_motion_node (executor: MoveIt qua 'move_action',
                                                             IK-oracle SDK execute=False,
                                                             gripper bridge / PWM fallback)
```

- `hri_task` KHÔNG biết MoveIt; `hri_motion` KHÔNG biết detection → test từng lớp riêng
  bằng `ros2 topic pub`.
- Pitch (DOF định hướng duy nhất của 5-DoF) đóng gói trong quaternion quanh trục Y:
  `pitch = 2·atan2(qy, qw)` (xem `scripts/hri_common.py`).

## Roadmap từng bước

| Bước | Nội dung | Trạng thái |
| ---- | -------- | ---------- |
| B1 | Gắp tại điểm cố định `pick_*` → nhả tại điểm cố định `place_*` (không camera) | **xong** |
| B2 | Camera đọc `/yolo/detected_tubes` → chọn vật ổn định gần base → gắp, nhả place | tới |
| B3 | Gesture chọn vật (tái dùng `/hand_gesture/selected_target` + `ok_sign`) | sau |
| B4 | Handover (OK-sign khi cầm ống → trao cho người) + RViz markers | sau |

## Chạy (BƯỚC 1)

Terminal 1 — motion stack (KHÔNG cần camera ở B1):

```bash
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=false
```

Terminal 2:

```bash
ros2 launch rx150_hri hri.launch.py mode:=fixed perception:=false
```

Chu kỳ: HOME → APPROACH → DESCEND → GRASP → LIFT → TRANSPORT → PLACE → RELEASE →
RETREAT → HOME. Điều chỉnh `pick_*` / `place_*` trong `config/hri_params.yaml` theo vị
trí vật thật (x ≈ 0.2–0.3 m, z ≈ 0.05 m). `auto_start:=false` (yaml) + kích hoạt từng
chu kỳ bằng `ros2 topic pub --once /hri/start std_msgs/msg/Empty {}`.

## Test interface executor thủ công (không cần hri_task)

```bash
ros2 run rx150_hri hri_motion_node.py
# tới điểm (pitch=0 → quaternion identity):
ros2 topic pub --once /hri/cmd_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: rx150/base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.15}}}'
ros2 topic pub --once /hri/cmd_gripper std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /hri/cmd_home std_msgs/msg/Empty '{}'
ros2 topic echo /hri/status      # POSE_DONE #1, GRIP_DONE #2, HOME_DONE #3, IDLE #3...
```

## Topic /hri/*

| Topic | Kiểu | Hướng | Ý nghĩa |
| --- | --- | --- | --- |
| `/hri/cmd_pose` | `geometry_msgs/PoseStamped` | task → motion | tới vị trí xyz + pitch (quaternion-Y), frame `rx150/base_link` |
| `/hri/cmd_gripper` | `std_msgs/Bool` | task → motion | True = kẹp, False = nhả |
| `/hri/cmd_home` | `std_msgs/Empty` | task → motion | về `home_joints` |
| `/hri/set_vel_scale` | `std_msgs/Float32` | task → motion | velocity scale cho cmd_pose kế tiếp |
| `/hri/status` | `std_msgs/String` | motion → task | `<KIND> #<seq>`: READY/IDLE/POSE_DONE/POSE_FAILED/GRIP_DONE/GRIP_FAILED/HOME_DONE/HOME_FAILED/REJECTED |
| `/hri/start` | `std_msgs/Empty` | user → task | kích hoạt 1 chu kỳ khi `auto_start:=false` |

Đồng bộ: seq tăng đơn điệu cho mỗi lệnh hoàn tất/từ chối → task gửi lệnh rồi chờ status
có `seq > seq đã thấy` (timeout `status_timeout_s`).

## Tham số chính (`config/hri_params.yaml`)

- `mode` (`fixed`), `auto_start` (true), `cycles` (1; −1 = vô hạn)
- `pick_x/y/z`, `pick_pitch`, `place_x/y/z`, `place_pitch` — điểm gắp/nhả B1 (TUNE)
- `approach_delta` (0.05), `finger_grasp_offset` (0.02), `retreat_m` (0.10)
- `velocity_scale_cruise` (0.3) / `velocity_scale_delicate` (0.1)
- `use_gripper_bridge` (true), `add_table_collision` (true) + `table_*`
- `home_joints`, `allowed_time_s` (5), `cmd_queue_depth` (4)

## Ghi chú

- Primitive motion port từ `rx150_pick_place/scripts/pick_place_moveit_node.py` (bản đã
  chạy ổn) — **SAU khi rx150_hri chạy ổn trên máy thật sẽ `git rm -r src/rx150_pick_place`**
  (đã kiểm tra: không code nào khác phụ thuộc package cũ).
- Smoke test helper thuần: `python3 module_tests/run_test.py perception/hri_helpers_test.py`.
