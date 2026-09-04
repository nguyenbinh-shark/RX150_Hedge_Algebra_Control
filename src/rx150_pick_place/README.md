# rx150_pick_place — Layer 2: QUYẾT ĐỊNH

Đọc **nhận diện (Layer 1)** → chọn vật → gọi **MoveIt (Layer 3)** gắp & thả.
Mọi chuyển động đi qua `move_group`; **không bao giờ** publish trực tiếp
`/rx150/commands/*` cho cánh tay (sẽ đánh nhau với `fuzzy_node`).

Gói này gồm **thư viện dùng chung** (`rx150_pick_place/`) + **2 task node**
(`scripts/`) + **công cụ kiểm tra offline** (`rx150_reach_check.py`).

```
rx150_pick_place/
  kinematics.py  IK/FK giải tích 5-DoF (nghiệm đóng, chọn nhánh theo seed)
  motion.py      MoveGroup client: verify pose thật, retry, E-stop, đi thẳng Descartes
  gripper.py     đóng/mở + XÁC NHẬN có vật trong ngón (part-present)
  scene.py       planning scene: box/cylinder, attach/detach vật đang kẹp, ACM
  detection.py   nguồn YOLO: kiểm tra frame/tuổi dữ liệu, median ghép theo khoảng cách
  skills.py      các verb pick-place (plan_grasp / grasp_at / release_at / recover)
  status.py      state machine + /status JSON + thống kê chu kỳ
  params.py      1 bộ tham số chung + build_stack()
scripts/
  pick_place_moveit_node.py  gắp vật được chỉ bằng cử chỉ tay → thả chỗ cố định
  tube_rack_node.py          gắp ống nghiệm → cắm lên giá, phân theo màu
  rx150_reach_check.py       kiểm tra tầm với / config, KHÔNG cần robot
test/test_kinematics.py      đối chiếu IK/FK với modern_robotics (pytest)
```

## Pipeline 3 layer

```
D435 ──rs_launch──► /camera/camera/{color, aligned_depth_to_color, depth/color/points}
        │                                              └──► OctoMap ──► move_group collision
        ▼
[L1 rx150_perception] yolo_tube_detector_node ──► /yolo/detected_tubes  (PoseArray base_link)
                                              ──► /yolo/tube_classes   (String JSON)
                      hand_gesture_node       ──► /hand_gesture/selected_target (Int32)
                                              ──► /hand_gesture/event "ok_sign"
        ▼
[L2 rx150_pick_place]  pick_place_moveit_node | tube_rack_node
        │   IK giải tích → joint goal / quỹ đạo thẳng
        │   /move_action (interbotix_arm)      ──► OMPL ──► arm_bridge ──► fuzzy_node ──► xs_sdk
        │   /move_action (interbotix_gripper)  ──► gripper_bridge ──► xs_sdk
        │   /apply_planning_scene: box bàn/giá + attach vật đang kẹp
        ▼
[L3 rx150_fuzzy_controller + MoveIt]  hoạch định + thực thi PWM
```

## Chạy

**T1 — motion stack + camera + hand-eye:**
```bash
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
    use_camera:=true rs_camera_pointcloud_enable:=true \
    use_camera_static_tf:=false use_handeye_publisher:=true
```

**T2 — chọn 1 trong 2 task:**
```bash
ros2 launch rx150_pick_place pick_place.launch.py            # gắp theo cử chỉ tay
ros2 launch rx150_pick_place tube_rack.launch.py             # ống nghiệm → giá
```
Thêm `dry_run:=true` để chạy **toàn bộ** state machine (nhận diện, IK, kế hoạch,
log từng waypoint) mà **không gửi goal** nào tới robot. Nên làm việc này trước.

> ⚠️ **Trước T1:** `pkill -f xs_sdk` (2 driver trên 1 bus → crash); cánh tay thoáng
> (`fuzzy_node` PWM-drive về home ngay khi launch); e-stop trong tầm tay.

### Kiểm tra TRƯỚC khi cấp điện (không cần robot, ~1 giây)
```bash
ros2 run rx150_pick_place rx150_reach_check.py                       # bảng tầm với
ros2 run rx150_pick_place rx150_reach_check.py --config \
  $(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config/tube_rack_params.yaml
```
rx150 **không** chúc thẳng đứng (pitch 90°) được ở mọi nơi — hết tầm từ r ≈ 0.29 m,
còn 64° ở r = 0.36 m. Rất nhiều ca "gắp không được" chỉ là điểm gắp nằm ngoài bao
hình khả thi. Bảng trên cho biết ngay.

### Vận hành khi đang chạy
| Việc | Lệnh |
|---|---|
| Xem trạng thái | `ros2 topic echo /tube_rack/status` (hoặc `/pick_place_moveit/status`) |
| Dừng khẩn cấp (mềm) | `ros2 service call /tube_rack/stop std_srvs/srv/Trigger` |
| Chạy lại sau khi dừng | `ros2 service call /tube_rack/reset std_srvs/srv/Trigger` |
| Về home | `ros2 service call /tube_rack/home std_srvs/srv/Trigger` |
| Mở gripper (lấy vật ra) | `ros2 service call /tube_rack/open_gripper std_srvs/srv/Trigger` |
| Chạy 1 chu kỳ | `ros2 service call /tube_rack/run std_srvs/srv/Trigger` |
| Gắp vật đang chỉ tay | `ros2 service call /pick_place_moveit/pick std_srvs/srv/Trigger` |

`/status` là JSON: `state`, `detail`, `attempted`, `succeeded`, `faults`,
`success_rate`, `last_cycle_s`, `last_error` — đủ cho HMI/log dây chuyền.

## State machine

```
IDLE → SCANNING → PLANNING → APPROACH → DESCEND → GRASP(+verify) → LIFT
     → TRANSPORT → PLACE/INSERT → RELEASE → RETRACT → HOMING → DONE → IDLE
                                                    ↘ FAULT → RECOVERY ↗
```

Hai nguyên tắc quan trọng:

1. **PLANNING xong toàn chuỗi rồi mới động.** IK của cả pre-grasp → grasp → lift →
   hover → place được giải TRƯỚC. Thiếu một pose nào ⇒ báo lỗi khi tay còn rỗng,
   thay vì kẹp vật rồi mới phát hiện chỗ thả không với tới.
2. **RECOVERY không thả vật bừa.** Rút thẳng đứng, nếu còn kẹp vật thì mang tới
   `reject_x/y/z` mới nhả, rồi về home.

## Tham số

Toàn bộ tham số nằm trong `config/*.yaml` (có chú thích từng dòng). Nhóm quan trọng:

| Nhóm | Tham số | Ý nghĩa |
|---|---|---|
| Hình học gắp | `approach_delta`, `grasp_z_offset`, `retract_height`, `min_ee_z` | `min_ee_z` là chặn cứng chống đâm bàn khi depth nhiễu |
| Pitch | `grasp_pitch_ladder`, `place_pitch_ladder` | thử dốc nhất trước, nới dần — thay cho 1 giá trị `grasp_pitch` cố định |
| Tốc độ | `*_speed_mps` (đoạn đi thẳng), `velocity_scale_*` (đoạn tự do) | mm/s là đơn vị nói được với người vận hành |
| Gripper | `grasp_empty_margin_m`, `grasp_retries`, `regrasp_z_step` | ngưỡng phát hiện "kẹp vào không khí" + regrasp |
| Nhận diện | `detection_max_age_s`, `median_frames`, `detection_min_hits_ratio` | từ chối dữ liệu cũ / nhấp nháy |
| Giá (tube_rack) | `slot0_*`, `rack_yaw_deg`, `rack_tilt_deg`, `slot_spacing`, `slot_dz` | xem phần dưới |
| Sự cố | `reject_x/y/z`, `max_consecutive_failures` | chỗ đặt vật khi bỏ dở, ngưỡng dừng dây chuyền |

### Quy ước hướng (5-DoF) — đã kiểm chứng bằng FK trong `test/`
- `waist = atan2(y, x)`; `pitch = shoulder + elbow + wrist_angle`.
- `pitch = +pi/2` → `ee_x` chúc thẳng xuống, trục `ee_z` **nằm ngang** (gắp vật nằm).
- `pitch = 0` → `ee_x` nằm ngang, trục `ee_z` **thẳng đứng** (vật kẹp dựng đứng).
- Vật hình ống kẹp ngang giữa 2 ngón nằm dọc `ee_z`. Để 2 ngón ép ⟂ trục vật:
  `tan(wrist_rotate) = tan(waist − yaw) / sin(pitch)`; ở `pitch = 90°` là
  `wrist_rotate = waist − yaw`.
- `yolo_tube_detector_node` lấy yaw từ `cv2.minAreaRect` (hệ **ảnh**). Nếu trục ảnh
  không trùng trục base_link, bù bằng `detection_yaw_offset_deg` / `detection_invert_yaw`:
  đặt 1 ống dọc trục +x của robot, xem log yaw, lấy hiệu.

### Hình học giá (`tube_rack_params.yaml`)
- `slot0_x/y/z` — **miệng lỗ** của slot đầu tiên (frame `rx150/base_link`).
- `rack_yaw_deg` — hướng **hàng slot** trong mặt phẳng XY (90° = hàng dọc trục y).
- `rack_tilt_deg` — nghiêng **trục lỗ** so với phương **thẳng đứng**
  (giá nghiêng 62° so với mặt bàn ⇒ đặt `28.0`). Pitch lúc cắm được suy từ đây.
- `slot_dz` — chênh cao mỗi slot (giá bậc thang).
- Node in bảng "slot nào với tới được" **mỗi lần khởi động**.

## Gripper: bridge vs PWM
- **Bridge (mặc định):** goal nhóm `interbotix_gripper` (joint `left_finger`,
  0.015 = grasp / 0.037 = release) → `gripper_trajectory_bridge` (stall-aware, giữ
  lực khi vận chuyển).
- **PWM fallback (`use_gripper_bridge: false`):** `JointSingleCommand(name='gripper')`.
  Bản này **chờ tới khi ngón dừng hẳn** rồi mới trả về (không `sleep(2.0)` mù).
- **Xác nhận kẹp:** sau khi đóng, đọc `left_finger`:
  sát giới hạn dưới ⇒ kẹp vào không khí (FAIL); sát giới hạn trên ⇒ chưa đóng (FAIL);
  dừng ở giữa ⇒ có vật. Hỏng thì regrasp `grasp_retries` lần rồi mới RECOVERY.

## Collision
- `table` (box) + `rack` (box) qua `/apply_planning_scene`.
- **Vật đang kẹp được attach vào `ee_gripper_link`** trong lúc vận chuyển ⇒ planner
  tính cả vật, không quét vật qua giá/bàn.
- Các vật **không** gắp (detection còn lại) được thêm làm cylinder vật cản
  (`add_detected_obstacles`) ⇒ không gạt đổ hàng bên cạnh.
- Pha đặt/cắm: nới ACM cho `grasped_object` (đọc ACM hiện tại rồi mới sửa — gửi ACM
  tự chế trong 1 diff sẽ **xoá** toàn bộ `disable_collisions` của SRDF), xong thì trả lại.
- OctoMap: từ `/camera/camera/depth/color/points`, cần T1 bật pointcloud.

## Build & test
```bash
cd ~/interbotix_ws && colcon build --packages-select rx150_pick_place --symlink-install
source install/setup.bash

# kiểm chứng toán học (không cần robot / ROS runtime)
cd src/rx150_pick_place && python3 -m pytest test -q
```
Thư viện được cài vào `lib/rx150_pick_place/` và đưa vào `PYTHONPATH` bằng env-hook
(không dùng `ament_python_install_package`: setuptools 84 trên máy này không hợp với
`packaging` 21.3 nên bước build egg fail).

## Những gì đã sửa so với bản trước (0.1.0)

| Lỗi/thiếu | Hậu quả thực tế |
|---|---|
| `box.dim` thay vì `box.dimensions` (SolidPrimitive dùng `__slots__`) | `_ensure_scene()` throw ngay dòng đầu ⇒ **`tube_rack` chết đúng lúc mở chu kỳ**, chưa gửi goal nào; `hri_motion` chết luôn worker thread (đã sửa cả ở `rx150_hri`) |
| `wrist_rotate = yaw − waist` (sai dấu) | ngón kẹp lệch 2·(yaw−waist) so với trục ống ⇒ kẹp vào đầu ống, đẩy ống lăn |
| Nhận `error_code = −4 (CONTROL_FAILED)` là SUCCESS cho MỌI nhóm khớp | tay chưa tới đích mà chuỗi vẫn chạy tiếp ⇒ kẹp không khí / thả ra ngoài giá. Nay đối chiếu `joint_states` thật rồi mới quyết định |
| Không kiểm tra kẹp được vật hay không | chu kỳ báo "✓ thành công" trong khi ngón rỗng |
| IK-oracle của SDK (`set_ee_pose_components(execute=False)`) | `_check_joint_limits` so cả **vận tốc** với `joint_commands` (không cập nhật khi `execute=False`) ⇒ báo "IK fail" ở pose với tới được. Nay dùng nghiệm đóng, chọn nhánh theo seed |
| Không attach vật vào planning scene | planner coi tay là tay không khi đang mang vật |
| `rack_angle_deg` vừa là "góc nghiêng" vừa dùng làm góc XY; `slot_spacing 0.12` + 4 slot | slot 3 nằm ở r = 0.51 m > tầm với 0.47 m — **không thể tới**, mà chỉ biết khi chạy tới ống thứ 4 |
| Ống đã nằm trong giá chỉ bị loại khỏi danh sách gắp | slot đó vẫn coi là trống ⇒ cắm ống mới **chồng lên** ống cũ |
| Không kiểm tra `frame_id` / tuổi của detection | detector chết là tay vẫn lao xuống theo pose cũ; sai frame thì gắp sai chỗ mà không ai biết |
| median lấy theo **chỉ số** mảng | detector đổi thứ tự là trộn toạ độ của 2 ống với nhau |
| PWM fallback `sleep(2.0); return True` | không biết ngón đã đóng chưa |
| Không có E-stop, không huỷ được goal, không có `/status` | không vận hành được như một trạm thật |
| 3 bản copy-paste MoveGroup primitive (`pick_place`, `tube_rack`, `hri_motion`) | sửa 1 bản không lan sang bản khác |
| Không có test nào | quy ước hướng/IK không ai kiểm chứng được ngoài cách thử trên robot |

Nhớ chạy lại `rx150_reach_check.py --config …` sau khi đo lại vị trí giá/bàn:
`place_x/y/z`, `slot0_*`, `reject_*` trong config hiện tại là **giá trị mẫu đã kiểm
tra là với tới được**, không phải số đo bàn của bạn.
