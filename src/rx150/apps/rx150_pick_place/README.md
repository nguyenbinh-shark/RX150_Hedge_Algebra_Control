# rx150_pick_place — Layer 2: QUYẾT ĐỊNH

Đọc **nhận diện (Layer 1)** → chọn vật → gọi **MoveIt (Layer 3)** gắp & thả.
Mọi chuyển động đi qua `move_group`; **không bao giờ** publish trực tiếp
`/rx150/commands/*` cho cánh tay (sẽ đánh nhau với `fuzzy_node`).

Gói này gồm **thư viện dùng chung** (`rx150_pick_place/`) + **2 task node**
(`scripts/`) + **công cụ kiểm tra offline** (`rx150_reach_check.py`).

> 🔧 **Chạy trên phần cứng thật?** Đọc [`docs/RUNBOOK.md`](docs/RUNBOOK.md) —
> thang bậc bring-up B0→B6 có tiêu chí GO/NO-GO, bảng triage triệu chứng, và
> quy trình thu log. README này là *tham chiếu*; RUNBOOK là *quy trình*.

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

**T1 + T2 — motion stack + camera + TF calib.** Dùng wrapper có sẵn ở gốc workspace
(nó tự `source_all.sh`, tự đặt đúng cờ):
```bash
./rx150.sh t1      # robot + MoveIt + camera
./rx150.sh t2      # perception: static_trans_pub (TF calib) + RViz
```
Hoặc gõ tay:
```bash
source ~/interbotix_ws/source_all.sh
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
    use_camera:=true use_camera_static_tf:=false use_handeye_publisher:=false
ros2 launch rx150_perception rx150_perception.launch.py use_camera:=false use_rviz:=true
```

> ⚠️ **Phải có đúng MỘT nguồn phát TF `world ↔ camera`**, và nó là `static_trans_pub`
> (do `rx150_perception.launch.py` khởi động, đọc `rx150_perception/config/static_transforms.yaml`).
> `fuzzy_moveit.launch.py` **không** chạy node đó. Vì thế:
> - `use_camera_static_tf:=true` → TF cứng `(1, 0, 1)` chỉ là placeholder chưa hiệu chuẩn.
> - `use_handeye_publisher:=true` → cần `~/.ros/easy_handeye2/rx150_eob.yaml`; **file
>   này chưa tồn tại** cho tới khi bạn chạy `handeye_calibrate.launch.py` và bấm Save.
>
> Bật cả hai hoặc bật nhầm một cái ⇒ TF nhảy hoặc **không có TF nào**. Không có TF thì
> `yolo_tube_detector` lookup `rx150/base_link ← camera_color_optical_frame` thất bại và
> `/yolo/detected_tubes` im lặng — trông y hệt "YOLO không nhận được gì". Kiểm bằng:
> ```bash
> ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
> ```

Cần đám mây điểm (chỉ để nhìn trong RViz / OctoMap) thì `./rx150.sh t1-pcl`, hoặc thêm
`rs_camera_pointcloud_enable:=true` — ~295 MB/s, YOLO **không** cần.

**T3 — chọn 1 trong 2 task:**
```bash
ros2 launch rx150_pick_place tube_rack.launch.py             # ống nghiệm → giá
ros2 launch rx150_pick_place pick_place.launch.py            # gắp theo cử chỉ tay
```
> ⚠️ Hai launch này **tự chạy `yolo_tube_detector`** (`detector` / `enable_detector`
> mặc định `true`). Nếu bạn đang đi đường một-terminal `fuzzy_moveit_perception.launch.py`
> (vốn đã có `use_yolo:=true`) thì **bắt buộc** thêm `detector:=false` —
> hai node trùng tên `yolo_tube_detector` sẽ nạp YOLO 2 lần lên GPU và publish
> `/yolo/detected_tubes` từ 2 nguồn. Đường `./rx150.sh t1 → t2 → tubes` thì không
> dính, vì `rx150_perception.launch.py` không chạy YOLO.

Thêm `dry_run:=true` để chạy **toàn bộ** state machine (nhận diện, IK, kế hoạch,
log từng waypoint) mà **không gửi goal** nào tới robot. Nên làm việc này trước.

Không có camera thì bơm ống giả để vẫn chạy hết chuỗi:
```bash
ros2 launch rx150_pick_place tube_rack.launch.py \
    detector:=false dry_run:=true \
    fake_tubes:='[{"x":0.20,"y":-0.15,"z":0.03,"yaw":0.0,"class":"pink"}]'
```
> `ros2 run … -p fake_tubes:='[{…}]'` **không dùng được**: `rcl` parse giá trị `-p`
> theo YAML nên JSON array thành flow-sequence → `Unknown YAML event`. Dùng launch
> arg ở trên, hoặc `--params-file`.

### Backend chấp hành (`motion_backend`)

| | `moveit` (mặc định) | `direct` |
|---|---|---|
| Đường đi | node → `move_action` → OMPL → `execute_trajectory` → bridge → fuzzy PWM → xs_sdk | node → **`arm_controller/follow_joint_trajectory`** → fuzzy PWM → xs_sdk |
| Tránh vật cản | có (planning scene, ACM, attach/detach) | **không** — an toàn nằm ở waypoint |
| Gripper | qua `gripper_trajectory_bridge` (move_group) | **PWM trực tiếp** (tự chuyển, có log) |
| Khi nào dùng | mặc định | `move_group` hỏng/không có, hoặc muốn hành vi y hệt bản `key_point` đã chạy thật |

```bash
ros2 launch rx150_pick_place tube_rack.launch.py motion_backend:=direct
```
Đường **phần cứng không đổi** giữa 2 backend — `direct` chỉ bỏ khâu lập kế hoạch,
vẫn đi qua đúng `fuzzy_trajectory_bridge` mà MoveIt vẫn dùng để chấp hành.

> ⚠️ **Trước T1:** `pkill -f xs_sdk` (2 driver trên 1 bus U2D2 → tranh chấp serial);
> dọn thoáng quanh robot; e-stop trong tầm tay.
>
> `fuzzy_node` **không** phải bring-up thụ động: ngay khi `/rx150/get_robot_info`
> trả lời, nó đặt cả nhóm `arm` sang **PWM mode + bật torque** rồi timer 100 Hz bắt
> đầu bơm PWM. Vì `fuzzy_moveit.launch.py` ép `enable_profile: False`, chừng nào
> bridge chưa gửi setpoint thì `q_ref = reference_pose = [0, −1.80, 1.55, 0.8, 0]`
> (tư thế **sleep**, không phải home) được đưa vào **như một bước nhảy**, không qua
> Ruckig làm mượt. Tay đang ở xa sleep ⇒ giật. Hãy bắt đầu từ tư thế gần sleep.

### Kiểm tra TRƯỚC khi cấp điện (không cần robot, ~1 giây)

```bash
./rx150.sh reach        # bảng tầm với + CẢ HAI config + tư thế trung chuyển
```

Hoặc từng phần:

```bash
ros2 run rx150_pick_place rx150_reach_check.py                       # bảng tầm với
ros2 run rx150_pick_place rx150_reach_check.py --config \
  $(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config/tube_rack_params.yaml
ros2 run rx150_pick_place rx150_reach_check.py --point 0.18 0 0.18 --pitch 0
```

Dòng cuối kiểm **tư thế trung chuyển** (`home_xyz_pitch`). IK điểm đó fail thì
`resolve_home()` chỉ log ERROR rồi **âm thầm quay lại `home_joints` thô
`[0,0,0,0,0]`** — FK của nó là `(0.359, 0, 0.255)`, tức tay duỗi thẳng **vượt QUA
giá ở x = 0.26**, đúng cái mà `home_xyz_pitch` sinh ra để tránh.

rx150 **không** chúc thẳng đứng (pitch 90°) được ở mọi nơi — hết tầm từ r ≈ 0.29 m,
còn 64° ở r = 0.36 m. Rất nhiều ca "gắp không được" chỉ là điểm gắp nằm ngoài bao
hình khả thi. Bảng trên cho biết ngay.

### Smoke-test theo tầng (không phát lệnh tới robot)

```bash
./rx150.sh check        # joint_states + action server + TF/detection
```

Mỗi bài exit 0 = GO, khác 0 = NO-GO **kèm nguyên nhân gốc + lệnh kiểm tiếp theo**.
Chi tiết: [`docs/RUNBOOK.md`](docs/RUNBOOK.md) §4.

### Khi báo lỗi — thu bằng chứng

```bash
./rx150.sh diag b5a-home-fail       # -> diag_<nhãn>_<ts>.tar.gz
./tools/record_pickplace.sh b5a     # bag 5 tầng, replay trong PlotJuggler
```

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
| Chấp hành | `motion_backend`, `arm_action`, `direct_joint_speed_rad_s` | `moveit` (có planner) hay `direct` (không planner) — xem bảng ở mục Chạy |
| Tư thế trung chuyển | `home_xyz_pitch`, `home_joints` | ghi **toạ độ**, `home_joints` được tính lại bằng IK lúc khởi động |
| Hình học gắp | `approach_delta`, `grasp_z_offset`, `retract_height`, `retreat_back_m`, `via_staging`, `min_ee_z` | `min_ee_z` là chặn cứng chống đâm bàn khi depth nhiễu |
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
  `motion_backend: direct` **tự chuyển sang đường này** (và log ra lý do): đường
  bridge của gripper đi qua chính `move_group`, nên giữ nó ở direct mode là mâu thuẫn.
  Nhớ để `set_gripper_pwm_mode: true` để motor thực sự ở PWM mode.
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

## Áp từ bản `key_point` đã chạy thật (0.2.1)

Bản `key_point/task3_4tubes.py` điều khiển tay bằng `bot.arm.set_ee_pose_components()`
— không planner, không planning scene — nhưng đã gắp-cắm được 4 ống thật. Ba thứ nó
làm mà bản này thiếu, nay đã đưa vào:

| key_point làm | Bản này trước đây | Nay |
|---|---|---|
| Mọi pha bắt đầu/kết thúc ở "fake home pose" `(0.18, 0, 0.18)` — thu gọn, **sau lưng giá** | `home_joints: [0,0,0,0,0]` ⇒ FK = `(0.359, 0, 0.255)`: tay **duỗi thẳng, vượt QUA giá ở x = 0.26**. `go_home()` chạy đầu mỗi chu kỳ và sau mỗi `recover()` ⇒ mỗi lần "về home" là quét ngang đúng chỗ cắm ống | `home_xyz_pitch: [0.18, 0, 0.18, 0]`, `home_joints` **tính lại bằng IK** lúc khởi động (log ra góc thật) |
| Vào giá luôn ghé cột `(0.18, 0, z+0.08)` trước, không quét thẳng từ chỗ gắp sang slot | `LIFT → HOVER` một nhát ở đúng cao độ miệng lỗ | `PickPlaceSkill.plan_via()` + `release_at(via=…)`, bật bằng `via_staging` |
| Nhả xong: lên `z+0.05` → **lùi `x−0.05`** → mới về home | chỉ rút thẳng lên rồi về home — ngón vẫn ở ngay trên miệng lỗ khi bắt đầu xoay | `PickPlaceSkill.retreat_radial()`, `retreat_back_m: 0.05`; lùi theo **bán kính** (trùng `−x` khi `y ≈ 0` như bố cục key_point, vẫn đúng khi giá lệch sang bên) |
| Không qua MoveIt chút nào | bắt buộc phải có `move_group` | `motion_backend: direct` — xem bảng ở mục Chạy |

Hai thứ **không** đổi vì kiểm tra lại thì bản này đã đúng:
- `wrist_rotate`: key_point dùng `roll + atan2(y,x)` với `roll = −atan2(dy,dx)`, rút gọn
  ra đúng `waist − yaw` — trùng `wrist_rotate_for_axis()`.
- pitch gắp `π/2` (ống nằm, trục `ee_z` ngang) / pitch cắm `0` (ống dựng, trục `ee_z`
  đứng) — trùng `grasp_pitch_ladder` / `place_pitch_ladder`.

`grasp_z_offset` thì **cố ý giữ `0.0`**: key_point kẹp ở `z + 0.01` (cao hơn tâm phát
hiện 1 cm), nhưng con số đó phụ thuộc quy ước z của detector bên đó. Đo trên bàn của
bạn rồi đặt `grasp_z_offset: -0.01` nếu thấy ngón kẹp thấp quá.

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
