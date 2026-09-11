# Sơ đồ cấu trúc điều khiển RX150 — và khoảng cách so với bản cũ

Tài liệu này trả lời hai câu:

1. **Hệ điều khiển hiện tại nối với nhau như thế nào** — tên node, tên topic/action,
   tần số, ai đóng vòng ở đâu.
2. **Bản cũ (`key_point/task4_full.py`) khác chỗ nào** — đây là mã của bài báo trước,
   giữ lại làm **baseline đối chiếu**, KHÔNG phải thứ sẽ đưa vào tầng Application.
   Bảng chênh lệch ở mục 5 và 6 chính là phần cũ → mới.

Nguồn: `src/rx150/**` (đọc ngày 2026-09-08) và `/home/hust/Desktop/key_point/task4_full.py`.

> **Vị thế của `task4_full.py`**: mã của bài báo bị từ chối, dùng để đối chiếu. Tầng
> Application của hệ mới là `rx150_pick_place` + `rx150_hri`, đã chạy. Không port file
> này vào kho.

---

## 1. Sơ đồ cấu trúc điều khiển hiện tại

Năm tầng IRROS, điền tên thật. Mũi tên đặc `═►` là **lệnh đi xuống**, mũi tên
`◄──` là **phản hồi đi lên**.

```
┌── APPLICATION ─────────────────────────────────────────────────────────────────┐
│  rx150_pick_place                          rx150_hri                           │
│   ├ tube_rack_node        gắp ống → cắm     ├ hri_task_node   (quyết định)     │
│   ├ pick_place_moveit_node  gắp vật được    │      │ /hri/cmd_pose  (PoseStamped)│
│   │                         chỉ bằng tay    │      │ /hri/cmd_gripper (Bool)    │
│   └ rx150_reach_check     kiểm tầm, offline │      │ /hri/cmd_home  (Empty)     │
│                                             │      ▼ /hri/status ◄── (String)   │
│                                             └ hri_motion_node (chấp hành)      │
│  Chỉ quyết định: gắp CÁI GÌ, đặt Ở ĐÂU, KHI NÀO. Không tự viết primitive.      │
└───────────────┬────────────────────────────────────────────────────────────────┘
                │ gọi hàm Python in-process (import rx150_modules.*)
                ▼
┌── APPLICATION SUPPORT ─────────────────────────────────────────────────────────┐
│  rx150_modules  (thư viện dùng chung — build_stack() dựng đủ 6 mảnh)           │
│   ├ Rx150Kinematics   IK GIẢI TÍCH 5-DoF (pitch = s+e+w; wrist = waist−yaw)    │
│   ├ MoveItExecutor    gửi goal + verify_reached() + retry                       │
│   ├ Gripper           close(verify=True) → xác nhận CÓ VẬT trong ngón           │
│   ├ SceneManager      planning scene: box bàn, attach/detach ống                │
│   ├ PickPlaceSkill    APPROACH→DESCEND→GRASP→LIFT→TRANSPORT→PLACE→RETREAT       │
│   └ TaskStatus        state machine + thống kê ra ~/status (JSON)               │
│                                                                                 │
│  rx150_perception                                                               │
│   ├ yolo_tube_detector_node  ─► /yolo/detected_tubes  (PoseArray, base_link)    │
│   │   YOLO-seg + mask→contour→endpoint + nắp màu → yaw, EMA + tracker min_hits  │
│   │                          ─► /yolo/tube_classes (String JSON) , /yolo/markers │
│   ├ hand_gesture_node        ─► /hand_gesture/selected_target (Int32)           │
│   │   MediaPipe Hands: tia lm5→lm7, OK-sign  ─► /hand_gesture/event (String)    │
│   └ apriltag_ros + easy_handeye2 ─► TF  rx150/base_link ← camera_color_optical  │
│                                                                                 │
│  move_group  (MoveIt 2: OMPL plan → TOTG time-parameterize → planning scene)    │
└───────────────┬────────────────────────────────────────────────────────────────┘
                │ action  /rx150/arm_controller/follow_joint_trajectory
                │        (control_msgs/FollowJointTrajectory)
                ▼
┌── CONTROL ─────────────────────────────────────────────────────────────────────┐
│  rx150_trajectory_bridge  (= fuzzy_trajectory_bridge, ns=rx150)                │
│     nhận quỹ đạo → NỘI SUY TUYẾN TÍNH theo thời gian, phát 100 Hz               │
│     ─► /rx150/fuzzy/setpoint   (Float64MultiArray, 5 khớp)                      │
│                                                                                 │
│  fuzzy_node  (C++, 100 Hz)  — MỘT trong ba biến thể, đổi được không sửa app     │
│     e = q_ref − q ,  ė = q̇_ref − q̇  →  Mamdani type-1 (fuzzy_type1.c)          │
│     u = K_u·fuzzy(K_e·e, K_ed·ė) + g(q)   (bù trọng lực Pinocchio)              │
│     ─► /rx150/commands/joint_group  (JointGroupCommand, **PWM**)                │
│     debug ─► fuzzy/error, fuzzy/edot, fuzzy/effort, fuzzy/gravity, fuzzy/ref    │
│                                                                                 │
│  gripper_trajectory_bridge ─► /rx150/commands/joint_single (effort PWM)         │
│                                                                                 │
│  (thay bằng rx150_ff_controller: fuzzy PD + K_v·q̇ + K_a·q̈, không bù trọng lực) │
│  (thay bằng rx150_hac_controller: HAC tuyến tính + Ruckig + bù trọng lực)       │
└───────────────┬────────────────────────────────────────────────────────────────┘
                │ /rx150/commands/joint_group  (PWM)
                ▼
┌── DRIVER ──────────────────────────────────────────────────────────────────────┐
│  xs_sdk (interbotix_xs_sdk)   ghi PWM xuống bus, đọc encoder                    │
│      ─► /rx150/joint_states  ◄══ PHẢN HỒI CHUNG cho cả 3 vòng ở trên            │
│      srv /rx150/set_operating_modes  (đặt group arm = pwm, gripper = pwm)       │
│  realsense2_camera ─► /camera/camera/color/image_raw                            │
│                    ─► /camera/camera/aligned_depth_to_color/image_raw + info    │
└───────────────┬────────────────────────────────────────────────────────────────┘
                ▼
┌── HARDWARE ────────────────────────────────────────────────────────────────────┐
│  RX150: 5×Dynamixel XL430/XM430 + gripper, qua U2D2 (USB, 1 Mbps)              │
│  RealSense D435i · AprilTag dán trên đế (hand-eye)                             │
└────────────────────────────────────────────────────────────────────────────────┘
```

### Ba vòng lồng nhau

Điểm quan trọng nhất của sơ đồ: **ba vòng kín chạy ở ba tần số khác nhau**, và cả ba
đều đóng qua cùng một `/rx150/joint_states`.

```
   ┌──────────────────────────── VÒNG 1 · NHẬN THỨC · ~5 Hz ─────────────────────┐
   │  camera ─► YOLO / MediaPipe ─► PoseArray + chỉ số vật ─► app CHỌN VẬT       │
   │                                                              │              │
   │   ┌───────────────────── VÒNG 2 · QUỸ ĐẠO · 1 lần / waypoint ┴────────┐     │
   │   │  app ─► IK giải tích ─► move_group (OMPL + TOTG) ─► FJT goal      │     │
   │   │                                              │                    │     │
   │   │   ┌──────────────── VÒNG 3 · SERVO · 100 Hz ─┴───────────────┐    │     │
   │   │   │  setpoint ─► e, ė ─► fuzzy + g(q) ─► PWM ─► motor        │    │     │
   │   │   │        ▲                                        │        │    │     │
   │   │   │        └──────────── /rx150/joint_states ◄──────┘        │    │     │
   │   │   └──────────────────────────────────────────────────────────┘    │     │
   │   │  verify_reached() đọc joint_states để xác nhận ĐÃ TỚI ĐÍCH THẬT   │     │
   │   └───────────────────────────────────────────────────────────────────┘     │
   │  Gripper.close(verify) đọc left_finger để xác nhận KẸP ĐƯỢC VẬT             │
   └────────────────────────────────────────────────────────────────────────────┘
```

Luật của kho: **tầng trên gọi tầng dưới, không bao giờ ngược lại**. App không được
nói thẳng với `fuzzy_node`; nó chỉ biết `move_group` và `rx150_modules`.

---

## 2. Bản cũ (`task4_full.py`) là cái gì — baseline đối chiếu

501 dòng, **một process, không tầng nào cả**. Nó tự cắm từ Application thẳng xuống
Hardware, đi vòng qua toàn bộ Control và Application Support. Vẽ ra để thấy rõ hình dạng
của hệ trong bài cũ:

```
┌── task4_full.py (1 file, 1 process) ───────────────────────────────────────────┐
│                                                                                 │
│  pyrealsense2.pipeline ──────────────────────────► mở THẲNG D435i (USB)  L173  │
│  YOLO('/home/quang/…/best.pt')  mask→contour→centroid→deproject          L185  │
│  HSV nắp cam → roll = −atan2(Δy, Δx)                                     L273  │
│  MediaPipe Hands + EMA(α=0.2) → tia lm5→lm7 ; OK-sign d<0.03             L183  │
│  M02 = M0·M1·M2 (θ=40° ĐO TAY) ; cam_to_robot(p) = M02⁻¹·p               L42   │
│  rack_bases / rack_slots / rack_status=[1,0,1,0]  HARDCODE               L70   │
│  workspace_limits = hộp AABB  HARDCODE                                   L99   │
│                                                                                 │
│  ┌── FSM chỉ tay (PHẦN DUY NHẤT THUỘC VỀ TẦNG APPLICATION) ──────┐  L237–456   │
│  │  chưa cầm:  chỉ vào slot có ống  (d<0.08, giữ 1.3 s) → pick_rack│            │
│  │             chỉ vào ống trên bàn (d<0.08, giữ 1.5 s) → pick     │            │
│  │  đang cầm:  chỉ vào giá          (d<0.12, giữ 15 s ) → place    │            │
│  │             OK-sign              (giữ 2.0 s)          → handover│            │
│  └───────────────────────────┬────────────────────────────────────┘            │
│                              ▼  robot_queue.put(('pick', …))                    │
│  robot_worker (thread)  4 hành vi, mỗi hành vi là 5–8 lệnh tuyệt đối     L116  │
│                              ▼                                                  │
│  InterbotixManipulatorXS.set_ee_pose_components(x,y,z,roll,pitch,yaw)    L107  │
│      └─ mr.IKinSpace (Newton, 3 seed cố định)                                  │
│      └─ publish JointGroupCommand ─► /rx150/commands/joint_group  **POSITION** │
└────────────────────────────────┬────────────────────────────────────────────────┘
                                 ▼
                              xs_sdk ─► Dynamixel (PID nội trong firmware)
```

Không có: planning scene, tránh vật cản, verify tới đích, verify kẹp được, TF,
không có tham số hoá — mọi con số nằm trong code.

---

## 3. Vì sao bản cũ không chạy chung được với stack mới

(Ghi lại để khỏi ai thử lại — cả bốn đều là hệ quả của việc bản cũ không có tầng.)

| # | Đụng độ | Triệu chứng thật |
| :- | :--- | :--- |
| 1 | **Cùng ghi `/rx150/commands/joint_group`.** `fuzzy_node` phát PWM 100 Hz; `set_ee_pose_components` phát POSITION. Motor đang ở **pwm mode** (`set_operating_modes` đặt lúc bring-up) nên con số vị trí bị diễn giải là PWM. | Tay giật/đâm, hoặc "không nhúc nhích rồi văng" — hai nguồn lệnh đánh nhau ở 100 Hz. |
| 2 | **Hai driver cùng mở D435i.** `fuzzy_moveit.launch.py` đã bật `realsense2_camera` (`use_camera:=true`); `pyrealsense2.pipeline.start()` mở lại thiết bị đó. | `RuntimeError: Device or resource busy`. |
| 3 | **Hai nguồn TF camera→robot.** `M02` hardcode θ=40° vs. TF từ AprilTag/`easy_handeye2`. | Toạ độ lệch vài cm, không ai biết bản nào đúng; đã có ghi chú "TF publisher phải có đúng MỘT nguồn". |
| 4 | **Đường dẫn model của máy khác**: `/home/quang/Colab_tube/runs/segment/train/weights/best.pt`. | `FileNotFoundError` ngay dòng 185. |

Ngoài ra, đọc kỹ FSM còn mấy chỗ nên sửa khi port (không phải lỗi kiến trúc, nhưng
sẽ đi theo nếu copy nguyên):

* `L345` dwell **15 s** cho `place`, trong khi pick là 1.3 s / 1.5 s — gần như chắc
  là số debug bỏ quên; người dùng sẽ tưởng chức năng place hỏng.
* `L346` log in `slot_idx % 3 + 1` còn logic dùng `% 2` → số slot in ra sai.
* `L389` nhánh OK-sign: `else: selected_id = -1` (reset) thay vì bắt đầu đếm giờ →
  `gesture_start` chỉ chạy từ frame thứ hai trở đi.
* `L458` `gesture_detected` không bao giờ được đặt lại `False` sau khi handover xong →
  chữ "OK SIGN DETECTED" dính vĩnh viễn trên ảnh.
* `L475`+`L491` `imshow`/`waitKey` bị lặp hai lần; `break` ở lần đầu khiến khối publish
  ROS ở `L482` không bao giờ chạy ở frame cuối.
* `node = FalconTubeNode()` được tạo nhưng **không ai spin** — publisher vẫn phát được,
  còn lại thì không.
* `holding` trong `robot_worker` gán rồi không đọc.

---

## 4. Chức năng của bản cũ nằm ở đâu trong hệ mới

Trong 501 dòng của bản cũ, phần **thực sự thuộc tầng Application** chỉ là FSM chỉ tay
(`L237–456`, ~90 dòng logic quyết định). Toàn bộ phần còn lại đã có sẵn ở tầng dưới của
hệ mới. Nếu muốn dựng lại đúng kịch bản HRI đó để đo A/B với bài cũ, nó là **một node app
duy nhất** — ba tầng dưới không đổi một dòng:

```
┌── APPLICATION ─────────────────────────────────────────────────────────────────┐
│                                                                                 │
│   ★ hri_pointing_node   ← nếu dựng lại kịch bản cũ: CHỈ CÒN FSM               │
│     ┌─────────────────────────────────────────────────────────────────┐        │
│     │ state: holding? · rack_status[4] · selected_id · dwell timer     │        │
│     │                                                                   │        │
│     │  ─◄ /yolo/detected_tubes         (PoseArray, đã ở base_link)     │        │
│     │  ─◄ /yolo/tube_classes           (màu nắp)                        │        │
│     │  ─◄ /hand_gesture/selected_target(Int32 — vật đang bị chỉ)        │        │
│     │  ─◄ /hand_gesture/event          ("ok_sign")                      │        │
│     │  ─◄ /hand_gesture/pointing_ray   (★ THÊM: tia để chỉ vào GIÁ)     │        │
│     │                                                                   │        │
│     │  luật: chưa cầm + chỉ slot đầy (1.3 s) → PICK_RACK                │        │
│     │        chưa cầm + chỉ ống bàn  (1.5 s) → PICK                     │        │
│     │        đang cầm + chỉ giá      (1.5 s) → PLACE (slot trống đầu)   │        │
│     │        đang cầm + OK-sign      (2.0 s) → HANDOVER tại cổ tay      │        │
│     └───────────────────────────────┬─────────────────────────────────┘        │
│                                     │ gọi hàm, KHÔNG tự viết chuyển động       │
└─────────────────────────────────────┼──────────────────────────────────────────┘
                                      ▼
┌── APPLICATION SUPPORT (KHÔNG SỬA GÌ) ──────────────────────────────────────────┐
│  rx150_modules.PickPlaceSkill                                                   │
│    plan_grasp(x,y,z,yaw) → GraspPlan(pre, grasp, lift)   ← thay 8 lệnh tuyệt đối│
│    grasp_at(plan, attach=True)                            ← thay bot.gripper…   │
│    release_at(insert, hover)                              ← thay lệnh place      │
│    go_home() / retract_up() / retreat_radial() / recover()                       │
│  Rx150Kinematics.ik / reach_report / max_pitch  ← thay workspace_limits AABB     │
│  SceneManager  ← THÊM MỚI so với task4: giá + bàn thành vật cản, ống được attach │
│  rx150_perception (yolo + gesture + TF hand-eye)  ← thay YOLO/MediaPipe/M02      │
│  move_group (OMPL + TOTG)                        ← thay mr.IKinSpace             │
└─────────────────────────────────────┬──────────────────────────────────────────┘
                                      ▼   /rx150/arm_controller/follow_joint_trajectory
┌── CONTROL ─── trajectory_bridge 100 Hz → fuzzy_node PWM ───────────────────────┐
└─────────────────────────────────────┬──────────────────────────────────────────┘
                                      ▼
┌── DRIVER ─── xs_sdk · realsense2_camera ───────────────────────────────────────┐
└─────────────────────────────────────┬──────────────────────────────────────────┘
                                      ▼   RX150 · D435i · AprilTag
```

So với sơ đồ mục 1, **chỉ có duy nhất khối ★ là mới**. Ba tầng dưới không đổi một dòng.
Đó chính là mục đích của cấu trúc phân tầng: đổi ứng dụng không phải đụng vào điều khiển.

---

## 5. Bảng ánh xạ từng khối của bản cũ

Đọc theo chiều "bài cũ làm thế này → hệ mới thay bằng cái gì":

| Khối trong `task4_full.py` | Dòng | Đi về đâu | Thay bằng |
| :--- | :--- | :--- | :--- |
| `pyrealsense2` pipeline / align / intrinsics | 173–180 | Driver | `realsense2_camera` (đã có trong `fuzzy_moveit.launch.py`) |
| `YOLO(...)`, mask → contour → centroid → deproject | 185, 253–292 | App Support | `yolo_tube_detector_node` → `/yolo/detected_tubes` |
| HSV nắp cam → `roll` | 273–290 | App Support | phần `cap_*` của node YOLO → `yaw` trong PoseArray + `/yolo/tube_classes` |
| MediaPipe Hands, EMA, tia chỉ, OK-sign | 183–184, 206–224, 294–321 | App Support | `hand_gesture_node` → `selected_target` + `event` |
| `M0·M1·M2`, `cam_to_robot()` | 42–68, 188 | App Support (TF) | AprilTag + `easy_handeye2` → TF `rx150/base_link ← camera_color_optical_frame` |
| `workspace_limits`, `closest_point_in_workspace` | 99–104, 226–235 | App Support | `Rx150Kinematics.reach_report()` / `max_pitch()`, `PickPlaceSkill.clamp_z()` |
| `rack_bases`, `rack_slots`, `rack_status` | 70–95 | **config** | `tube_rack_params.yaml`: `slot0_x/y/z`, `rack_yaw_deg`, `slot_spacing`, `num_slots` |
| `InterbotixManipulatorXS`, `set_ee_pose_components` | 107, 125–165 | **BỎ** | `MoveItExecutor` + `PickPlaceSkill` qua `move_group` |
| `bot.gripper.grasp()/release()` | 129, 140… | App Support | `rx150_modules.Gripper.close(verify=True)` (xác nhận kẹp được) |
| `robot_queue` + `robot_worker` thread | 114–170 | Application | hàng lệnh trong node — theo mẫu `hri_motion_node._cmds` |
| **FSM chỉ tay + dwell + `rack_status`** | **237–456** | **Application — GIỮ** | thân của `hri_pointing_node` |
| `FalconTubeNode` publish ảnh | 23–31, 482–489 | **BỎ** | driver RealSense đã publish sẵn |

Đếm dòng: 501 dòng → còn khoảng **90 dòng logic quyết định** + ~60 dòng khai báo tham
số/topic. Phần biến mất không phải bị xoá, mà là **đã tồn tại sẵn ở tầng dưới**.

---

## 6. Cái gì đã đổi, cái gì còn thiếu

**Đã đổi so với bài cũ** — số liệu chống lưng nằm sẵn trong `docs/tuning/`:

| Trục | Bản cũ (`task4_full.py`) | Hệ mới | Bằng chứng trong kho |
| :--- | :--- | :--- | :--- |
| Điều khiển khớp | POSITION mode, PID trong firmware Dynamixel — không quan sát, không sửa được | PWM vòng kín 100 Hz trên host: fuzzy Mamdani + bù trọng lực Pinocchio; đổi sang FF hoặc HAC mà app không phải sửa | `docs/tuning/comparison_data.csv` (606 dòng: e/ė/PWM/gravity của fuzzy vs HAC), `control_surface_3d_*.png`, `trajectory_comparison.png` |
| Sinh quỹ đạo | 5–8 lệnh pose tuyệt đối nối tiếp, không planner | OMPL + TOTG qua `move_group`, có planning scene + Octomap | `rx150_modules/motion.py` |
| Động học ngược | `mr.IKinSpace` — Newton lặp, 3 seed cố định, **fail ngẫu nhiên ở điểm biên** | IK giải tích 5-DoF, tất định, có pytest phủ | `rx150_modules/test/test_kinematics.py` |
| Xác nhận | tin `error_code`, không kiểm tra gì | `verify_reached()` đọc `joint_states`; `Gripper.close(verify)` đo `left_finger` → xác nhận **kẹp được vật** | `rx150_modules/motion.py`, `gripper.py` |
| Tránh va chạm | không có | `SceneManager`: bàn + giá là vật cản, ống được attach khi cầm | `rx150_modules/scene.py` |
| Hand-eye | ma trận `M02` cứng, θ = 40° đo tay | AprilTag + `easy_handeye2` → TF | `rx150_perception/launch/handeye_*.launch.py` |
| Nhận diện | YOLO in-process, 1 frame, không lọc | node riêng: EMA + tracker `min_hits`, ROI box, cổng độ sâu, chạy GPU | `yolo_detector_params.yaml`; đo được: **5.7 ms** (A4000) vs **68 ms** (CPU) |
| Tái lập | mọi hằng số nằm trong code | `config/*.yaml`; `rx150_reach_check.py` kiểm tầm với **trước khi** cấp điện | `./rx150.sh reach` (B0, không cần robot) |

**Còn thiếu, nếu muốn dựng lại kịch bản HRI của bài cũ trên hệ mới để đo A/B:**

1. `hand_gesture_node` mới phát *chỉ số vật gần tia nhất* (`~/selected_target`), chưa phát
   **bản thân tia**. FSM cũ cần tia để chỉ vào **giá** — giá không phải vật YOLO. Dữ liệu
   đã có sẵn trong `_sync_cb`, chỉ thiếu một publisher `~/pointing_ray`.
2. Chưa có app node cho luồng pick/place/handover theo cử chỉ. Gần nhất là
   `pick_place_moveit_node` (gắp vật được chỉ → thả điểm cố định); thiếu nhánh
   place-vào-giá-được-chỉ và nhánh handover.
3. Sáu lỗi nhỏ của FSM cũ ở mục 3 — nếu dựng lại thì sửa, đừng chép.
