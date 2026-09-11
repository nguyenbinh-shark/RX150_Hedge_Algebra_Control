# Sơ đồ cấu trúc điều khiển RX150

Hệ điều khiển hiện tại nối với nhau như thế nào: tên node, tên topic/action, tần số, ai
đóng vòng ở đâu. Nguồn: `src/rx150/**` (đọc ngày 2026-09-08).

---

## Sơ đồ cấu trúc điều khiển

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

## Đối chiếu với bản cũ

Phần so sánh với `key_point/task4_full.py` (bản cũ một-process, dùng làm baseline) đã
tách sang [docs/lich_su/doi_chieu_key_point.md](lich_su/doi_chieu_key_point.md) — gồm sơ
đồ bản cũ, bảng ánh xạ từng khối, và bảng "đã đổi những gì".
