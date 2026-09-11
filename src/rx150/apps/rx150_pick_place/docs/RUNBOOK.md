# RUNBOOK — chạy & debug RX150 trên phần cứng thật

Tài liệu này để **cầm khi đứng cạnh robot**. Trọng tâm: nhiệm vụ `tube_rack`
(ống nghiệm → giá) với `motion_backend: moveit`. Các đường khác nằm ở §7.

Nguyên tắc duy nhất: **chạy theo thang bậc, hỏng ở bậc nào thì dừng ở bậc đó.**
Mỗi bậc có tiêu chí GO/NO-GO và lệnh thu bằng chứng. Đừng nhảy cóc — 80% thời gian
debug bị mất là do chạy B6 rồi mới phát hiện B2 chưa GO.

Tài liệu liên quan: [`../README.md`](../README.md) (kiến trúc + tham số),
[`rx150_perception/docs/PERCEPTION_GUIDE.md`](../../../rx150_toolbox/rx150_perception/docs/PERCEPTION_GUIDE.md)
(nhận diện + hiệu chuẩn TF, có nhật ký sự cố).

---

## 0. An toàn — đọc trước khi cấp điện

**Controller bật torque và bơm PWM ngay khi launch, không có cổng arming.**
`fuzzy_node` (và `hac_node`, `ff_node`) đặt cả nhóm `arm` sang PWM mode rồi
`torque_enable=true` ngay khi `/rx150/get_robot_info` trả lời. Sau đó timer 100 Hz
chạy liên tục.

`fuzzy_moveit.launch.py` ép `enable_profile: False`, nên **chừng nào bridge chưa gửi
setpoint**, `q_ref = reference_pose = [0, −1.80, 1.55, 0.8, 0]` (tư thế **sleep**)
được đưa vào **như một bước nhảy**, không qua Ruckig làm mượt. Tay đang ở xa sleep
⇒ giật mạnh.

**Checklist mỗi lần launch:**

1. `pkill -f xs_sdk` — hai driver trên một bus U2D2 là tranh chấp serial, không phải
   lỗi sạch. Mọi `*_moveit.launch.py` và `*_control.launch.py` đều tự khởi động một
   `xs_sdk` riêng.
2. Đặt tay về gần tư thế sleep trước khi cấp nguồn.
3. Dọn thoáng bán kính ~0.5 m quanh gốc robot.
4. E-stop / công tắc nguồn trong tầm với.

**Gripper giữ lực sau khi kẹp.** `gripper_trajectory_bridge` **cố ý không** đưa PWM
về 0 sau một lần grasp thành công (để giữ vật lúc vận chuyển). Nghĩa là sau một chu
kỳ lỗi, ngón kẹp có thể còn đang bóp vô thời hạn. Luôn kết thúc phiên bằng:

```bash
ros2 service call /tube_rack/open_gripper std_srvs/srv/Trigger
# hoặc dứt khoát hơn:
ros2 service call /rx150/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
    "{cmd_type: 'group', name: 'arm', enable: false}"
```

**Dừng khẩn cấp mềm** (huỷ goal đang chạy, chặn goal mới):

```bash
ros2 service call /tube_rack/stop  std_srvs/srv/Trigger
ros2 service call /tube_rack/reset std_srvs/srv/Trigger   # chạy lại sau khi dừng
```

---

## 1. Quy tắc môi trường

```bash
source ~/RX150_Hedge_Algebra_Control/source_all.sh
```

`source install/setup.bash` **là chưa đủ**: `apriltag_ros` nằm ở `~/apriltag_ws`,
`easy_handeye2` ở `~/easy_handeye2_ws`. `source_all.sh` source đúng 4 tầng
(ros → apriltag_ws → easy_handeye2_ws → interbotix_ws). Wrapper `./rx150.sh` tự làm
việc này.

### ⚠️ Kiểm tra overlay `~/ws_moveit` trước khi launch

`~/ws_moveit` là overlay MoveIt **cũ**: nó che MoveIt hệ thống và cần
`libgeometric_shapes.so.2.3.2` trong khi Humble ship 2.3.4 (đang được vá bằng symlink
`~/.local/lib/ros_shim/` — chạy được, nhưng là ABI nói dối).

`.bashrc` **không** source nó (chỉ có alias `moveit`). Vấn đề là env **kế thừa**: nếu
terminal nào đó đã gõ `moveit` rồi từ đó mở IDE / mở terminal con, thì mọi shell con
đều mang theo overlay đó. Đã quan sát thấy đúng tình huống này trên máy bạn.

**Luôn kiểm một dòng trước khi launch:**

```bash
ros2 pkg prefix moveit_ros_move_group
```

- `/opt/ros/humble` → đúng.
- `/home/hust/ws_moveit/install/...` → **SAI**. Mở một terminal **mới hoàn toàn**
  (không phải terminal con của phiên đang có overlay) rồi `source_all.sh` lại.

So sánh với shell sạch để chắc chắn:

```bash
env -i HOME=$HOME bash -lc 'source ~/RX150_Hedge_Algebra_Control/source_all.sh >/dev/null; \
    ros2 pkg prefix moveit_ros_move_group'
```

`tools/collect_diag.sh` tự kiểm mục này và ghi vào `00_env.txt`.

**Symlink install:** `rx150_pick_place` và `rx150_perception` cài bằng
`--symlink-install`, nên sửa `.py`/`.yaml` trong `src/` **có hiệu lực ngay**, không
cần build lại. **Ngoại lệ:** `interbotix_xsarm_moveit` (SRDF, `rx150_controllers.yaml`,
`joint_limits`) cài bằng **bản sao thật** — sửa nguồn thì phải:

```bash
colcon build --packages-select interbotix_xsarm_moveit
```

---

## 2. Quy tắc TF camera — MỘT nguồn, không hơn không kém

Đúng **một** trong ba thứ sau được phát `world ↔ camera`:

| Nguồn | Bật bằng | Ghi chú |
|---|---|---|
| `static_trans_pub` | chạy `rx150_perception.launch.py` | **Đây là nguồn dùng hằng ngày.** Đọc `rx150_perception/config/static_transforms.yaml` |
| `camera_static_tf` | `use_camera_static_tf:=true` | TF **cứng** `(1, 0, 1)` — chỉ là placeholder chưa hiệu chuẩn |
| `handeye_publisher` | `use_handeye_publisher:=true` | Cần `~/.ros/easy_handeye2/rx150_eob.yaml` — **file này chưa tồn tại** |

`fuzzy_moveit.launch.py` **không** chạy `static_trans_pub`. Nên tổ hợp
`use_camera_static_tf:=false use_handeye_publisher:=true` mà không chạy perception
sẽ cho **không có TF nào cả**.

Chiều TF là `camera_color_optical_frame → world` (camera làm **cha**), cố ý ngược
trực giác: RealSense đã sở hữu `camera_color_frame → camera_color_optical_frame`, nếu
đặt `world` làm cha thì `camera_color_optical_frame` có **2 cha** và cây TF gãy.
Chuỗi đúng: `camera_link → camera_color_optical_frame → world → rx150/base_link`.

**Lệnh kiểm tra duy nhất cần nhớ:**

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
```

Không ra số ⇒ mọi thứ ở tầng nhận diện đều vô nghĩa, đừng đi tune `conf_threshold`.

**Các frame AprilTag KHÔNG nằm trong luật này.** `rack_tag`, `ee_tag`, `tube_rack`,
`tube_rack/slot*` đều là frame **lá** (con của `camera_color_optical_frame` hoặc của
`rx150/base_link`), không ai khác phát chúng, nên chạy `rack-calib` / `eetag` song song
với `t2` là an toàn. Chỉ cạnh `world ↔ camera` mới bị giới hạn một nguồn.

Ngược lại: các phép đo bằng tag **nằm trên nền** TF camera. Calib camera sai ⇒
`rack_pose.yaml` chỉ chép lại đúng cái sai đó. Luôn xong B2 trước B2b.

---

## 3. Bản đồ hệ thống

Namespace: **mọi thứ dưới `/rx150`, TRỪ `move_group` và `rviz2`** (chúng ở namespace
toàn cục — là `/move_group`, không phải `/rx150/move_group`).

```
[L0]  U2D2 /dev/ttyDXL ── Dynamixel ── 5 khớp + gripper   |   RealSense D435i (USB3)

[L1]  /rx150/xs_sdk        ──► /rx150/joint_states   (100 Hz, sensor_msgs/JointState)
      motor cfg: rx150_motion_common/config/rx150_motor.yaml
                           ◄── /rx150/commands/joint_group  (JointGroupCommand, PWM)
                           ◄── /rx150/commands/joint_single (JointSingleCommand, PWM)
        srv /rx150/{torque_enable, set_operating_modes, get_robot_info, reboot_motors,
                    set_motor_pid_gains, set_motor_registers, get_motor_registers}
      /rx150/robot_state_publisher ──► /tf, robot_description
      /camera/camera/… ──► color/image_raw, aligned_depth_to_color/image_raw,
                           color/camera_info

[L2]  /rx150/fuzzy_node   100 Hz PWM vòng kín (PD mờ + bù trọng lực Pinocchio)
        ──► /rx150/fuzzy/{reference, error, edot, effort, gravity}
        ◄── /rx150/fuzzy/setpoint  (std_msgs/Float64MultiArray)
      /rx150/fuzzy_trajectory_bridge   (rx150_motion_common/rx150_trajectory_bridge.py)
        action /rx150/arm_controller/follow_joint_trajectory
      /rx150/gripper_trajectory_bridge (rx150_fuzzy_controller)
        action /rx150/gripper_controller/follow_joint_trajectory
        đóng = PWM ÂM, mở = PWM DƯƠNG; stall lúc đóng ⇒ SUCCESS (và GIỮ nguyên lực)

[L3]  /move_group  ──► action move_action, execute_trajectory
                   srv /apply_planning_scene, /get_planning_scene
      nhóm: interbotix_arm (5 khớp) | interbotix_gripper (left_finger)
      moveit_manage_controllers = False

[L4]  /yolo_tube_detector ──► /yolo/detected_tubes (PoseArray, frame rx150/base_link)
                           ──► /yolo/tube_classes (String JSON)
                           ──► /yolo/markers, /yolo/image_debug
      /hand_gesture        ──► /hand_gesture/selected_target (Int32)
                           ──► /hand_gesture/event (String, "ok_sign")
      /static_trans_pub    ──► TF camera_color_optical_frame → world
      /rack_tag            ──► /rack_tag/tag_detections (chỉ trong phiên hiệu chuẩn giá)
      /rack_calib          ──► rack_pose.yaml + TF rx150/base_link → tube_rack
                           ──► /rack_calib/markers (4 lỗ, viền giá, trục lỗ)
                           ──► /rack_calib/image_debug (4 lỗ chiếu ngược lên ảnh)

[L5]  /tube_rack  |  /pick_place_moveit
        ──► /tube_rack/status  (String JSON: state, detail, attempted, succeeded,
                                faults, success_rate, last_cycle_s, last_error, slots)
        srv ~/run ~/stop ~/reset ~/home ~/open_gripper   (+ ~/pick cho pick_place)
```

**Nguyên tắc chẩn đoán:** mỗi mũi tên là một chỗ cắt được. Debug = tìm mũi tên **đầu
tiên tính từ dưới lên** mà dữ liệu không đi qua. Thang bậc §4 chính là cách quét đó.

---

## 4. Thang bậc bring-up

| Bậc | Mục tiêu | Robot? | GO khi |
|---|---|---|---|
| B0 | Toán học + config | Không | pytest 20/20; `rx150_reach_check` exit 0 cả 2 config |
| B1 | Driver + TF | Có | `/rx150/joint_states` ~100 Hz, 6 khớp; không frame 2 cha |
| B2 | Camera + TF calib | Có | color + aligned_depth có hz; `tf2_echo` ra số ổn định |
| B2b | Vị trí giá (tùy chọn) | Có | `rack_pose.yaml` có 4 lỗ, tản mát ≤ 3 mm; `reach` exit 0 |
| B3 | Nhận diện | Có | `/yolo/detected_tubes` frame `rx150/base_link`, khớp thước ±1 cm |
| B4 | Chuyển động khô | dry_run | Chạy hết state machine, đủ waypoint, không gửi goal |
| B5 | Chuyển động thật, tay không | Có | `~/home`, `~/open_gripper` đúng |
| B6 | Chu kỳ đầy đủ | Có | 1 ống → 1 slot, `/status` `succeeded ≥ 1` |

### B0 — không cần robot

```bash
source ~/RX150_Hedge_Algebra_Control/source_all.sh
# Test đã dời sang rx150_modules khi tách tầng Application Support.
python3 -m pytest src/rx150/rx150_toolbox/rx150_modules/test -q        # 20 passed
cd ~/RX150_Hedge_Algebra_Control && ./rx150.sh reach                                 # gộp cả 3 lệnh dưới
```

`./rx150.sh reach` chạy sẵn đúng bộ này:

```bash
ros2 run rx150_pick_place rx150_reach_check.py            # bảng bao hình pitch/r/z
for f in tube_rack_params pick_place_params; do
  ros2 run rx150_pick_place rx150_reach_check.py --config \
    $(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config/$f.yaml
done

# Tư thế TRUNG CHUYỂN — kiểm riêng, xem giải thích bên dưới
ros2 run rx150_pick_place rx150_reach_check.py --point 0.18 0 0.18 --pitch 0
```

**GO:** pytest 20 passed; cả hai `--config` exit 0; điểm `(0.18, 0, 0.18)` với tới được.

**Vì sao dòng cuối quan trọng:** `home_xyz_pitch: [0.18, 0, 0.18, 0]` được giải IK lúc
khởi động để thay `home_joints`. Nếu IK **fail**, `resolve_home()` chỉ log ERROR rồi
**âm thầm quay lại `home_joints` thô `[0,0,0,0,0]`** — FK của nó là `(0.359, 0, 0.255)`,
tức tay **duỗi thẳng vượt QUA giá ở x = 0.26**. Mỗi lần "về home" sẽ quét ngang đúng
chỗ cắm ống — đúng cái mà `home_xyz_pitch` sinh ra để tránh. Đây là chế độ hỏng âm
thầm nguy hiểm nhất.

rx150 **không** chúc thẳng đứng (pitch 90°) được ở mọi nơi — hết tầm từ r ≈ 0.29 m,
còn ~64° ở r = 0.36 m. Rất nhiều ca "gắp không được" chỉ là điểm gắp nằm ngoài bao
hình khả thi; bảng của `rx150_reach_check` cho biết ngay, miễn phí.

Chạy lại B0 **mỗi khi đo lại vị trí giá/bàn** (`slot0_*`, `place_*`, `reject_*`).

### B1 — driver + TF

```bash
pkill -f xs_sdk
./rx150.sh t1
```

Kiểm ở terminal khác:

```bash
ros2 topic hz /rx150/joint_states                 # ~100 Hz
ros2 topic echo /rx150/joint_states --once        # 6 tên: 5 khớp + left_finger (+right)
ros2 node list | sort
ros2 action list | grep follow_joint_trajectory   # arm_controller + gripper_controller
ros2 run tf2_tools view_frames                    # sinh frames.pdf — tìm frame 2 cha
```

**GO:** `joint_states` ~100 Hz với đủ tên khớp; có **cả hai** action server
`/rx150/arm_controller/follow_joint_trajectory` và `/rx150/gripper_controller/…`;
`view_frames` không có frame nào 2 cha.

**NO-GO thường gặp:** thiếu `gripper_controller` ⇒ bạn đang chạy nhánh
`ff_moveit.launch.py` (nhánh này không khởi động gripper bridge).

### B2 — camera + TF calib

```bash
./rx150.sh t2
```

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
```

**GO:** cả hai ảnh có hz ổn định; `tf2_echo` ra translation/rotation **không nhảy**.

**NO-GO:** `tf2_echo` báo không tìm được ⇒ xem §2. TF nhảy liên tục ⇒ đang có 2 nguồn.

Cần hiệu chuẩn lại (đã xê dịch camera): `./rx150.sh calib`, đưa AprilTag vào khung
hình, đặt Snapshots = 10, bấm **Snap Pose**. Kết quả tự ghi vào `static_transforms.yaml`.
Xong thì Ctrl+C và quay về `./rx150.sh t2` — đừng để GUI mở lâu (rò timer 20 Hz).

> **Snap Pose chỉ nên coi là bản nháp.** Nó giải 6 ẩn từ **một** tư thế duy nhất. Đo
> 2026-09-11: bản Snap Pose đang dùng lệch **5.61° và 29 mm**, làm toạ độ 4 miệng lỗ sai
> tới **19 mm** và bịa ra 4° "nghiêng giá" không có thật. Bản chuẩn dùng tag trên tay gắp
> với 39 tư thế:
>
> ```bash
> ./rx150.sh eetag           # T3: detector tag tay gắp
> ./rx150.sh rack-calib      # biết giá đang đứng đâu để lưới pose tránh
> ./rx150.sh eetag-calib     # ~5 phút → tuning_runs/<run>/static_transforms_refined.yaml
> # TẮT T2 rồi mới chép đè lên config/static_transforms.yaml, sau đó bật lại T2
> ./rx150.sh rack-calib      # BẮT BUỘC snap lại: rack_pose nằm TRÊN NỀN TF vừa đổi
> ```
>
> Kiểm chứng: `./rx150.sh eetag-pick`. Số đo:
> [docs/tuning/do_chinh_xac_camera_robot.md](../../../../../docs/tuning/do_chinh_xac_camera_robot.md).
>
> **Gá tag tự thiết kế thì phải đo vị trí tag TRƯỚC** (`./rx150.sh eetag-tagcal`):
> `ar_tag_link` trong URDF là vị trí gá CHÍNH HÃNG, và Snap Pose sai đúng bằng độ
> lệch của gá. Thứ tự bắt buộc **tag → camera → giá**; quy trình đầy đủ:
> [docs/tuning/hieu_chuan_tag_va_camera.md](../../../../../docs/tuning/hieu_chuan_tag_va_camera.md).

### B2b — vị trí giá đỡ ống nghiệm (chỉ khi giá đã dán AprilTag)

Cùng khuôn với B2, khác đối tượng đo:

| | đo cái gì | công cụ | file kết quả | ai đọc file |
|---|---|---|---|---|
| B2 | camera ở đâu | armtag GUI | `static_transforms.yaml` | `static_trans_pub` → TF |
| B2b | **giá ở đâu** | `rack_calib` | `rack_pose.yaml` | `tube_rack_node` (đọc thẳng) |

**Nhìn trước, tin sau.** Bước đầu tiên là mở GUI, KHÔNG ghi file gì:

```bash
./rx150.sh rack-gui          # T3 — T1 + T2 phải đang chạy. Không ghi đè gì cả.
```

RViz mở ra hai khung:

| Khung | Trả lời câu hỏi |
|---|---|
| 3D (marker `/rack_calib/markers`) | 4 lỗ tính ra nằm đúng chỗ so với robot chưa |
| Ảnh (`/rack_calib/image_debug`) | **4 lỗ đó chiếu ngược lên ảnh camera có TRÙNG lỗ thật không** |

Khung ảnh mới là bằng chứng thật: nó đi qua **cả** TF hiệu chuẩn camera, **cả** pose
tag, **cả** hình học khai trong `rack_calib.yaml`. Bốn vòng tròn nằm khít 4 miệng lỗ
⇒ toàn chuỗi đúng. Lệch đều một hướng ⇒ calib camera (B2). Lệch kiểu xoay/gương ⇒
hình học khai sai (`yaw_offset_deg`, `slot_index_map`).

**Lục = nghiệm đang đo. Cam = nghiệm đã lưu trong `rack_pose.yaml`.** Hai bộ chồng
khít nhau nghĩa là giá chưa xê dịch từ lần hiệu chuẩn trước; tách ra bao nhiêu thì
dòng `lech live vs file: … mm` trên góc ảnh nói thẳng.

Nhìn thấy 4 vòng tròn đúng chỗ rồi mới snap:

```bash
./rx150.sh rack-calib             # ghi rack_pose.yaml
./rx150.sh rack-calib rviz:=true  # vừa snap vừa soi
```

> **Bản đồ id tag của hệ** — đổi một chỗ là phải đổi đủ:
>
> | id | Tag | Khai trong |
> |---|---|---|
> | **1** | tay gắp (`rx150/ar_tag_link`, con của `ee_arm_link`) | `tags.yaml`, `apriltag_calib.yaml`, `ee_tag.yaml` |
> | **0** | giá đỡ ống nghiệm | `rack_tag.yaml`, `rack_calib.yaml` (`tag_id`) |
>
> Hai id khác nhau nên mỗi detector chỉ khai id của mình ⇒ **không bao giờ nhìn thấy
> tag của bên kia**, dù cả hai cùng nằm trong khung hình. Đây cũng là thứ giữ an
> toàn cho `./rx150.sh calib`: Interbotix armtag lấy `detections[0]` — tag **đầu
> tiên** trong khung, **không** lọc theo id (`apriltag.py:161`, `TODO: support for
> multiple tags`); nó chỉ an toàn vì `tags.yaml` khai đúng một id.
>
> `rack_calib` còn một lọc dự phòng theo TF (`ignore_near_frames`) cho trường hợp
> buộc phải cho hai tag dùng chung id — mặc định **TẮT**, xem `rack_calib.yaml`.

Tag phải nằm trong khung hình và không bị che. Node lấy ~30 mẫu (~1,5 s), lấy **trung
vị** vị trí + **trung bình quaternion**, rồi in bảng 4 miệng lỗ trong `rx150/base_link`
và ghi `rx150_perception/config/rack_pose.yaml`.

**GO:** tản mát vị trí ≤ 3 mm, tản mát góc ≤ 1,5°, 4 miệng lỗ khớp thước trong ±5 mm.

**NO-GO thường gặp:**

| Log | Nguyên nhân |
|---|---|
| `KHÔNG có frame nào trên /rack_tag/tag_detections` | detector chưa lên, hoặc thiếu remap `~/image_rect` |
| `thấy tag id [1] nhưng cấu hình đang đòi id 0` | đang chĩa vào tag TAY GẮP chứ không phải tag giá — đưa giá vào khung hình |
| `Còn 2 tag id 0 … — BỎ khung này` | có tag id 0 thứ hai trong khung (dán nhầm / in trùng) — cất đi rồi snap lại |
| `TF rx150/base_link←camera_color_optical_frame` | chưa chạy T2 (§2) |
| tản mát > 3 mm | tag quá xa / quá nghiêng / ảnh mờ — kéo giá lại gần, snap lại |
| tản mát **hàng trăm mm** | không phải nhiễu ảnh: đang có **2 nguồn TF** `world↔camera` (§2), TF nhảy giữa hai giá trị |
| khung ảnh trống, không có vòng tròn | chưa có `camera_info`, hoặc `image_topic` sai — `ros2 topic hz /rack_calib/image_debug` |
| 4 lỗ ra **đối xứng gương** so với thực tế | tag dán quay ngược ⇒ `yaw_offset_deg: 180`, đừng đảo dấu từng trục |

Hình học giá khai trong `rx150_perception/config/rack_calib.yaml` — mặc định là 4 lỗ
ở 4 đỉnh hình chữ nhật **120 × 50 mm**, tag ở **trung điểm cạnh dài gần**:

```
    xa   (2)─────────────────(3)     v = rect_short_m  = 0.050
          │                   │
          │        ▲ v        │
    gần   (0)──────┼────────(1)      u = ±rect_long_m/2 = ±0.060
                  TAG ──► u
```

Chỉ số 0..3 chính là chỉ số `color_slot_map` đang dùng (pink→0, blue→1, green→2,
yellow→3). Đổi thứ tự bằng `slot_index_map`, **đừng** sửa hình học.

Sau khi snap:

```bash
./rx150.sh reach             # 4 lỗ có với tới được không (dùng LUÔN số vừa đo)
./rx150.sh rack-pub          # tùy chọn: hiện frame tube_rack + 4 slot trong RViz
./rx150.sh rack-watch        # kiểm tra giá có bị xê dịch (đo, KHÔNG ghi đè file)
```

`rack_pose.yaml` **thắng** `slot0_*`/`slot_spacing`/`rack_yaw_deg` trong
`tube_rack_params.yaml`. `tube_rack_node` in rõ nguồn nào đang dùng lúc khởi động:

```
Hình học giá lấy từ HIỆU CHUẨN: …/rack_pose.yaml (tag 1, 30 mẫu, 2026-09-10T…)
CHƯA hiệu chuẩn giá (không có …/rack_pose.yaml) ⇒ dùng slot0_*/… đo bằng thước
```

Chưa dán tag lên giá thì **bỏ qua cả mục này** — đường đo thước cũ chạy y như trước.

**Bao lâu phải hiệu chuẩn lại?** Đây là việc **một lần cho mỗi lần đặt giá**, không
phải việc làm mỗi chu kỳ:

| Tình huống | Cần `rack-calib` lại? |
|---|---|
| Tắt máy, hôm sau bật lại, giá không ai đụng | **Không** — số nằm trong file, không nằm trong RAM |
| Chạy `tubes` nhiều chu kỳ liên tiếp | **Không** — node đọc file một lần lúc khởi động |
| Giá bị đụng / dời / kê lại | **Có** |
| Vừa chạy `./rx150.sh calib` (hiệu chuẩn lại camera) | **Có** — toạ độ giá tính trên nền TF camera |
| Đổi giá khác, đổi kích thước hình chữ nhật | **Có** (sửa `rack_calib.yaml` trước) |

Không chắc thì đừng đoán, hỏi thẳng: `./rx150.sh rack-watch` in độ lệch mm giữa giá
**đang thấy** và giá **trong file**. Lệch ≤ 5 mm là còn dùng được; hơn thì snap lại.

`tube_rack_node` **cố ý** đọc file chứ không bám TF `tube_rack` lúc chạy: một chu kỳ
gắp không được đổi mục tiêu giữa chừng vì camera rung hay vì tay máy che mất tag.

> Lần đầu snap, file được ghi vào `src/` và chép tạm một bản sang `install/`. Chạy
> `./tools/build.sh --packages-select rx150_perception` một lần để `install/` trỏ
> symlink về `src/`; sau đó snap lại là ăn ngay, không cần build.

### B3 — nhận diện

```bash
./rx150.sh test                                   # 14 test tự động của detector, ~5 s
ros2 topic hz /yolo/detected_tubes                # ~5 Hz (detect_rate_hz)
ros2 topic echo /yolo/detected_tubes --once       # frame_id PHẢI = rx150/base_link
ros2 topic echo /yolo/tube_classes --once
```

Đặt **một** ống ở vị trí đo được bằng thước, so toạ độ trong `/yolo/detected_tubes`.

**GO:** `frame_id = rx150/base_link`, hz ổn định, toạ độ khớp thước trong ±1 cm.

**Lệch có hệ thống** (mọi ống đều lệch cùng một hướng) ⇒ lỗi calib TF, **không** phải
lỗi YOLO. Quay lại B2.

Công cụ tune khi cần:

```bash
ros2 param set /yolo_tube_detector log_gate_metrics true   # in số đo từng detection
ros2 param set /yolo_tube_detector enable_roi_box false    # bisect khi KHÔNG thấy gì
ros2 param dump /yolo_tube_detector | grep -E 'roi_|enable_roi_box'
```

Hộp ROI (`rx150_perception/config/roi_box_params.yaml`) lọc theo toạ độ trong
`rx150/base_link`, nên **calib sai ⇒ hộp nằm sai chỗ dù số trong file đúng**. Tắt ROI
là cách phân biệt "không detect được" với "detect được nhưng bị hộp loại".

### B4 — chuyển động khô (dry_run), không cần camera

```bash
./rx150.sh dry-fake
```

tương đương:

```bash
ros2 launch rx150_pick_place tube_rack.launch.py \
    detector:=false dry_run:=true \
    fake_tubes:='[{"x":0.20,"y":-0.15,"z":0.03,"yaw":0.0,"class":"pink"}]'
```

Chạy **toàn bộ** state machine (nhận diện giả, IK, kế hoạch, log từng waypoint) mà
không gửi goal nào. Đây là chỗ bắt lỗi hình học rẻ nhất.

Trong log phải thấy đủ ba thứ:

1. `Tư thế trung chuyển (0.180,0.000,0.180) pitch=0° → home_joints = [...]°`
   — nếu thay vào đó là `home_xyz_pitch … KHÔNG với tới`, dừng lại và xử lý (xem B0).
2. Bảng slot nào với tới được (in mỗi lần khởi động).
3. `Kế hoạch: … → VIA-GIÁ … → HOVER … → INSERT → slot k`
   — thiếu `VIA-GIÁ` nghĩa là `via_staging` tắt hoặc cột trung chuyển không với tới.

> `ros2 run … -p fake_tubes:='[{…}]'` **không dùng được**: `rcl` parse giá trị `-p`
> theo YAML nên JSON array thành flow-sequence → `Unknown YAML event`. Dùng launch
> arg như trên, hoặc `--params-file`.

### B5 — chuyển động thật, tay không cầm gì

**B5a — backend `moveit` (baseline, làm trước):**

```bash
ros2 launch rx150_pick_place tube_rack.launch.py detector:=false auto_start:=false
```

```bash
ros2 topic echo /tube_rack/status                        # terminal riêng, để chạy suốt
ros2 service call /tube_rack/home std_srvs/srv/Trigger
ros2 service call /tube_rack/open_gripper std_srvs/srv/Trigger
```

**GO:** tay về đúng tư thế trung chuyển (thu gọn, **sau lưng** giá — không duỗi thẳng
vượt qua giá); ngón kẹp mở; `/status` về `IDLE` không có `last_error`.

**B5b — backend `direct`, CHỈ sau khi B5a GO:**

```bash
ros2 launch rx150_pick_place tube_rack.launch.py detector:=false auto_start:=false \
    motion_backend:=direct
```

Khác biệt cần biết trước, **không phải lỗi**:

- Mỗi thao tác planning scene **treo ~3 s rồi WARN** ("Planner sẽ KHÔNG biết vật cản
  này") nếu `move_group` không chạy — `SceneManager` vẫn gọi `/apply_planning_scene`
  với `service_timeout = 3.0`.
- Gripper **tự chuyển sang PWM trực tiếp** (có log giải thích): đường bridge của
  gripper đi qua chính `move_group`. Nhớ để `set_gripper_pwm_mode: true`.
- **Không có tránh vật cản.** An toàn nằm ở waypoint (cột trung chuyển, rút thẳng
  đứng, lùi bán kính), không ở planner.

So sánh 2 backend **ở đây**, không phải ở B6 — cùng waypoint, tay không cầm gì. Bag
lại rồi so quỹ đạo đặt vs thực:

```bash
./tools/record_pickplace.sh moveit     # rồi chạy B5a
./tools/record_pickplace.sh direct     # rồi chạy B5b
```

### B6 — chu kỳ đầy đủ

```bash
./rx150.sh t1        # terminal 1
./rx150.sh t2        # terminal 2
./rx150.sh tubes     # terminal 3  (detector chạy ở đây — đúng MỘT instance)
```

Tay để trên `~/stop`. Bắt đầu bằng **một** ống.

```bash
ros2 topic echo /tube_rack/status
```

**GO:** `attempted = 1`, `succeeded = 1`, ống nằm đúng slot theo `color_slot_map`.

> Nếu bạn đi đường một-terminal `fuzzy_moveit_perception.launch.py` (có sẵn
> `use_yolo:=true`) thì **bắt buộc** `tube_rack.launch.py detector:=false`, nếu không
> sẽ có **hai** node tên `yolo_tube_detector`.

---

## 5. Bảng triage: triệu chứng → nghi ngờ → lệnh xác minh

| Triệu chứng | Nghi ngờ đầu tiên | Lệnh xác minh |
|---|---|---|
| `xs_sdk` crash / stack smashing | 2 instance trên 1 bus | `pgrep -af xs_sdk` → `pkill -f xs_sdk` |
| `ModuleNotFoundError: rx150_pick_place` | chưa source (env-hook PYTHONPATH) | `echo $PYTHONPATH \| tr : '\n' \| grep rx150` |
| `handeye_publish` fail lúc launch | chưa calib easy_handeye2 | `ls ~/.ros/easy_handeye2/` |
| **YOLO không ra detection nào** | **không có TF camera↔world (§2)** | `ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link` |
| Detection có nhưng bị bỏ hết | ROI box loại / calib lệch | `ros2 param set /yolo_tube_detector enable_roi_box false` |
| `… ở frame X nhưng node làm việc trong rx150/base_link` | detector sai `target_frame` | `ros2 topic echo /yolo/detected_tubes --once \| head -5` |
| `… dữ liệu cũ Ns` | detector chậm / `detection_max_age_s` chặt | `ros2 topic hz /yolo/detected_tubes` |
| Toạ độ gắp lệch **có hệ thống** | calib `static_transforms.yaml` | so thước; `./rx150.sh calib` snap lại |
| Gắp ống OK nhưng **cắm trượt lỗ** | vị trí giá sai, không phải calib camera | `./rx150.sh rack-watch`; lệch > 5 mm ⇒ `./rx150.sh rack-calib` |
| Cắm trượt **đối xứng gương** (slot 0 ↔ 1) | tag giá dán quay ngược | `yaw_offset_deg: 180` trong `rack_calib.yaml`, snap lại |
| PointCloud mất khi Fixed Frame = `world` | frame có 2 cha | `ros2 run tf2_tools view_frames` |
| `home_xyz_pitch … KHÔNG với tới` | IK home fail ⇒ âm thầm dùng home duỗi thẳng | `rx150_reach_check.py --point 0.18 0 0.18 --pitch 0` |
| Tay quét ngang qua giá khi về home | ↑ cùng nguyên nhân, hoặc `via_staging` tắt | grep log `Tư thế trung chuyển` / `VIA-GIÁ` |
| `move_action chưa sẵn sàng` | `move_group` chưa lên / crash | `ros2 action list \| grep move_action` |
| `move_group` chết ngay / lỗi thư viện | overlay `~/ws_moveit` kế thừa vào env (§1) | `ros2 pkg prefix moveit_ros_move_group` — phải là `/opt/ros/humble` |
| `/rx150/arm_controller/… chưa sẵn sàng` (direct) | bridge chưa chạy | `ros2 action list \| grep follow_joint_trajectory` |
| Mỗi bước treo ~3 s + WARN planning scene | direct mode không có `move_group` | bình thường — xem B5b |
| Sửa SRDF/controllers mà không thấy đổi | installed là **bản sao**, không symlink | `colcon build --packages-select interbotix_xsarm_moveit` |
| Kẹp không khí mà vẫn báo OK | `left_finger` sát giới hạn dưới | `ros2 topic echo /rx150/joint_states` xem `left_finger` |
| Gripper không nhúc nhích (direct) | motor chưa ở PWM mode | `set_gripper_pwm_mode: true` trong yaml |
| Ngón kẹp bóp mãi sau khi lỗi | bridge **cố ý** giữ PWM sau grasp | `~/open_gripper` hoặc `torque_enable enable:=false` |
| GPU đầy, pose nhảy | 2 detector cùng chạy | `ros2 node list \| grep -c yolo_tube_detector` |
| Tay giật mạnh lúc launch | `enable_profile=False` + tay xa sleep | bắt đầu từ tư thế gần sleep (§0) |
| **Bước ĐẦU chạy, mọi bước SAU đều `CONTROL_FAILED(-4)`** | `allowed_start_tolerance` 0.01 chặt hơn sai số fuzzy | tìm log `Validating trajectory with allowed_start_tolerance` → ABORT sau ~9ms **không** kèm `sending trajectory` |
| `CONTROL_FAILED` kèm sai số 27° / 178° | tay CHƯA nhúc nhích — số đó là khoảng cách tới đích | như trên; đừng đi tune gain |
| `GOAL_TOLERANCE_VIOLATED … tol=0.0200` | ngưỡng đích của bridge chặt hơn fuzzy | `default_goal_tolerance` trong `fuzzy_moveit.launch.py` |
| `param set` ngưỡng xong vẫn hỏng y hệt | MoveIt/bridge **cache** lúc init | log vẫn in giá trị cũ ⇒ sửa launch rồi relaunch T1 |
| `attempted = 0`, `[DONE] không còn ống nào cần gắp` | ống nằm trong `rack_filter_xy_m` quanh 1 slot | log `Slot k đã có ống … đánh dấu đã đầy`; dời ống ra ngoài `x∈[0.21,0.31] ∧ y∈[±0.125]` |
| `GRASP: z=… dưới min_ee_z` in ở MỌI chu kỳ | `min_ee_z` cao hơn tâm vật nằm ngang | ống r=8.5mm ⇒ tâm ~0.009; đặt `min_ee_z: 0.010` |
| Ngón khép hết, `left_finger` ~0.010–0.013 | kẹp vào không khí — nghi yaw sai trục | hiệu chuẩn `detection_yaw_offset_deg`: đặt ống dọc +x, đọc yaw trong log |
| `topic hz` im dù topic vẫn chạy | subscriber RELIABLE vào sau topic 100 Hz | kiểm chéo: `fuzzy_node` KHÔNG lặp `stale joint_states` ⇒ vẫn tốt |
| Slot 3/4 luôn "không với tới" | `slot_spacing` × `num_slots` vượt tầm | `rx150_reach_check.py --config tube_rack_params.yaml` |
| Cắm ống chồng lên ống cũ | `rack_filter_xy_m` quá nhỏ | xem log `_mark_occupied_from_detections` |

---

## 6. Thu bằng chứng khi báo lỗi

### 6.1 Chụp trạng thái

```bash
./rx150.sh diag <nhãn>                  # ghi ra diag_<nhãn>_<ts>/ + .tar.gz
# tương đương: ./tools/collect_diag.sh <nhãn>
```

Chạy được cả khi stack **không** chạy (ghi "không tìm thấy node") lẫn khi đang chạy.

### 6.2 Ghi bag để so quỹ đạo

```bash
./tools/record_pickplace.sh <nhãn>      # Ctrl+C để dừng
```

Bag gồm `/rx150/joint_states`, `/rx150/fuzzy/{reference,error,edot,effort}`,
`/tube_rack/status`, `/yolo/detected_tubes`, `/yolo/tube_classes`, `/tf`, `/tf_static`.
Mở trong PlotJuggler với layout sẵn có:

```bash
ros2 run plotjuggler plotjuggler -l ~/RX150_Hedge_Algebra_Control/data_analysis/layouts/fuzzy_plotjuggler_layout.xml
```

So `/rx150/fuzzy/reference` (đặt) với `/rx150/joint_states` (thực) — đây là thứ log
text không bao giờ nói được.

### 6.3 Log từng node

ROS 2 đã tách sẵn: `~/.ros/log/<launch>/<node>/stdout.log`. Gửi file của **đúng node
hỏng**, và 30 dòng quanh dòng lỗi **ĐẦU TIÊN** (không phải dòng cuối — dòng cuối
thường chỉ là hệ quả).

### 6.4 Mẫu báo cáo

```
Bậc: B5a
Lệnh: <copy nguyên lệnh đã gõ>
Kỳ vọng: tay về tư thế trung chuyển
Thực tế: <1 câu; kèm ảnh/video nếu là vấn đề hình học>
/status: <dán JSON tại thời điểm lỗi>
Log node hỏng: <30 dòng quanh lỗi ĐẦU TIÊN>
Đính kèm: diag_<timestamp>.tar.gz
```

---

## 7. Các đường khác

### 7.1 Nhiệm vụ gắp theo cử chỉ tay

```bash
ros2 launch rx150_pick_place pick_place.launch.py
ros2 service call /pick_place_moveit/pick std_srvs/srv/Trigger   # hoặc ra dấu OK
```

Thêm hai bậc kiểm tra vào giữa B3 và B4:

```bash
ros2 topic echo /hand_gesture/selected_target     # Int32, chỉ số trong PoseArray
ros2 topic echo /hand_gesture/event               # String "ok_sign"
```

MediaPipe chạy **CPU**, không GPU. Node tên `hand_gesture` là **bắt buộc** (nó publish
topic private nên tên node quyết định `/hand_gesture/...`).

### 7.2 Một terminal

```bash
ros2 launch rx150_fuzzy_controller fuzzy_moveit_perception.launch.py \
    rs_camera_pointcloud_enable:=false
ros2 launch rx150_pick_place tube_rack.launch.py detector:=false   # BẮT BUỘC false
```

### 7.3 Hiệu chuẩn hand-eye (easy_handeye2)

Chỉ cần nếu bạn muốn bỏ `static_transforms.yaml` sang đường easy_handeye2.

```bash
ros2 launch rx150_perception handeye_calibrate.launch.py     # cần T1 đang chạy
# ... thu mẫu, bấm Save calibration → ~/.ros/easy_handeye2/rx150_eob.yaml
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
    use_camera:=true use_camera_static_tf:=false use_handeye_publisher:=true
```

Khi đó **không** chạy `static_trans_pub` nữa (§2).

### 7.4 Smoke-test theo tầng

```bash
./rx150.sh check                        # chạy cả ba, tổng kết GO/NO-GO
```

Từng bài riêng, kèm tuỳ chọn:

```bash
python3 module_tests/run_test.py --list
python3 module_tests/run_test.py hardware/joint_states_test.py -- --min-hz 80
python3 module_tests/run_test.py moveit/action_servers_test.py -- --backend direct
python3 module_tests/run_test.py perception/tf_and_detection_test.py -- --allow-empty
```

Mỗi test exit 0 = GO, khác 0 = NO-GO, và in ra chính xác cái gì thiếu.

### 7.5 Nhánh KHÔNG dùng cho đợt này

- `rx150_hac_controller` — controller thay thế (PD tuyến tính + bù ma sát). Dùng cho
  thí nghiệm A/B, không phải đường pick-place.
- `rx150_ff_controller` — **không có gripper bridge**, mọi kế hoạch nhóm
  `interbotix_gripper` sẽ treo. Chỉ dùng cho thí nghiệm feedforward.
- `rx150_hri` — `hri.launch.py perception:=true` **lỗi** (include một launch file
  không tồn tại); `mode:=camera` chưa hiện thực. Chỉ `perception:=false mode:=fixed`
  chạy được.
- `interbotix_xsarm_moveit/launch/xsarm_moveit.launch.py` (bản vendor) — dùng
  `ros2_control`, sẽ tranh `/rx150/arm_controller/follow_joint_trajectory` với bridge.
  **Đừng chạy chung.**
