# rx150_pick_place — nhật ký sự cố phần cứng & đối chiếu bản `key_point`

Tách ra từ `rx150_pick_place/README.md` khi dọn kho. Đây là **ghi chép lịch sử** của đợt
đưa node lên phần cứng thật, giữ lại vì nó là bằng chứng đo được, không phải mô tả hệ
đang chạy. Mô tả hệ hiện tại ở
[rx150_pick_place/README.md](../../src/rx150/apps/rx150_pick_place/README.md).

---

## Sự cố đã gặp trên phần cứng (2026-09-09)

Chạy thử một ống nghiệm vàng trên bàn, theo thang bậc RUNBOOK. **B0–B3 GO ngay**
(20/20 pytest; `joint_states` 100.1 Hz; TF `camera_link → camera_color_optical_frame →
world → rx150/base_link` đúng chiều, không frame 2 cha; detector ra `yellow` ở frame
`rx150/base_link`, 4.3 Hz, tản mát XY 0.2–1.5 mm). Lỗi nằm ở **tầng chấp hành**, và là
**bốn nguyên nhân xếp chồng** — sửa một cái vẫn không chạy, nên phải sửa đủ.

### 1. Ống đặt trúng vùng cấm của giá ⇒ `attempted = 0`

Ống ở `(0.2495, −0.0235)`, cách slot 1 `(0.26, −0.025)` đúng **1.06 cm** < `rack_filter_xy_m`
0.05 ⇒ bị coi là đã cắm. Xem ô cảnh báo ở mục *Hình học giá* bên trên. Đây **không** phải
bug: đúng bố cục mặc định thì vùng cấm là `x ∈ [0.21,0.31]` ∧ `y ∈ [−0.125,+0.125]`.

### 2. `Ku` của fuzzy quá thấp ⇒ ngay `~/home` cũng báo thất bại

Với `Ku: [600, 800, 800, 700, 700]`, `elbow` dừng lại khi còn dư **3.78°**, vượt
`verify_tolerance_rad` 3.44°. Nhân **2.5** (giữ nguyên tỉ lệ giữa các khớp) đưa sai số lớn
nhất về **1.49°**; mức phẳng 3000 chỉ được 1.40° — chênh lệch trong nhiễu, nghĩa là phần dư
còn lại là **ma sát/rơ**, không phải thiếu gain. Đã ghi vào
`rx150_fuzzy_controller/config/rx150_fuzzy_gains.yaml`.

### 3. Nguyên nhân chính — ba tầng ngưỡng chồng nhau

Hai tầng phía MoveIt chặt hơn khả năng thật của vòng PWM, nên **lệnh đầu chạy được, để lại
~2° dư, rồi mọi lệnh sau bị loại sạch**:

| Tầng | Đặt ở đâu | Cũ | Nay |
|---|---|---|---|
| Kiểm tư thế **xuất phát** (MoveIt) | `fuzzy_moveit.launch.py` → `trajectory_execution.allowed_start_tolerance` | 0.01 rad (0.57°) | **0.10** |
| Kiểm **đích** (bridge) | cùng launch → `default_goal_tolerance` của `rx150_trajectory_bridge.py` | 0.02 rad (1.15°) | **0.10** |
| Nghiệm thu (task) | `*_params.yaml` → `verify_tolerance_rad` | 0.06 rad (3.44°) | **giữ nguyên** |

Cả hai giá trị MoveIt thừa hưởng từ launch vendor `xsarm_moveit.launch.py`, vốn dùng
`ros2_control` bám vị trí tới <0.01 rad. Bộ fuzzy PWM để lại 1.4–2.5° ở home và tới **6.5°
ở `wrist_angle`** khi tay vươn ra — không bao giờ đạt nổi.

**Đừng nới `verify_tolerance_rad`.** Đó là tầng duy nhất đối chiếu `joint_states` thật và
là thứ chặn tay gắp không khí. Chỉ nới hai tầng MoveIt cho khớp năng lực bộ điều khiển.

**Dấu hiệu nhận ra trong log** — quan trọng vì nó khác hẳn lỗi bám kém:

```
Validating trajectory with allowed_start_tolerance 0.01
[ERROR] moveit_ros.trajectory_execution_manager:          <- message RỖNG
Execution completed: ABORTED
```

abort sau **~9 ms** và **không có** dòng `sending trajectory to /rx150/arm_controller`.
Quỹ đạo bị loại *trước khi* xuống tới bridge. Task sau đó báo `CONTROL_FAILED(-4)` kèm sai
số khổng lồ (27°, thậm chí 178°) — con số đó là khoảng cách tới **đích**, vì tay chưa hề
nhúc nhích. Phân biệt:

- abort **nhanh**, không có dòng `sending trajectory` ⇒ bị loại ở khâu **xuất phát**.
- abort **chậm**, kèm `GOAL_TOLERANCE_VIOLATED: … err=… > tol=0.0200` ⇒ tầng **bridge**.

> ⚠️ **Bẫy: `ros2 param set` KHÔNG có tác dụng với hai tham số này.**
> `ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.10` trả
> "Set parameter successful" và `param get` đọc ra 0.1, **nhưng executor vẫn dùng 0.01** —
> `TrajectoryExecutionManager` cache lúc khởi tạo, log vẫn in 0.01. Bridge cũng vậy: đọc
> `default_goal_tolerance` một lần vào `self._default_tol` lúc init. **Phải sửa launch rồi
> relaunch `./rx150.sh t1`.** (Ngược lại, `Ku` thì chỉnh nóng được — `fuzzy_node.cpp:87`
> có `on_set_parameters_callback`.)

Sau khi sửa đủ 3 tầng, chu kỳ lần đầu đi hết chuỗi thật:
`APPROACH ✓ → DESCEND ✓ → GRASP ✓ → LIFT`, không còn `CONTROL_FAILED` ở bước di chuyển nào.

### 4. Còn treo: kẹp hụt ống nằm ngang

Đã sửa một phần: `min_ee_z` là **0.015**, trong khi ống bán kính 8.5 mm nằm trên bàn có tâm
ở ~0.009 (detector đo 0.010–0.013) ⇒ mọi lần kẹp đều bị nâng lên **cao hơn tâm ống 5 mm**
(log in `GRASP: z=0.010m dưới min_ee_z=0.015m → nâng lên min_ee_z` ở **mọi** chu kỳ).
Đã hạ xuống `0.010`. **Vẫn hụt**: ngón khép hết, `left_finger` = 0.0100–0.0133 m — tức
thấp hơn cả `finger_closed_m: 0.015` mà config giả định.

Đã loại trừ: nhiễu nhận diện (lặp lại ±3 mm giữa các chu kỳ) và sai lệch depth (đo 0.010 vs
hình học 0.009).

**Nghi vấn còn lại: `detection_yaw_offset_deg` chưa bao giờ được hiệu chuẩn.** Detector lấy
yaw từ `cv2.minAreaRect` trong hệ **ảnh**; trục ảnh không trùng trục `base_link` thì ngón
khép **dọc** theo ống thay vì cắt ngang, gạt ống văng ra. Cách hiệu chuẩn nằm ngay ở mục
*Quy ước hướng* bên trên: đặt 1 ống dọc trục **+x**, đọc yaw trong log, lấy hiệu bỏ vào
`detection_yaw_offset_deg`. Làm việc này **trước** khi tinh chỉnh bất cứ thứ gì khác trong
đường gắp.

### Việc vặt phát hiện kèm

- Camera D435i đang cắm **cổng USB 2.1** (`Device 243322073847 is connected using a 2.1
  port`) ⇒ color/aligned_depth chỉ đạt **18.9/19.9 Hz** thay vì 30. Đủ cho detector 5 Hz,
  nhưng nên đổi sang cổng USB 3.
- `./rx150.sh t2` mở luôn `armtag_tuner_gui` (mặc định của `rx150_perception.launch.py`),
  mà RUNBOOK §B2 dặn không để GUI này mở lâu vì rò timer 20 Hz.
- `open_gripper` trả `success=False` kèm message `'Đã mở gripper.'` — mâu thuẫn đã biết,
  xem mục *Gripper*. Mở dứt điểm bằng PWM trực tiếp:
  ```bash
  ros2 topic pub -1 /rx150/commands/joint_single \
      interbotix_xs_msgs/msg/JointSingleCommand "{name: 'gripper', cmd: 250.0}"
  ```

### File đã đổi trong đợt này

| File | Đổi gì |
|---|---|
| `rx150_fuzzy_controller/launch/fuzzy_moveit.launch.py` | `allowed_start_tolerance` 0.01→0.10; thêm `default_goal_tolerance: 0.10` cho bridge |
| `rx150_fuzzy_controller/config/rx150_fuzzy_gains.yaml` | `Ku` ×2.5 → `[1500, 2000, 2000, 1750, 1750]` |
| `rx150_pick_place/config/pick_place_params.yaml` | `min_ee_z` 0.015→0.010 |

---

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

