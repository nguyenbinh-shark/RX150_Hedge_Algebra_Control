# Lệnh chạy `tube_rack` — bản dán thẳng vào terminal

Sổ lệnh để **copy-paste**, không giải thích dài. Vì sao mỗi bậc tồn tại, triệu chứng →
nguyên nhân, và cách thu bằng chứng: xem
[RUNBOOK](../src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md).

Mọi lệnh chạy từ `~/interbotix_ws`. Mỗi terminal mới đều phải `source`:

```bash
cd ~/interbotix_ws && source source_all.sh
```

`./rx150.sh` tự source, nên các lệnh `./rx150.sh …` không cần bước trên.

> **An toàn.** Torque BẬT ngay lúc launch T1 và tay nhảy về tư thế sleep **không qua
> Ruckig**. Trước khi launch: đặt tay gần sleep, dọn thoáng bán kính ~0.5 m, để E-stop
> trong tầm với. Kết thúc phiên bằng §7.

Thứ tự: **B0 → T1 → T2 → [giá] → B3 → task**. Hỏng ở bậc nào thì dừng ở bậc đó.

> **Nghiệm thu gần nhất: 2026-09-11 20:19** — gắp 1 ống và cắm vào giá theo màu, chạy
> thật với PWM `-300/+300`. Điều kiện để chạy được: cáp USB 3.x (§2), snap lại giá
> **sau** khi snap camera (§3), ống đặt cách mọi slot > 5 cm (§5.1). Sau khi chạy
> **phải** cắt dòng ngón kẹp (§7).

---

## 0. B0 — toán học + config (không cần robot)

```bash
cd ~/interbotix_ws && source source_all.sh
python3 -m pytest src/rx150/rx150_toolbox/rx150_modules/test -q
./rx150.sh reach
```

**GO:** `20 passed`; `./rx150.sh reach` in `✅ Mọi điểm trong config đều với tới.`

Chạy lại B0 **mỗi lần** snap lại vị trí giá.

---

## 1. Terminal 1 — robot + MoveIt + camera

```bash
cd ~/interbotix_ws
pkill -f xs_sdk          # 2 driver trên 1 bus U2D2 = tranh chấp serial
./rx150.sh t1-hac        # hoặc ./rx150.sh t1 để dùng bộ fuzzy
```

Kiểm ở terminal khác:

```bash
source ~/interbotix_ws/source_all.sh
ros2 topic hz /rx150/joint_states                 # ~100 Hz
ros2 topic echo /rx150/joint_states --once        # 8 tên: 5 khớp + gripper + 2 finger
ros2 action list | grep follow_joint_trajectory   # PHẢI có cả arm_ và gripper_controller
```

**GO:** 100 Hz, đủ 8 tên, đủ **hai** action server.

---

## 2. Terminal 2 — perception (nạp TF hiệu chuẩn)

```bash
./rx150.sh t2
```

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

**GO:** `tf2_echo` ra số **không nhảy**; cả hai ảnh có hz đều. Dòng
`Invalid frame ID … does not exist` ở đầu là bình thường (buffer chưa đầy).

**Ảnh không có hz, hoặc hz thấp bất thường ⇒ kiểm tốc độ cổng USB.**
Lỗi này đã nuốt trọn một buổi (2026-09-11):

```bash
for d in /sys/bus/usb/devices/*/; do
  [ "$(cat $d/idVendor 2>/dev/null)" = 8086 ] && \
    echo "$(cat $d/speed) Mbps  $(cat $d/product)"
done
```

`5000` ⇒ USB 3.x, đúng. `480` ⇒ **USB 2.0, phải đổi cáp hoặc cổng.**

Ở USB 2.0, color 640×480×30 + depth 640×480×30 + `align_depth` cần ~295 Mbps trên
đường 480 Mbps ⇒ luồng **sập xuống ~0–1 Hz**, không phải chậm đi một chút. Đo được:
**0 khung trong 25 s**. Hạ xuống 25 fps **không cứu được** — chỉ bớt 17%, mà D435i
cũng không có 25 fps (`rs-enumerate-devices` chỉ liệt 60/30/15/6).

Sau khi thay cáp, cả ba luồng chạy đủ **30 Hz** (khoảng cách stamp nhỏ nhất 33.4 ms).

> `ros2 topic hz` hay câm vì QoS BEST_EFFORT và đệm stdout. Không ra số **chưa chắc**
> là camera chết — đo lại bằng `--once` hoặc so `header.stamp` trước khi kết luận.

**Không có số ⇒ dừng lại**, mọi thứ tầng trên đều vô nghĩa. Xem RUNBOOK §2
(chỉ **một** nguồn được phát `world ↔ camera`).

---

## 3. Vị trí giá — hỏi trước, snap sau

```bash
./rx150.sh rack-watch    # ĐO, không ghi file. Ctrl+C để thoát.
```

- lệch **≤ 5 mm** → bỏ qua cả mục này, đi tiếp §4.
- lệch lớn → giá đã bị xê dịch, snap lại:

```bash
./rx150.sh rack-calib    # ghi rack_pose.yaml (~3 s, cần T1 + T2 đang chạy)
./rx150.sh reach         # BẮT BUỘC: 4 lỗ mới có với tới được không
```

**GO của snap:** tản mát vị trí ≤ 3 mm, tản mát góc ≤ 1.5°.
(Snap 2026-09-11 20:19 đạt 35 mẫu, 1.35 mm / 0.20°.)

**Kiểm THỨ TỰ hiệu chuẩn** — `rack_pose.yaml` phải **mới hơn** `static_transforms.yaml`:

```bash
stat -c '%y  %n' \
  src/rx150/rx150_toolbox/rx150_perception/config/static_transforms.yaml \
  src/rx150/rx150_toolbox/rx150_perception/config/rack_pose.yaml
```

Vị trí giá đo **trên nền** TF camera, và `tube_rack_node` đọc thẳng toạ độ base_link
trong file chứ **không tra TF lúc chạy**. Nên snap camera sau khi snap giá ⇒ số giá
thành rác mà **không có cảnh báo nào**. Đúng lỗi này làm hỏng chu kỳ 19:59 ngày
2026-09-11 (giá snap 19:29, camera snap 19:58); snap lại giá thì `y` nhảy **125 mm**
và `long_edge_yaw` nhảy **37°**.

Muốn nhìn tận mắt trước khi tin (4 lỗ chiếu ngược lên ảnh camera):

```bash
./rx150.sh rack-gui      # lục = đang đo, cam = đã lưu trong file
```

Hoặc gộp luôn vào T2 — cùng marker và ảnh chồng hình, nhưng trong **RViz của T2**
thay vì mở thêm một RViz thứ hai:

```bash
./rx150.sh t2-rack       # = t2 + use_rack_watch:=true
```

Trong RViz bật hai Display **RackMarkers** (4 lỗ trong 3D) và **RackOverlay**
(4 vòng tròn chiếu ngược lên ảnh). Cả hai `mode:=watch` nên **chỉ đo, không ghi**
`rack_pose.yaml`.

⚠️ Dùng `t2-rack` thì **đừng** mở thêm `rack-gui`/`rack-calib` ở terminal khác:
cả hai cùng dựng node tên `rack_tag` và `rack_calib`, trùng tên trên một graph.

---

## 4. B3 — nhận diện

```bash
./rx150.sh test                              # 14 test YOLO + hình học giá, offline
ros2 topic hz /yolo/detected_tubes           # ~4-5 Hz
ros2 topic echo /yolo/detected_tubes --once  # frame_id PHẢI = rx150/base_link
ros2 topic echo /yolo/tube_classes --once
```

Detector chỉ chạy khi task node đã lên (§5), nên chạy hai lệnh `topic` sau §5.

Đường nối topic — **không phải khai báo tay**, đã có sẵn trong config:

| Khâu | Vị trí | Giá trị |
| :--- | :--- | :--- |
| detector publish | `yolo_tube_detector_node.py:279` | `/yolo/detected_tubes` (PoseArray) |
| detector publish | `yolo_tube_detector_node.py:280` | `/yolo/tube_classes` (String JSON) |
| config khai báo | `tube_rack_params.yaml:150-151` | `detection_topic` / `classes_topic` |
| task subscribe | `rx150_modules/params.py:230` | `DetectionSource(poses_topic=...)` |

Pose và class nằm ở **hai topic tách rời, ghép với nhau bằng chỉ số**. Lệch nhịp ⇒
ống xanh bị dán nhãn hồng ⇒ cắm sai slot. Đừng suy màu từ pose.

**Lệch có hệ thống** (mọi ống cùng lệch một hướng) ⇒ lỗi calib TF, **không** phải YOLO.
Quay về §6.

### 4.1 `[DONE] không còn ống nào cần gắp` mà không một cảnh báo

Chế độ hỏng **im lặng nhất** của cả hệ. Khi ROI bật, detector vẫn publish PoseArray
**RỖNG mỗi khung hình** — đúng `frame_id`, đúng nhịp — nên `DetectionSource` thấy dữ
liệu "tươi và hợp lệ", không có gì để cảnh báo; task node chờ hết `detection_wait_s`
rồi kết luận hết ống. Ba nguyên nhân khác hẳn nhau cho ra **cùng một dòng log**.

Tách bằng ba lệnh, đúng thứ tự:

```bash
ros2 topic hz /camera/camera/color/image_raw   # (1) có khung hình không?
ros2 topic hz /yolo/detected_tubes             # (2) detector có publish không?
ros2 topic echo /yolo/detected_tubes --once    # (3) publish nhưng poses rỗng?
```

| (1) | (2) | (3) | Kết luận |
| :-- | :-- | :-- | :--- |
| im | — | — | Camera/USB chết → §2 |
| có | im | — | Thiếu TF `world↔camera` → §6. Detector `return` im lặng, chỉ log mức `debug` |
| có | ~4 Hz | `poses: []` | Không có gì **trong ROI** — đặt lại ống, không phải lỗi |
| có | ~4 Hz | có pose | Nhận diện tốt; lỗi ở §5 (bẫy 5 cm) |

Bẫy phụ: khung hình chỉ về ~1–2 Hz thì `track_timeout_s: 1.0` xoá track **trước khi**
nó kịp đạt `min_hits: 2` ⇒ PoseArray rỗng vĩnh viễn **dù YOLO nhìn thấy ống rất rõ**.
Triệu chứng giống hệt "model kém", nguyên nhân thật là **băng thông USB**.

---

## 5. Terminal 3 — task

```bash
source ~/interbotix_ws/source_all.sh
ros2 launch rx150_pick_place tube_rack.launch.py auto_start:=false
```

`auto_start:=false` = node lên nhưng **chưa cử động**, chờ lệnh. Bỏ cờ này (hoặc dùng
`./rx150.sh tubes`) thì nó tự chạy ngay khi lên.

Lúc khởi động phải thấy đủ ba dòng:

```
Tư thế trung chuyển (0.180,0.000,0.180) pitch=0° → home_joints = [...]°
Hình học giá lấy từ HIỆU CHUẨN: …/rack_pose.yaml (tag 0, NN mẫu, …)
  slot 0..3: … [OK ]
```

Terminal 4 — để chạy suốt phiên:

```bash
ros2 topic echo /tube_rack/status
```

Điều khiển (terminal 5):

```bash
ros2 service call /tube_rack/home         std_srvs/srv/Trigger   # B5: về tư thế trung chuyển
ros2 service call /tube_rack/open_gripper std_srvs/srv/Trigger
ros2 service call /tube_rack/run          std_srvs/srv/Trigger   # B6: chạy chu kỳ
ros2 service call /tube_rack/stop         std_srvs/srv/Trigger   # dừng khẩn cấp mềm
ros2 service call /tube_rack/reset        std_srvs/srv/Trigger   # chạy lại sau khi stop
```

### 5.1 Bẫy 5 cm khi đặt ống

`rack_filter_xy_m: 0.05` — ống nằm trong bán kính 5 cm quanh **bất kỳ** slot nào bị coi
là **đã cắm rồi** và bỏ qua, **không một dòng log**. Đặt ống cách mọi slot **> 5 cm**.

```bash
ros2 topic echo /yolo/detected_tubes --once | grep -A3 position
grep -A6 '^slots:' install/rx150_perception/share/rx150_perception/config/rack_pose.yaml
```

Ví dụ thật 2026-09-11: ống ở `(0.190, 0.180)` cách slot 3 đúng **46.6 mm** ⇒ bị nuốt;
dời sang `(0.140, 0.172)` thì cách 97 mm ⇒ chạy bình thường.

**GO của B6:** `/status` có `"succeeded": 1`, `"faults": 0`, và ống nằm đúng slot.

`run` trả `Đang STOP — gọi ~/reset trước` ⇒ gọi `reset` rồi `run` lại.

---

## 6. Hiệu chuẩn lại camera — chỉ khi toạ độ lệch

Thứ tự **bắt buộc**: tag → camera → giá. Bỏ bước nào thì bước sau chỉ chép lại cái sai
của bước trước.

### 6.1 Snap Pose (nhanh, ~1 phút)

```bash
# TẮT T2 TRƯỚC — nếu không sẽ có HAI nguồn phát world↔camera
./rx150.sh calib
```

Trên GUI: đưa tag tay gắp (**id 1**) vào khung hình, đặt **Snapshots = 10**, bấm
**Snap Pose**. Kết quả tự ghi `static_transforms.yaml`. Xong thì Ctrl+C ngay
(GUI rò timer 20 Hz) và bật lại `./rx150.sh t2`.

### 6.2 Kiểm chứng — đừng tin bản snap khi chưa đo

```bash
./rx150.sh eetag         # terminal riêng: detector tag tay gắp, 30 Hz
```

```bash
# (a) đứng yên, không ra lệnh gì — 15 s
ros2 run rx150_motion_common rx150_ee_tag_bench.py watch \
    --duration 15 --tag-offset ee_tag_offset.yaml --label tfcheck --no-plot

# (b) 5 góc waist, quay TẠI CHỖ ở r=0.18 z=0.20 — bán kính này nằm trong giá
#     (giá bắt đầu từ x≈0.24) nên không chạm được giá lẫn ống đang cắm
ros2 run rx150_motion_common rx150_ee_tag_bench.py hold \
    --pose-set grid --grid-radii=0.18 --grid-heights=0.20 \
    --grid-waists-deg=-40,-20,0,20,40 --keepout-rack \
    --tag-offset ee_tag_offset.yaml --dwell 2.5 --settle 2.5 \
    --label tfcheck5 --no-plot
```

Chỉ (b) mới có giá trị: (a) đo đúng tư thế vừa snap nên khớp là hiển nhiên.

**GO:** `enc→cam` RMS ≲ 5 mm, hướng ≲ 1°. (Đo 2026-09-11: 3.89 mm / 0.81°.)

> **ĐỪNG áp phần "TF đề xuất" của (b).** Bộ pose quay waist tại chỗ có `tầm phủ của dữ
> liệu` rất hẹp — lệnh tự in `⚠ HẸP (<100 mm): hiệu chỉnh xoay lẫn với tịnh tiến`. Nó
> chỉ dùng để KIỂM CHỨNG. Muốn refine thật thì phải chạy lưới 39 pose (§6.3).

### 6.3 Lưới 39 pose (chuẩn, ~5 phút)

**Rút hết ống khỏi giá trước.** `--keepout-rack` chỉ biết hộp giá (nóc ~0.076 m), không
biết ống dựng đứng cao ~0.18 m — đúng tầm lưới quét.

```bash
./rx150.sh rack-calib    # biết giá đứng đâu để lưới tránh
./rx150.sh eetag-calib   # ~5 phút → tuning_runs/<run>/static_transforms_refined.yaml
# TẮT T2 rồi mới chép đè lên rx150_perception/config/static_transforms.yaml, bật lại T2
```

### 6.4 Sau MỌI lần đổi TF camera

```bash
./rx150.sh rack-calib    # BẮT BUỘC: vị trí giá nằm TRÊN NỀN TF vừa đổi
./rx150.sh rack-gui      # 4 vòng tròn phải khít 4 miệng lỗ thật
./rx150.sh reach
```

---

## 7. Kết thúc phiên

```bash
ros2 service call /tube_rack/open_gripper std_srvs/srv/Trigger

# BẮT BUỘC — chính open_gripper ở trên vừa latch PWM +300 lên motor
ros2 topic pub --once /rx150/commands/joint_single \
    interbotix_xs_msgs/msg/JointSingleCommand "{name: 'gripper', cmd: 0.0}"

ros2 service call /rx150/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
    "{cmd_type: 'group', name: 'arm', enable: false}"
```

Gripper **cố ý giữ lực** sau khi kẹp và `_pwm()` **không bao giờ hạ về 0** — nên
`open_gripper` vừa gọi ở trên chính là thứ để `+300` nằm lại trên motor. Dòng
`cmd: 0.0` không phải tuỳ chọn. Xem §8.2.

---

## 8. Sự cố phần cứng

### 8.1 Motor tự ngắt

Triệu chứng ở tầng trên **giống hệt** lỗi tolerance/gain:

```
GOAL_TOLERANCE_VIOLATED: waist: err=0.31 > tol=0.10
CONTROL_FAILED(-4) … Sai số còn lại waist=17.4° > tol 3.44°
```

Phân biệt trong **một dòng** — đọc `effort`:

```bash
ros2 topic echo /rx150/joint_states --once
```

**Đúng một khớp bằng `0.0`** trong khi các khớp khác có PWM ⇒ **motor đã ngắt**, đừng
đi tune gain. Xác nhận thêm: log T1 có `[RxPacketError] Hardware error occurred`.

Đọc cờ lỗi / nhiệt độ (phải hỏi **từng khớp**; `cmd_type: 'group'` trả `values=[]` rỗng):

```bash
ros2 service call /rx150/get_motor_registers interbotix_xs_msgs/srv/RegisterValues \
    "{cmd_type: 'single', name: 'waist', reg: 'Hardware_Error_Status'}"
ros2 service call /rx150/get_motor_registers interbotix_xs_msgs/srv/RegisterValues \
    "{cmd_type: 'single', name: 'gripper', reg: 'Present_Temperature'}"
```

Khôi phục — **không restart T1** (restart cắt torque cả nhóm, tay đang giơ cao sẽ **rơi**):

```bash
# waist là trục THẲNG ĐỨNG ⇒ không có mô-men trọng lực ⇒ reboot riêng nó thì tay không rơi
ros2 service call /rx150/reboot_motors interbotix_xs_msgs/srv/Reboot \
    "{cmd_type: 'single', name: 'waist', enable: false, smart_reboot: false}"

# reboot XOÁ THANH GHI RAM ⇒ mất PWM mode. Đặt lại đúng thứ hac_node/fuzzy_node dùng:
ros2 service call /rx150/set_operating_modes interbotix_xs_msgs/srv/OperatingModes \
    "{cmd_type: 'group', name: 'arm', mode: 'pwm', profile_type: 'time', \
      profile_velocity: 0, profile_acceleration: 0}"
ros2 service call /rx150/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
    "{cmd_type: 'group', name: 'arm', enable: true}"
```

Kiểm lại: `effort` của khớp đó khác 0 và `Hardware_Error_Status` = 0.

### 8.2 Ngón kẹp nóng

`Gripper._pwm()` publish PWM rồi **không bao giờ hạ về 0** (cố ý, để giữ lực lúc mang
vật). Nếu ngón không chạm được cữ — bánh răng trượt — thì mỗi lần
`RELEASE (PWM): hết 4.0s mà ngón chưa dừng` sẽ để PWM nằm lại **vô thời hạn**. Đo
2026-09-11: gripper **63°C** trong khi các khớp tay 37–43°C.

Cắt dòng ngay:

```bash
ros2 topic pub --once /rx150/commands/joint_single \
    interbotix_xs_msgs/msg/JointSingleCommand "{name: 'gripper', cmd: 0.0}"
```

**Đây không phải sự cố hiếm — nó lặp lại SAU MỖI CHU KỲ.** Đo 2026-09-11, hai lần
liên tiếp, mỗi lần đều `effort = +909.2` đứng yên (một lần kéo dài ~7 phút trước khi
phát hiện). Lần thứ hai ngón đã mở hết cữ `±0.0335` mà vẫn bị ép tiếp.

Kiểm sau **mỗi** lần chạy — `effort` của `gripper` phải là `+0.0`:

```bash
ros2 topic echo /rx150/joint_states --once | grep -A2 effort
```

### 8.3 Chạy khi ngón kẹp đang hỏng

Trong `src/rx150/apps/rx150_pick_place/config/tube_rack_params.yaml`:

| khoá | giá trị khi ngón hỏng | trả lại khi đã thay bánh răng |
| :--- | :--- | :--- |
| `use_gripper_bridge` | `false` | `true` (chỉ khi SRDF/URDF `left_finger` khớp phần cứng) |
| `grasp_verify` | `false` | `true` |
| `gripper_pwm_grasp` | `0.0` | `-300.0` |
| `gripper_pwm_release` | `0.0` | `300.0` |

> **Trạng thái hiện tại (2026-09-11): `-300.0` / `300.0` — kẹp thật, bánh răng CHƯA
> thay.** Bật lại theo yêu cầu vận hành, rủi ro đã biết và chấp nhận. Hệ quả bắt buộc:
> **không để chạy không người trông**, và chạy lệnh cắt dòng ở §8.2 sau mỗi chu kỳ.
> Dấu quy ước phần cứng: **ĐÓNG = PWM ÂM, MỞ = PWM DƯƠNG** (−350 → 0.0133 m,
> +350 → 0.0330 m).

PWM = 0 thì chu kỳ **vẫn chạy trọn** state machine (mỗi thao tác ngón tốn 4 s chờ
timeout) nhưng **không gắp thật** — chỉ nghiệm thu được quỹ đạo và hình học, không
nghiệm thu cú kẹp. Đừng đọc `succeeded: 1` ở chế độ này thành "nhiệm vụ chạy được".

Package `rx150_pick_place` cài bằng `--symlink-install`, nên sửa `.yaml` xong chỉ cần
**khởi động lại node task**, không cần build.

---

## 9. Thu dữ liệu và vẽ đồ thị

Hai lệnh, tách hẳn nhau: thu xong rồi vẽ, vẽ offline không cần ROS.

```bash
# terminal riêng, BẬT TRƯỚC khi bấm chạy chu kỳ
./rx150.sh record --label run1
# ... chạy chu kỳ ở terminal task ... rồi Ctrl+C ở đây

./rx150.sh plot tuning_runs/run1_<ts>
```

Muốn có thêm đường **tag** (camera nhìn thấy tay gắp ở đâu) thì bật
`./rx150.sh eetag` trước; không có thì thêm `--no-tag` cho đỡ cảnh báo.
Chạy bộ fuzzy thay vì HAC: `--ref-topic /rx150/fuzzy/reference
--err-topic /rx150/fuzzy/error`.

`record` **không ra lệnh gì cho robot** — an toàn chạy song song với `tubes`.

### 9.1 Ba nguồn quỹ đạo — đừng gộp chúng

| Đường | Nguồn | Nghĩa |
| :--- | :--- | :--- |
| `ref` | `/rx150/hac/reference` → FK | Ruckig **muốn** tay ở đâu |
| `enc` | `/rx150/joint_states` → FK | encoder **nói** tay đang ở đâu |
| `tag` | TF `base_link→ee_tag` | camera **nhìn thấy** tay ở đâu |

Từ đó ra **hai** sai số có nguyên nhân khác hẳn nhau:

- `enc − ref` = sai số **BÁM** — lỗi của bộ điều khiển (gain, ma sát, trọng lực).
- `tag − enc` = sai số **HIỆU CHUẨN** — encoder tưởng một đằng, thực tế một nẻo.

Cộng hai thứ này lại là vô nghĩa. Tay gắp trượt lỗ mà `enc−ref` nhỏ trong khi
`tag−enc` lớn ⇒ đi hiệu chuẩn (§6), đừng tune gain. Ngược lại thì tune gain,
đừng hiệu chuẩn.

### 9.2 File sinh ra

```
tuning_runs/<nhãn>_<ts>/
  traj.csv   quỹ đạo lấy mẫu đều 50 Hz: ref/enc/tag + góc khớp + /hac/error
  det.csv    mỗi pose ống YOLO, kèm nhãn màu và yaw
  meta.json  toạ độ 4 lỗ giá, rack_pose, đếm message từng nguồn
  traj3d.png      quỹ đạo 3D + 4 lỗ giá + ống nhận diện
  traj_2d.png     chiếu XY (từ trên) và XZ (ngang)
  error_time.png  sai số theo thời gian, nền tô theo pha state machine
  rms.png         RMS theo trục và theo khớp
  joints.png      ref vs enc từng khớp
  detections.png  toạ độ ống theo t + tản mát XY kèm vòng 5 cm
  summary.txt     bảng số
```

`meta.json` đếm message từng nguồn — **đọc nó trước khi tin đồ thị**. `reference: 0`
nghĩa là cột `ref_*` rỗng và mọi RMS bám đều vô nghĩa, không phải "sai số bằng 0".

### 9.3 Hai con số trong `counts` phải xem

`tf_fallback` — số mẫu tag phải **lùi về TF mới nhất** vì TF trễ hơn dấu thời gian
ảnh. Mẫu lùi đã mất căn thời gian: tay chạy 50 mm/s mà lệch 40 ms là 2 mm, nên
đừng dùng chúng để kết luận sai lệch **hằng**. Tỉ lệ lùi cao ⇒ chỉ đọc hình dạng
quỹ đạo, không đọc con số tuyệt đối.

`tf_fail` — tra TF hỏng hẳn. Khác `tf_fallback`: đây là mất mẫu thật.

Nếu `tag: 0` mà `tag_msgs` lớn thì thấy tag nhưng TF hỏng — thông báo in ra kèm
luôn câu lỗi TF, không phải đoán.

Detector tag chậm (dưới ~4 Hz) thì nới `--tag-max-age` (mặc định 0.25 s), nếu không
mọi mẫu đều quá hạn và cột `tag_*` ra NaN dù tag vẫn đang được thấy.

Đo lúc đứng yên ở home 2026-09-11: `enc−ref` RMS ‖Δ‖ = **7.3 mm**, gần như toàn bộ
nằm ở trục z (7.30 mm) — đó là **độ võng tĩnh**, không phải lỗi bám động.

---

## 10. Sổ lệnh một trang

| Việc | Lệnh |
| :--- | :--- |
| Test offline | `python3 -m pytest src/rx150/rx150_toolbox/rx150_modules/test -q` |
| Tầm với + config | `./rx150.sh reach` |
| T1 robot (HAC) | `pkill -f xs_sdk && ./rx150.sh t1-hac` |
| T1 robot (fuzzy) | `pkill -f xs_sdk && ./rx150.sh t1` |
| T2 perception | `./rx150.sh t2` |
| Giá có xê dịch? | `./rx150.sh rack-watch` |
| Snap vị trí giá | `./rx150.sh rack-calib` |
| Soi 4 lỗ bằng mắt | `./rx150.sh rack-gui` |
| Soi 4 lỗ NGAY TRONG T2 | `./rx150.sh t2-rack` |
| Thu data một lần chạy | `./rx150.sh record --label <nhãn>` |
| Vẽ đồ thị | `./rx150.sh plot tuning_runs/<thư mục>` |
| Hiệu chuẩn camera nhanh | `./rx150.sh calib` (tắt T2 trước) |
| Hiệu chuẩn camera chuẩn | `./rx150.sh eetag` + `./rx150.sh eetag-calib` |
| Test perception offline | `./rx150.sh test` |
| Task (chưa chạy) | `ros2 launch rx150_pick_place tube_rack.launch.py auto_start:=false` |
| Task (tự chạy) | `./rx150.sh tubes` |
| Chạy khô, không cần camera | `./rx150.sh dry-fake` |
| Smoke-test 3 tầng | `./rx150.sh check` |
| Chụp trạng thái gửi kèm lỗi | `./rx150.sh diag <nhãn>` |
| Ghi bag so quỹ đạo | `./tools/record_pickplace.sh <nhãn>` |
| Kiểm cổng USB camera | xem §2 (`480` = hỏng, `5000` = đúng) |
| Kiểm thứ tự calib | `stat -c '%y %n' …/static_transforms.yaml …/rack_pose.yaml` |
| **Cắt dòng ngón kẹp** | `ros2 topic pub --once /rx150/commands/joint_single interbotix_xs_msgs/msg/JointSingleCommand "{name: 'gripper', cmd: 0.0}"` |
