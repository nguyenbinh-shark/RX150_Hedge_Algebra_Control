# Hiệu chuẩn tag tay gắp + TF camera — quy trình dùng lại

Hai phép hiệu chuẩn khác nhau, hay bị lẫn. Bảng này là thứ cần nhớ:

| | đo cái gì | lệnh | ghi vào | chạy lại khi |
| :-- | :-- | :-- | :-- | :-- |
| **A** | tag nằm ở đâu **trên tay gắp** | `./rx150.sh eetag-tagcal` | `rx150_perception/config/ee_tag_offset.yaml` | in/lắp lại gá, dán lại tag, đổi tag, đổi độ phân giải camera |
| **B** | **camera** nằm ở đâu so với robot | `./rx150.sh eetag-calib` | `rx150_perception/config/static_transforms.yaml` | camera xê dịch, đổi ống kính/độ phân giải, **sau khi chạy A** |
| **C** | **giá** nằm ở đâu | `./rx150.sh rack-calib` | `rx150_perception/config/rack_pose.yaml` | giá xê dịch, **sau khi chạy B** |

Thứ tự **A → B → C** là bắt buộc: B đứng trên A, C đứng trên B. Chạy ngược thì
mỗi bước chỉ chép lại cái sai của bước dưới.

Kiểm chứng cuối cùng: `./rx150.sh eetag-pick`
(xem [do_chinh_xac_camera_robot.md](do_chinh_xac_camera_robot.md)).

---

## Vì sao không khai vị trí tag trong URDF

URDF của Interbotix có sẵn `rx150/ar_tag_link` (`ar_tag.urdf.xacro`), nhưng đó là
vị trí **gá AR tag chính hãng**. Gá tự thiết kế thì con số đó sai — đo
2026-09-11: lệch **4.82 mm và 2.09°**.

Điều đó không vô hại, vì chuỗi armtag Snap Pose tính

```
T_camera←base = T_camera←tag (đo bằng ảnh) ∘ T_tag←base (tra TF)
```

Vế thứ hai lấy ở đâu thì sai số chỗ đó chui **nguyên** vào hiệu chuẩn camera.
Riêng phần **xoay** đi đúng 1:1 bất kể lúc snap tay máy đứng ở tư thế nào — số
hiệu chỉnh là một phép liên hợp `T_base←ee · X_urdf · X_thật⁻¹ · T_base←ee⁻¹`, mà
liên hợp không làm đổi độ lớn góc.

Nên **nguồn sự thật là `ee_tag_offset.yaml`**, đo từ dữ liệu. `rx150_perception.launch.py`
phát nó thành frame TF `rx150/ee_tag_link` và trỏ `arm_tag_frame` vào đó
(`arm_tag_frame:=auto`). `ar_tag_link` lui về đúng vai trò khối hình học cho va
chạm/hiển thị. Không file nào trong `src/vendor/` bị đụng ⇒ kéo lại vendor không mất.

Không có file đo thì launch tự quay về `rx150/ar_tag_link` và **in cảnh báo**.

---

## A. Đo vị trí tag trên tay gắp

### Chuẩn bị vật lý (làm trước, chỉ một lần cho mỗi lần đổi gá)

1. **Đo cạnh ô ĐEN của tag bằng thước cặp**, khai vào `config/ee_tag.yaml` →
   `standalone_tags.ee_tag.size`. Đây là thứ rẻ nhất mà sai nhiều nhất: khai
   28 mm cho tag in ra 28.6 mm là sai cự ly 2%, ở 0.6 m thành **12 mm** — lớn hơn
   toàn bộ thứ ta đang cố đo. Máy in giấy và in 3D đều co ngót vài phần trăm.
   (Tag giá dùng `config/rack_tag.yaml`, cùng vấn đề.)
2. **Dán tag lên mặt cứng phẳng** (nhôm/mica) rồi mới bắt lên gá in 3D. AprilTag
   giả định mặt tag phẳng tuyệt đối; nhựa in cong vênh là sai số không gỡ được.
3. Tag **to hơn thì chính xác hơn** (sai số góc tỉ lệ nghịch với số pixel trên
   cạnh tag). Hiện tag 28 mm ở tầm 0.36–0.56 m chỉ chiếm ~45–70 px.

### Chạy — CẦN BA LƯỢT, không phải một

Bộ `tagcal` có 20 pose. Kiểm tra chéo chia đôi theo pose, nên một lượt chỉ còn
~9 pose mỗi nửa — không đủ. Đo 2026-09-11: lượt đơn cho hai nửa lệch **4.2–5.5 mm**
(`DỮ LIỆU CHƯA ĐỦ`), ba lượt gộp lại cho **0.8 mm** (`ĐÁNG TIN`). Ba lượt dùng
động lực học **khác nhau** để sai số do quán tính/ma sát không cùng dấu ở cả ba.

```bash
./rx150.sh t1-hac       # T1
./rx150.sh t2           # T2
./rx150.sh eetag        # T3  detector tag tay gắp
./rx150.sh eetag-tagcal # T4  ~17 phút: 3 lượt + gộp + cài, một lệnh
./rx150.sh t2           # khởi động lại T2 để frame ee_tag_link dùng số mới
```

`eetag-tagcal` tự chạy ba lượt (bậc thang → trườn chậm chiều + → trườn chậm
chiều −) vào `tuning_runs/tagcal_<ts>/pass{1,2,3}/`, rồi gộp và cài. Một lượt
hỏng là nó **dừng, không gộp**.

Muốn tự tay từng bước (vd chỉ đo lại một lượt, hoặc gộp thêm lượt cũ):

```bash
ros2 run rx150_motion_common rx150_ee_tag_bench.py tagoffset \
    tuning_runs/<a>/tagcal_hold.csv \
    tuning_runs/<b>/tagcal_hold.csv \
    tuning_runs/<c>/tagcal_hold.csv --install
```

Đưa nhiều CSV thì nhãn pose được thêm tiền tố theo lượt, nên 3 × 20 thành ~56–60
nhóm độc lập — kiểm tra chéo và jackknife đều tính theo **nhóm**, không theo mẫu.
`--install` ghi vào `ee_tag_offset.yaml` đang dùng, sao lưu bản cũ thành
`ee_tag_offset_previous.yaml` cạnh CSV đầu tiên.

### Đọc kết quả

```
tịnh tiến t_X = (  -62.06,    -2.91,   +39.19) mm
   ± (jackknife) (    0.63,     0.81,     0.43) mm
   xoay R_X    = roll -1.63°  pitch +0.22°  yaw -89.82°   ± 0.16°
dư sau khớp: vị trí 4.29 mm RMS, hướng 0.97° RMS
Kiểm tra chéo: hai nửa lệch 0.8 mm, sai số 1.0 mm — ĐÁNG TIN
```

* **GO** khi dòng cuối nói `ĐÁNG TIN`. `--install` **từ chối cài** nếu nó nói
  `DỮ LIỆU CHƯA ĐỦ` — đó là lúc cần thêm lượt, không phải lúc ép cài.
* Sai số mong đợi ở cấu hình hiện tại (3 lượt): **±1 mm / ±0.2°**.
* Ba lượt riêng lẻ phải cho `t_X` gần nhau. Đo 2026-09-11: ba lượt lệch nhau
  **< 0.9 mm** dù động lực học khác hẳn ⇒ con số không phải artefact của cách tay
  máy đi tới pose. Nếu ba lượt lệch nhau nhiều mm thì đừng gộp, đi tìm nguyên nhân.
* Cột `enc→cam` mà phần `analyze` in ra với bộ pose này là **RÁC** (~17 mm) —
  trung vị `X_i` hỏng khi tư thế trải rộng. Số đúng là `dư sau khớp` ở trên.

### Vì sao phải là bộ pose `tagcal`, không dùng bộ khác

Phương trình là `R_bc·p_raw + t_bc = p_ee + R_ee·t_X`. `t_X` chỉ tách được khỏi
`t_camera` nhờ **`R_ee` thay đổi** giữa các pose. Mọi bộ pose khác (`default`,
`calib`, `grid`) đều để `wrist_rotate = 0` ⇒ `R_ee·t_X` gần như hằng và lẫn hoàn
toàn vào `t_camera`. `tagcal` quét `wrist_rotate` ±1.2 rad, chọn bằng tìm kiếm
trên 2059 ứng viên để tối ưu trực tiếp phương sai của `t_X`.

---

## B. Hiệu chuẩn TF camera

```bash
./rx150.sh rack-calib     # snap thô: để lưới pose biết đường TRÁNH cái giá
./rx150.sh eetag-calib    # ~5 phút, 39 pose lưới, tự chạy refine
```

Đọc phần `Kiểm tra chéo` trong kết quả:

* `hai nửa cho cùng một hiệu chỉnh ⇒ lệch THẬT` → **áp dụng**
* `hiệu chuẩn hiện tại ĐÃ ỔN` → **đừng** áp dụng, đã hội tụ
* `hai nửa LỆCH NHAU nhiều` → dữ liệu chưa đủ, đo lại

Áp dụng — **thứ tự này quan trọng**:

```bash
# 1. TẮT T2 TRƯỚC (static_trans_pub ghi đè file lúc THOÁT — chép khi nó còn chạy là mất)
# 2. chép đè
cp tuning_runs/<run>/static_transforms_refined.yaml \
   src/rx150/rx150_toolbox/rx150_perception/config/static_transforms.yaml
# 3. bật lại T2
./rx150.sh t2
# 4. BẮT BUỘC: snap lại giá, vì rack_pose nằm TRÊN NỀN TF vừa đổi
./rx150.sh rack-calib
```

Không cần `colcon build`: `install/` là symlink về `src/`.

### Vì sao không dùng armtag Snap Pose làm chuẩn

Snap Pose giải 6 ẩn từ **một** tấm ảnh. Đo 2026-09-11: bản Snap Pose đang dùng
lệch **5.61° và 29 mm**, trong đó ~2.09° là do gá tag (phần A ở trên) và phần còn
lại (~3.5°) là nhiễu một-khung-hình của chính nó. Sửa xong phần A thì Snap Pose
bớt sai nhưng **vẫn kém xa** đường 39 tư thế.

Snap Pose vẫn hữu ích như phép **nghiệm thu**: chạy `./rx150.sh calib` 5 lần ở 5
tư thế khác nhau rồi so với bản `eetag-calib`. Trung bình độ lệch = sai số gá tag
còn sót; độ tản giữa 5 lần = nhiễu của Snap Pose.

---

## Trần độ chính xác hiện tại, và cách nâng

**Camera đang chạy trên link USB 2.0 (480 Mbps).** Kiểm tra:

```bash
for d in /sys/bus/usb/devices/*/idProduct; do [ "$(cat $d)" = 0b3a ] && \
  echo "$(cat $(dirname $d)/speed) Mbps"; done
```

Ở 480 Mbps, D435i **không** cấp nổi 1280×720×30 kèm depth — đã thử
`rs_camera_rgb_profile:=1280x720x30` ngày 2026-09-11 và driver **âm thầm tụt
xuống 640×480×15**, tức tệ hơn mặc định, không báo lỗi gì ngoài một dòng
`Device is connected using a 2.1 port`.

⇒ Muốn tăng độ phân giải thì **trước hết phải cắm camera vào cổng USB 3 bằng cáp
USB 3** (cổng xanh / có ký hiệu SS). Sau đó mới:

```bash
ros2 launch rx150_hac_controller hac_moveit.launch.py ... rs_camera_rgb_profile:=1280x720x30
ros2 run ... rgb_camera.color_profile   # XÁC NHẬN nó thật sự nhận, đừng tin là xong
```

Lưu ý 640×480 là 4:3 còn 720p là 16:9 — **kiểm lại khung hình** còn thấy đủ cả
tag tay lẫn tag giá không. Đổi độ phân giải xong phải chạy lại **A rồi B**.

Kỳ vọng: 720p cho ~2× cả vị trí lẫn góc; kèm tag 40 mm nữa thì ~3×.

---

## Bẫy đã trả giá

* **`install/` là symlink về `src/`**, mà `static_trans_pub` ghi hiệu chuẩn vào
  `transform_filepath` — nên nó ghi thẳng vào mã nguồn, cả lúc nhận transform lẫn
  **lúc tắt**. Chép file mới khi T2 còn chạy thì bị đè lại khi T2 thoát.
* **`armtag_tuner_gui` mặc định BẬT** trong `rx150_perception.launch.py` gốc ⇒ mỗi
  phiên `t2` thường ngày đều có thể đổi hiệu chuẩn. Đã đổi mặc định thành `false`;
  phiên `./rx150.sh calib` vẫn tự bật.
* **Đừng đặt tên frame là `ee_tag`**: detector trong `ee_tag.launch.py` đã phát
  `camera_color_optical_frame → ee_tag`. Trùng tên là frame có **hai cha**.
* **`apriltag_ros_continuous_detector_node` subscribe topic PRIVATE** `~/image_rect`.
  Trong launch file thì remap `('~/image_rect', ...)` ăn; qua `ros2 run --ros-args`
  thì phải viết tên đầy đủ `/rack_tag/image_rect:=...`, không thì node lên, không
  báo lỗi, và **không bao giờ nhận được một tấm ảnh nào**.
* **`tag_bundles: {bundle_names: []}`** trong YAML làm node chết lúc khởi động
  (rclcpp không suy được kiểu list rỗng). Bỏ hẳn khoá đó.
* **Id tag nằm ở nhiều file**: id 1 = tag tay gắp (`tags.yaml`,
  `apriltag_calib.yaml`, `ee_tag.yaml`); id 0 = tag giá (`rack_tag.yaml` +
  `tag_id` trong `rack_calib.yaml`).
* **U2D2 tự đổi cổng** (`ttyUSB0 → ttyUSB1`) giữa phiên: `xs_sdk` vẫn sống nhưng
  `joint_states` về −π và tay máy mềm. Bench sẽ báo "không thấy tag" chứ không báo
  lỗi bus. Kiểm nhanh:
  `ros2 topic echo /rx150/joint_states --field position --once`. Khắc phục: khởi
  động lại T1 (udev `/dev/ttyDXL` đã tự trỏ sang cổng mới).

## Điều không phép đo nào lấy được

`t_X` lấy `ee_gripper_link` làm gốc nên nó **hấp thụ luôn sai số khâu cuối của
URDF** — hai thứ đồng nhất về mặt toán học (cả hai vào dưới dạng `R_ee · δ`).
Con số đo ra là *"tag nằm đâu theo cách nhìn của chính mô hình robot"* — đúng thứ
armtag cần, nhưng **đừng** đem đối chiếu với bản vẽ CAD cái gá rồi kết luận gá in
sai mấy mm. Muốn biết gá có đúng bản vẽ không thì đo bằng thước cặp.
