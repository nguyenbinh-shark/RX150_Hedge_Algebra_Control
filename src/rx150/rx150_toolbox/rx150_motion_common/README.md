# rx150_motion_common — hạ tầng & công cụ dùng chung cho các bộ điều khiển

**Tầng IRROS:** Application Support · **Ngôn ngữ:** Python

Nơi chứa những thứ cả ba bộ điều khiển (`fuzzy`, `ff`, `hac`) đều cần, để chúng không phải
copy của nhau. **Package này cố tình không có thư viện C++**: mỗi controller tự biên dịch
engine riêng, chỉ dùng chung phần hạ tầng Python + cấu hình.

## Cấu trúc

| Đường dẫn | Nội dung |
| :--- | :--- |
| `scripts/rx150_trajectory_bridge.py` | `FollowJointTrajectory` action server → setpoint. Cầu nối MoveIt ↔ controller |
| `scripts/rx150_tuning_gui.py` | GUI Tkinter tune gain trực tiếp khi đang chạy, lưu thẳng vào YAML |
| `scripts/rx150_tuning_session.py` | Ghi một phiên tune: `data.csv` 46 cột + `plots/*.png` |
| `scripts/rx150_gravity_id.py` | Nhận dạng hệ số trọng lực (B4: `identify` → `validate`) |
| `scripts/rx150_friction_id.py` | Nhận dạng ma sát theo từng khớp và từng tốc độ |
| `scripts/rx150_steady_state_bench.py` | Sai số xác lập theo khớp trên bộ pose cố định (trước/sau hiệu chuẩn) |
| `scripts/rx150_stiction_hysteresis.py` | Trễ do ma sát tĩnh: cùng pose, tiếp cận từ hai phía |
| `scripts/rx150_ee_tag_bench.py` | **Đo bằng camera**: vị trí tay gắp theo AprilTag vs encoder vs lệnh |
| `scripts/compare_fuzzy_vs_hac.py` | Mặt điều khiển 3D + overlay quỹ đạo, xuất CSV so sánh |
| `scripts/plot_hac_velocity_analysis.py` | Phân tích 4 góc phần tư (e, ė) của mặt HAC |
| `scripts/rx150_run_compare.py` | Overlay nhiều lần chạy đã ghi |
| `scripts/tuning_lib.py` | Hằng số khớp + đọc/ghi CSV dùng chung cho các script trên |
| `config/rx150_motor.yaml` | Cấu hình động cơ Dynamixel — **một nguồn sự thật** cho mọi controller |
| `config/sensors_3d.yaml` | Cấu hình Octomap cho MoveIt (đọc point cloud từ D435i) |
| `env-hooks/geometric_shapes_shim.sh` | Shim cho overlay `~/ws_moveit` cũ (cần `libgeometric_shapes.so.2.3.2` trong khi Humble ship 2.3.4) |

## Dùng

```bash
ros2 run rx150_motion_common rx150_tuning_gui.py          # tune gain live
ros2 run rx150_motion_common rx150_gravity_id.py identify # nhận dạng trọng lực
ros2 run rx150_motion_common compare_fuzzy_vs_hac.py      # so sánh A/B
```

### Đo vị trí tay gắp bằng camera (`rx150_ee_tag_bench.py`)

Encoder chỉ biết góc khớp — nó **không thấy** võng khâu, rơ bánh răng hay sai số hình học.
Dán một AprilTag (tag36h11 **id 1** — id 0 dành cho tag của giá đỡ) lên tay gắp và để camera đo, ta có nguồn đo **độc lập**
với robot. Bench so ba quỹ đạo của cùng điểm `rx150/ee_gripper_link`:

| | nguồn | trả lời câu gì |
| :--- | :--- | :--- |
| `cmd` | FK giải tích từ setpoint gửi controller | — |
| `enc` | TF `base_link → ee_gripper_link` (joint_states) | `cmd→enc`: controller bám setpoint tốt đến đâu |
| `cam` | AprilTag, quy về EE qua offset gắn tag | `enc→cam`: phần **encoder không thấy**; `cmd→cam`: sai số pick_place thực chịu |

Offset gắn tag `X = T(ee_gripper_link → tag)` mặc định **được fit từ chính dữ liệu** (trung
vị vị trí + trung bình quaternion trên các mẫu đứng yên), nên không cần đo tay xem tag dán
lệch bao nhiêu. Đổi lại, mọi sai lệch **hằng** (dán lệch, bias hiệu chuẩn camera) bị hấp thụ
vào `X`: khi đó bench chỉ đo được độ **không nhất quán giữa các pose**.

Đo lại chính `X` đó: `./rx150.sh eetag-tagcal` (bộ pose `tagcal`, ~6 phút, tự cài kết quả
và sao lưu bản cũ). Quy trình đầy đủ + tiêu chí GO/NO-GO:
[docs/tuning/hieu_chuan_tag_va_camera.md](../../../../docs/tuning/hieu_chuan_tag_va_camera.md).

Muốn số tuyệt đối thì truyền `--tag-offset ee_tag_offset.yaml` — bản `X` đo riêng bằng bài
AX=ZB (`mode tagoffset`, ±1.0 mm). `X` cố định ⇒ cột `enc→cam` trở lại là **sai lệch thật**,
và câu hỏi "có offset hằng không" mới trả lời được. Mọi phép đo hiệu chuẩn/kiểm tra dưới đây
đều chạy với cờ này.

Bốn terminal:

```bash
./rx150.sh t1-hac      # T1  robot + MoveIt + camera        (fuzzy: dùng t1)
./rx150.sh t2          # T2  TF hiệu chuẩn world ↔ camera (static_transforms.yaml)
./rx150.sh eetag       # T3  detector AprilTag liên tục 30 fps → /ee_tag/tag_detections
./rx150.sh eetag-hold  # T4  robot đi qua bộ pose tĩnh rồi về sleep
./rx150.sh eetag-watch # T4' hoặc: chỉ GHI trong lúc pick_place chạy, không ra lệnh
```

Với controller fuzzy, thêm `--setpoint-topic /rx150/fuzzy/setpoint`. Chế độ `sweep` quét
quỹ đạo chậm để xem trễ/vọt lố động; `analyze <csv>` tính lại + vẽ mà không cần robot.
Kết quả (CSV + JSON + PNG) vào `tuning_runs/eetag_<ts>/`.

#### Hiệu chuẩn lại TF camera (`refine`)

Nếu `enc→cam` là một **hàm tuyến tính của vị trí** (lệch y tỉ lệ với z, v.v.) thì thủ phạm
không phải cơ khí robot mà là phép quay trong TF camera: armtag Snap Pose chỉ có **một** tư
thế để giải. Tag trên tay gắp cho hàng nghìn tư thế, đủ để giải lại cả extrinsic:

```bash
./rx150.sh eetag-calib     # 39 pose lưới (tránh hộp giá) -> tự chạy refine luôn
# hoặc tự gọi:
ros2 run rx150_motion_common rx150_ee_tag_bench.py refine <csv> --tag-offset ee_tag_offset.yaml
```

Có `--tag-offset` thì `refine` **chỉ giải 6 ẩn extrinsic**, `t_X` giữ nguyên. Quan trọng:
tịnh tiến của extrinsic và `t_X` đổi chác gần như 1:1 khi cả hai cùng tự do, nên bài 9 ẩn
luôn cho residual đẹp nhưng chia phần sai lệch **tuỳ ý** giữa hai bên — không dùng để kết
luận "camera lệch bao nhiêu" được. Không có cờ đó thì nó quay về bài 9 ẩn như cũ.

`--keepout-rack` bỏ những pose rơi trúng hộp vật cản của giá (đọc `rack_box` trong
`rack_pose.yaml`) — **snap giá trước** thì vùng cấm mới đúng chỗ.

`refine` giải `T_ref_cam` bằng least-squares, **tự kiểm tra chéo**
(fit một nửa số pose, đo trên nửa chưa thấy) và tách phần dư còn phụ thuộc tư thế. Nó ghi
`static_transforms_refined.yaml` **nhưng không ghi đè** bản đang dùng — TF đó đổi cả toạ độ
vật mà pick_place nhìn thấy, nên việc áp dụng là quyết định có ý thức.

Đọc kết quả: chênh góc giữa hai lần kiểm tra chéo < 1° ⇒ lệch là thật; tầm phủ dữ liệu dưới
100 mm theo trục nào thì hiệu chỉnh xoay quanh trục đó lẫn với tịnh tiến, đừng tin.

**Đo lại 2026-09-11** (HAC, 39 pose lưới phủ 221×470×162 mm): hiệu chuẩn đang dùng lệch
**5.61°** và **29 mm**; sau khi áp dụng, đo lại đúng 34 nhóm pose đó thì `enc→cam` đi từ
16.63 → **4.15 mm** RMS, bias 11.5 → **1.6 mm**, sai hướng 5.62° → **1.23°**, và lần refine
kế tiếp chỉ đòi 0.52° với kiểm tra chéo *tệ đi* ⇒ đã hội tụ. Chi tiết + bài test quỹ đạo:
[docs/tuning/do_chinh_xac_camera_robot.md](../../../../docs/tuning/do_chinh_xac_camera_robot.md).

**Đo trên máy 2026-09-09** (HAC, 27 pose lưới, 5470 mẫu): hiệu chuẩn armtag lệch **4.48°**
(chủ yếu pitch +3.44°); kiểm tra chéo 7.06→3.37 và 6.17→3.34 mm trên pose chưa thấy, chênh
0.64° ⇒ lệch thật. Đã **áp dụng** và đo lại đúng 27 pose đó:

| | enc→cam RMS | max | cmd→cam RMS |
| :--- | ---: | ---: | ---: |
| armtag Snap Pose | 7.84 mm | 15.42 mm | 8.09 mm |
| sau `refine` | **3.15 mm** | **7.02 mm** | **4.98 mm** |

Lần refine thứ hai chỉ đòi sửa thêm 0.17° và kiểm tra chéo *tệ đi* 0.30 mm ⇒ đã hội tụ,
không còn gì để chỉnh. Phần dư còn lại có cấu trúc **−16 mm/m theo tầm với ở trục z** — tay
võng xuống ~1.6 mm mỗi 100 mm vươn ra, đúng thứ encoder mù và `refine` không sửa được.

#### Bám quỹ đạo + tới điểm trên đường gắp (`pick`)

```bash
./rx150.sh eetag-pick                       # 4 lỗ × 2 vòng, ~2 phút
./rx150.sh eetag-pick -- --slots 1,3 --cycles 3 --cart-vel 0.03
ros2 run rx150_motion_common rx150_ee_tag_bench.py pickreport <csv> --tag-offset ee_tag_offset.yaml
```

Đi **đường thẳng trong không gian Descartes** qua đúng chuỗi điểm của `pick_place`: treo
trên lỗ → hạ → dừng → nhấc → sang lỗ kế, tốc độ lấy thẳng từ `tube_rack_params.yaml`. Điểm
thấp nhất dừng cách miệng lỗ `--clearance` (mặc định 15 mm) nên **không bao giờ chạm giá**,
kể cả khi `rack_pose.yaml` lệch.

Toàn tuyến được **kiểm khô trước khi động cơ nhúc nhích**: IK từng bước, bước khớp lớn nhất
(bắt lật nhánh IK), sàn `z`, và góc tới mặt tag. Cái cuối là ràng buộc thật: camera treo cao
mà mặt tag hướng theo `+z` của `ee_gripper_link`, nên tư thế gắp thật (pitch ≈ 90°) quay mặt
tag **ngang** và camera gần như không đọc được. Mỗi điểm chốt vì thế tự chọn pitch lớn nhất
trong ladder còn giữ góc tới ≤ `--max-incidence` (60°) — hình dạng đường đi giữ nguyên, chỉ
độ chúc cổ tay giảm (thực tế ra 37–54°).

Báo cáo tách ba câu hỏi:

1. **Tới điểm** — ở các điểm dừng, tách `enc→cam` thành phần **hằng** và phần **thay đổi
   theo điểm**; bỏ `--settle-skip` giây đầu (mặc định 0.5) để quá độ không bị đọc thành sai
   số tĩnh, và khử mẫu trùng khung ảnh (bench lấy mẫu nhanh hơn 30 fps của camera).
2. **Bám quỹ đạo** — ước lượng **trễ** và **lệch hằng** *đồng thời* bằng cách khớp
   `cam(t) ≈ enc(t−τ) + b`. Không tách thì trễ 60 ms ở 50 mm/s đội lên 3 mm và bị đọc nhầm
   thành offset. Kèm độ **cong** của đường đi so với đoạn thẳng nối hai đầu (không dính trễ).
3. **Hai tag** — vector tay↔giá do camera đo trong **cùng một khung hình** so với vector
   robot tin. Hiệu hai pose trong một ảnh không đi qua phần tịnh tiến của hiệu chuẩn
   hand-eye, nên đây là phép kiểm độc lập ở đúng chỗ sắp gắp. Cần detector tag giá chạy
   (`rack_calib.launch.py mode:=watch`).

**Đo 2026-09-11** (HAC, sau hiệu chuẩn): `enc→cam` hằng **3.22 mm**, thay đổi giữa các điểm
chỉ ~1 mm, **trễ 0 ms**; kiểm chéo hai tag cho 3.60 mm — khớp nhau. Nhưng `lệnh→enc` còn
**10.14 mm** và **đảo dấu theo chiều đi** (hạ xuống dừng cao hơn lệnh 10.6 mm, nhấc lên nằm
thấp hơn 10.8 mm, không hội tụ sau 3 s) ⇒ **ma sát tĩnh**, không phải camera, mới là thứ
chặn độ chính xác gắp hiện nay.

> **Bẫy đã trả giá**: `install/` ở workspace này là **symlink về `src/`**, còn `static_trans_pub`
> lưu hiệu chuẩn vào `transform_filepath` — nên nó ghi thẳng vào mã nguồn, cả khi nhận
> transform mới lẫn **lúc tắt** (chép file mới trong khi T2 còn chạy thì bị đè lại khi T2
> thoát: tắt T2 trước, chép sau). Nguồn ghi là `armtag_tuner_gui`, vốn mặc định BẬT trong
> `rx150_perception.launch.py` ⇒ mỗi phiên `t2` thường ngày đều có thể đổi hiệu chuẩn.
> Đo được: bản đầu phiên và bản sau một lần khởi động lại T2 lệch nhau **9.75° và 30 mm**,
> bản trong git HEAD lại khác cả hai. Mặc định GUI nay là `false`; phiên hiệu chuẩn
> (`./rx150.sh calib`) vẫn tự truyền `use_armtag_tuner_gui:=true`.

**Kích thước tag phải đúng** (`rx150_perception/config/ee_tag.yaml`): sai size làm sai
khoảng cách camera→tag theo đúng tỉ lệ đó — khai 28 mm cho tag in 30 mm là lệch ~35 mm ở
tầm 0.5 m, lớn hơn mọi sai số điều khiển đang đo.

Các script vẽ đồ thị ghi kết quả vào **thư mục đang đứng** (đổi bằng `RX150_PLOT_DIR`),
không ghi vào `scripts/`. Kết quả tham chiếu: [docs/tuning/](../../../../docs/tuning/).

## Lưu ý khi sửa

`rx150_motor.yaml` được cả ba controller nạp. Đổi `operating_mode` hay giới hạn ở đây là
đổi cho **tất cả** — đó là chủ ý, đừng fork thành bản riêng cho từng controller.
