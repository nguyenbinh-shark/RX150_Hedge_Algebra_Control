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

Offset gắn tag `X = T(ee_gripper_link → tag)` **được fit từ chính dữ liệu** (trung vị vị trí
+ trung bình quaternion trên các mẫu đứng yên), nên không cần đo tay xem tag dán lệch bao
nhiêu. Đổi lại, mọi sai lệch **hằng** (dán lệch, bias hiệu chuẩn camera) bị hấp thụ vào `X`:
bench đo được độ **không nhất quán giữa các pose**, không đo được bias tuyệt đối của cả hệ.

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
./rx150.sh eetag-hold --pose-set grid --dwell 3.5 --settle 1.5   # 27 điểm lưới sinh bằng IK
ros2 run rx150_motion_common rx150_ee_tag_bench.py refine tuning_runs/eetag_*/grid27_hold.csv
```

`refine` giải đồng thời `T_ref_cam` (6) + `t_X` (3) bằng least-squares, **tự kiểm tra chéo**
(fit một nửa số pose, đo trên nửa chưa thấy) và tách phần dư còn phụ thuộc tư thế. Nó ghi
`static_transforms_refined.yaml` **nhưng không ghi đè** bản đang dùng — TF đó đổi cả toạ độ
vật mà pick_place nhìn thấy, nên việc áp dụng là quyết định có ý thức.

Đọc kết quả: chênh góc giữa hai lần kiểm tra chéo < 1° ⇒ lệch là thật; tầm phủ dữ liệu dưới
100 mm theo trục nào thì hiệu chỉnh xoay quanh trục đó lẫn với tịnh tiến, đừng tin.

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
