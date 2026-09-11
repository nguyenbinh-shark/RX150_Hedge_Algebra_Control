# Độ chính xác toạ độ camera ↔ robot trên quỹ đạo gắp — đo 2026-09-11

Trả lời một câu hỏi cụ thể: **toạ độ camera báo về có trùng chỗ tay gắp thật sự
tới không, và phần lệch có phải là một offset hằng do đo đạc cơ khí không.**

Dụng cụ: AprilTag **id 1** dán trên tay gắp, **id 0** dán trên giá đỡ ống nghiệm,
một camera D435i treo cao. Bộ điều khiển **HAC**. Toàn bộ số liệu thô nằm trong
`tuning_runs/tfcal_20260911_103546/`, `tuning_runs/tfval_20260911_105433/` và
`tuning_runs/pick_20260911_104810/`.

Cách đọc ba cột dùng xuyên suốt:

| cột | nghĩa |
| :--- | :--- |
| `lệnh→enc` | controller có tới chỗ được lệnh không (encoder tự báo) |
| `enc→cam` | camera và robot có **cùng một hệ toạ độ** không |
| `lệnh→cam` | tổng — sai số mà cú gắp thật sự chịu |

Offset gắn tag `X = T(ee_gripper_link → tag)` lấy **cố định** từ
`ee_tag_offset.yaml` (đo riêng bằng bài AX=ZB ngày 2026-09-11, ±1.0 mm). Đây là
điều kiện bắt buộc để câu hỏi "có offset hằng không" trả lời được: nếu để `X` tự
fit từ chính dữ liệu thì mọi sai lệch hằng bị nó nuốt và cột `enc→cam` chỉ còn đo
độ *không nhất quán* giữa các pose.

## 1. Có offset. Nó lớn 11 mm và 5.6°.

Hiệu chuẩn đang dùng trước hôm nay (armtag Snap Pose) lệch **thật**, đo trên 39
pose lưới phủ 221 × 470 × 162 mm:

```
enc→cam   bias = (−11.3, +2.1, +0.6) mm   RMS 16.63   max 36.72   hướng 5.62°
```

`refine` giải lại extrinsic (6 ẩn, `t_X` cố định) đòi sửa **5.61°** và
**(+16.3, −20.8, −12.9) mm**. Kiểm tra chéo — fit một nửa số pose rồi đo trên nửa
chưa thấy — cho cùng một hiệu chỉnh (chênh 0.21°, lợi 11.45 mm), nên đây là lệch
thật chứ không phải khớp nhiễu.

Sau khi áp dụng và **đo lại đúng 34 nhóm pose đó**:

| | enc→cam bias | RMS | max | sai hướng |
| :--- | ---: | ---: | ---: | ---: |
| armtag Snap Pose | 11.5 mm | 16.63 mm | 36.72 mm | 5.62° |
| sau `refine` | **1.6 mm** | **4.15 mm** | **7.43 mm** | **1.23°** |

Chạy `refine` lần nữa trên dữ liệu mới chỉ đòi thêm 0.52°, mà kiểm tra chéo *tệ đi*
0.02 mm ⇒ **đã hội tụ**, không còn gì để chỉnh.

Hai bằng chứng độc lập rằng bản mới đúng hơn, không phải chỉ "khớp đẹp hơn":

* Độ **nghiêng của giá** đo qua camera: 6.4° → **2.4°**. Giá đặt phẳng trên bàn,
  nên phần "nghiêng" biến mất chính là sai số hiệu chuẩn cũ.
* Vị trí 4 miệng lỗ dịch tới **19 mm** khi snap lại trên nền TF mới — đúng bằng
  lượng mà tay gắp trước đây bị đưa sai chỗ.

## 2. Sau hiệu chuẩn: camera và robot lệch nhau 3 mm, gần như hằng

Quỹ đạo mô phỏng gắp (`./rx150.sh eetag-pick`): treo trên lỗ → hạ 65 mm →
dừng → nhấc, qua cả 4 lỗ, 2 vòng, tốc độ đúng của `pick_place`
(0.05 m/s đi ngang, 0.025 m/s lên xuống). Điểm thấp nhất dừng **cách miệng lỗ
15 mm** nên bài test không bao giờ chạm giá. 24 điểm dừng, 3742 mẫu, camera thấy
tag ở **100%** số khung.

```
Ở các điểm dừng:  enc→cam phần HẰNG = (−2.94, −1.21, −0.52) mm  ‖3.22‖
                  phần THAY ĐỔI theo điểm (std) = (1.08, 1.03, 1.34) mm
Lúc đang chạy:    enc→cam  trễ 0 ms · lệch hằng 3.2 mm · dư 2.15 mm RMS
```

Ba điều rút ra:

1. **Trễ bằng 0.** Dấu thời gian của chuỗi ảnh và của `joint_states` khớp nhau —
   toàn bộ phần lệch là hình học, không phải đồng bộ thời gian. (Nếu không tách
   trễ ra thì 60 ms ở 50 mm/s đã thành 3 mm và bị đọc nhầm thành offset.)
2. **Phần còn lại gần như hằng**: dao động giữa các điểm chỉ ~1 mm, trong khi sàn
   nhiễu của một mẫu AprilTag đơn ở tầm này đã là 3–4 mm.
3. Phần thay đổi ít ỏi đó **có cấu trúc theo tư thế** (tầm với, độ cao), không
   phải nhiễu trắng — đây là võng cơ khí, thứ hiệu chuẩn không sửa được.

### Kiểm chéo không đi qua hiệu chuẩn hand-eye

Hai tag trong **cùng một khung hình**: hiệu hai pose không dính phần tịnh tiến của
`T_camera→base`, nên nó kiểm được đúng đại lượng cú gắp phải chịu.

```
vector tay→giá, camera đo TRỪ robot tin = (−2.05, −2.27, +1.88) mm  ‖3.60‖
                                          độ tản (1.23, 0.58, 1.23) mm
tag giá đo trực tiếp lệch rack_pose.yaml = (−0.35, +0.09, +2.30) mm
```

3.60 mm này khớp với 3.22 mm đo bằng đường khác ở trên ⇒ **toạ độ camera→robot
hiện sai khoảng 3 mm ở chỗ giá**, và con số đó ổn định.

## 3. Thủ phạm còn lại KHÔNG phải camera — là ma sát tĩnh

`lệnh→cam` vẫn còn **10.26 mm RMS**, nhưng `lệnh→enc` cũng **10.14 mm**: robot
không tới chỗ được lệnh, còn camera thì xác nhận chính xác chỗ robot đang đứng.

| loại điểm | `lệnh→enc` trung bình (x, y, z) mm |
| :--- | ---: |
| treo (`hover`) | (+0.19, −1.26, −0.31) |
| **đáy (`low`)** | (−0.26, −2.67, **−10.61**) |
| nhấc (`lift`) | (+1.05, −1.19, **+10.78**) |

Sai số **đảo dấu theo chiều đi**: hạ xuống thì tay dừng cao hơn lệnh 10.6 mm,
nhấc lên thì nằm thấp hơn lệnh 10.8 mm, và **không hội tụ** suốt 3 giây dừng.
Trọng lực chưa bù hết sẽ kéo lệch *một chiều*; đảo dấu là chữ ký của **ma sát
tĩnh / vùng chết**. Độ lớn tăng theo tầm với: lỗ gần (r = 0.232 m) lệch 7.0 mm,
lỗ xa (r = 0.320 m) lệch 12.9–16.1 mm.

Lúc đang chạy, controller bám setpoint **trễ 208 ms** — ở 25 mm/s là ~5 mm tụt sau,
và phần dư sau khi trừ trễ vẫn còn 7.17 mm RMS.

Hệ quả trực tiếp cho `pick_place`: khi hạ xuống gắp, tay gắp dừng **cao hơn chỗ
được lệnh ~10 mm ở lỗ gần và ~13–16 mm ở lỗ xa**. Sai số này lớn gấp ba lần toàn
bộ sai số camera, nên **mọi nỗ lực hiệu chuẩn camera thêm nữa đều vô nghĩa cho tới
khi bù được ma sát** — xem [friction_hac.md](friction_hac.md) và
`rx150_friction_id.py` (hệ số bù ma sát của HAC hiện vẫn = 0).

## 4. Cái bài test này KHÔNG trả lời được

Hiệu chuẩn hand-eye dùng chính cánh tay làm thước, nên **một offset hằng chung cho
cả hệ camera–robot là không quan sát được** — nó bị chính phép hiệu chuẩn hấp thụ.
Ba con số 3.22 / 3.60 mm ở trên là lệch *đo được* giữa hai nguồn sau khi đã cố định
`t_X`; nếu bản thân URDF hay `t_X` lệch hằng thì phần đó vẫn nằm ngoài tầm với của
phép đo. Muốn chốt tuyệt đối thì phải có **chân trị vật lý**: tắt torque, dắt tay
gắp cắm vào từng lỗ, ghi FK encoder rồi so với vị trí lỗ mà camera báo.

## Chạy lại

```bash
./rx150.sh t1-hac                 # T1
./rx150.sh t2                     # T2
./rx150.sh eetag                  # T3  tag tay gắp
ros2 launch rx150_perception rack_calib.launch.py mode:=watch   # T4 tag giá (cho phép kiểm chéo)

./rx150.sh rack-calib             # biết giá đang ở đâu (lưới hiệu chuẩn cần tránh nó)
./rx150.sh eetag-calib            # 39 pose → refine → static_transforms_refined.yaml
#   tắt T2 TRƯỚC khi chép đè, rồi bật lại T2  (static_trans_pub ghi đè file lúc thoát)
./rx150.sh rack-calib             # snap lại giá trên nền TF mới — BẮT BUỘC
./rx150.sh eetag-pick             # bài test quỹ đạo
```
