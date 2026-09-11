<div align="center">

# RX150 · Hedge Algebra Control

**Bảng luật mờ có thực sự đáng giá không?**
Một cánh tay robot thật, ba bộ điều khiển thay nhau cùng một chỗ, và phép đo để trả lời.

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Robot](https://img.shields.io/badge/Robot-Interbotix%20RX150-0A7E8C)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

**Tiếng Việt** · [English](README.en.md)

</div>

---

## Ý tưởng

Cánh tay **Interbotix ReactorX-150** mặc định đóng vòng vị trí **bên trong firmware** của
động cơ Dynamixel. Vòng đó không quan sát được, không sửa được, và vì thế không so sánh
được — muốn hỏi "luật điều khiển này có tốt hơn luật kia không" thì không có chỗ để hỏi.

Dự án này **kéo vòng đó ra ngoài**. Động cơ chuyển sang `operating_mode: pwm`, firmware
chỉ còn đóng vai trò khuếch đại công suất, còn toàn bộ vòng kín vị trí chạy trên host ở
**100 Hz**. Luật điều khiển trở thành một thứ **tháo ra lắp vào được** — và đó là chỗ
để đặt câu hỏi.

Vào đúng vị trí đó, kho này cắm **ba luật khác nhau**:

| Bộ điều khiển | Luật | Feedforward |
| :--- | :--- | :--- |
| [`rx150_fuzzy_controller`](src/rx150/controllers/rx150_fuzzy_controller/README.md) | Fuzzy Mamdani type-1, sinh từ `.fis` sang C thuần | Bù trọng lực `g(q)` (Pinocchio RNEA) |
| [`rx150_ff_controller`](src/rx150/controllers/rx150_ff_controller/README.md) | **Cùng** bảng luật mờ đó | `Kv·q̇ + Ka·q̈` — động học thuần, không cần mô hình động lực học |
| [`rx150_hac_controller`](src/rx150/controllers/rx150_hac_controller/README.md) | **HAC — Đại số gia tử**, mặt tuyến tính ba hệ số | Bù trọng lực `g(q)` |

### Câu hỏi trung tâm

Bộ **HAC** thu gọn **toàn bộ** bảng luật Mamdani — tập mờ, hàm thuộc, luật hợp thành,
giải mờ — xuống còn **ba số vô hướng** `a`, `b`, `c`, qua ánh xạ định lượng ngữ nghĩa
của đại số gia tử:

$$u \;=\; \frac{2c}{3a}\,e \;+\; \frac{c}{3b}\,\dot e$$

Cả mặt điều khiển gói trong một dòng C ([`hac.c`](src/rx150/controllers/rx150_hac_controller/src/hac/hac.c)),
và `a/b/c` chỉnh được trực tiếp khi robot đang chạy qua ROS parameter.

> **Nếu HAC bám ngang bộ fuzzy, thì phần "mờ" đã không đóng góp gì ngoài một mặt phi
> tuyến.** Đó chính là câu hỏi mà kho này dựng ra để trả lời — bằng số đo trên phần cứng
> thật, không phải bằng mô phỏng.

Phép so sánh chỉ có nghĩa khi **mọi thứ còn lại được giữ nguyên**: cùng chu kỳ 100 Hz,
cùng đường lệnh PWM, cùng nguồn phản hồi `joint_states`, cùng profile Ruckig, và
`error_limit`/`error_dot_limit` của HAC đặt đúng bằng `1/Ke`, `1/Ked` của fuzzy.

### Đo dưới tải thật, không phải trên bàn thí nghiệm

Ba bộ điều khiển không chạy không tải. Bên trên chúng là một ứng dụng hoàn chỉnh:

* **Gắp ống nghiệm cắm lên giá** — YOLO segmentation tìm ống, mask → contour → endpoint →
  góc yaw, nắp màu để phân loại, gripper **xác nhận có vật trong ngón** trước khi nhấc.
* **Tương tác người–máy bằng cử chỉ** — MediaPipe Hands, tia ngón trỏ chỉ vào vật cần gắp,
  dấu OK để xác nhận.
* **Hiệu chuẩn hand-eye bằng AprilTag** + planning scene MoveIt để tránh vật cản động.

## Đã đo được gì

Hiệu chuẩn mô hình trọng lực trực tiếp trên phần cứng (132 mẫu, 2026-09-09) thay mô hình
Pinocchio + hệ số datasheet bằng mô hình lượng giác fit thẳng ở đơn vị PWM:

| Khớp | Residual RMS — Pinocchio + `Gff` | Residual RMS — model đã fit |
| :--- | ---: | ---: |
| shoulder | 53.8 | **35.4** |
| elbow | 80.0 | **60.2** |

Sai số xác lập xấu nhất trên 6 tư thế: **2.55° → 1.95°**.

**Nhưng phần dư còn lại không phải trọng lực.** Tiếp cận cùng một tư thế từ hai phía thì
sai số **đảo dấu** (elbow: −1.65° từ phía này, +1.43° từ phía kia). Đó là **ma sát tĩnh** —
và không lượng hiệu chuẩn trọng lực nào hạ được nó. Đòn bẩy tiếp theo là
`friction_coulomb`/`friction_viscous`, hiện vẫn để `[0,0,0,0,0]`.

→ Số liệu đầy đủ: [rx150_hac_controller/README.md](src/rx150/controllers/rx150_hac_controller/README.md) ·
dữ liệu thô: `tuning_runs/gravity_identify_20260909_182815/`

## Kiến trúc

Năm tầng theo chuẩn [IRROS](https://github.com/Interbotix/interbotix_ros_core#code-structure)
của Interbotix, để code dự án ghép được vào hệ sinh thái upstream mà không phải sửa:

```
Application         rx150_pick_place · rx150_hri          ← quyết định gắp GÌ, đặt Ở ĐÂU
Application Support rx150_modules · rx150_motion_common · rx150_perception
Control             rx150_fuzzy_controller │ rx150_ff_controller │ rx150_hac_controller
Driver              interbotix_xs_sdk · realsense2_camera · apriltag_ros
Hardware            RX150 (XL430/XM430) + U2D2 · RealSense D435i · AprilTag
```

**Tầng trên gọi tầng dưới, không bao giờ ngược lại.** Ba bộ điều khiển nằm cùng một chỗ
vì chúng là **ba biến thể của một bài toán**, chọn một trong ba — không phải chạy cả ba.

→ Cây thư mục đầy đủ và luật phụ thuộc: [docs/cau_truc_kho.md](docs/cau_truc_kho.md) ·
sơ đồ node/topic/tần số: [docs/so_do_dieu_khien.md](docs/so_do_dieu_khien.md)

## Bắt đầu nhanh

Cần Ubuntu 22.04 + ROS 2 Humble.

```bash
git clone https://github.com/nguyenbinh-shark/RX150_Hedge_Algebra_Control.git ~/RX150_Hedge_Algebra_Control && cd ~/RX150_Hedge_Algebra_Control
./tools/setup_vendor.sh      # kéo vendor Interbotix (KHÔNG commit) — bắt buộc trước khi build
./tools/build.sh             # colcon build, đã gỡ sẵn bẫy setuptools/packaging
source source_all.sh
```

Mọi chế độ chạy đi qua [`rx150.sh`](rx150.sh); gọi không tham số để xem danh sách.

```bash
./rx150.sh reach      # B0  bảng tầm với + kiểm config  — KHÔNG cần robot
./rx150.sh t1         # B1  robot + MoveIt + camera     (torque BẬT, tay tự về home)
./rx150.sh t2         # B2  perception + TF hiệu chuẩn
./rx150.sh check      # B3  smoke-test: joint_states + action server + TF/detection
./rx150.sh dry-fake   # B4  chạy hết state machine với ống giả — không cần camera
./rx150.sh tubes      # B5  gắp ống nghiệm thật
```

> ⚠️ Ở PWM mode, **mất node là tay rơi** — firmware không còn giữ vị trí nữa. Thang bậc
> B0→B6 có nguyên tắc: **hỏng ở bậc nào thì dừng ở bậc đó.**

→ Cài đặt chi tiết: [docs/cai_dat.md](docs/cai_dat.md) ·
quy trình chạy thật: [RUNBOOK](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md)

## Tài liệu

README này chỉ giới thiệu ý tưởng. Toàn bộ tài liệu chuyên sâu — cài đặt, kiến trúc,
điều khiển, hiệu chuẩn, nhận diện, kiểm thử — có mục lục ở **[docs/](docs/README.md)**.

## Đóng góp

Xem [CONTRIBUTING.md](CONTRIBUTING.md). Tóm tắt: package mới đặt đúng tầng IRROS, **mỗi
package phải có README riêng**, và không tự viết lại primitive đã có trong `rx150_modules`.

## Giấy phép & trích dẫn

BSD 3-Clause — xem [LICENSE](LICENSE). `src/vendor/` giữ giấy phép gốc của Interbotix và
các bên thứ ba. Nếu dùng kho này trong công bố khoa học, xem [CITATION.cff](CITATION.cff).
