# Tài liệu · Documentation

Mục lục toàn bộ tài liệu của kho. README ở gốc chỉ giới thiệu ý tưởng — mọi thứ sâu hơn
nằm ở đây.

*Index of every document in the repository. The root README introduces the idea only;
everything deeper lives here.*

---

## Bắt đầu · Getting started

| Tài liệu · Document | Trả lời câu gì · Answers |
| :--- | :--- |
| [cai_dat.md](cai_dat.md) | Cài đặt, dựng vendor, bẫy `setuptools`/`packaging`, gỡ rối *· Install, vendor setup, the setuptools/packaging trap, troubleshooting* |
| [cau_truc_kho.md](cau_truc_kho.md) | Cây thư mục đầy đủ, 5 tầng IRROS, luật phụ thuộc, cái gì commit *· Full tree, the 5 IRROS layers, dependency law, what gets committed* |
| [lenh_chay_tube_rack.md](lenh_chay_tube_rack.md) | Sổ lệnh dán thẳng vào terminal: bring-up, hiệu chuẩn, khôi phục motor *· Copy-paste command book* |
| [RUNBOOK.md](../src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md) | Thang bậc bring-up B0→B6, tiêu chí GO/NO-GO, bảng triệu chứng → nguyên nhân *· Bring-up ladder, GO/NO-GO gates, symptom triage* |

## Điều khiển · Control

| Tài liệu · Document | Trả lời câu gì · Answers |
| :--- | :--- |
| [so_do_dieu_khien.md](so_do_dieu_khien.md) | Sơ đồ cấu trúc điều khiển: node, topic/action, tần số, 3 vòng kín *· Node/topic/action map, rates, the three loops* |
| [rx150_fuzzy_controller](../src/rx150/controllers/rx150_fuzzy_controller/README.md) | Fuzzy Mamdani type-1 + bù trọng lực; vì sao PWM mode chứ không phải position mode *· Mamdani type-1 + gravity comp; why PWM mode* |
| [rx150_hac_controller](../src/rx150/controllers/rx150_hac_controller/README.md) | Mặt HAC ba hệ số, quy trình hiệu chuẩn trọng lực 4 bước, số đo phần cứng *· The three-coefficient HAC surface, 4-step gravity calibration, hardware numbers* |
| [rx150_ff_controller](../src/rx150/controllers/rx150_ff_controller/README.md) | Fuzzy PD + feedforward động học; vì sao droop tĩnh **không** phải bug *· Fuzzy PD + kinematic feedforward; why static droop is not a bug* |
| [fuzzy_codegen/](../fuzzy_codegen/README.md) | Thiết kế luật mờ, sinh `fuzzy_type1.c` từ `.fis`, xem mặt 3D *· Rule design, FIS → C codegen, 3D surface viewer* |

## Đo đạc & hiệu chuẩn · Measurement & calibration

| Tài liệu · Document | Trả lời câu gì · Answers |
| :--- | :--- |
| [tuning/](tuning/README.md) | Kết quả đo camera↔robot, quy trình hiệu chuẩn A→B→C, lệnh sinh lại đồ thị *· Camera↔robot accuracy, A→B→C calibration, plot regeneration* |
| [tuning/do_chinh_xac_camera_robot.md](tuning/do_chinh_xac_camera_robot.md) | Toạ độ camera báo về có trùng chỗ tay gắp thật tới không *· Does the reported camera pose match where the gripper actually lands* |
| [tuning/hieu_chuan_tag_va_camera.md](tuning/hieu_chuan_tag_va_camera.md) | Quy trình hiệu chuẩn tag trên tay gắp → camera → giá, và khi nào phải chạy lại *· Hand-eye chain and when to redo it* |
| [data_analysis/](../data_analysis/README.md) | Quy ước topic/CSV telemetry, ghi và vẽ đồ thị điều khiển *· Telemetry conventions, logging and plotting* |

## Nhận diện & ứng dụng · Perception & applications

| Tài liệu · Document | Trả lời câu gì · Answers |
| :--- | :--- |
| [PERCEPTION_GUIDE.md](../src/rx150/rx150_toolbox/rx150_perception/docs/PERCEPTION_GUIDE.md) | Hiệu chuẩn hand-eye, tune PCL, tham số YOLO *· Hand-eye calibration, PCL tuning, YOLO parameters* |
| [rx150_modules](../src/rx150/rx150_toolbox/rx150_modules/README.md) | Thư viện dùng chung: IK giải tích, MoveIt executor, gripper, planning scene *· Shared library: analytic IK, MoveIt executor, gripper, scene* |
| [rx150_pick_place](../src/rx150/apps/rx150_pick_place/README.md) | Gắp ống nghiệm lên giá; gắp vật chỉ bằng cử chỉ tay *· Tube-to-rack pick & place; gesture-pointed grasp* |
| [rx150_hri](../src/rx150/apps/rx150_hri/README.md) | Tách task (quyết định) ↔ executor (chấp hành) qua topic *· Task/executor split over topics* |
| [apps/README.md](../src/rx150/apps/README.md) | Cách thêm một ứng dụng mới cho đúng tầng *· How to add a new application at the right layer* |

## Kiểm thử & lịch sử · Testing & history

| Tài liệu · Document | Trả lời câu gì · Answers |
| :--- | :--- |
| [module_tests/](../module_tests/README.md) | Smoke-test theo tầng, chạy độc lập với colcon *· Per-layer smoke tests, independent of colcon* |
| [lich_su/](lich_su/) | Nhật ký đưa lên phần cứng + đối chiếu baseline cũ. **Không mô tả hệ đang chạy** *· Hardware bring-up log and comparison against the old baseline — not a description of the current system* |

---

## Quy ước · Conventions

* Tài liệu chuyên sâu viết bằng **tiếng Việt**; hai README ở gốc song ngữ VI/EN.
  *Deep docs are in Vietnamese; the two root READMEs are bilingual VI/EN.*
* Tên file dùng `snake_case` không dấu. *File names use unaccented `snake_case`.*
* Mỗi package ROS **phải có README riêng** — yêu cầu của upstream Interbotix.
  *Every ROS package must carry its own README — an upstream Interbotix requirement.*
* Số đo phải kèm ngày và đường dẫn dữ liệu thô trong `tuning_runs/`.
  *Every measurement cites its date and the raw-data path under `tuning_runs/`.*
