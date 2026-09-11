# Changelog

Mọi thay đổi đáng kể của kho này. Định dạng theo
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

*Notable changes to this repository, following [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).*

Kho chưa phát hành phiên bản đánh số — các mốc dưới đây theo ngày.
*No numbered releases yet; milestones below are dated.*

---

## [Chưa phát hành · Unreleased]

### Thêm · Added
- README song ngữ VI/EN, mục lục tài liệu `docs/README.md`, và bộ tệp chuẩn GitHub
  (`CONTRIBUTING`, `CODE_OF_CONDUCT`, `CITATION.cff`, mẫu issue/PR)
  *· Bilingual VI/EN READMEs, a `docs/` index, and the standard GitHub file set*
- `docs/cau_truc_kho.md` và `docs/cai_dat.md` — tách phần chuyên sâu ra khỏi README
  *· deep repo-structure and installation docs split out of the README*

## 2026-09-11

### Thêm · Added
- Dữ liệu hiệu chuẩn TF và tag (`tfcal`, `tfval`, `tagcal`) trong `tuning_runs/`

### Sửa · Changed
- Dọn dữ liệu tuning cũ, sửa các đường dẫn còn theo bố cục tiền-IRROS, gọn lại tài liệu
- `.gitignore`: loại CSV thô khỏi `tuning_runs/` (≈98% dung lượng) và PNG sinh lại được
  khỏi `docs/tuning/`

## 2026-09-09

### Thêm · Added
- Nhận dạng mô hình trọng lực trên phần cứng: `rx150_gravity_id.py`, model lượng giác fit
  ở đơn vị PWM, và benchmark sai số xác lập trước/sau
  *· hardware gravity identification, PWM-unit fitted model, before/after steady-state bench*
- Phát hiện **sàn sai số là ma sát tĩnh, không phải trọng lực** — residual đảo dấu theo
  hướng tiếp cận (`rx150_stiction_hysteresis.py`)
  *· identified the error floor as stiction, not gravity*

## 2026-09-06

### Thêm · Added
- `rx150_modules` — tầng Application Support dùng chung: IK giải tích, MoveIt executor có
  verify, gripper xác nhận kẹp, planning scene, skills, task status
- Metapackage `rx150`, README cho từng package theo yêu cầu upstream

### Sửa · Changed
- **Tái cấu trúc `src/rx150` theo 5 tầng IRROS** — `controllers/`, `rx150_toolbox/`, `apps/`
- `hri_motion` dùng `rx150_modules`, bỏ ~250 dòng primitive tự viết
- Gỡ 762 file vendor khỏi git, quản bằng `rx150.repos` + `tools/setup_vendor.sh`

## 2026-08-22 → 2026-08-27

### Thêm · Added
- `rx150_hac_controller` — bộ điều khiển đại số gia tử + bù trọng lực + công cụ phân tích
  quỹ đạo *· the Hedge Algebra Controller, the point of the whole repository*
- `tube_rack_node` — gắp ống nghiệm và cắm lên giá, phân theo màu nắp
- `rx150_hri` — tách task (quyết định) ↔ executor (chấp hành) qua topic

## 2026-08-10 → 2026-08-18

### Thêm · Added
- `rx150_fuzzy_controller` — fuzzy Mamdani type-1 ở PWM mode, 100 Hz, tích hợp MoveIt 2
- Bù trọng lực bằng Pinocchio RNEA từ URDF
- `fuzzy_codegen/` — sinh `fuzzy_type1.c` từ file `.fis` của MATLAB
- `rx150_perception` — YOLO segmentation, cử chỉ tay, PCL, AprilTag hand-eye
- `rx150_pick_place`, `data_analysis/`, cấu hình Octomap cho camera 3D
