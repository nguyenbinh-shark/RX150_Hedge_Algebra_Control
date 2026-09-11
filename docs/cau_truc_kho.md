# Cấu trúc kho & phân tầng IRROS

> Tài liệu này là bản chi tiết của phần "Kiến trúc" trong [README](../README.md).
> README chỉ nói *ý tưởng*; ở đây là *bản đồ đầy đủ*.

Kho được tổ chức theo chuẩn [IRROS của Interbotix](https://github.com/Interbotix/interbotix_ros_core#code-structure)
để code của dự án ghép được vào hệ sinh thái upstream mà không phải sửa.

---

## 1. Cây thư mục

```
RX150_Hedge_Algebra_Control/
├── rx150.sh                   Điểm vào duy nhất — mọi chế độ chạy đều qua đây
├── rx150.repos                Pin phiên bản vendor Interbotix + third-party
├── source_all.sh              Source 4 overlay (ROS → apriltag → easy_handeye2 → ws này)
├── docs/                      Tài liệu dài
│   ├── cau_truc_kho.md          ← bạn đang đọc
│   ├── cai_dat.md               Cài đặt, vendor, bẫy setuptools/packaging
│   ├── so_do_dieu_khien.md      Sơ đồ node/topic/action, tần số, 3 vòng kín
│   ├── lenh_chay_tube_rack.md   Sổ lệnh dán thẳng vào terminal
│   ├── tuning/                  Kết quả đo & quy trình hiệu chuẩn
│   └── lich_su/                 Đối chiếu với bản cũ — KHÔNG mô tả hệ đang chạy
├── tuning_runs/               Dữ liệu từng lần đo (CSV thô KHÔNG commit)
├── module_tests/              Smoke-test theo tầng, chạy độc lập với colcon
├── tools/                     setup_vendor.sh, build.sh, collect_diag.sh, record/plot_tube_run.py
├── data_analysis/             Ghi CSV + vẽ đồ thị (PlotJuggler layout)
├── fuzzy_codegen/             Sinh fuzzy_type1.c từ file .fis (MATLAB FIS → C)
└── src/
    ├── vendor/                KHÔNG commit — khôi phục bằng ./tools/setup_vendor.sh
    │   ├── interbotix_ros_core            Driver layer (xs_sdk, xs_driver, xs_msgs)
    │   ├── interbotix_ros_manipulators    Mô tả + MoveIt config cho X-Series
    │   ├── interbotix_ros_toolboxes       Module Python hỗ trợ của Interbotix
    │   └── moveit_visual_tools
    └── rx150/
        ├── rx150/                         Metapackage — depend toàn bộ bên dưới
        ├── controllers/                   ── Control layer ──
        │   ├── rx150_fuzzy_controller       Fuzzy Mamdani type-1 + bù trọng lực (Pinocchio)
        │   ├── rx150_ff_controller          Fuzzy PD + feedforward vận tốc/gia tốc
        │   └── rx150_hac_controller         HAC tuyến tính + Ruckig + bù trọng lực
        ├── rx150_toolbox/                 ── Application Support layer ──
        │   ├── rx150_modules                Thư viện dùng chung: IK giải tích, MoveIt
        │   │                                executor, gripper có xác nhận kẹp, scene, skills
        │   ├── rx150_motion_common          Hạ tầng chung cho controller: trajectory bridge,
        │   │                                tuning GUI, nhận dạng ma sát/trọng lực, cấu hình motor
        │   └── rx150_perception             YOLO + cử chỉ tay + PCL + AprilTag hand-eye
        └── apps/                          ── Application layer ──
            ├── rx150_pick_place             Gắp ống nghiệm lên giá; gắp vật chỉ bằng cử chỉ
            └── rx150_hri                    Tương tác người–máy (task ↔ executor qua topic)
```

Ba cấp `src/rx150/<nhóm>/<package>` là bản áp dụng của cấu trúc ba tầng upstream:
*landing page* → *thư mục nhóm* → *package*.

---

## 2. Năm tầng IRROS, điền cho RX150

**I**nterbotix **R**esearch **R**obotics **O**pen **S**tandard:

| Tầng | Nội dung trong kho này |
| :--- | :--- |
| **Hardware** | RX150 (Dynamixel XL430/XM430) qua U2D2; RealSense D435i; AprilTag gắn trên đế |
| **Driver** | `interbotix_xs_sdk`, `interbotix_xs_driver` (vendor); `realsense2_camera`, `apriltag_ros` (bên thứ ba) |
| **Control** | `rx150_fuzzy_controller`, `rx150_ff_controller`, `rx150_hac_controller` — mỗi package có `config/` (gain, giới hạn) + launch bring-up riêng |
| **Application Support** | `rx150_modules` (IK/motion/gripper/scene dùng chung), `rx150_motion_common` (hạ tầng controller), `rx150_perception` (nhận diện) |
| **Application** | `rx150_pick_place`, `rx150_hri` — code người dùng cuối, chỉ gọi xuống tầng dưới |

---

## 3. Luật phụ thuộc

```
apps/  ──depend──▶  rx150_toolbox/  ──depend──▶  vendor (xs_sdk, MoveIt, RealSense)
                          ▲
controllers/ ─────────────┘
```

**Tầng trên gọi tầng dưới, không bao giờ ngược lại.** Cụ thể:

* `rx150_toolbox/*` **không được** import từ `apps/*`. Nếu một app cần dùng lại logic của
  app khác thì logic đó phải đi lên `rx150_modules` trước.
* `controllers/*` không phụ thuộc `apps/*`, và không phụ thuộc lẫn nhau ngoài phần hạ tầng
  chung ở `rx150_motion_common`.
* Ứng dụng mới **không được** tự viết lại primitive MoveIt/gripper/scene — dùng
  `rx150_modules`. Thiếu primitive thì **thêm vào `rx150_modules`**, đừng thêm vào app.
  Xem [src/rx150/apps/README.md](../src/rx150/apps/README.md).

---

## 4. Vì sao gom ba controller vào một thư mục

`fuzzy`, `ff` và `hac` là **ba biến thể của cùng một bài toán** (PWM vòng kín 100 Hz cho
RX150), tồn tại song song để so sánh A/B chứ không phải ba tính năng khác nhau. Gom lại
để rõ rằng **chọn một trong ba**, không phải chạy cả ba.

Điều kiện để ba bộ so sánh được với nhau — đổi một bên thì phải đổi bên kia:

* cùng chu kỳ 100 Hz, cùng đường lệnh `/rx150/commands/joint_group` (PWM);
* cùng nguồn phản hồi `/rx150/joint_states`;
* `error_limit` / `error_dot_limit` của HAC đặt bằng `1/Ke` / `1/Ked` của fuzzy;
* cùng profile Ruckig trước bộ điều khiển (`enable_profile`).

---

## 5. Dữ liệu: cái gì commit, cái gì không

| Nơi | Nội dung | Commit? |
| :--- | :--- | :--- |
| `tuning_runs/<nhãn>_<ts>/` | Dữ liệu thô từng lần đo | `*.csv` **không** (≈98% dung lượng); `meta.json`, `*.png`, `summary.txt`, `*.yaml`, `*.json` **có** |
| `docs/tuning/*.png` | Đồ thị mặt điều khiển & quỹ đạo | **Không** — sinh lại được hoàn toàn bằng tính toán, không cần robot |
| `docs/tuning/*.md` | Kết luận đã viết thành văn | **Có** |
| `src/vendor/` | Interbotix upstream + MoveIt | **Không** — pin trong `rx150.repos`, khôi phục bằng `./tools/setup_vendor.sh` |
| `build/`, `install/`, `log/` | Sản phẩm colcon | **Không** |

Lệnh sinh lại đồ thị nằm ở [docs/tuning/README.md](tuning/README.md).
