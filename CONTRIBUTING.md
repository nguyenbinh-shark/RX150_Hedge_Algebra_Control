# Đóng góp · Contributing

[Tiếng Việt](#tiếng-việt) · [English](#english)

---

## Tiếng Việt

### Trước khi viết dòng code đầu tiên

Đọc ba thứ này, theo thứ tự:

1. [docs/cau_truc_kho.md](docs/cau_truc_kho.md) — package mới thuộc tầng IRROS nào
2. [docs/so_do_dieu_khien.md](docs/so_do_dieu_khien.md) — node/topic hiện có, đừng dựng trùng
3. [src/rx150/apps/README.md](src/rx150/apps/README.md) — nếu bạn định thêm một ứng dụng

### Bốn luật không thương lượng

1. **Tầng trên gọi tầng dưới, không bao giờ ngược lại.**
   `rx150_toolbox/*` không được import từ `apps/*`. `controllers/*` không phụ thuộc
   `apps/*` và không phụ thuộc lẫn nhau, ngoài hạ tầng chung ở `rx150_motion_common`.

2. **Dùng lại, đừng viết lại.**
   Không tự cài đặt MoveGroup client, gripper, planning scene hay IK — tất cả đã có trong
   [`rx150_modules`](src/rx150/rx150_toolbox/rx150_modules/README.md). Thiếu primitive thì
   **thêm vào `rx150_modules`**, đừng thêm vào app.

3. **Mỗi package phải có `README.md` riêng.** Đây là yêu cầu của upstream Interbotix, không
   phải sở thích của kho này.

4. **Tham số vào `config/*.yaml`, không hardcode trong code.**

### Thêm một package mới

Dùng `ament_cmake` (không dùng `ament_python` — xem `rx150_perception/CMakeLists.txt` để
biết lý do), đặt đúng thư mục nhóm, và thêm một dòng `<exec_depend>` vào
[`src/rx150/rx150/package.xml`](src/rx150/rx150/package.xml) để metapackage kéo được.

### Kiểm thử trước khi mở PR

```bash
./tools/build.sh          # phải xanh
./rx150.sh reach          # kiểm config, KHÔNG cần robot
./rx150.sh test           # smoke-test theo tầng
```

Nếu thay đổi chạm vào phần cứng, ghi rõ trong PR **bậc nào trong thang B0→B6 bạn đã chạy
qua** ([RUNBOOK](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md)). Không có phần cứng
cũng đóng góp được — nói rõ là chưa test trên máy thật.

### Số đo

Mọi con số đưa vào tài liệu phải kèm **ngày đo** và **đường dẫn dữ liệu thô** trong
`tuning_runs/`. Không commit CSV thô (xem `.gitignore`) — commit `meta.json`, `*.png`,
`summary.txt`.

### Commit

Theo [Conventional Commits](https://www.conventionalcommits.org/):

```
feat(hac): thêm feedforward ma sát Coulomb
fix(perception): sửa dấu yaw khi nắp ống nằm bên trái mask
docs(tuning): số đo hysteresis ngày 2026-09-09
refactor(modules): tách SceneManager khỏi PickPlaceSkill
chore(vendor): pin lại interbotix_ros_toolboxes
```

Scope dùng tên package rút gọn (`hac`, `fuzzy`, `ff`, `modules`, `perception`, `hri`,
`pick_place`, `toolbox`, `vendor`, `docs`, `tuning`). Nội dung tiếng Việt hay tiếng Anh
đều được, nhưng **nhất quán trong một PR**.

### Quy trình PR

1. Nhánh từ `main`: `feat/...`, `fix/...`, `docs/...`
2. Commit nhỏ, mỗi commit một việc
3. Mở PR theo [mẫu](.github/PULL_REQUEST_TEMPLATE.md), điền đủ phần "đã kiểm thử gì"
4. Không commit `build/`, `install/`, `log/`, `src/vendor/`, hay CSV thô

---

## English

### Before writing any code

Read these three, in order:

1. [docs/cau_truc_kho.md](docs/cau_truc_kho.md) — which IRROS layer your package belongs to
2. [docs/so_do_dieu_khien.md](docs/so_do_dieu_khien.md) — existing nodes and topics, so you don't duplicate one
3. [src/rx150/apps/README.md](src/rx150/apps/README.md) — if you are adding an application

> Deep documentation is in Vietnamese. If that is a barrier, open an issue in English —
> maintainers answer in English.

### Four non-negotiable rules

1. **Upper layers call lower layers, never the reverse.**
   `rx150_toolbox/*` must not import from `apps/*`. `controllers/*` do not depend on
   `apps/*` and do not depend on each other, apart from shared infrastructure in
   `rx150_motion_common`.

2. **Reuse, do not reimplement.**
   Never write your own MoveGroup client, gripper handling, planning scene, or IK —
   [`rx150_modules`](src/rx150/rx150_toolbox/rx150_modules/README.md) has all of them. If a
   primitive is missing, **add it to `rx150_modules`**, not to your app.

3. **Every package carries its own `README.md`.** This is an upstream Interbotix
   requirement, not a local preference.

4. **Parameters go in `config/*.yaml`, never hardcoded.**

### Adding a package

Use `ament_cmake` (not `ament_python` — see `rx150_perception/CMakeLists.txt` for why),
put it in the correct group directory, and add an `<exec_depend>` line to
[`src/rx150/rx150/package.xml`](src/rx150/rx150/package.xml) so the metapackage pulls it in.

### Test before opening a PR

```bash
./tools/build.sh          # must be green
./rx150.sh reach          # config check, NO robot needed
./rx150.sh test           # per-layer smoke tests
```

If your change touches hardware behaviour, state in the PR **which rungs of the B0→B6
ladder you actually ran** ([RUNBOOK](src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md)). You
can contribute without hardware — just say clearly that it is untested on a real robot.

### Measurements

Every number that enters the documentation must cite its **measurement date** and the
**raw-data path** under `tuning_runs/`. Raw CSVs are not committed (see `.gitignore`) —
commit `meta.json`, `*.png`, `summary.txt` instead.

### Commits

Follow [Conventional Commits](https://www.conventionalcommits.org/), with the package short
name as scope (`hac`, `fuzzy`, `ff`, `modules`, `perception`, `hri`, `pick_place`,
`toolbox`, `vendor`, `docs`, `tuning`). Vietnamese or English bodies are both fine, but
**stay consistent within one PR**.

### PR workflow

1. Branch from `main`: `feat/...`, `fix/...`, `docs/...`
2. Small commits, one concern each
3. Open the PR using the [template](.github/PULL_REQUEST_TEMPLATE.md), filling in what you tested
4. Never commit `build/`, `install/`, `log/`, `src/vendor/`, or raw CSVs
