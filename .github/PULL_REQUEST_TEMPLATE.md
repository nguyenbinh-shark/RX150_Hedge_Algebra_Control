## Thay đổi gì · What changed

<!-- Một đoạn ngắn. Vì sao cần thay đổi này, không chỉ là nó làm gì. -->
<!-- A short paragraph. Why this change is needed, not just what it does. -->

## Tầng IRROS bị ảnh hưởng · IRROS layers touched

- [ ] Application — `rx150_pick_place`, `rx150_hri`
- [ ] Application Support — `rx150_modules`, `rx150_motion_common`, `rx150_perception`
- [ ] Control — `rx150_fuzzy_controller`, `rx150_ff_controller`, `rx150_hac_controller`
- [ ] Công cụ / tài liệu · tooling / docs
- [ ] Vendor pin (`rx150.repos`)

## Đã kiểm thử gì · What was tested

<!-- Bắt buộc. "Chưa test trên phần cứng" là câu trả lời hợp lệ — nói rõ ra. -->
<!-- Required. "Not tested on hardware" is a valid answer — just say so. -->

- [ ] `./tools/build.sh` xanh · green
- [ ] `./rx150.sh reach` — kiểm config, không cần robot · config check, no robot
- [ ] `./rx150.sh test` — smoke-test theo tầng · per-layer smoke tests
- [ ] `./rx150.sh dry-fake` — state machine với ống giả · state machine on fake tubes
- [ ] Chạy trên phần cứng thật · ran on real hardware — bậc B__ → B__ trong [RUNBOOK](../src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md)

## Số đo · Measurements

<!-- Nếu PR đổi hành vi điều khiển: số trước/sau + đường dẫn tuning_runs/. Không thì xoá mục này. -->
<!-- If this changes control behaviour: before/after numbers + the tuning_runs/ path. Otherwise delete this section. -->

## Checklist

- [ ] Package mới đặt đúng tầng IRROS, và có `README.md` riêng
      *· new packages sit at the right layer and carry their own README*
- [ ] Không import ngược tầng (`toolbox/` không gọi `apps/`)
      *· no upward imports*
- [ ] Không viết lại primitive đã có trong `rx150_modules`
      *· no reimplemented primitives*
- [ ] Tham số nằm trong `config/*.yaml`, không hardcode
      *· parameters live in `config/*.yaml`*
- [ ] Không commit `build/`, `install/`, `log/`, `src/vendor/`, CSV thô
      *· none of the ignored artefacts are committed*
- [ ] Tài liệu liên quan đã cập nhật · related docs updated
