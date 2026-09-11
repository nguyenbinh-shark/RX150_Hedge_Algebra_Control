# rx150_hac_controller — HAC tuyến tính + Ruckig + bù trọng lực

**Tầng IRROS:** Control · **Ngôn ngữ:** C++ (`hac_node`)

Bộ điều khiển PWM vòng kín dùng mặt điều khiển **HAC tuyến tính** thay cho luật mờ, giữ
nguyên phần còn lại của kiến trúc (profile Ruckig + bù trọng lực Pinocchio) để so sánh
công bằng với [`rx150_fuzzy_controller`](../rx150_fuzzy_controller/README.md).

Mặt điều khiển có ba hệ số vô hướng `a`, `b`, `c` (live-tunable qua ROS parameter), thay
cho toàn bộ bảng luật Mamdani. Ý nghĩa của việc này: nếu HAC bám ngang fuzzy thì phần
"mờ" không đóng góp gì ngoài một mặt phi tuyến — đó chính là câu hỏi mà package này dựng
ra để trả lời.

## Cấu trúc

| Thư mục | Nội dung |
| :--- | :--- |
| `src/hac_node.cpp` | Node chính, 100 Hz: `joint_states` → HAC + g(q) → `commands/joint_group` |
| `src/hac/hac.c` | Mặt điều khiển HAC thuần C |
| `src/gravity_comp.cpp` | Bù trọng lực bằng Pinocchio RNEA từ URDF |
| `config/rx150_hac_gains.yaml` | `a/b/c`, `error_limit`, `error_dot_limit`, `u_max`, hệ số N·m→PWM |
| `config/rx150_hac_gains_safe.yaml` | Bộ gain rón rén, dùng khi thử trên máy thật lần đầu |
| `config/rx150_gravity_model.yaml` | Model trọng lực ĐÃ hiệu chuẩn trên phần cứng (sinh tự động, đừng sửa tay) |
| `launch/hac_control.launch.py` | Chỉ controller (không MoveIt) |
| `launch/hac_moveit.launch.py` | Bring-up đầy đủ kèm MoveIt + camera |

`error_limit` và `error_dot_limit` được đặt bằng `1/Ke` và `1/Ked` của bộ fuzzy — đây là
điều kiện để hai bộ so sánh được với nhau, đổi một bên thì phải đổi bên kia.

## Chạy

```bash
# Lần đầu trên máy thật: dùng bộ gain an toàn
ros2 launch rx150_hac_controller hac_control.launch.py \
    gains_file:=$(ros2 pkg prefix rx150_hac_controller)/share/rx150_hac_controller/config/rx150_hac_gains_safe.yaml

# Đầy đủ với MoveIt, dùng model trọng lực đã hiệu chuẩn
ros2 launch rx150_hac_controller hac_moveit.launch.py \
    gravity_model_file:=rx150_gravity_model.yaml
# ...hoặc gọn hơn, đã bao sẵn arg trên:
./rx150.sh t1-hac
```

Bỏ `gravity_model_file` để quay về đường Pinocchio + `Gff` — cách so A/B hai mô hình.

> ⚠️ Bật torque và đưa tay về home ngay khi khởi động.

## Hiệu chuẩn bù trọng lực

`enable_gravity_comp` mặc định dùng Pinocchio RNEA rồi nhân `Gff` (N·m→PWM ước lượng từ
datasheet). Ước lượng đó lệch hệ thống trên máy thật. Quy trình bốn bước dưới đây thay nó
bằng model lượng giác fit thẳng ở đơn vị PWM, hấp thụ luôn sai số `Gff` lẫn sai số khối
lượng trong URDF.

```bash
# B0 — kiểm dấu. BẮT BUỘC gravity comp TẮT, nếu không script tự hủy.
ros2 launch rx150_hac_controller hac_control.launch.py     gains_file:=rx150_hac_gains_safe.yaml enable_gravity_comp:=false
ros2 run rx150_motion_common rx150_gravity_id.py signcheck

# B1 — quét lưới (~11 phút, 132 lượt) rồi ridge-fit. Bật lại gravity comp.
ros2 launch rx150_hac_controller hac_control.launch.py gains_file:=rx150_hac_gains_safe.yaml
ros2 run rx150_motion_common rx150_gravity_id.py plan --out grid.json
ros2 run rx150_motion_common rx150_gravity_id.py identify --grid grid.json

# B2 — nạp model vừa sinh (file ghi thẳng vào config/, cần colcon build vì là file MỚI)
colcon build --packages-select rx150_hac_controller --symlink-install
ros2 launch rx150_hac_controller hac_control.launch.py     gains_file:=rx150_hac_gains_safe.yaml gravity_model_file:=rx150_gravity_model.yaml

# B3 — đối chiếu, không cần động cơ
ros2 run rx150_motion_common rx150_gravity_id.py validate     --model src/rx150/controllers/rx150_hac_controller/config/rx150_gravity_model.yaml     --data ~/interbotix_ws/tuning_runs/gravity_identify_*/identify_data.csv
```

`enable_gravity_comp`, `gravity_sign`, `gravity_model_source` **chỉ đọc lúc khởi tạo** —
`onParamChange` của `hac_node` chỉ nhận `a/b/c`, `friction_eps`, `error_limit`,
`error_dot_limit`, `u_max`, `Gff`, `friction_*`, `fitted_gravity_coeffs`,
`max_velocities`, `max_accelerations`. Đổi ba cái đầu phải relaunch, `ros2 param set`
sẽ báo thành công mà không có tác dụng.

### Đo lại hai con số dưới đây

```bash
# sai số xác lập trên 6 tư thế cố định (dùng chung trước/sau hiệu chuẩn)
ros2 run rx150_motion_common rx150_steady_state_bench.py <nhãn> [out.json]
# tiếp cận cùng tư thế từ hai phía -> sai số có đảo dấu không
ros2 run rx150_motion_common rx150_stiction_hysteresis.py
```

Dữ liệu thô của lần chạy 2026-09-09 nằm ở `tuning_runs/gravity_identify_20260909_182815/`
(`identify_data.csv` 132 mẫu, `model_fit.json`, `ss_bench_before_pinocchio.json`,
`ss_bench_after_fitted.json`).

### Kết quả đo được (phần cứng, 2026-09-09)

Residual RMS khi dự đoán PWM giữ tư thế, 132 mẫu:

| khớp | PWM thô | Pinocchio + `Gff` | model fitted |
| :--- | ---: | ---: | ---: |
| shoulder | 78.0 | 53.8 (lệch −13.7) | **35.4** (lệch −0.1) |
| elbow | 122.3 | 80.0 (lệch −53.0) | **60.2** (lệch −1.5) |
| wrist_angle | 94.0 | 44.4 (lệch +2.7) | **44.3** (lệch −0.5) |

Sai số xác lập trên 6 tư thế: xấu nhất **2.55° → 1.95°**, shoulder 1.92° → 1.13°.

### Sàn sai số: ma sát tĩnh, không phải trọng lực

Bộ fit tách riêng hệ số theo hướng tiếp cận (`stiction_bias_pwm`): shoulder −29.4,
elbow −58.8, wrist_angle −44.3 PWM. Với `Kp = 2c/3a = 2667 PWM/rad` chúng tương đương
±0.63° / ±1.26° / ±0.95° — tức **bằng đúng cỡ toàn bộ sai số còn lại**.

Kiểm chứng bằng cách tiếp cận cùng một tư thế từ hai phía: sai số **đảo dấu**.

| khớp | từ + | từ − | bề rộng dải |
| :--- | ---: | ---: | ---: |
| shoulder | −0.30° | +0.32° | 0.88° |
| elbow | −1.65° | +1.43° | 3.07° |
| wrist_angle | −1.25° | +1.12° | 2.38° |

Cùng model, cùng tư thế, chỉ khác hướng đi tới. Nên **không lượng hiệu chuẩn trọng lực
nào hạ được phần này** — đòn bẩy tiếp theo là `friction_coulomb`/`friction_viscous`
(`rx150_friction_id.py`), hiện vẫn là `[0,0,0,0,0]`.

## Phân tích

```bash
ros2 run rx150_motion_common plot_hac_velocity_analysis.py   # mặt 3D 4 góc phần tư, đường cắt 2D
ros2 run rx150_motion_common compare_fuzzy_vs_hac.py         # overlay fuzzy vs HAC
```

Hai script trên **không** cần robot. Đặt `RX150_PLOT_DIR` để chọn nơi ghi ảnh, nếu không
ảnh rơi ra thư mục hiện hành — xem [docs/tuning/README.md](../../../../docs/tuning/README.md).

Về ma sát: mô hình trọng lực đã hiệu chuẩn (`gravity_model_source: fitted`) nhưng phần dư
còn lại **đảo dấu theo hướng tiếp cận** ⇒ đó là ma sát tĩnh, không phải trọng lực. Số đo
và giới hạn ghi ngay trong header của
[config/rx150_gravity_model.yaml](config/rx150_gravity_model.yaml); công cụ hạ nó là
`rx150_friction_id.py`, hệ số hiện vẫn bằng 0.
