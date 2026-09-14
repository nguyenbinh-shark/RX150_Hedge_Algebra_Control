# So sánh HAC vs Fuzzy — `ab_hactune_B`

- Task: **slow_cycle**
- Số lượt: HAC 1 [1] · Fuzzy 3 [1, 2, 3]
- Băng xác lập: max(2 % |Δq|, 0.5°) · cửa sổ xác lập 0.5 s · khớp coi là chuyển động khi |Δq| ≥ 2.0°

## Kiểm công bằng

✓ Mọi tham số chung giống nhau: `loop_rate`, `debug_publish_rate`, `velocity_filter_tau`, `enable_profile`, `max_velocities`, `max_accelerations`, `max_jerk`, `sync_mode`, `u_max`, `enable_gravity_comp`, `Gff`, `gravity_sign`, `gravity_model_source`.

## Kết quả theo lượt

Δ% = (HAC − Fuzzy) / |Fuzzy|. Âm = HAC nhỏ hơn. Mọi chỉ số ở đây: **nhỏ hơn là tốt hơn**.

| chỉ số | HAC (mean ± std) | Fuzzy (mean ± std) | Δ% | Cohen d | p Welch | p MWU | tốt hơn |
|---|---|---|---|---|---|---|---|
| RMSE bám [°] | 0.677 ± — | 0.967 ± 0.11 | -30 | — | — | — | HAC |
| Sai số bám lớn nhất [°] | 2.3 ± — | 4.58 ± 2.1 | -49.8 | — | — | — | HAC |
| Sai số xác lập TB [°] | 0.757 ± — | 1.08 ± 0.097 | -29.7 | — | — | — | HAC |
| Sai số xác lập xấu nhất [°] | 2.21 ± — | 3.09 ± 1.4 | -28.4 | — | — | — | HAC |
| Thời gian xác lập TB [s] | 7.92 ± — | 7.45 ± 0.51 | 6.32 | — | — | — | Fuzzy |
| Số lần không vào băng | 4 ± — | 9 ± 1 | -55.6 | — | — | — | HAC |
| Vọt lố TB [%] | 0 ± — | 0.0461 ± 0.046 | -100 | — | — | — | HAC |
| RMS PWM tổng [PWM] | 115 ± — | 119 ± 2.8 | -2.76 | — | — | — | HAC |
| RMS PWM phản hồi [PWM] | 66 ± — | 73.2 ± 10 | -9.85 | — | — | — | HAC |
| Biến thiên PWM [PWM/s] | 96.1 ± — | 77.4 ± 9.2 | 24.2 | — | — | — | Fuzzy |
| Tỉ lệ bão hoà [%] | 0 ± — | 0 ± 0 | — | — | — | — | = |
| Khối luật 5 khớp TB [µs] | 1.28 ± — | 178 ± 0.83 | -99.3 | — | — | — | HAC |
| Khối luật p99 [µs] | 7.7 ± — | 310 ± 8.4 | -97.5 | — | — | — | HAC |
| Chu kỳ tính TB [µs] | 26.7 ± — | 203 ± 1.1 | -86.8 | — | — | — | HAC |
| Chu kỳ tính p99 [µs] | 79.2 ± — | 363 ± 11 | -78.2 | — | — | — | HAC |
| CPU tiến trình [% 1 lõi] | 6.05 ± — | 13 ± 0.06 | -53.4 | — | — | — | HAC |

`*` p < 0.05 (Welch). Với n < 5 mỗi bộ, p-value chỉ mang tính tham khảo — Mann-Whitney với 3 vs 3 lượt không thể nhỏ hơn 0.1.

## Chi phí tính toán

| đại lượng | HAC | Fuzzy | Fuzzy / HAC |
|---|---|---|---|
| Khối luật, 5 khớp [µs] | 1.28 | 178 | **140×** |
| Khối luật, 1 khớp [µs] | 0.255 | 35.6 | **140×** |
| Chu kỳ tính [µs] | 26.7 | 203 | **7.58×** |
| Chu kỳ tính / ngân sách chu kỳ [%] | 1.07 | 8.11 | **7.58×** |
| Khối luật / ngân sách chu kỳ [%] | 0.051 | 7.13 | **140×** |
| CPU cả tiến trình [% 1 lõi] | 6.05 | 13 | **2.15×** |

- **Khối luật** = vòng từng khớp trong `onTimer`, đo bằng `steady_clock` trong node. Bên HAC khối này còn gồm bù ma sát (`tanh`) mà Fuzzy không có — tức là đo thiệt cho HAC.
- **Chu kỳ tính** = từ sau watchdog tới khi publish lệnh: Ruckig + Pinocchio (hai bộ như nhau) + khối luật. Phần chung càng lớn thì tỉ lệ ở dòng này càng gần 1 — đó là thật, không phải lỗi đo.
- **CPU cả tiến trình** (`/proc/<pid>/stat`) gồm cả DDS và publish debug; HAC phát 7 topic debug, Fuzzy 5.
- Build hiện tại không bật tối ưu (`-O0`). Chi phí riêng của luật ở `-O0` và `-O2`, không nhiễu ROS: `./rx150.sh ab-bench`.

## Theo khớp (trung bình giữa các lượt)

| khớp | RMSE bám HAC / Fuzzy [°] | xác lập HAC / Fuzzy [°] | settle HAC / Fuzzy [s] | RMS PWM phản hồi HAC / Fuzzy |
|---|---|---|---|---|
| waist | 0.294 / 0.699 | 0.195 / 0.563 | 7.32 / 8.18 | 27.6 / 27.6 |
| shoulder | 1.04 / 1.24 | 1.21 / 1.43 | 7.46 / 7.17 | 97 / 102 |
| elbow | 0.613 / 0.747 | 0.723 / 0.878 | 7.29 / 7.34 | 57.2 / 68 |
| wrist_angle | 0.436 / 0.823 | 0.544 / 1.24 | 9.46 / 6.66 | 41 / 46.4 |
| wrist_rotate | 0.00118 / 0.0185 | — / — | — / — | 0.11 / 1.27 |

## Hình

![tracking](fig_tracking.png)

![trials](fig_trials.png)

![joints](fig_joints.png)

## Đọc số thế nào

- `e` là sai số so với quỹ đạo Ruckig (ref), không phải so với đích. RMSE bám thấp mà sai số xác lập cao nghĩa là bám tốt khi chạy nhưng thiếu lực ở cuối (ma sát / bù trọng lực).
- `RMS PWM phản hồi` = u − u_grav: phần do luật điều khiển tạo ra. So effort nên dùng cột này; PWM tổng bị bù trọng lực (giống nhau ở hai bộ) chi phối.
- `Biến thiên PWM` cao kèm dao động nhỏ quanh đích = rung/limit-cycle.
- Dữ liệu ở debug_publish_rate (thường 50 Hz): overshoot và max |e| có thể thấp hơn thực tế một chút vì đỉnh ngắn hơn 20 ms bị bỏ lỡ, như nhau ở cả hai bộ.
- File thô: `segments.csv` (từng segment × khớp), `trials.csv`, `summary.csv`, `per_joint.csv`.
