# So sánh HAC vs Fuzzy — `ab_slow3`

- Task: **slow_cycle**
- Số lượt: HAC 3 [1, 2, 3] · Fuzzy 3 [1, 2, 3]
- Băng xác lập: max(2 % |Δq|, 0.5°) · cửa sổ xác lập 0.5 s · khớp coi là chuyển động khi |Δq| ≥ 2.0°

## Kiểm công bằng

✓ Mọi tham số chung giống nhau: `loop_rate`, `debug_publish_rate`, `velocity_filter_tau`, `enable_profile`, `max_velocities`, `max_accelerations`, `max_jerk`, `sync_mode`, `u_max`, `enable_gravity_comp`, `Gff`, `gravity_sign`, `gravity_model_source`.

## Kết quả theo lượt

Δ% = (HAC − Fuzzy) / |Fuzzy|. Âm = HAC nhỏ hơn. Mọi chỉ số ở đây: **nhỏ hơn là tốt hơn**.

| chỉ số | HAC (mean ± std) | Fuzzy (mean ± std) | Δ% | Cohen d | p Welch | p MWU | tốt hơn |
|---|---|---|---|---|---|---|---|
| RMSE bám [°] | 0.629 ± 0.011 | 0.967 ± 0.11 | -35 | -4.3 | 0.032 * | 0.1 | HAC |
| Sai số bám lớn nhất [°] | 2.12 ± 0 | 4.58 ± 2.1 | -53.6 | -1.6 | 0.18 | 0.064 | HAC |
| Sai số xác lập TB [°] | 0.733 ± 0.031 | 1.08 ± 0.097 | -31.9 | -4.8 | 0.018 * | 0.1 | HAC |
| Sai số xác lập xấu nhất [°] | 2.04 ± 0 | 3.09 ± 1.4 | -34.1 | -1.1 | 0.31 | 0.064 | HAC |
| Thời gian xác lập TB [s] | 7.83 ± 0.24 | 7.45 ± 0.51 | 5.11 | 0.95 | 0.33 | 0.2 | Fuzzy |
| Số lần không vào băng | 4.33 ± 0.58 | 9 ± 1 | -51.9 | -5.7 | 0.0048 * | 0.077 | HAC |
| Vọt lố TB [%] | 0.0154 ± 0.027 | 0.0461 ± 0.046 | -66.7 | -0.82 | 0.39 | 0.48 | HAC |
| RMS PWM tổng [PWM] | 118 ± 2.2 | 119 ± 2.8 | -0.546 | -0.26 | 0.77 | 1 | HAC |
| RMS PWM phản hồi [PWM] | 70.7 ± 0.83 | 73.2 ± 10 | -3.36 | -0.34 | 0.72 | 0.7 | HAC |
| Biến thiên PWM [PWM/s] | 102 ± 7.5 | 77.4 ± 9.2 | 31.5 | 2.9 | 0.025 * | 0.1 | Fuzzy |
| Tỉ lệ bão hoà [%] | 0 ± 0 | 0 ± 0 | — | — | — | — | = |
| Khối luật 5 khớp TB [µs] | 1.27 ± 0.013 | 178 ± 0.83 | -99.3 | -3e+02 | 7.3e-06 * | 0.1 | HAC |
| Khối luật p99 [µs] | 7.3 ± 0.5 | 310 ± 8.4 | -97.6 | -51 | 0.00024 * | 0.1 | HAC |
| Chu kỳ tính TB [µs] | 26.8 ± 0.3 | 203 ± 1.1 | -86.8 | -2.1e+02 | 3.7e-06 * | 0.1 | HAC |
| Chu kỳ tính p99 [µs] | 77.3 ± 2.1 | 363 ± 11 | -78.7 | -36 | 0.00033 * | 0.1 | HAC |
| CPU tiến trình [% 1 lõi] | 6.09 ± 0.037 | 13 ± 0.06 | -53.1 | -1.4e+02 | 1.2e-07 * | 0.1 | HAC |

`*` p < 0.05 (Welch). Với n < 5 mỗi bộ, p-value chỉ mang tính tham khảo — Mann-Whitney với 3 vs 3 lượt không thể nhỏ hơn 0.1.

## Chi phí tính toán

| đại lượng | HAC | Fuzzy | Fuzzy / HAC |
|---|---|---|---|
| Khối luật, 5 khớp [µs] | 1.27 | 178 | **140×** |
| Khối luật, 1 khớp [µs] | 0.254 | 35.6 | **140×** |
| Chu kỳ tính [µs] | 26.8 | 203 | **7.57×** |
| Chu kỳ tính / ngân sách chu kỳ [%] | 1.07 | 8.11 | **7.57×** |
| Khối luật / ngân sách chu kỳ [%] | 0.0509 | 7.13 | **140×** |
| CPU cả tiến trình [% 1 lõi] | 6.09 | 13 | **2.13×** |

- **Khối luật** = vòng từng khớp trong `onTimer`, đo bằng `steady_clock` trong node. Bên HAC khối này còn gồm bù ma sát (`tanh`) mà Fuzzy không có — tức là đo thiệt cho HAC.
- **Chu kỳ tính** = từ sau watchdog tới khi publish lệnh: Ruckig + Pinocchio (hai bộ như nhau) + khối luật. Phần chung càng lớn thì tỉ lệ ở dòng này càng gần 1 — đó là thật, không phải lỗi đo.
- **CPU cả tiến trình** (`/proc/<pid>/stat`) gồm cả DDS và publish debug; HAC phát 7 topic debug, Fuzzy 5.
- Build hiện tại không bật tối ưu (`-O0`). Chi phí riêng của luật ở `-O0` và `-O2`, không nhiễu ROS: `./rx150.sh ab-bench`.

## Theo khớp (trung bình giữa các lượt)

| khớp | RMSE bám HAC / Fuzzy [°] | xác lập HAC / Fuzzy [°] | settle HAC / Fuzzy [s] | RMS PWM phản hồi HAC / Fuzzy |
|---|---|---|---|---|
| waist | 0.366 / 0.699 | 0.313 / 0.563 | 7.39 / 8.18 | 25.7 / 27.6 |
| shoulder | 0.932 / 1.24 | 1.12 / 1.43 | 7.38 / 7.17 | 109 / 102 |
| elbow | 0.528 / 0.747 | 0.629 / 0.878 | 7.15 / 7.34 | 61.7 / 68 |
| wrist_angle | 0.51 / 0.823 | 0.624 / 1.24 | 9.37 / 6.66 | 43.1 / 46.4 |
| wrist_rotate | 0 / 0.0185 | — / — | — / — | 0 / 1.27 |

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
