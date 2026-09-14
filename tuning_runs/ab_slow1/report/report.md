# So sánh HAC vs Fuzzy — `ab_slow1`

- Task: **slow_cycle**
- Số lượt: HAC 3 [1, 2, 3] · Fuzzy 3 [1, 2, 3]
- Băng xác lập: max(2 % |Δq|, 0.5°) · cửa sổ xác lập 0.5 s · khớp coi là chuyển động khi |Δq| ≥ 2.0°

## Kiểm công bằng

✓ Mọi tham số chung giống nhau: `loop_rate`, `debug_publish_rate`, `velocity_filter_tau`, `enable_profile`, `max_velocities`, `max_accelerations`, `max_jerk`, `sync_mode`, `u_max`, `enable_gravity_comp`, `Gff`, `gravity_sign`, `gravity_model_source`.

## Kết quả theo lượt

Δ% = (HAC − Fuzzy) / |Fuzzy|. Âm = HAC nhỏ hơn. Mọi chỉ số ở đây: **nhỏ hơn là tốt hơn**.

| chỉ số | HAC (mean ± std) | Fuzzy (mean ± std) | Δ% | Cohen d | p Welch | p MWU | tốt hơn |
|---|---|---|---|---|---|---|---|
| RMSE bám [°] | 1.43 ± 0.014 | 0.967 ± 0.11 | 47.5 | 5.9 | 0.017 * | 0.1 | Fuzzy |
| Sai số bám lớn nhất [°] | 4.44 ± 0.049 | 4.58 ± 2.1 | -3.16 | -0.096 | 0.92 | 0.7 | HAC |
| Sai số xác lập TB [°] | 1.71 ± 0.032 | 1.08 ± 0.097 | 59.2 | 8.8 | 0.0041 * | 0.1 | Fuzzy |
| Sai số xác lập xấu nhất [°] | 4.35 ± 0.1 | 3.09 ± 1.4 | 40.8 | 1.3 | 0.25 | 0.7 | Fuzzy |
| Thời gian xác lập TB [s] | 10.2 ± 0.27 | 7.45 ± 0.51 | 37.5 | 6.8 | 0.0033 * | 0.1 | Fuzzy |
| Số lần không vào băng | 15 ± 0 | 9 ± 1 | 66.7 | 8.5 | 0.0091 * | 0.064 | Fuzzy |
| Vọt lố TB [%] | 0 ± 0 | 0.0461 ± 0.046 | -100 | -1.4 | 0.23 | 0.2 | HAC |
| RMS PWM tổng [PWM] | 138 ± 0.96 | 119 ± 2.8 | 16 | 9.1 | 0.0036 * | 0.1 | Fuzzy |
| RMS PWM phản hồi [PWM] | 67.6 ± 0.95 | 73.2 ± 10 | -7.68 | -0.77 | 0.44 | 0.7 | HAC |
| Biến thiên PWM [PWM/s] | 50.7 ± 6.3 | 77.4 ± 9.2 | -34.5 | -3.4 | 0.019 * | 0.1 | HAC |
| Tỉ lệ bão hoà [%] | 0 ± 0 | 0 ± 0 | — | — | — | — | = |
| Khối luật 5 khớp TB [µs] | 1.26 ± 0.025 | 178 ± 0.83 | -99.3 | -3e+02 | 7.2e-06 * | 0.1 | HAC |
| Khối luật p99 [µs] | 7.39 ± 0.31 | 310 ± 8.4 | -97.6 | -51 | 0.00025 * | 0.1 | HAC |
| Chu kỳ tính TB [µs] | 26.5 ± 0.48 | 203 ± 1.1 | -86.9 | -2e+02 | 5.9e-07 * | 0.1 | HAC |
| Chu kỳ tính p99 [µs] | 80.9 ± 4.2 | 363 ± 11 | -77.7 | -34 | 0.00011 * | 0.1 | HAC |
| CPU tiến trình [% 1 lõi] | 6.07 ± 0.068 | 13 ± 0.06 | -53.2 | -1.1e+02 | 2.5e-08 * | 0.1 | HAC |

`*` p < 0.05 (Welch). Với n < 5 mỗi bộ, p-value chỉ mang tính tham khảo — Mann-Whitney với 3 vs 3 lượt không thể nhỏ hơn 0.1.

## Chi phí tính toán

| đại lượng | HAC | Fuzzy | Fuzzy / HAC |
|---|---|---|---|
| Khối luật, 5 khớp [µs] | 1.26 | 178 | **142×** |
| Khối luật, 1 khớp [µs] | 0.252 | 35.6 | **142×** |
| Chu kỳ tính [µs] | 26.5 | 203 | **7.65×** |
| Chu kỳ tính / ngân sách chu kỳ [%] | 1.06 | 8.11 | **7.65×** |
| Khối luật / ngân sách chu kỳ [%] | 0.0503 | 7.13 | **142×** |
| CPU cả tiến trình [% 1 lõi] | 6.07 | 13 | **2.14×** |

- **Khối luật** = vòng từng khớp trong `onTimer`, đo bằng `steady_clock` trong node. Bên HAC khối này còn gồm bù ma sát (`tanh`) mà Fuzzy không có — tức là đo thiệt cho HAC.
- **Chu kỳ tính** = từ sau watchdog tới khi publish lệnh: Ruckig + Pinocchio (hai bộ như nhau) + khối luật. Phần chung càng lớn thì tỉ lệ ở dòng này càng gần 1 — đó là thật, không phải lỗi đo.
- **CPU cả tiến trình** (`/proc/<pid>/stat`) gồm cả DDS và publish debug; HAC phát 7 topic debug, Fuzzy 5.
- Build hiện tại không bật tối ưu (`-O0`). Chi phí riêng của luật ở `-O0` và `-O2`, không nhiễu ROS: `./rx150.sh ab-bench`.

## Theo khớp (trung bình giữa các lượt)

| khớp | RMSE bám HAC / Fuzzy [°] | xác lập HAC / Fuzzy [°] | settle HAC / Fuzzy [s] | RMS PWM phản hồi HAC / Fuzzy |
|---|---|---|---|---|
| waist | 0.635 / 0.699 | 0.529 / 0.563 | 7.99 / 8.18 | 29.7 / 27.6 |
| shoulder | 2.24 / 1.24 | 2.55 / 1.43 | 12.3 / 7.17 | 105 / 102 |
| elbow | 1.57 / 0.747 | 1.72 / 0.878 | 12.7 / 7.34 | 73.1 / 68 |
| wrist_angle | 1.02 / 0.823 | 1.34 / 1.24 | — / 6.66 | 47.6 / 46.4 |
| wrist_rotate | 0.000853 / 0.0185 | — / — | — / — | 0.0397 / 1.27 |

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
