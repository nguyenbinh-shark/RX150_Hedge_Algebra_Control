# So sánh HAC vs Fuzzy — `ab_hactune_D`

- Task: **slow_cycle**
- Số lượt: HAC 1 [1] · Fuzzy 3 [1, 2, 3]
- Băng xác lập: max(2 % |Δq|, 0.5°) · cửa sổ xác lập 0.5 s · khớp coi là chuyển động khi |Δq| ≥ 2.0°

## Kiểm công bằng

✓ Mọi tham số chung giống nhau: `loop_rate`, `debug_publish_rate`, `velocity_filter_tau`, `enable_profile`, `max_velocities`, `max_accelerations`, `max_jerk`, `sync_mode`, `u_max`, `enable_gravity_comp`, `Gff`, `gravity_sign`, `gravity_model_source`.

## Kết quả theo lượt

Δ% = (HAC − Fuzzy) / |Fuzzy|. Âm = HAC nhỏ hơn. Mọi chỉ số ở đây: **nhỏ hơn là tốt hơn**.

| chỉ số | HAC (mean ± std) | Fuzzy (mean ± std) | Δ% | Cohen d | p Welch | p MWU | tốt hơn |
|---|---|---|---|---|---|---|---|
| RMSE bám [°] | 0.601 ± — | 0.967 ± 0.11 | -37.9 | — | — | — | HAC |
| Sai số bám lớn nhất [°] | 2.12 ± — | 4.58 ± 2.1 | -53.6 | — | — | — | HAC |
| Sai số xác lập TB [°] | 0.654 ± — | 1.08 ± 0.097 | -39.2 | — | — | — | HAC |
| Sai số xác lập xấu nhất [°] | 2.04 ± — | 3.09 ± 1.4 | -33.9 | — | — | — | HAC |
| Thời gian xác lập TB [s] | 7.79 ± — | 7.45 ± 0.51 | 4.51 | — | — | — | Fuzzy |
| Số lần không vào băng | 4 ± — | 9 ± 1 | -55.6 | — | — | — | HAC |
| Vọt lố TB [%] | 0 ± — | 0.0461 ± 0.046 | -100 | — | — | — | HAC |
| RMS PWM tổng [PWM] | 120 ± — | 119 ± 2.8 | 0.784 | — | — | — | Fuzzy |
| RMS PWM phản hồi [PWM] | 73.1 ± — | 73.2 ± 10 | -0.097 | — | — | — | HAC |
| Biến thiên PWM [PWM/s] | 179 ± — | 77.4 ± 9.2 | 131 | — | — | — | Fuzzy |
| Tỉ lệ bão hoà [%] | 0 ± — | 0 ± 0 | — | — | — | — | = |
| Khối luật 5 khớp TB [µs] | 1.28 ± — | 178 ± 0.83 | -99.3 | — | — | — | HAC |
| Khối luật p99 [µs] | 7.26 ± — | 310 ± 8.4 | -97.7 | — | — | — | HAC |
| Chu kỳ tính TB [µs] | 26.9 ± — | 203 ± 1.1 | -86.7 | — | — | — | HAC |
| Chu kỳ tính p99 [µs] | 86.4 ± — | 363 ± 11 | -76.2 | — | — | — | HAC |
| CPU tiến trình [% 1 lõi] | 6.12 ± — | 13 ± 0.06 | -52.9 | — | — | — | HAC |

`*` p < 0.05 (Welch). Với n < 5 mỗi bộ, p-value chỉ mang tính tham khảo — Mann-Whitney với 3 vs 3 lượt không thể nhỏ hơn 0.1.

## Chi phí tính toán

| đại lượng | HAC | Fuzzy | Fuzzy / HAC |
|---|---|---|---|
| Khối luật, 5 khớp [µs] | 1.28 | 178 | **140×** |
| Khối luật, 1 khớp [µs] | 0.255 | 35.6 | **140×** |
| Chu kỳ tính [µs] | 26.9 | 203 | **7.54×** |
| Chu kỳ tính / ngân sách chu kỳ [%] | 1.08 | 8.11 | **7.54×** |
| Khối luật / ngân sách chu kỳ [%] | 0.0511 | 7.13 | **140×** |
| CPU cả tiến trình [% 1 lõi] | 6.12 | 13 | **2.12×** |

- **Khối luật** = vòng từng khớp trong `onTimer`, đo bằng `steady_clock` trong node. Bên HAC khối này còn gồm bù ma sát (`tanh`) mà Fuzzy không có — tức là đo thiệt cho HAC.
- **Chu kỳ tính** = từ sau watchdog tới khi publish lệnh: Ruckig + Pinocchio (hai bộ như nhau) + khối luật. Phần chung càng lớn thì tỉ lệ ở dòng này càng gần 1 — đó là thật, không phải lỗi đo.
- **CPU cả tiến trình** (`/proc/<pid>/stat`) gồm cả DDS và publish debug; HAC phát 7 topic debug, Fuzzy 5.
- Build hiện tại không bật tối ưu (`-O0`). Chi phí riêng của luật ở `-O0` và `-O2`, không nhiễu ROS: `./rx150.sh ab-bench`.

## Theo khớp (trung bình giữa các lượt)

| khớp | RMSE bám HAC / Fuzzy [°] | xác lập HAC / Fuzzy [°] | settle HAC / Fuzzy [s] | RMS PWM phản hồi HAC / Fuzzy |
|---|---|---|---|---|
| waist | 0.312 / 0.699 | 0.107 / 0.563 | 7.28 / 8.18 | 38.1 / 27.6 |
| shoulder | 0.94 / 1.24 | 1.13 / 1.43 | 7.35 / 7.17 | 110 / 102 |
| elbow | 0.528 / 0.747 | 0.629 / 0.878 | 7.11 / 7.34 | 61.6 / 68 |
| wrist_angle | 0.326 / 0.823 | 0.392 / 1.24 | 9.27 / 6.66 | 38.3 / 46.4 |
| wrist_rotate | 0.000833 / 0.0185 | — / — | — / — | 0.0969 / 1.27 |

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
