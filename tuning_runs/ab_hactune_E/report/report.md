# So sánh HAC vs Fuzzy — `ab_hactune_E`

- Task: **slow_cycle**
- Số lượt: HAC 1 [1] · Fuzzy 3 [1, 2, 3]
- Băng xác lập: max(2 % |Δq|, 0.5°) · cửa sổ xác lập 0.5 s · khớp coi là chuyển động khi |Δq| ≥ 2.0°

## Kiểm công bằng

✓ Mọi tham số chung giống nhau: `loop_rate`, `debug_publish_rate`, `velocity_filter_tau`, `enable_profile`, `max_velocities`, `max_accelerations`, `max_jerk`, `sync_mode`, `u_max`, `enable_gravity_comp`, `Gff`, `gravity_sign`, `gravity_model_source`.

## Kết quả theo lượt

Δ% = (HAC − Fuzzy) / |Fuzzy|. Âm = HAC nhỏ hơn. Mọi chỉ số ở đây: **nhỏ hơn là tốt hơn**.

| chỉ số | HAC (mean ± std) | Fuzzy (mean ± std) | Δ% | Cohen d | p Welch | p MWU | tốt hơn |
|---|---|---|---|---|---|---|---|
| RMSE bám [°] | 0.625 ± — | 0.967 ± 0.11 | -35.3 | — | — | — | HAC |
| Sai số bám lớn nhất [°] | 2.12 ± — | 4.58 ± 2.1 | -53.7 | — | — | — | HAC |
| Sai số xác lập TB [°] | 0.715 ± — | 1.08 ± 0.097 | -33.5 | — | — | — | HAC |
| Sai số xác lập xấu nhất [°] | 2.04 ± — | 3.09 ± 1.4 | -34.1 | — | — | — | HAC |
| Thời gian xác lập TB [s] | 7.94 ± — | 7.45 ± 0.51 | 6.63 | — | — | — | Fuzzy |
| Số lần không vào băng | 4 ± — | 9 ± 1 | -55.6 | — | — | — | HAC |
| Vọt lố TB [%] | 0 ± — | 0.0461 ± 0.046 | -100 | — | — | — | HAC |
| RMS PWM tổng [PWM] | 119 ± — | 119 ± 2.8 | 0.00984 | — | — | — | Fuzzy |
| RMS PWM phản hồi [PWM] | 70.6 ± — | 73.2 ± 10 | -3.56 | — | — | — | HAC |
| Biến thiên PWM [PWM/s] | 117 ± — | 77.4 ± 9.2 | 51.6 | — | — | — | Fuzzy |
| Tỉ lệ bão hoà [%] | 0 ± — | 0 ± 0 | — | — | — | — | = |
| Khối luật 5 khớp TB [µs] | 1.29 ± — | 178 ± 0.83 | -99.3 | — | — | — | HAC |
| Khối luật p99 [µs] | 7.86 ± — | 310 ± 8.4 | -97.5 | — | — | — | HAC |
| Chu kỳ tính TB [µs] | 26.6 ± — | 203 ± 1.1 | -86.9 | — | — | — | HAC |
| Chu kỳ tính p99 [µs] | 78.4 ± — | 363 ± 11 | -78.4 | — | — | — | HAC |
| CPU tiến trình [% 1 lõi] | 6.06 ± — | 13 ± 0.06 | -53.4 | — | — | — | HAC |

`*` p < 0.05 (Welch). Với n < 5 mỗi bộ, p-value chỉ mang tính tham khảo — Mann-Whitney với 3 vs 3 lượt không thể nhỏ hơn 0.1.

## Chi phí tính toán

| đại lượng | HAC | Fuzzy | Fuzzy / HAC |
|---|---|---|---|
| Khối luật, 5 khớp [µs] | 1.29 | 178 | **138×** |
| Khối luật, 1 khớp [µs] | 0.258 | 35.6 | **138×** |
| Chu kỳ tính [µs] | 26.6 | 203 | **7.62×** |
| Chu kỳ tính / ngân sách chu kỳ [%] | 1.06 | 8.11 | **7.62×** |
| Khối luật / ngân sách chu kỳ [%] | 0.0515 | 7.13 | **138×** |
| CPU cả tiến trình [% 1 lõi] | 6.06 | 13 | **2.14×** |

- **Khối luật** = vòng từng khớp trong `onTimer`, đo bằng `steady_clock` trong node. Bên HAC khối này còn gồm bù ma sát (`tanh`) mà Fuzzy không có — tức là đo thiệt cho HAC.
- **Chu kỳ tính** = từ sau watchdog tới khi publish lệnh: Ruckig + Pinocchio (hai bộ như nhau) + khối luật. Phần chung càng lớn thì tỉ lệ ở dòng này càng gần 1 — đó là thật, không phải lỗi đo.
- **CPU cả tiến trình** (`/proc/<pid>/stat`) gồm cả DDS và publish debug; HAC phát 7 topic debug, Fuzzy 5.
- Build hiện tại không bật tối ưu (`-O0`). Chi phí riêng của luật ở `-O0` và `-O2`, không nhiễu ROS: `./rx150.sh ab-bench`.

## Theo khớp (trung bình giữa các lượt)

| khớp | RMSE bám HAC / Fuzzy [°] | xác lập HAC / Fuzzy [°] | settle HAC / Fuzzy [s] | RMS PWM phản hồi HAC / Fuzzy |
|---|---|---|---|---|
| waist | 0.41 / 0.699 | 0.31 / 0.563 | 7.34 / 8.18 | 28.8 / 27.6 |
| shoulder | 0.924 / 1.24 | 1.11 / 1.43 | 7.36 / 7.17 | 108 / 102 |
| elbow | 0.53 / 0.747 | 0.636 / 0.878 | 7.19 / 7.34 | 62 / 68 |
| wrist_angle | 0.461 / 0.823 | 0.544 / 1.24 | 9.73 / 6.66 | 39 / 46.4 |
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
