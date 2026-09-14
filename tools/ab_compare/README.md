# So sánh HAC vs Fuzzy trên cùng một task

Pipeline 3 bước: **chạy HAC → chạy Fuzzy → xử lý số liệu**. Hai bộ nhận đúng một
chuỗi waypoint, cùng giới hạn Ruckig, cùng đường bù trọng lực, và báo cáo tự kiểm
lại điều đó từ tham số đang chạy trên node.

```
tools/ab_compare/
├── tasks/
│   ├── pick_cycle.yaml    chu kỳ gắp-đặt, mọi khớp cùng chạy (~26 s/lượt)
│   └── joint_steps.yaml   bậc thang từng khớp ±, đo rise/overshoot/settling sạch (~28 s/lượt)
├── ab_run.py              ra lệnh + ghi (cần ROS, cần robot)
└── ab_report.py           chỉ số + thống kê + đồ thị (offline)
```

## 1. Chuẩn bị (một lần)

```bash
./rx150.sh ab-run --task pick_cycle --check     # kiểm giới hạn khớp, độ cao EE, hold đủ dài — không cần ROS
```

- Sửa gains YAML xong thì **`colcon build`** lại: node đọc bản trong `install/`.
  `ab_run` chụp tham số thật từ node vào `params.yaml`, nên nếu quên build thì báo cáo vẫn ghi đúng
  gain đã dùng — chỉ là không phải gain bạn định thử.
- Dọn chỗ quanh robot. Task chỉ kiểm giới hạn khớp và EE z ≥ 5 cm, **không kiểm vật cản**.

## 2. Chạy — mỗi bộ một lần, cùng `--session`

```bash
# ── HAC ──
./rx150.sh ab-hac                                                    # terminal 1: robot + hac_node
./rx150.sh ab-run --ctrl hac   --task pick_cycle --session pc1 --trials 5   # terminal 2
# Ctrl+C terminal 1 khi xong

# ── Fuzzy ──
./rx150.sh ab-fuzzy                                                  # terminal 1: robot + fuzzy_node
./rx150.sh ab-run --ctrl fuzzy --task pick_cycle --session pc1 --trials 5   # terminal 2
```

`ab-run` từ chối chạy nếu không thấy node đích, hoặc thấy **cả hai** node cùng sống.
Chạy lại cùng `--session` sẽ **nối thêm** lượt (`trial_06`, …) chứ không ghi đè.

**Giảm sai lệch hệ thống** (motor nóng dần, pin/nguồn sụt): chạy xen kẽ khối
HAC 3 lượt → Fuzzy 3 lượt → HAC 3 → Fuzzy 3 thay vì 10 lượt một bộ rồi mới đổi; cùng
`--session` là đủ, các lượt tự nối. Lượt đầu sau khi bật torque thường lệch — bỏ bằng
`--drop-first 1` lúc report.

Mỗi task nên có **session riêng** (`pc1`, `steps1`): report từ chối trộn hai chuỗi waypoint.

## 3. Xử lý số liệu

```bash
./rx150.sh ab-report tuning_runs/ab_pc1
./rx150.sh ab-report tuning_runs/ab_pc1 --drop-first 1 --band-deg 0.5 --ss-window 0.5
```

Ra `tuning_runs/ab_pc1/report/`:

| file | nội dung |
|---|---|
| `report.md` | kiểm công bằng tham số, bảng mean ± std, Δ%, Cohen d, p Welch / Mann-Whitney, bảng theo khớp |
| `summary.csv` | bảng tổng hợp ở trên, dạng máy đọc |
| `trials.csv` | 1 dòng / bộ / lượt — dùng để tự vẽ hoặc đưa vào thống kê khác |
| `per_joint.csv` | mean ± std theo khớp |
| `segments.csv` | chi tiết nhất: bộ × lượt × segment × khớp |
| `fig_tracking.png` | q / e / PWM theo thời gian, lượt trung vị của mỗi bộ chồng lên nhau |
| `fig_trials.png` | box + điểm từng lượt cho mọi chỉ số |
| `fig_joints.png` | cột theo khớp với thanh std |

### Dữ liệu thô

`tuning_runs/ab_<session>/<ctrl>/trial_NN/data.csv`, 1 dòng mỗi tick debug của node (50 Hz):
`t, seg`, rồi mỗi khớp `{pos, vel, ref_pos, ref_vel, err, edot, pwm, grav}`. Góc rad, PWM −885…885.
`seg` = chỉ số waypoint đang chạy (khớp với `meta.json → segments`).

Vì sao 50 Hz mà không 400 Hz: node chỉ phát ref/err/effort ở `debug_publish_rate`. Ghép ref 50 Hz
với encoder 400 Hz sinh sai số giả tới v·20 ms (≈ 3° ở 2.6 rad/s), nên chỉ số dùng `err` do node
tự tính trong cùng chu kỳ.

### Chỉ số

| chỉ số | định nghĩa | ý nghĩa |
|---|---|---|
| RMSE bám, max \|e\| | e = q_ref − q (ref Ruckig) | chất lượng bám trong lúc chạy |
| Sai số xác lập | \|q_đích − q\| TB trong 0.5 s cuối segment | độ chính xác tới điểm |
| Thời gian xác lập | từ lúc gửi waypoint tới khi vào băng max(2 %·\|Δq\|, 0.5°) mãi mãi | nhanh-chậm; `Số lần không vào băng` đếm segment không bao giờ vào |
| Vọt lố | vượt đích theo chiều chạy / \|Δq\| | |
| RMS PWM phản hồi | RMS(u − u_grav) | effort do luật điều khiển — **dùng cột này** khi so năng lượng |
| Biến thiên PWM | Σ\|Δu\| / thời gian | rung / chattering |
| Bão hoà | % mẫu \|u\| ≥ 0.98·u_max | |

Xác lập / vọt lố chỉ tính cho khớp có \|Δq\| ≥ 2° trong segment đó.

## Chi phí tính toán (CPU time)

Ba tầng, trả lời ba câu hỏi khác nhau — báo cáo nên đưa cả ba:

| tầng | đo gì | lấy từ | cần robot |
|---|---|---|---|
| **Luật** | ns / lần gọi `hac_eval` vs `fuzzy_type1_eval`, ở `-O0` và `-O2` | `./rx150.sh ab-bench` | không |
| **Trong node** | thời gian khối luật (vòng 5 khớp) và cả chu kỳ tính, mỗi chu kỳ | topic `<ctrl>/timing` → `ab-run` → `ab-report` | có |
| **Tiến trình** | CPU% một lõi của cả node trong lượt chạy | `/proc/<pid>/stat` trong `ab-run` | có |

```bash
./rx150.sh ab-bench                    # -> tuning_runs/bench_law_<ts>/{report.md, bench.csv, fig_bench.png}
./tools/build.sh --packages-select rx150_hac_controller rx150_fuzzy_controller   # một lần, để có <ctrl>/timing
# rồi ab-hac / ab-fuzzy / ab-run / ab-report như trên — report có thêm mục "Chi phí tính toán"
```

Đọc số cho trung thực:
- `fuzzy_type1_eval` dựng lại bảng 7 hàm thuộc đầu ra × 201 điểm **ở mỗi lần gọi**, rồi gộp 25 luật × 201
  điểm; `hac_eval` là 2 phép nhân. Chênh lệch lớn nhất nằm ở tầng **luật**.
- Ở tầng **chu kỳ** và **tiến trình**, Ruckig, Pinocchio và DDS là phần chung của hai bộ, nên tỉ lệ nhỏ lại.
  Đó là số thật cần báo cáo cùng, không phải lỗi đo.
- Khối luật của HAC còn gồm bù ma sát `tanh`; tiến trình HAC còn luôn tính Pinocchio và phát 7 topic debug
  (Fuzzy 5) — các chênh lệch này nghiêng **về phía bất lợi cho HAC**.
- Máy đang ở governor `powersave`: số tuyệt đối dao động. Tỉ lệ ổn định hơn vì hai luật được đo xen kẽ.
- Timer `steady_clock` tốn cỡ 20 ns mỗi lần gọi; không đáng kể so với chu kỳ 2 500 µs.

## Công bằng — những gì pipeline giữ và những gì không

Giữ và **tự kiểm** (report cảnh báo nếu khác): `loop_rate`, `debug_publish_rate`,
`velocity_filter_tau`, `enable_profile`, `max_velocities`, `max_accelerations`, `max_jerk`,
`sync_mode`, `u_max`, `enable_gravity_comp`, `Gff`, `gravity_sign`, `gravity_model_source`.

- `./rx150.sh t1-hac` nạp model trọng lực **fitted**; fuzzy_node chỉ có Pinocchio + Gff. Vì vậy
  `ab-hac` **không** nạp model fitted. So `t1-hac` với `t1` là so hai bộ bù trọng lực chứ không phải
  hai luật điều khiển.
- Gain không "tương đương" theo nghĩa tuyệt đối: `rx150_fuzzy_gains.yaml` đã chỉnh Ke/Ked/Ku để
  khớp độ dốc mặt HAC (xem comment trong file). Nếu đổi a/b/c của HAC mà không chỉnh lại fuzzy, kết
  quả so hai mức gain chứ không phải hai luật.
- Không kiểm: nhiệt độ motor, điện áp nguồn, tải lên tay gắp. Ghi vào ghi chú phiên nếu có đổi.

## Thêm task mới

Chép một file trong `tasks/`, sửa `start` / `waypoints` (rad), rồi `--check`. `hold` phải lớn
hơn thời gian Ruckig cần + 1 s (`--check` tính và báo). Bắt đầu task ở `reference_pose` của node
(`[0, -1.80, 1.55, 0.80, 0]`) thì lúc vào lượt đầu tay không giật.
