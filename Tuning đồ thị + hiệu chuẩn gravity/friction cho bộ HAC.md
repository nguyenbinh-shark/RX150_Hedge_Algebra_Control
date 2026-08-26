# Kế hoạch: Tuning đồ thị + hiệu chuẩn gravity/friction cho bộ HAC (RX150)
  
## Bối cảnh
  
Bộ HAC ([rx150_hac_controller](src/rx150_hac_controller/ )) điều khiển RX150 bằng l luật PD bão hòa `u = (2c/3a)·sat(e) + (c/3b)·sat(ė) + PWM_gravity`, clamp ±u_max, ra lệnh PWM trực tiếp qua xs_sdk. Người dùng muốn:
1. **Tool tuning trên đồ thị riêng** — kiểu tune PID: đổi gain → thấy ngay response góc trên plot (chọn: live plot nhúng trong GUI có sẵn).
2. **Hiệu chuẩn gravity comp chính xác** — hiện chỉ là ước lượng.
  
**Chẩn đoán sau khảo sát** (quan trọng, sửa lại hiểu lầm ban đầu):
- Gravity comp **đã** phụ thuộc góc (Pinocchio `computeGeneralizedGravity` mỗi chu kỳ — [gravity_comp.cpp:34-57](src/rx150_hac_controller/src/gravity_comp.cpp#L34-L57 )). Cái chưa tối ưu là **3 lớp ước lượng**: (a) hệ số Gff N·m→PWM đoán từ datasheet, (b) dấu `gravity_sign` chưa kiểm chứng (shoulder/elbow/wrist_angle có Drive_Mode 1), (c) khối lượng URDF gốc chưa tính payload thực (gripper/RealSense). Cộng thêm (d) **friction hoàn toàn chưa được bù**.
- **BUG GUI**: [rx150_tuning_gui.py](src/rx150_motion_common/scripts/rx150_tuning_gui.py ) với `target:=hac` gửi `Ke/Ked/Ku` mà hac_node **không khai báo** → rclcpp từ chối ngầm (GUI không đọc response) → mọi lần tune HAC qua GUI trước nay **không có tác dụng**.
- `/joint_states.effort` **CÓ** được publish (100 Hz) nhưng sai đơn vị (Present_Load×2.69) — đơn điệu theo mô-men, dùng được cho nhận dạng tương đối; fit chính thống làm ở **đơn vị PWM** (đơn vị node thật sự áp dụng).
- Đã có sẵn hạ tầng tái sử dụng: CSV logger 41 cột ([data_analysis/csv_logger.py](data_analysis/csv_logger.py )), plotter offline ([data_analysis/plot_control_csv.py](data_analysis/plot_control_csv.py )), 5 topic telemetry của hac_node (`hac/{reference,error,edot,effort,gravity}`), live param qua `set_parameters`.
  
**Quyết định của người dùng**: live plot trong GUI ✓ · ID tự động (arm tự chạy) ✓ · gộp friction ✓ · phạm vi chỉ HAC ✓.
  
## Kiến trúc tổng thể
  
```
GUI (slider gains + tab Plots live) ──set_parameters──> hac_node (100 Hz)
        │                                                  │ PWM
        │ subscribe hac/*, joint_states                    v
        v                                            xs_sdk → Goal_PWM
  Ring buffer + FigureCanvasTkAgg (10 Hz redraw)
Session tool (excitation step/sine + CSV + metrics) ──> tuning_runs/<label>/ 
Compare tool (A/B các run, Δmetrics + overlay)
Gravity/Friction ID tool ──LSQ fit──> rx150_gravity_model.yaml ──> hac_node
```
  
Hai lớp bổ trợ nhau: **GUI live plot** = cảm nhận định tính tức thời khi kéo gain; **session + metrics** = bằng chứng định lượng lặp lại được (chuẩn xác — yêu cầu cốt lõi của người dùng).
  
---
  
## Giai đoạn 0 — Sửa lỗi nền tảng (bắt buộc làm trước)
  
1. **Sửa tab Gains của GUI cho HAC** ([rx150_tuning_gui.py](src/rx150_motion_common/scripts/rx150_tuning_gui.py )):
   - Hàng gain cho hac: `a`, `b`, `c` (scalar — thêm đường gửi `PARAMETER_DOUBLE`, hiện chỉ có DOUBLE_ARRAY), `error_limit`, `error_dot_limit`, `u_max`, `Gff` (mảng 5). Bỏ Ke/Ked/Ku với target=hac.
   - Gắn done-callback cho future `set_parameters` — param bị reject → ô trạng thái đỏ (hết fire-and-forget).
   - Sửa "Read current" hỗn hợp scalar/array; sửa lưu-yaml cho dòng scalar (`a: 0.3`) — regex hiện chỉ khớp dòng mảng; lưu vào **bản src** (tìm `src/<pkg>/config/...` đi lên từ install) chứ không chỉ install share (build lại là mất).
2. **Đồng bộ default lệch nhau**: dead-init trong [hac_node.hpp](src/rx150_hac_controller/include/rx150_hac_controller/hac_node.hpp ) (0.3/0.4/3000) và default Gff trong code (885/632…) vs yaml (150/255…) — thống nhất theo yaml.
  
## Giai đoạn A — Tuning trên đồ thị
  
3. **Tab "Plots" live trong GUI** (file trên): `FigureCanvasTkAgg` + refresh bằng `root.after()` 10 Hz (GUI đã single-thread: `_tick()` gọi `spin_once` — không cần thread mới, không conflict). Ring buffer ~10 s × mỗi khớp, subplot per khớp: q vs q_ref; error; PWM tách gravity/phần điều khiển. Drain `spin_once` nhiều lần mỗi tick để không tụt backlog (QoS best-effort depth 1 như csv_logger).
4. **`rx150_tuning_session.py`** (mới, [scripts/](src/rx150_motion_common/scripts/ )): kịch bản kích thích lặp lại được (step Δ tuỳ chọn, sine A/f tuỳ chọn, hold), ghi CSV **đúng schema mở rộng 46 cột** (41 cột hiện có + `{j}_fric` — mở rộng ở bước 4b; plot_control_csv.py mở được luôn), kèm `gains_snapshot.yaml` (đọc lại từ node qua get_parameters — chắc chắn là gain đã áp), `metrics.json` (rise time, overshoot, settling 2%, RMSE, ss_err, RMS PWM), `plots/*.png`, `meta.json` (marker thời điểm gửi step để align khi so sánh). Output `~/interbotix_ws/tuning_runs/<label>_<ts>/`.
4b. **Mở rộng schema CSV cho friction** ([csv_logger.py](data_analysis/csv_logger.py), [plot_control_csv.py](data_analysis/plot_control_csv.py)): logger hiện 41 cột = `timestamp` + 5 khớp × 8 field (`{j}_pos/vel/ref_pos/ref_vel/err/edot/pwm/grav`) — **chưa có friction**. Thêm subscribe `hac/friction` (JointState .effort) → 5 cột `{j}_fric` appended cuối (backward-compatible: CSV cũ vẫn mở được); plotter vẽ thêm panel PWM tách gravity/friction/phần HAC. Không có bước này thì mục tiêu "tách bạch PWM = HAC + gravity + friction" khi phân tích không thực hiện được.
5. **`rx150_run_compare.py`** (mới): nhận 2+ thư mục run, align theo marker trong meta.json, overlay per-joint, bảng Δmetrics (console + markdown). *(Lưu ý: scripts/ đã có `compare_fuzzy_vs_hac.py` + `comparison_data.csv` — viết mới vì tool cũ chỉ hard-code so sánh fuzzy↔hac từ 1 CSV gộp, không align theo marker / không tính metrics; reuse phần code plot của nó nếu hợp.)*
6. **`tuning_lib.py`** (mới): dùng chung — CSV recorder (schema 46 cột sau bước 4b), FK nhẹ tính chiều cao EE cho lưới pose an toàn, metrics, đọc joint limits từ [rx150_motor.yaml](src/rx150_motion_common/config/rx150_motor.yaml ) (cửa sổ tick → rad, trừ margin 0.2 rad).
7. Cài đặt script mới vào [CMakeLists.txt](src/rx150_motion_common/CMakeLists.txt ) (install PROGRAMS như 2 script hiện có). Tùy chọn: layout PlotJuggler cho hac (copy [fuzzy_plotjuggler_layout.xml](data_analysis/layouts/fuzzy_plotjuggler_layout.xml ), đổi prefix).
  
## Giai đoạn B — Hiệu chuẩn gravity + friction (ID tự động)
  
8. **`rx150_gravity_id.py`** (mới) — 4 mode `signcheck | plan | identify | validate`:
   - **B0 signcheck** (làm đầu tiên, ~2 phút): chạy với `enable_gravity_comp:=false` (PD thuần giữ pose — tránh arm giật nếu Gff/sign đang sai; `joint_states.effort` = Present_Load×2.69 là tải đo thật, độc lập với gravity comp nên correlation vẫn đo được). 5 pose, tính correlation giữa effort đo và τ Pinocchio từng khớp. Âm → đổi `gravity_sign[i]=-1` trong yaml, restart, chạy lại.
   - **B1 identify**: lưới ~60 pose (q2∈{-1.6..0}, q3∈{0.6..1.8}, q4∈{-0.4..0.6}), lọc trong joint limits + EE cao ≥5 cm; mỗi pose tiếp cận từ **2 hướng** (triệt stiction), settle detect |qd|<0.02 rad/s trong 0.5 s, lấy trung bình 1 s: effort đo + PWM tổng (`hac/effort`) + q. Di chuyển qua `/rx150/hac/setpoint` với profile ON, tốc độ ≤0.5 rad/s. Đọc nhiệt độ động cơ (service xs_sdk `/rx150/get_motor_temps`) đầu & cuối B1 — ghi vào provenance.
   - **Mô hình fit** (tuyến tính theo tham số, ridge λ nhỏ): với m2=q2, m3=q2+q3, m4=q2+q3+q4 —
     - shoulder: [cos m2, sin m2, cos m3, sin m3, cos m4, sin m4] (6 hệ số)
     - elbow: [cos m3, sin m3, cos m4, sin m4] (4); wrist_angle: [cos m4, sin m4] (2)
     - waist/wrist_rotate: bằng kết cấu 0 (trục đứng/trục roll)
     - Fit song song **đơn vị PWM** (để node dùng — hấp thụ luôn sai số Gff lẫn sai số khối lượng URDF) và đơn vị effort (mô hình vật lý). Thêm regressor hướng tiếp cận d_k để tách thiên lệch stiction.
   - Chất lượng: R², residual RMS, chia 80/20 train/held-out; báo cáo Gff tương đương + đối chiếu ngưỡng vật lý ~590 PWM/N·m của XL430.
   - **B4 validate**: chạy lại lưới với mô hình fitted — residual RMS phải giảm ≥2× so với trước hiệu chuẩn. Chạy thêm 1 lần lúc động nguội (~15 phút nghỉ): residual không được tăng rõ (chênh <30% so với lúc nóng) — nếu lệch lớn, ghi rõ nhiệt hiệu chuẩn vào provenance yaml và chuẩn hóa quy trình ID về cùng trạng thái nhiệt.
9. **`rx150_friction_id.py`** (mới): từng khớp, các khớp khác đậu ở pose trọng trường thấp; cruise vận tốc không đổi 0.1/0.2/0.4 rad/s **2 hướng** (hạ tạm `max_velocities[i]`, gửi setpoint xa); trong cửa sổ cruise: `u − u_grav(q) = f_c·sign(qd) + f_v·qd` → LSQ cho Coulomb + viscous. Xuất R², hệ số.
10. **Tích hợp vào hac_node** ([hac_node.cpp](src/rx150_hac_controller/src/hac_node.cpp )):
    - Param mới: `friction_coulomb[5]`, `friction_viscous[5]`, `friction_eps` (0.05), `gravity_model_source` ('pinocchio'|'fitted'), `fitted_gravity_coeffs[12]` — nạp từ `config/rx150_gravity_model.yaml` mới (do ID tool ghi ra, kèm provenance: ngày, payload đang gắn).
    - [gravity_comp.cpp](src/rx150_hac_controller/src/gravity_comp.cpp ): thêm `computeFitted(q)` đánh giá basis trig ở trên; Pinocchio giữ làm fallback + cross-check.
    - Friction FF trong `onTimer` **sau** gravity, **trước** clamp ±u_max sẵn có (~dòng 496-497): `u_fric = f_c·tanh(qd_meas/eps) + f_v·qd_meas` (dùng velocity đo, không phải error_dot; nếu `hac/edot` rung thì **tăng `friction_eps`** chứ không thêm filter — filter gây trễ phá tác dụng FF). Topic mới `hac/friction` (JointState .effort) — tách bạch PWM = HAC + gravity + friction khi ghi hình.
    - Mở rộng `onParamChange` cho friction_coulomb/friction_viscous (live-tune như Gff).
    - Thêm `config/rx150_hac_gains_safe.yaml` (u_max≈[500,600,500,400,400], max_vel 0.5, max_acc 1.0) + launch arg `gains_file` trong [hac_control.launch.py](src/rx150_hac_controller/launch/hac_control.launch.py ) để nạp khi ID.
  
## Giai đoạn C — Tune lại gain (quy trình kiểu PID)
  
11. Chạy từ bộ gain yaml hiện tại với gravity+friction đã chuẩn. **Mỗi thay đổi ĐÚNG MỘT thông số = 1 session có label**, so bằng `rx150_run_compare.py`:
    - `a` ↓ = cứng hơn (Kp=2c/3a): 0.3 → 0.25 → 0.2… dừng khi overshoot >10% hoặc PWM đập, lùi 20-30%.
    - `b` ↓ = dämp hơn (Kd=c/3b): giảm tới khi `hac/edot` rung/nhiễu thì lùi.
    - `error_limit` ≈ sai số cho phép (0.1-0.2 rad) để bão hòa chỉ hoạt động trong transient.
    - `u_max` giữ 600, chỉ giảm cổ tay nếu rung. Giữ `c` cố định để tách ảnh hưởng.
12. Lưu bộ gain cuối (GUI "Lưu vào yaml" đã sửa) + commit `rx150_hac_gains.yaml` & `rx150_gravity_model.yaml` kèm ghi chú ngày hiệu chuẩn + payload.
  
## An toàn (thuộc lòng trước khi chạy)
  
- **Kill đúng bậc**: (1) Ctrl-C script kích thích — hac_node giữ pose cuối (an toàn); (2) đỡ tay bằng tay rồi `ros2 service call /rx150/torque_enable ... enable: false`. **Tuyệt đối không Ctrl-C hac_node khi tay chịu lực** (shutdown hook zero PWM + torque off → rơi).
- Trước khi bật/tắt: đưa arm về pose thẳng [0,0,0,0,0] hoặc sleep pose. Bàn trống, người luôn trong tầm.
- ID chạy với gains_safe.yaml, tốc độ ≤0.5 rad/s; motor nóng → nghỉ 2-3 phút.
- Đổi payload (RealSense/gripper/vật cầm) ⇒ phải chạy lại B1 (10-15 phút).
  
## Files
  
**Tạo**: [scripts/tuning_lib.py](src/rx150_motion_common/scripts/tuning_lib.py ), `scripts/rx150_tuning_session.py`, `scripts/rx150_run_compare.py`, `scripts/rx150_gravity_id.py`, `scripts/rx150_friction_id.py`, [config/rx150_gravity_model.yaml](src/rx150_hac_controller/config/rx150_gravity_model.yaml ) (do tool sinh), `config/rx150_hac_gains_safe.yaml`.
  
**Sửa**: [csv_logger.py](data_analysis/csv_logger.py ) + [plot_control_csv.py](data_analysis/plot_control_csv.py ) (bước 4b — cột `{j}_fric` + panel tách gravity/friction), [rx150_tuning_gui.py](src/rx150_motion_common/scripts/rx150_tuning_gui.py ) (Giai đoạn 0 + tab Plots), [hac_node.cpp](src/rx150_hac_controller/src/hac_node.cpp ) + [hac_node.hpp](src/rx150_hac_controller/include/rx150_hac_controller/hac_node.hpp ), [gravity_comp.cpp](src/rx150_hac_controller/src/gravity_comp.cpp ) + [gravity_comp.hpp](src/rx150_hac_controller/include/rx150_hac_controller/gravity_comp.hpp ), [hac_control.launch.py](src/rx150_hac_controller/launch/hac_control.launch.py ), [CMakeLists.txt](src/rx150_motion_common/CMakeLists.txt ).
  
**Tái sử dụng**: schema CSV của [csv_logger.py](data_analysis/csv_logger.py ) (mở rộng 46 cột ở bước 4b; session CSV mở được bằng [plot_control_csv.py](data_analysis/plot_control_csv.py )), mẫu QoS/plot style của data_analysis, cơ chế live-param có sẵn của hac_node.
  
## Kiểm tra & nghiệm thu
  
1. **Round-trip GUI**: `ros2 param get /rx150/hac_node a` trả đúng giá trị vừa Apply; ô trạng thái đỏ khi gửi tên param không tồn tại (test cố ý).
2. **Tab Plots**: kéo slider setpoint → đồ thị q/q_ref respond <0.5 s; đổi `a` → thấy rõ thay đổi độ cứng ngay trên đồ thị error.
3. **Session smoke**: `--label smoke --delta 0.1` → đủ data.csv (46 cột ~100 Hz), gains_snapshot trùng node, metrics.json, plots đủ 5 khớp; plot_control_csv.py mở được file.
4. **B0**: correlation(effort, τ_Pinocchio) > 0 cho cả 5 khớp.
5. **B1**: R² > 0.9 (shoulder/elbow/wrist_angle), held-out ≈ train, residual vs góc còn cấu trúc; Gff tương đương < ~590 PWM/N·m.
6. **B2**: R² > 0.8 ở khớp ma sát lớn; f_c dương đúng dấu hướng chuyển động.
7. **B4 trước/sau**: residual RMS giảm ≥2× (kiểm cả lúc nóng lẫn lúc nguội — chênh <30%, coi là drift nhiệt chấp nhận được); droop test (giảm a một nửa, giữ 8 pose): |q−q_ref| < 0.02 rad; sine RMSE giảm ≥30% so baseline.
8. **Nghiệm thu cuối Phase C**: step Δ0.3 rad — overshoot ≤10%, settling 2% ≤1.5 s, ss_err ≤0.02 rad; hold 60 s pose nặng nhất không trôi >0.02 rad; 1 chu trình MoveIt pick-place qua hac_moveit không mất tracking.
  
## Rủi ro chính & giảm thiểu
  
- Đơn vị effort sai (load×2.69) → fit ở đơn vị PWM làm chuẩn, effort chỉ làm đối chiếu.
- Stiction đối xứng không tách khỏi gravity bằng giữ-pose → 2 hướng tiếp cận + 1 vòng lặp B1↔B2.
- Không có e-stop phần cứng; watchdog stale joint_states zero PWM làm tay rơi → quy trình kill bậc trên, luôn ngồi tư thế đỡ được tay.
- Overfit hộp pose hẹp → 80/20 held-out, mở rộng grid trong an toàn.
- Payload đổi vô hiệu hoá hiệu chuẩn → provenance trong yaml + quy tắc chạy lại B1.
- Drift nhiệt (điện trở cuộn, viscosity mỡ thay đổi theo nhiệt) đổi hệ số PWM → đọc nhiệt đầu/cuối B1, B4 validate lúc nguội, provenance ghi nhiệt hiệu chuẩn.
  