# Kế hoạch: Node gắp ống nghiệm bỏ lên giá (rack) — rx150_pick_place

## Context

Nghiên cứu trong `~/Downloads/key_point` (task3_4tubes.py, task5.py...) đã giải quyết bài toán bằng script đơn thể: SDK Interbotix điều khiển trực tiếp, ma trận hand-eye cứng, lệnh gắp tuần tự. Trong workspace `interbotix_ws` đã có kiến trúc 3 lớp chuẩn hóa và **đã có sẵn phần "xác định hướng ống"**:

- `rx150_perception/scripts/yolo_tube_detector_node.py` publish `/yolo/detected_tubes` (PoseArray, frame `rx150/base_link`, position = điểm kẹp giữa thân ống, orientation = quaternion yaw quanh Z trục ống) + `/yolo/tube_classes` (String JSON, index song song).
- Mo-típ chuyển động đã được kiểm chứng trên máy thật: **IK-oracle + joint-goal qua MoveIt action `move_action`** (`pick_place_moveit_node.py:185-244`, `sort_tubes_by_color.py:140-210`), gripper qua `gripper_trajectory_bridge` (stall-khi-kẹp = SUCCESS, giữ effort khi vận chuyển).

Theo yêu cầu: *giả sử hướng ống đã xác định xong* — node mới chỉ consume yaw có sẵn. Chức năng: **tự quét → gắp tuần tự từng ống (top-down, bù wrist-roll theo yaw) → giữ ống thẳng đứng → cắm vào slot trên giá nghiêng theo màu → nhả → rút → ống kế tiếp**.

⚠️ Trạng thái vụt vỡ hiện tại (commit `60a6dde`): đã xóa `yolo_detector_node.py` + các launch cũ → `pick_place.launch.py` include launch không tồn tại, topic `/yolo/detected_objects` không còn publisher. Node mới **không** phụ thuộc phần này (dùng `/yolo/detected_tubes`), kế hoạch vá минимум cho phần include chết.

## Tạo mới

### 1. `src/rx150_pick_place/scripts/tube_rack_node.py` (file chính)

Node ROS chuẩn, modeled trên `pick_place_moveit_node.py` (worker thread + state machine) + math gắp ống của `src/rx150_perception/demos/sort_tubes_by_color.py`.

**Nhận diện (input):**
- Sub `/yolo/detected_tubes` (PoseArray) + `/yolo/tube_classes` (String JSON); sync khi 2 độ dài bằng nhau (`_sync()` như `sort_tubes_by_color.py:115-129`); decode `yaw = 2·atan2(qz, qw)`.

**Tham số (đọc từ yaml, mục 2):** topics, velocity scales, `approach_delta=0.08`, ladder pitch gắp `[1.5708, 1.40, 1.25, 1.10, 0.95]`, tham số giá (đáy slot0, `rack_angle_deg=62`, khoảng cách slot `0.12 m` dọc theo phương nghiêng), bảng màu→slot (`pink:0, blue:1, green:2, yellow:3`, unknown → slot trống bất kỳ), `dry_run` (chỉ tính IK + log, không gửi goal — để test an toàn), `fake_tubes` (danh sách (x,y,z,yaw,class) JSON — dry-run/verify IK + chọn slot không cần camera/ống thật), `tube_length (0.10)` + `insert_depth` — **hình học thả ống**: ống kẹp giữa thân, pitch=0 → ống thẳng đứng, đáy ống thấp hơn điểm EE đúng `tube_length/2` → `z_hover = z_slot_top + tube_length/2 + margin(0.03)`, `z_insert = z_hover − insert_depth`; `place_pitch=0.0` với ladder dự phòng `0.0 → 0.15 → 0.3` nếu IK fail ở slot xa.

**Định nghĩa slot giá (tính từ tham số, theo mô-típ `task3_4tubes.py:65-99`):**
- `slot_k = slot0 + k·spacing·(sin(angle), −cos(angle))`, z tăng dần theo k (giá nghiêng: slot sau cao hơn).

**Luồng chính (chạy 1 lần khi khởi động, auto):**
1. `HOME_JOINTS` + mở gripper (`HOME_JOINTS=[0,0,0,0,0]`).
2. SCAN: chờ detection `detection_wait_s`; chụp snapshot dùng **median ~5 frame trong ~1 s** cho (x,y,z,yaw) từng ống (chống gắp lệch do 1 frame nhiễu — thay cho EMA của key_point); **bỏ qua ống đã nằm trong vùng giá** (khoảng cách XY tới slot bất kỳ < 5 cm — như `find_tubes_on_rack` của key_point) để không gắp lại ống đã cắm.
3. Với mỗi ống trong snapshot (theo thứ tự phát hiện):
   - **Toán gắp** (copy `sort_tubes_by_color.py:305-327`): `z_approach = max(tz+0.08, 0.12)`, `z_grasp = max(tz, 0.02)`; `solve_ik_top_down()` thử ladder pitch; `wrist_rot = normalize(yaw − waist_angle)` với `waist_angle = j_grasp[0]`; ghi đè `j[4] = wrist_rot` cho cả approach lẫn grasp (lỗi 180° không sao — ngón vẫn vuông góc trục ống).
   - **State machine mỗi ống:** APPROACH (v 0.3) → DESCEND (v 0.12) → GRASP (bridge, stall=success) → LIFT về approach (v 0.2) → chọn slot theo màu (slot ưu tiên đầu tiên còn trống; đánh dấu occupied) → PLACE_HOVER trên slot tại `z_hover = z_slot_top + tube_length/2 + 0.03` (IK ladder quanh `place_pitch` 0.0→0.15→0.3 → ống treo thẳng đứng giữa 2 ngón) → INSERT hạ chậm (v 0.1) xuống `z_insert = z_hover − insert_depth` → RELEASE mở kẹp → **nhấc THẲNG ĐỨNG** vượt đỉnh ống đã cắm (≥ `z_slot_top + tube_length + 0.02`) → chỉ SAU đó mới RETREAT ngang → HOME. TRANSPORT + mọi hover giữ EE cao hơn vật cản tối thiểu `tube_length/2 + 0.03` vì ống treo dọc dưới EE (ống không được model trong MoveIt — phải kiểm tra quỹ đạo bằng mắt ở dry-run/RViz).
   - Fail bước nào → mở gripper + home + bỏ qua ống đó, sang ống kế (`_abort()` như `pick_place_moveit_node.py:366-370`).
4. Hết ống → log tổng kết (thành công/thất bại theo slot) → HOME → idle.
5. **Interface chạy lại (demo không cần relaunch):** service `std_srvs/Trigger` `/tube_rack/run` — chạy lại chu kỳ mới (quét lại detection mới, reset slot occupancy theo vùng giá); topic `/tube_rack/status` (String JSON: slot occupancy + bước state machine hiện tại). Tự chạy 1 lần khi khởi động, tắt được bằng param `auto_start:=false` (khi muốn trigger tay/test từng phần).

**Tái sử dụng nguyên văn mo-típ từ `pick_place_moveit_node.py`:** `move_to_joint_target()` (JointConstraint/joint, `OK_CODES=(1,-4)`, workspace ±1 m), `_wait_future()`, `_ik()` (`set_ee_pose_components(execute=False)` — tuyệt đối không publish `/rx150/commands/*` trực tiếp, sẽ đánh fuzzy_node/hac_node), `_gripper()` (bridge nhóm `interbotix_gripper`/`left_finger` 0.015/0.037, PWM fallback `JointSingleCommand`), setup `main()` (global node, `robot_startup`, set gripper PWM mode).
**Collision box cho giá (nên có):** mo-típ `_ensure_scene` của table — thêm `CollisionObject` box `rack` (frame `world`), kích thước **thu nhỏ một chút so với thực**, đỉnh box = đỉnh slot: planner tránh quét ngang qua giá khi TRANSPORT mà không cản INSERT (EE không xuống dưới đỉnh slot; phần ống xuống lỗ không được model nên không bị cản).

### 2. `src/rx150_pick_place/config/tube_rack_params.yaml`
Toàn bộ tham số trên + `home_joints`. Ghi chú đo đạc: slot0 đo bằng tay khi setup (có thể dùng RViz click / log detection ống cắm sẵn trong giá để hiệu chỉnh).

### 3. `src/rx150_pick_place/launch/tube_rack.launch.py`
- Chạy `tube_rack_node.py` với params yaml.
- `detector:=true` (mặc định): chạy luôn `rx150_perception` `yolo_tube_detector_node.py` (đã được install qua CMakeLists của rx150_perception) — self-contained, không đụng launch chết.

## Sửa

### 4. `src/rx150_pick_place/CMakeLists.txt`
Thêm install cho `scripts/tube_rack_node.py`, `config/tube_rack_params.yaml`, `launch/tube_rack.launch.py` (theo pattern install hiện có của package).

### 5. Vá wiring chết trong `src/rx150_pick_place/launch/pick_place.launch.py`
Bỏ/giảm include `yolo_detector.launch.py` **và `pcl_detector.launch.py`** — cả hai đều không còn tồn tại trong `src/rx150_perception/launch/` (đã xác nhận). Nhánh yolo: thay bằng chạy trực tiếp `yolo_tube_detector_node.py` + `detection_topic:=/yolo/detected_tubes`; nhánh pcl: bỏ hẳn (thu hẹp `choices=['yolo']`) hoặc giữ như tính năng đã biết hỏng. (Phần gesture `/yolo/detected_objects` của hand_gesture_node ngoài phạm vi — node mới không dùng.)

## Ghi chú kỹ thuật / rủi ro

- **Model hiện tại**: chỉ có `models/best_color.pt` (bbox) → yaw từ fallback `minAreaRect`. Node không quan tâm nguồn yaw; khi bỏ thêm `best_keypoint.pt` vào `rx150_perception/models/` thì hướng chính xác hơn tự động.
- **Dấu yaw**: key_point dùng `roll = −atan2(...)` (tâm→nắp), detector workspace dùng `atan2` (nắp→đáy) — khác dấu nhưng grasp chỉ cần vuông góc (đối xứng mod 180°) nên sai dấu không làm hỏng; vẫn phải xác nhận bằng `dry_run` + `/yolo/image_debug` trước khi chạy thật.
- **Cắm vào giá nghiêng 62° với ống thẳng đứng** là cách demo key_point đã chạy ổn (thả rơi vào lỗ); `place_pitch` là tham số để tinh chỉnh (ví dụ −0.3 rad nghiêng theo giá nếu cắm kẹn).
- **Backend-agnostic**: T1 chạy được trên `fuzzy_moveit.launch.py` HOẶC `hac_moveit.launch.py` (twin launch gần giống hệt — node chỉ dùng `move_action` + trajectory bridge, không quan tâm controller bên dưới). Sau khi hiệu chuẩn gravity/friction cho HAC xong (kế hoạch "friction cho bộ HAC"), demo ống NÊN chuyển sang hac_moveit — gravity chuẩn giúp giữ ống thẳng đứng ổn định ở pitch=0 (pose nặng); 2 kế hoạch bổ trợ nhau.
- **An toàn**: fuzzy/hac_moveit.launch.py bật torque + kéo tay về home khi launch; chỉ 1 xs_sdk trên bus; source đủ 4 overlay qua `source_all.sh`.
- Sau khi đặt ống, detector vẫn thấy ống trong giá → đã xử lý bằng snapshot + lọc khoảng cách 5 cm tới slot.

## Verification

1. Build: `cd ~/interbotix_ws && colcon build --packages-select rx150_pick_place && source install/setup.bash`.
2. **Dry-run (không cần ống thật):** T1 `ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=true use_camera_static_tf:=false` (hoặc `rx150_hac_controller hac_moveit.launch.py` cùng các cờ); T2 `ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true` (không camera thì cấp `fake_tubes`) → kiểm tra log: mỗi ống in được IK ok, `wrist_rot`, slot được chọn; đối chiếu hướng mũi tên trên `/yolo/image_debug` với yaw.
3. **Chạy thật từng ống:** đặt 1 ống → chạy không dry_run → quan sát: ngón vuông góc trục ống khi hạ, gripper stall giữ effort khi nhấc, ống thẳng đứng khi tới giá, cắm vào slot, nhả + nhấc lên không kéo đổ ống. Tinh chỉnh `insert_depth`, `place_pitch`, z slot theo quan sát.
4. **Full cycle:** 4 ống 4 màu → kiểm tra phân loại đúng slot ưu tiên, không gắp lại ống đã cắm, log tổng kết đủ 4/4.
5. Nếu IK fail ở slot xa: hạ ladder pitch hoặc dịch slot0 gần hơn (param, không sửa code).
6. **Hiệu chỉnh slot bằng ống cắm sẵn:** cắm sẵn 1 ống vào slot đã biết → chạy detector → so (x,y,z) đo được với công thức slot → hiệu chỉnh `slot0/spacing/rack_angle_deg` trong yaml (không sửa code).
7. Dry-run bật RViz: kiểm tra quỹ đạo TRANSPORT không lượn thấp (ống treo dọc không được model — quan sát bằng mắt), collision box giá hiện đúng vị trí/độ cao.
