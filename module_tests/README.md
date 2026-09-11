# Module tests

Thư mục này chứa các chương trình kiểm tra từng chức năng độc lập của hệ thống
RX150. File `COLCON_IGNORE` giúp `colcon` bỏ qua thư mục này khi dò ROS package.

## Cấu trúc

- `fuzzy_controller/`: kiểm tra luật mờ, gains, setpoint và trajectory bridge.
- `perception/`: kiểm tra camera, depth, YOLO, TF và xử lý point cloud.
- `moveit/`: kiểm tra planning, IK và thực thi trajectory.
- `hardware/`: kiểm tra riêng camera, motor, gripper và thiết bị ngoại vi.
- `common/`: mã tiện ích dùng chung cho nhiều bài test.

> `run_test.py` chỉ dò 4 nhóm `fuzzy_controller`, `perception`, `moveit`,
> `hardware` (hằng `GROUPS`). File đặt ngoài các thư mục đó sẽ không hiện trong
> `--list`.

Mỗi bài test nên chỉ kiểm tra một chức năng và tự kiểm tra điều kiện đầu vào trước
khi tác động lên phần cứng. **Không** đặt model, rosbag hay log dung lượng lớn vào đây:
model YOLO nằm trong `rx150_perception/models/` (commit cùng package), bag ghi ra `bags/`
và dữ liệu đo của một lần chạy vào `tuning_runs/<nhãn>_<ts>/` — hai chỗ sau đã có luật
`.gitignore` riêng.

## Chạy test

Sau khi build và source workspace:

```bash
source source_all.sh
python3 module_tests/run_test.py --list
python3 module_tests/run_test.py perception/example_import_test.py
```

## Bộ smoke-test theo tầng (dùng khi bring-up phần cứng)

Ba bài dưới đây bám đúng thang bậc của
[`rx150_pick_place/docs/RUNBOOK.md`](../src/rx150/apps/rx150_pick_place/docs/RUNBOOK.md).
Không bài nào phát lệnh tới robot — chỉ nghe và hỏi discovery. Exit 0 = GO.

```bash
python3 module_tests/run_test.py hardware/joint_states_test.py        # B1
python3 module_tests/run_test.py moveit/action_servers_test.py        # B1 / B5
python3 module_tests/run_test.py perception/tf_and_detection_test.py  # B2 / B3
```

Khi NO-GO, mỗi bài in ra **nguyên nhân gốc + lệnh kiểm tiếp theo**, không chỉ
"failed". Vài tuỳ chọn hay dùng:

```bash
# ngưỡng tần số riêng (mặc định FAIL nếu < 50 Hz, danh nghĩa 100)
python3 module_tests/run_test.py hardware/joint_states_test.py -- --seconds 5 --min-hz 80

# backend direct chỉ cần arm + driver; move_group/scene thiếu chỉ là WARN
python3 module_tests/run_test.py moveit/action_servers_test.py -- --backend direct

# bàn đang trống thì PoseArray rỗng là bình thường
python3 module_tests/run_test.py perception/tf_and_detection_test.py -- --allow-empty
```

Xem camera và kết quả nhận diện YOLO trực tiếp:

```bash
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py
```

Mặc định bài test chỉ dùng ảnh màu. Để kiểm tra cả luồng depth:

```bash
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py -- --with-depth
```

Đổi model hoặc ngưỡng confidence:

```bash
python3 module_tests/run_test.py perception/yolo_camera_gui_test.py -- \
  --weights src/rx150/rx150_toolbox/rx150_perception/models/best_color.pt --confidence 0.35
```

Có thể truyền đối số cho bài test sau dấu `--`:

```bash
python3 module_tests/run_test.py hardware/my_motor_test.py -- --joint wrist_angle
```

## Quy ước file test

- Đặt tên `test_<chuc_nang>.py` hoặc `<chuc_nang>_test.py`.
- Test dùng robot thật phải có cảnh báo trong docstring và mặc định không phát lệnh.
- Trả mã thoát `0` khi thành công, khác `0` khi thất bại.
- Ghi rõ topic, service, action và phần cứng cần thiết ở đầu file.
