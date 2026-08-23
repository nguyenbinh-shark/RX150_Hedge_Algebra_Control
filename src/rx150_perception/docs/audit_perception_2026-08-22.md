# Báo cáo audit pipeline perception — rx150_perception

**Ngày:** 2026-08-22 · **Phạm vi:** package `rx150_perception` + toàn bộ chuỗi upstream mà nó include (`interbotix_perception_modules`, `interbotix_perception_pipelines`, `interbotix_tf_tools`, fork `apriltag_ros` 3.2.1 của Interbotix tại `~/apriltag_ws`).
**Cấu hình vật lý:** arm RX150, camera RealSense D435i, AprilTag **tag36h11 ID 0, cạnh 30 mm** (0.03 m).
**Phương pháp:** đọc source toàn bộ chuỗi + chạy thử live `ros2 launch` + review đa góc độ (12 agent, mọi phát hiện đều được đối chiếu chéo với file thật).

> Hướng dẫn tham chiếu: [Trossen — Perception Pipeline Configuration](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/perception_pipeline_configuration.html).
> Nguyên nhân gốc của hầu hết lỗi: hướng dẫn mô tả file **tổng** `xsarm_perception.launch.py` (gồm robot bring-up với `use_world_frame:=false`, driver camera, `pc_filter`, `armtag`, **`static_transform_pub`**, RViz), trong khi ta port từng mảnh rời nên mất các m nối quan trọng.

---

## 1. Tóm tắt

| # | Mức | File | Triệu chứng | Nguyên nhân |
| --- | ----- | ------ | ------------- | ------------- |
| L1 | 🔴 critical | `launch/armtag.launch.py` | Bấm **Snap Pose** → GUI báo success nhưng TF tree không đổi, không lưu đâu | Kết quả publish lên topic thường `/static_transforms`; node tiêu thụ duy nhất (`static_trans_pub`) không được launch |
| L2 | 🟠 high | chạy armtag không có robot | Publish **transform rác** (base_link đặt tại vị trí tag), GUI vẫn báo success | `armtag.py:225-240` trả ma trận đơn vị khi lookup TF fail; caller không kiểm tra |
| L3 | 🟠 high | `armtag.launch.py` (topology) | Sau khi sửa L1: `rx150/base_link` có 2 parent; chạy kèm static TF/handeye → **cycle TF** | URDF MoveIt có `world→rx150/base_link`; armtag mặc định publish `camera→rx150/base_link` |
| L4 | 🟡 medium | `armtag.launch.py`, `standalone_perception.launch.py` | Profile 640x480x30 **không được áp dụng**, driver tự chọn resolution | realsense2_camera **4.58.3** đổi tên param: `rgb_camera.color_profile` / `depth_module.depth_profile`; tên cũ bị drop silently |
| L5 | 🟡 medium | `config/standalone_perception.rviz` | 2 display "Filtered PointCloud" + "Object Markers" luôn luôn trống | Trỏ topic không tồn tại (`pointcloud_clusters`, `object_markers`) + sai loại message (MarkerArray vs Marker) |
| L6 | 🟡 medium | upstream `apriltag.launch.py` | Đổi `camera_info_topic`/`camera_frame` qua launch arg → không tác dụng | Args chỉ truyền cho `picture_snapper` (không dùng); GUI đọc param `/apriltag/camera_info_topic` default cứng |
| L7 | ⚪ nhỏ | nhiều file | Xem §3.7 | Thiếu dep `rviz2`, config chết, install `__pycache__`, header hướng dẫn sai… |
| L8 | ⚪ upstream | upstream only | Ghi nhận, không sửa trong phạm vi này | Euler averaging qua ±π, dirname quirk, semantics của tag `size` |

**Những gì ĐÃ ĐÚNG từ đầu** (đã kiểm chứng live, không cần sửa): `config/tags.yaml` đúng schema fork Interbotix; node `ar_tracker` khởi động sạch với `tag36h11`; tên service `/apriltag/snap_picture` + `/apriltag/single_image_tag_detection` khớp nhau; topic `/camera/camera/...` và frame `camera_color_optical_frame` khớp driver 4.58.3.

---

## 2. Chuỗi dữ liệu "Snap Pose" (đã kiểm chứng live)

```text
[GUI ArmTag Tuner]  (node /armtag/armtag_tuner_gui)
   │  bấm nút Snap Pose (num_samples mặc định 5)
   ▼
service /apriltag/snap_picture          (node /apriltag/picture_snapper)
   │  lưu ảnh hiện tại vào /tmp/get_image.png
   │  ⚠ service chỉ được tạo SAU khi node nhận được ảnh từ camera
   ▼
service /apriltag/single_image_tag_detection   (node /apriltag/ar_tracker — fork
   │  apriltag_ros của Interbotix, ~/apriltag_ws, tags.yaml = params)
   │  detect tag pose trong ảnh; intrinsics lấy từ CameraInfo trong request
   ▼
tag pose (Pose) theo frame của camera_info.header.frame_id (= camera_color_optical_frame)
   │  apriltag.py: trung bình num_samples mẫu (position + Euler RPY)
   │  armtag.py: T_RefBase = T_RefCam · T_CamTag · (arm_base→arm_tag_tag)⁻¹ …
   │             ↳ TF `arm_base_frame → arm_tag_frame` PHẢI có từ robot driver
   ▼
publish TransformStamped lên topic thường ❶ /static_transforms
   ▼
node /static_trans_pub  (interbotix_tf_tools — phải được launch riêng ❷)
   │  broadcast lên /tf_static  +  lưu static_transforms.yaml
   ▼
TF tree: camera_color_optical_frame → arm_base_frame
```

❶ `apriltag.py:103-107` — publisher topic `/static_transforms`, **không phải** `/tf`.
❷ Chỉ `xsarm_perception.launch.py` gốc của Trossen include `static_transform_pub.launch.py`; cả upstream `armtag.launch.py` lẫn wrapper cũ của ta đều không.

---

## 3. Chi tiết lỗi & bằng chứng

### 3.1 L1 — Snap Pose chết vì thiếu consumer `/static_transforms` (critical)

- `armtag.py:207`: `self.apriltag_inf.pub_transforms.publish(self.trans)` → topic `/static_transforms` (`apriltag.py:103-107`).
- Node duy nhất subscribe topic này: `static_trans_pub.py:101` (`interbotix_tf_tools`) → broadcast `/tf_static` (dòng 118) + lưu yaml (dòng 123-124).
- Wrapper cũ trả về đúng 3 action: camera + armtag + rviz — không có `static_transform_pub`.
- GUI vẫn in *"Successfully found and published transform."* (`armtag_tuner_gui.py:151`) → người dùng tưởng đã xong.

### 3.2 L2 — Thiếu robot TF → transform rác, vẫn báo success (high)

- `armtag.py:225-240`: `get_transform()` bắt `LookupException/Connectivity/Extrapolation` → log error → `return np.identity(4)`.
- `find_ref_to_arm_base_transform` (176-209) **không kiểm tra** kết quả: `T_CamBase = T_CamTag · I` = chính pose của tag → publish làm pose của base_link.
- Lệnh ghi trong header file cũ (`use_camera:=true`, không robot) guaranteed dính lỗi này.
- `rx150/ee_gripper_link` nằm sau các khớp quay → cần **cả driver lẫn joint_states** (chỉ robot_state_publisher thôi chưa đủ).

### 3.3 L3 — Xung đột topology TF (high, lộ ra ngay khi sửa L1)

- URDF bring-up của ta (MoveIt) có `world → rx150/base_link` (fixed).
- armtag mặc định cũ `arm_base_frame=rx150/base_link` → sau snap, `base_link` có **2 parent** (`world` và `camera_color_optical_frame`).
- Nếu `use_camera_static_tf:=true` (default của fuzzy_moveit) hoặc handeye publisher đang publish `world → camera_link` → tạo **vòng lặp** `world → camera_link → camera_color_optical_frame → world`.
- Hợp lệ với Trossen vì stack gốc chạy `use_world_frame:=false` (gốc URDF là `base_link`) — khác cấu trúc với hệ của ta.

### 3.4 L4 — realsense2_camera 4.58.3 đổi tên param profile (medium)

- Grep trực tiếp `/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py:37,44`: param là `rgb_camera.color_profile` và `depth_module.depth_profile`.
- Launch cũ truyền `rgb_camera.profile` / `depth_module.profile` (tên thời 4.5x sớm — vẫn được dùng trong launch file của chính Trossen) → `rs_launch.py` warn *"Parameter … is not supported"* rồi **bỏ im lặng**, driver tự chọn profile theo firmware.
- Hệ quả: không đảm bảo color/depth 640x480x30 như ý; ảnh/tag có thể ở resolution khác dự kiến.

### 3.5 L5 — RViz standalone sub sai topic (medium)

- `pointcloud_pipeline.cpp:438-441` (node ns `pc_filter`) publish: `pointcloud/objects`, `pointcloud/filtered`, `markers/objects` (**Marker**, không phải MarkerArray), `markers/crop_box`.
- rviz config cũ sub `/pc_filter/pointcloud_clusters` + MarkerArray `/pc_filter/object_markers` (tên kiểu ROS1) → hai display này không bao giờ có dữ liệu, dù pipeline chạy đúng.

### 3.6 L6 — `camera_info_topic` / `camera_frame` là launch args chết (medium, upstream)

- Upstream `apriltag.launch.py:63-78` chỉ truyền 2 args trên cho `picture_snapper`, node này chỉ declare `camera_color_topic`, `apriltag_ns`, `image_save_dir`.
- Node thực sự cần camera_info là `InterbotixAprilTagInterface` chạy trong process GUI: declare param tuyệt đối `/apriltag/camera_info_topic` với **default cứng** `/camera/camera/color/camera_info` (`apriltag.py:70-75`); frame lấy từ `camera_info.header.frame_id` (`apriltag.py:134`).
- Hiện chạy được vì default trùng hệ thống; đổi launch arg sẽ không có tác dụng — phải sửa param ở GUI node (upstream) hoặc giữ default.

### 3.7 L7 — Các lỗi nhỏ

| File | Vấn đề |
| ------ | -------- |
| `package.xml` | Thiếu `<exec_depend>rviz2</exec_depend>` dù 3 launch file chạy rviz2 |
| `config/sensors_3d.yaml` | Bản chết — bản sống do `fuzzy_moveit.launch.py:224` load từ `rx150_motion_common/config/sensors_3d.yaml` |
| `CMakeLists.txt` | `install(DIRECTORY launch config models …)` copy cả `launch/__pycache__` (bytecode cũ) vào share |
| `armtag.launch.py` (cũ) | RViz `-f camera_color_optical_frame` trong khi `use_camera` default false → tree trống |
| `perception.launch.py` | Header không ghi điều kiện chạy (fuzzy_moveit phải chạy camera trước); rviz không có `-f` |
| Header `armtag.launch.py` (cũ) | Lệnh mẫu `use_camera:=true` standalone → nếu fuzzy_moveit đang chạy sẽ 2 driver tranh D435i; và thiếu robot TF → dính L2 |

### 3.8 L8 — Ghi nhận upstream (không sửa trong phạm vi này)

- **Euler averaging** (`armtag.py:149-154`): trung bình từng thành phần RPY — sai khi góc quay qua ±π. Khắc phục thực tế: tăng Num Samples, giữ tag gần thẳng với camera, đối chiếu RPY hiển thị trên GUI với trực giác trước khi dùng.
- **`picture_snapper.py:97`**: `os.path.dirname(dir_param)` làm hỏng path nếu đổi `image_save_dir` thành chuỗi không có `/` cuối (mặc định `/tmp/` vẫn chạy đúng).
- **Tag `size`** = chiều dài cạnh ô đen (viền đen-trắng chung), theo README fork apriltag_ros — đo đúng tag in 30 mm, không đo cả viền trắng.

---

## 4. Phương án sửa (đã áp dụng — mọi sửa nằm trong `rx150_perception`, không đụng upstream)

| Lỗi | Sửa | File |
| ------ | ----- | ------ |
| L1 | Thêm include `interbotix_tf_tools/launch/static_transform_pub.launch.py` với 3 args mới: `load_static_transforms` (default **false** — tránh replay edge cũ), `save_transforms` (true), `static_transforms_path` (mặc định `config/static_transforms.yaml` của package) | `launch/armtag.launch.py` |
| L2 | Không sửa upstream; phòng tránh bằng: viết lại header usage (bắt buộc 2 terminal + preflight `tf2_echo`), ghi rõ hậu quả transform rác | `launch/armtag.launch.py` |
| L3 | Đổi default `arm_base_frame`: `rx150/base_link` → **`world`** (publish `camera_color_optical_frame → world`, không 2 parent); quy tắc mutex 3 nguồn TF ghi trong header + §5 | `launch/armtag.launch.py`, `launch/perception.launch.py` |
| L4 | Đổi `rgb_camera.profile` → `rgb_camera.color_profile`; `depth_module.profile` → `depth_module.depth_profile` (kèm comment) | `launch/armtag.launch.py`, `launch/standalone_perception.launch.py` |
| L5 | Display "Filtered PointCloud" → `/pc_filter/pointcloud/filtered`; "Object Markers" → class `rviz_default_plugins/Marker`, topic `/pc_filter/markers/objects` | `config/standalone_perception.rviz` |
| L6 | Comment đánh dấu 2 args là chết upstream (giữ cho tương thích API include) | `launch/armtag.launch.py` |
| L7 | `+<exec_depend>rviz2</exec_depend>`; xóa `config/sensors_3d.yaml`; `PATTERN "__pycache__" EXCLUDE` + xóa thư mục; rviz `-f world` (arg `rviz_frame`); header `perception.launch.py` ghi điều kiện chạy + tham chiếu runbook armtag | `package.xml`, `CMakeLists.txt`, `config/`, 2 launch file |

Sau khi sửa: `colcon build --packages-select rx150_perception`.

---

## 5. Runbook vận hành đúng

### 5.1 Calibration camera → arm bằng ArmTag (một lần)

```bash
# T1 — robot + camera + robot TF (tắt 2 nguồn static TF còn lại):
ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
    use_camera:=true use_camera_static_tf:=false use_handeye_publisher:=false

# Preflight (bắt buộc) — phải in ra số trước khi tiếp tục:
ros2 run tf2_ros tf2_echo rx150/base_link rx150/ee_gripper_link

# T2 — armtag (use_camera mặc định false vì T1 đã chạy camera):
ros2 launch rx150_perception armtag.launch.py use_armtag_tuner_gui:=true

# Trong T2 mở terminal khác: torque off, xoay tay cho camera thấy tag, torque on:
ros2 service call /rx150/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
    "{cmd_type: 'group', name: 'arm', enable: false}"
ros2 service call /rx150/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
    "{cmd_type: 'group', name: 'arm', enable: true}"

# Bấm 'Snap Pose' (Num Samples ~10). Kiểm tra:
ros2 run tf2_ros tf2_echo world camera_color_optical_frame   # phải ra số hợp lý

# Lưu kết quả lâu dài (file trong install share bị xóa khi rebuild):
cp $(ros2 pkg prefix rx150_perception)/share/rx150_perception/config/static_transforms.yaml \
   ~/interbotix_ws/src/rx150_perception/config/
```

### 5.2 Tái dùng TF đã lưu (không cần tag)

```bash
ros2 launch rx150_perception armtag.launch.py \
    use_armtag_tuner_gui:=false load_static_transforms:=true
```

### 5.3 Quy tắc mutex TF (bắt buộc)

Chỉ **MỘT** trong 3 nguồn static `world ↔ camera` được bật tại một thời điểm:

1. `static_transforms.yaml` qua armtag (`load_static_transforms:=true`)
2. `use_camera_static_tf:=true` (fuzzy_moveit — static TF đo tay)
3. `use_handeye_publisher:=true` (fuzzy_moveit — kết quả easy_handeye2)

Bật ≥2 nguồn cùng lúc → `multiple parents` / cycle → `tf2_echo` và MoveIt thất bại ngẫu nhiên.

### 5.4 Pointcloud pipeline

```bash
# T1: fuzzy_moveit.launch.py use_camera:=true   (điều kiện bắt buộc)
# T2:
ros2 launch rx150_perception perception.launch.py enable_pipeline:=true \
    use_pointcloud_tuner_gui:=true use_rviz:=true
# Topic kết quả: /pc_filter/pointcloud/filtered, /pc_filter/pointcloud/objects,
#               /pc_filter/markers/objects (Marker)
# Lấy cluster theo yêu cầu (tiết kiệm CPU): enable_pipeline:=false + gọi service
ros2 service call /pc_filter/get_cluster_positions interbotix_perception_msgs/srv/ClusterPositions ...
```

### 5.5 Standalone (không robot — chỉ xem camera/tag/pointcloud)

```bash
ros2 launch rx150_perception standalone_perception.launch.py
```

---

## 6. Còn lại / rủi ro đã biết

- **`arm_tag_frame=rx150/ee_gripper_link` là xấp xỉ**: sai số = độ lệch vật lý giữa tag dán và gốc frame (đòn bẩy theo khoảng cách tag–gripper). Option chính xác hơn: bật `show_ar_tag:=true` trong robot description (fuzzy_moveit) để có `rx150/ar_tag_link` đúng vị trí mount rồi đổi default.
- **Transform rác vẫn có thể xảy ra** (L2) nếu quên preflight — upstream chưa sửa; luôn chạy `tf2_echo` trước khi Snap.
- **Chuỗi profile realsense** `'640x480x30'`: cần xác nhận với camera thật (nếu bị từ chối, thử `'640,480,30'` — format của default `0,0,0`).
- **static_transforms.yaml trong install share** bị xóa mỗi lần `colcon build` — nhớ copy về src (§5.1) hoặc truyền `static_transforms_path:=` trỏ ra ngoài install.
- **L6 (args chết)**: nếu sau này đổi namespace/topic camera_info, phải sửa ở upstream hoặc set param cho node GUI — launch arg của wrapper sẽ không tác dụng.
