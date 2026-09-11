# rx150_perception — nhận diện, camera và hiệu chuẩn hand-eye

**Tầng IRROS:** Application Support · **Ngôn ngữ:** Python

Toàn bộ phần "nhìn" của hệ RX150: RealSense D435i, YOLO segmentation cho ống nghiệm, cử
chỉ tay, lọc point cloud theo chuẩn Interbotix, và hiệu chuẩn hand-eye bằng AprilTag.
Package này **chỉ phát hiện và công bố toạ độ** — quyết định gắp gì, gắp thế nào là việc
của tầng Application ([`apps/`](../../apps/README.md)).

Tài liệu đầy đủ (hiệu chuẩn, tune PCL, tham số YOLO): **[docs/PERCEPTION_GUIDE.md](docs/PERCEPTION_GUIDE.md)**

## Cấu trúc

| Đường dẫn | Nội dung |
| :--- | :--- |
| `scripts/yolo_tube_detector_node.py` | YOLO-seg → `/yolo/detected_tubes` (PoseArray): tâm, hướng, lớp màu |
| `scripts/hand_gesture_node.py` | MediaPipe → cử chỉ tay + điểm chỉ 3D |
| `launch/rx150_perception.launch.py` | Camera + pc_filter + armtag + static_trans_pub + RViz |
| `launch/armtag.launch.py` | Chỉ nhánh AprilTag |
| `launch/ee_tag.launch.py` | Detector AprilTag **liên tục 30 fps** cho tag dán trên tay gắp (nguồn đo ngoài của `rx150_ee_tag_bench.py`) |
| `scripts/rack_calib_node.py` | **Hiệu chuẩn vị trí GIÁ ĐỠ ống nghiệm** bằng AprilTag → `rack_pose.yaml` |
| `launch/rack_calib.launch.py` | Phiên hiệu chuẩn giá: detector tag giá + `rack_calib` (`mode:=snap\|watch\|publish`) |
| `launch/handeye_calibrate.launch.py` | Phiên hiệu chuẩn easy_handeye2 |
| `launch/handeye_publish.launch.py` | Phát TF đã hiệu chuẩn |
| `launch/standalone_perception.launch.py` | Chạy nhận diện không cần robot |
| `config/yolo_detector_params.yaml` | `imgsz`, `iou`, cổng lọc theo độ sâu, tracker EMA |
| `config/static_transforms.yaml` | TF camera ↔ đế, kết quả của phiên hiệu chuẩn **camera** |
| `config/rack_tag.yaml` | id + size của AprilTag dán trên giá (detector liên tục) |
| `config/rack_calib.yaml` | Hình học giá trong khung tag: 4 lỗ ở 4 đỉnh 120 × 50 mm, tag ở trung điểm cạnh dài gần |
| `config/rack_pose.yaml` | 4 miệng lỗ + trục lỗ trong `rx150/base_link` — **node tự ghi**, `tube_rack_node` đọc |
| `rviz/rack_calib.rviz` | Khung 3D (marker 4 lỗ) + khung ảnh chồng hình, cho `./rx150.sh rack-gui` |
| `models/best.pt`, `models/best_color.pt` | Trọng số YOLO (~23 MB mỗi file) |
| `test/test_yolo_tube_detector.py` | 14 test offline, không cần camera |
| `test/test_rack_calib.py` | Chốt quy ước trục của hiệu chuẩn giá bằng số (không cần camera) |
| `demos/` | Script minh hoạ chạy tay (sắp ống theo màu, kiểm tra hướng ống) |

## Chạy

```bash
./rx150.sh t2          # perception thường ngày: nạp TF hiệu chuẩn + RViz
./rx150.sh calib       # phiên hiệu chuẩn CAMERA (ArmTag Snap Pose)
./rx150.sh rack-calib  # phiên hiệu chuẩn VỊ TRÍ GIÁ đỡ ống nghiệm
./rx150.sh rack-gui    # KIỂM BẰNG MẮT: RViz + 4 lỗ chiếu ngược lên ảnh camera
./rx150.sh rack-watch  # giá có bị xê dịch không (đo, không ghi đè)
./rx150.sh rack-pub    # phát TF tube_rack + 4 slot để soi trong RViz
./rx150.sh tune     # tune PCL — cần T1 đang chạy ở chế độ t1-pcl
./rx150.sh test     # 14 test offline, ~5 s
```

## Hai phép hiệu chuẩn, cùng một khuôn

| | đo cái gì | công cụ | file kết quả | ai đọc kết quả |
|---|---|---|---|---|
| Camera | camera đứng ở đâu so với đế | armtag GUI (`calib`) | `static_transforms.yaml` | `static_trans_pub` → TF |
| **Giá** | 4 miệng lỗ nằm ở đâu | `rack_calib` (`rack-calib`) | `rack_pose.yaml` | `tube_rack_node` (đọc thẳng file) |

Phép thứ hai **nằm trên nền** phép thứ nhất: TF camera sai thì `rack_pose.yaml` chỉ chép
lại đúng cái sai đó.

**Bản đồ id tag** (cùng một tấm tag vật lý ⇒ đổi một chỗ là phải đổi đủ):

| id | Tag | Khai trong |
|---|---|---|
| **1** | tay gắp (`rx150/ar_tag_link`) | `tags.yaml`, `apriltag_calib.yaml`, `ee_tag.yaml` |
| **0** | giá đỡ ống nghiệm | `rack_tag.yaml`, `rack_calib.yaml` (`tag_id`) |

Mỗi detector chỉ khai id của mình nên không bao giờ báo cáo tag của bên kia. Điều này
cũng giữ an toàn cho armtag: nó lấy `detections[0]` mà **không** lọc theo id. Chi tiết + cách đọc log NO-GO: [RUNBOOK §B2b](../../apps/rx150_pick_place/docs/RUNBOOK.md).

## Phụ thuộc pip

`ultralytics`, `mediapipe`, `torch` không có trong rosdep — cài riêng:

```bash
pip install --user -r $(ros2 pkg prefix rx150_perception)/share/rx150_perception/requirements.txt
```

> ⚠️ **Hai cái bẫy đã trả giá để biết:** (1) `numpy` phải ghim 1.26.4, bản 2.x làm hỏng
> OpenCV của apt. (2) `torch` phải khớp driver NVIDIA — lệch một cái là ultralytics **im
> lặng** rơi về CPU, 68 ms/frame thay vì 5.7 ms, và máy tưởng như treo. Xem
> [PERCEPTION_GUIDE.md](docs/PERCEPTION_GUIDE.md).
