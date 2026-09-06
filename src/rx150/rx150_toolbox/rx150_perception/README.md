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
| `launch/handeye_calibrate.launch.py` | Phiên hiệu chuẩn easy_handeye2 |
| `launch/handeye_publish.launch.py` | Phát TF đã hiệu chuẩn |
| `launch/standalone_perception.launch.py` | Chạy nhận diện không cần robot |
| `config/yolo_detector_params.yaml` | `imgsz`, `iou`, cổng lọc theo độ sâu, tracker EMA |
| `config/static_transforms.yaml` | TF camera ↔ đế, kết quả của phiên hiệu chuẩn |
| `models/best.pt`, `models/best_color.pt` | Trọng số YOLO (~23 MB mỗi file) |
| `test/test_yolo_tube_detector.py` | 14 test offline, không cần camera |
| `demos/` | Script minh hoạ chạy tay (sắp ống theo màu, kiểm tra hướng ống) |

## Chạy

```bash
./rx150.sh t2       # perception thường ngày: nạp TF hiệu chuẩn + RViz
./rx150.sh calib    # phiên hiệu chuẩn ArmTag (Snap Pose)
./rx150.sh tune     # tune PCL — cần T1 đang chạy ở chế độ t1-pcl
./rx150.sh test     # 14 test offline, ~5 s
```

## Phụ thuộc pip

`ultralytics`, `mediapipe`, `torch` không có trong rosdep — cài riêng:

```bash
pip install --user -r $(ros2 pkg prefix rx150_perception)/share/rx150_perception/requirements.txt
```

> ⚠️ **Hai cái bẫy đã trả giá để biết:** (1) `numpy` phải ghim 1.26.4, bản 2.x làm hỏng
> OpenCV của apt. (2) `torch` phải khớp driver NVIDIA — lệch một cái là ultralytics **im
> lặng** rơi về CPU, 68 ms/frame thay vì 5.7 ms, và máy tưởng như treo. Xem
> [PERCEPTION_GUIDE.md](docs/PERCEPTION_GUIDE.md).
