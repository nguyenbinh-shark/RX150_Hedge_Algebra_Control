#!/usr/bin/env bash
# rx150.sh — Chạy hệ RX150 với đúng tổ hợp cờ tối ưu cho từng mục đích.
#
# Dùng:   ./rx150.sh <chế_độ>
#
#   t1        Robot + MoveIt + camera (HẰNG NGÀY — không point cloud, không MoveIt-RViz)
#   t1-pcl    Như t1 nhưng BẬT point cloud (chỉ khi cần luồng PCL / OctoMap)
#   t2        Perception thường ngày: nạp TF calib (+ RViz)
#   calib     Phiên HIỆU CHUẨN ArmTag: armtag GUI + RViz (không cần point cloud)
#   tune      Phiên TUNE PCL: pointcloud tuner GUI + RViz (T1 phải là t1-pcl)
#   tubes     Gắp ống nghiệm (YOLO, Layer 2)
#   dry       Gắp ống nghiệm CHẠY THỬ: chỉ IK + log, không cử động
#   test      Bộ test tổng hợp perception (14 test, ~5s)
#
# Nguyên tắc tối ưu:
#   - Point cloud (~295 MB/s) chỉ bật khi thật sự dùng (t1-pcl + tune).
#   - MỘT RViz duy nhất (bên perception, chạy trên GPU NVIDIA); MoveIt-RViz tắt.
#   - Tuner GUI chỉ mở trong phiên tune/calib rồi TẮT — InterbotixRobotNode rò
#     timer 20Hz theo mỗi lần kéo slider, để GUI mở lâu là CPU tăng dần.
set -e
cd "$(dirname "$0")"
source ./source_all.sh >/dev/null

case "${1:-}" in
  t1)
    echo "⚠️  Tay máy sẽ BẬT TORQUE và tự về HOME khi launch. Dọn chỗ quanh robot."
    exec ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
        use_camera_static_tf:=false \
        use_moveit_rviz:=false
    ;;
  t1-pcl)
    echo "⚠️  Tay máy sẽ BẬT TORQUE và tự về HOME khi launch. Point cloud BẬT (~295 MB/s)."
    exec ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
        use_camera_static_tf:=false \
        use_moveit_rviz:=false \
        rs_camera_pointcloud_enable:=true
    ;;
  t2)
    exec ros2 launch rx150_perception rx150_perception.launch.py use_rviz:=true
    ;;
  calib)
    echo "Hiệu chuẩn: đưa AprilTag vào khung hình, chỉnh Snapshots=10, bấm Snap Pose."
    echo "Xong thì Ctrl+C và chạy lại './rx150.sh t2' (đừng để GUI mở lâu)."
    exec ros2 launch rx150_perception rx150_perception.launch.py \
        use_armtag_tuner_gui:=true \
        use_rviz:=true
    ;;
  tune)
    echo "Tune PCL: T1 phải đang chạy './rx150.sh t1-pcl' (cần point cloud)."
    echo "Xong thì bấm Save Config, Ctrl+C và quay về './rx150.sh t2'."
    exec ros2 launch rx150_perception rx150_perception.launch.py \
        use_pointcloud_tuner_gui:=true \
        use_rviz:=true
    ;;
  tubes)
    exec ros2 launch rx150_pick_place tube_rack.launch.py
    ;;
  dry)
    exec ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
    ;;
  test)
    exec ros2 run rx150_perception test_yolo_tube_detector.py
    ;;
  *)
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
