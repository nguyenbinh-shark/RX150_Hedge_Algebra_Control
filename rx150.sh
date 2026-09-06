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
#   tubes-direct  Như tubes nhưng KHÔNG qua move_group (motion_backend:=direct)
#   gesture   Gắp vật được chỉ bằng cử chỉ tay (pick_place + hand_gesture)
#   dry       Gắp ống nghiệm CHẠY THỬ: chỉ IK + log, không cử động
#   dry-fake  Như dry nhưng bơm 1 ống GIẢ — chạy hết chuỗi khi KHÔNG có camera
#   all       T1+T2+YOLO trong MỘT terminal (fuzzy_moveit_perception)
#   test      Bộ test tổng hợp perception (14 test, ~5s)
#   check     Smoke-test theo tầng: joint_states + action server + TF/detection
#   reach     Bảng tầm với + kiểm config (KHÔNG cần robot)
#   diag      Chụp trạng thái để gửi kèm báo lỗi -> diag_<ts>.tar.gz
#
# Thứ tự bring-up (chi tiết: src/rx150_pick_place/docs/RUNBOOK.md):
#   reach → t1 → t2 → check → dry-fake → tubes
#   Hỏng ở bậc nào thì DỪNG ở bậc đó, đừng chạy tiếp.
#
# Nguyên tắc tối ưu:
#   - Point cloud (~295 MB/s) chỉ bật khi thật sự dùng (t1-pcl + tune).
#   - MỘT RViz duy nhất (bên perception, chạy trên GPU NVIDIA); MoveIt-RViz tắt.
#   - Tuner GUI chỉ mở trong phiên tune/calib rồi TẮT — InterbotixRobotNode rò
#     timer 20Hz theo mỗi lần kéo slider, để GUI mở lâu là CPU tăng dần.
#   - MỘT yolo_tube_detector duy nhất: 'all' đã có sẵn nên task phải detector:=false;
#     đường t1→t2→tubes thì không dính (rx150_perception KHÔNG chạy YOLO).
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
  tubes-direct)
    echo "⚠️  motion_backend=direct: KHÔNG có tránh vật cản (không qua move_group)."
    echo "    An toàn nằm ở waypoint. Mỗi thao tác planning scene sẽ treo ~3 s rồi WARN"
    echo "    nếu move_group không chạy — đó là bình thường, xem RUNBOOK B5b."
    exec ros2 launch rx150_pick_place tube_rack.launch.py motion_backend:=direct
    ;;
  gesture)
    exec ros2 launch rx150_pick_place pick_place.launch.py
    ;;
  dry)
    exec ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
    ;;
  dry-fake)
    # detector:=false + ống giả ⇒ chạy hết state machine mà không cần camera lẫn
    # động cơ. Đây là bậc B4 của RUNBOOK — chỗ bắt lỗi hình học rẻ nhất.
    exec ros2 launch rx150_pick_place tube_rack.launch.py \
        detector:=false dry_run:=true \
        fake_tubes:='[{"x":0.20,"y":-0.15,"z":0.03,"yaw":0.0,"class":"pink"}]'
    ;;
  all)
    # Một terminal: fuzzy_moveit + perception + YOLO. Point cloud tắt (295 MB/s,
    # thuần hiển thị). Task chạy ở terminal khác và PHẢI có detector:=false —
    # launch này đã có yolo_tube_detector rồi, hai node trùng tên là hỏng.
    echo "⚠️  Tay máy sẽ BẬT TORQUE khi launch. Dọn chỗ quanh robot."
    echo "    Task ở terminal khác PHẢI thêm detector:=false (đã có YOLO ở đây)."
    exec ros2 launch rx150_fuzzy_controller fuzzy_moveit_perception.launch.py \
        rs_camera_pointcloud_enable:=false
    ;;
  test)
    exec ros2 run rx150_perception test_yolo_tube_detector.py
    ;;
  check)
    # Không exec: chạy cả ba rồi tổng kết. Không bài nào phát lệnh tới robot.
    rc=0
    for t in hardware/joint_states_test.py \
             moveit/action_servers_test.py \
             perception/tf_and_detection_test.py; do
      echo
      echo "════════ $t ════════"
      python3 module_tests/run_test.py "$t" || rc=1
    done
    echo
    [ $rc -eq 0 ] && echo "✅ Tất cả smoke-test GO." \
                  || echo "❌ Có smoke-test NO-GO — xem nguyên nhân gốc bên trên."
    exit $rc
    ;;
  reach)
    # Không cần robot. Chạy trước MỌI lần đo lại vị trí giá/bàn.
    # rx150_reach_check trả 1 khi có điểm ngoài tầm — đó là KẾT QUẢ, không phải
    # lỗi script, nên phải chặn `set -e` để còn chạy hết các config.
    rc=0
    ros2 run rx150_pick_place rx150_reach_check.py || rc=1
    share="$(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config"
    # --no-envelope: bảng bao hình đã in ở trên, không lặp lại 3 lần.
    for f in tube_rack_params pick_place_params; do
      echo
      echo "════════ $f.yaml ════════"
      ros2 run rx150_pick_place rx150_reach_check.py \
        --config "$share/$f.yaml" --no-envelope || rc=1
    done
    echo
    echo "════════ tư thế trung chuyển (home_xyz_pitch) ════════"
    echo "IK điểm này FAIL ⇒ resolve_home() âm thầm quay lại home duỗi thẳng"
    echo "[0,0,0,0,0] (FK = 0.359,0,0.255) — quét ngang QUA giá ở x=0.26."
    ros2 run rx150_pick_place rx150_reach_check.py \
      --point 0.18 0 0.18 --pitch 0 --no-envelope || rc=1
    echo
    [ $rc -eq 0 ] && echo "✅ Mọi điểm trong config đều với tới." \
                  || echo "❌ Có điểm NGOÀI tầm với — sửa config trước khi cấp điện."
    exit $rc
    ;;
  diag)
    shift
    exec ./tools/collect_diag.sh "$@"
    ;;
  *)
    # In khối comment đầu file cho tới dòng trống đầu tiên sau danh sách chế độ.
    sed -n '2,27p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
