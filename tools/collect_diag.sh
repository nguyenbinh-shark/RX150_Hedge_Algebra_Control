#!/usr/bin/env bash
# collect_diag.sh — chụp TOÀN BỘ trạng thái ROS + hệ thống vào một thư mục, rồi tar.
#
# Dùng:  ./tools/collect_diag.sh [nhãn]
#        ./tools/collect_diag.sh b5a-home-fail
#
# Chạy được cả khi stack KHÔNG chạy (mỗi mục ghi "không tìm thấy") lẫn khi đang
# chạy. Không bao giờ tự thoát giữa chừng: mỗi bước có timeout riêng và lỗi của
# bước này không chặn bước sau.
#
# Kết quả: diag_<nhãn>_<timestamp>.tar.gz ở thư mục hiện tại — đó là thứ gửi đi
# khi báo lỗi. Xem docs/RUNBOOK.md §6.

set -uo pipefail          # KHÔNG dùng -e: một lệnh ros2 fail không được giết script

LABEL="${1:-diag}"
TS="$(date +%Y%m%d_%H%M%S)"
OUT="diag_${LABEL}_${TS}"
mkdir -p "$OUT"

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Mỗi lệnh có trần thời gian riêng: `ros2 topic hz` không bao giờ tự dừng, và
# `ros2 service list` treo vô hạn khi không có node nào.
run() {                    # run <tên file> <giây> <lệnh...>
  local name="$1" secs="$2"; shift 2
  local f="$OUT/$name"
  { echo "\$ $*"; echo; } > "$f"
  if timeout --signal=INT "$secs" "$@" >> "$f" 2>&1; then
    echo "  ✓ $name"
  else
    local rc=$?
    # 124/130 = hết giờ; với `topic hz` đó là hành vi BÌNH THƯỜNG.
    if [ $rc -eq 124 ] || [ $rc -eq 130 ]; then
      echo "  ⏱ $name (hết $secs s — bình thường với hz/echo)"
    else
      echo "  ✗ $name (rc=$rc)"
      echo "[collect_diag] lệnh thất bại rc=$rc" >> "$f"
    fi
  fi
}

echo "📋 Thu thập vào $OUT/"

# ── 0. Bối cảnh: phiên bản mã + môi trường ───────────────────────────────
{
  echo "timestamp     : $TS"
  echo "label         : $LABEL"
  echo "hostname      : $(hostname)"
  echo "ROS_DISTRO    : ${ROS_DISTRO:-<chưa source>}"
  echo "RMW           : ${RMW_IMPLEMENTATION:-<mặc định>}"
  echo "ROS_DOMAIN_ID : ${ROS_DOMAIN_ID:-<mặc định 0>}"
  echo
  echo "--- AMENT_PREFIX_PATH ---"
  echo "${AMENT_PREFIX_PATH:-<trống>}" | tr ':' '\n'
  echo
  echo "--- PYTHONPATH ---"
  echo "${PYTHONPATH:-<trống>}" | tr ':' '\n'
  echo
  echo "--- cảnh báo overlay ---"
  case "${AMENT_PREFIX_PATH:-}" in
    *ws_moveit*)
      echo "⚠️⚠️  ~/ws_moveit ĐANG nằm trong overlay — nó che MoveIt hệ thống."
      echo "     Thường là do shell CHA (terminal khởi động IDE) đã chạy alias 'moveit',"
      echo "     rồi truyền env xuống. .bashrc KHÔNG source nó — kiểm bằng:"
      echo "       env -i HOME=\$HOME bash -lc 'ros2 pkg prefix moveit_ros_move_group'"
      echo "     Khắc phục: chạy stack robot từ terminal MỚI hoàn toàn."
      ;;
    *) echo "ok: ~/ws_moveit không nằm trong overlay." ;;
  esac
  case "${AMENT_PREFIX_PATH:-}" in
    *easy_handeye2_ws*) echo "ok: easy_handeye2_ws đã source." ;;
    *)                  echo "⚠️  easy_handeye2_ws CHƯA source (dùng source_all.sh)." ;;
  esac
  echo
  echo "--- move_group sẽ chạy từ đâu (PHẢI là /opt/ros/humble) ---"
  ros2 pkg prefix moveit_ros_move_group 2>&1 || echo "(không tra được)"
} > "$OUT/00_env.txt"
echo "  ✓ 00_env.txt"

{
  echo "--- git HEAD ---"
  git -C "$WS" rev-parse HEAD 2>&1
  git -C "$WS" log --oneline -5 2>&1
  echo
  echo "--- git status --short ---"
  git -C "$WS" status --short 2>&1
  echo
  echo "--- diff --stat (thay đổi CHƯA commit) ---"
  git -C "$WS" diff --stat 2>&1
} > "$OUT/01_git.txt"
echo "  ✓ 01_git.txt"

# ── 1. Phần cứng ────────────────────────────────────────────────────────
{
  echo "--- U2D2 / serial ---"
  ls -l /dev/ttyDXL /dev/ttyUSB* /dev/ttyACM* 2>&1
  echo
  echo "--- tiến trình xs_sdk (>1 dòng = TRANH CHẤP BUS) ---"
  pgrep -af xs_sdk 2>&1 || echo "(không có xs_sdk nào đang chạy)"
  echo
  echo "--- USB ---"
  lsusb 2>&1
  echo
  echo "--- GPU ---"
  nvidia-smi 2>&1 || echo "(không có nvidia-smi)"
  echo
  echo "--- hand-eye calib ---"
  ls -l "$HOME/.ros/easy_handeye2/" 2>&1 || echo "(chưa calib easy_handeye2)"
  echo
  echo "--- tải máy ---"
  uptime 2>&1
  free -h 2>&1
} > "$OUT/02_hardware.txt"
echo "  ✓ 02_hardware.txt"

# ── 2. Danh mục ROS ─────────────────────────────────────────────────────
if ! command -v ros2 >/dev/null 2>&1; then
  echo "⚠️  không có lệnh ros2 — bỏ qua phần ROS. Hãy 'source source_all.sh'."
  echo "không có lệnh ros2 trong PATH" > "$OUT/03_NO_ROS.txt"
else
  run 03_nodes.txt      15 ros2 node list
  run 04_topics.txt     15 ros2 topic list -t
  run 05_actions.txt    15 ros2 action list -t
  run 06_services.txt   20 ros2 service list -t

  # ── 3. Nhịp dữ liệu — hz tự dừng bằng timeout, đó là cách dùng đúng ──
  run 10_hz_joint_states.txt   8 ros2 topic hz /rx150/joint_states
  run 11_hz_detections.txt    10 ros2 topic hz /yolo/detected_tubes
  run 12_hz_color.txt          8 ros2 topic hz /camera/camera/color/image_raw
  run 13_hz_depth.txt          8 ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw

  # ── 4. Nội dung mẫu ─────────────────────────────────────────────────
  run 20_joint_states.txt      8 ros2 topic echo --once /rx150/joint_states
  run 21_detections.txt       10 ros2 topic echo --once /yolo/detected_tubes
  run 22_tube_classes.txt     10 ros2 topic echo --once /yolo/tube_classes
  run 23_status_tube_rack.txt  8 ros2 topic echo --once /tube_rack/status
  run 24_status_pick_place.txt 8 ros2 topic echo --once /pick_place_moveit/status

  # ── 5. TF — nguyên nhân số 1 của "YOLO không nhận được gì" ───────────
  run 30_tf_cam_to_base.txt    8 ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
  run 31_tf_world_to_base.txt  8 ros2 run tf2_ros tf2_echo world rx150/base_link
  run 32_tf_cam_to_world.txt   8 ros2 run tf2_ros tf2_echo camera_color_optical_frame world
  run 33_tf_static.txt         8 ros2 topic echo --once /tf_static

  # frames.pdf/gv rơi vào thư mục hiện tại → chạy trong $OUT rồi quay ra
  ( cd "$OUT" && timeout --signal=INT 20 ros2 run tf2_tools view_frames \
      > 34_view_frames.txt 2>&1 ) \
    && echo "  ✓ 34_view_frames.txt (+ frames.pdf)" \
    || echo "  ⏱ 34_view_frames.txt"

  # ── 6. Tham số các node then chốt ───────────────────────────────────
  run 40_params_yolo.txt        15 ros2 param dump /yolo_tube_detector
  run 41_params_tube_rack.txt   15 ros2 param dump /tube_rack
  run 42_params_pick_place.txt  15 ros2 param dump /pick_place_moveit
  run 43_params_fuzzy.txt       15 ros2 param dump /rx150/fuzzy_node
fi

# ── 7. Config đang thực sự được dùng (bản ĐÃ CÀI, không phải src) ───────
for f in \
  "$WS/install/rx150_pick_place/share/rx150_pick_place/config/tube_rack_params.yaml" \
  "$WS/install/rx150_pick_place/share/rx150_pick_place/config/pick_place_params.yaml" \
  "$WS/install/rx150_perception/share/rx150_perception/config/static_transforms.yaml" \
  "$WS/install/rx150_perception/share/rx150_perception/config/roi_box_params.yaml" \
  "$WS/install/rx150_perception/share/rx150_perception/config/yolo_detector_params.yaml" \
  "$WS/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/controllers/rx150_controllers.yaml"
do
  [ -e "$f" ] && cp -L "$f" "$OUT/cfg_$(basename "$f")" 2>/dev/null
done
echo "  ✓ cfg_*.yaml (bản đã cài)"

# ── 8. Log ROS mới nhất ─────────────────────────────────────────────────
LATEST_LOG="$(ls -dt "$HOME"/.ros/log/*/ 2>/dev/null | head -1)"
if [ -n "$LATEST_LOG" ]; then
  echo "$LATEST_LOG" > "$OUT/50_latest_log_dir.txt"
  # Chỉ lấy stdout của từng node, bỏ file lớn — đủ để đọc, không phình tar.
  find "$LATEST_LOG" -name 'stdout.log' -size -5M \
       -exec cp --parents {} "$OUT/" \; 2>/dev/null
  echo "  ✓ log node từ $LATEST_LOG"
else
  echo "(không có ~/.ros/log)" > "$OUT/50_latest_log_dir.txt"
fi

# ── 9. Đóng gói ─────────────────────────────────────────────────────────
tar czf "${OUT}.tar.gz" "$OUT" 2>/dev/null
echo
echo "✅ Xong: ${OUT}.tar.gz  ($(du -sh "${OUT}.tar.gz" | cut -f1))"
echo "   Thư mục còn nguyên ở $OUT/ nếu bạn muốn xem trước khi gửi."
