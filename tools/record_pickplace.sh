#!/usr/bin/env bash
# record_pickplace.sh — ghi ros2 bag ĐỦ CẢ NĂM TẦNG cho một lần chạy pick-place.
#
# Dùng:  ./tools/record_pickplace.sh [nhãn]        (mặc định: pickplace)
#        ./tools/record_pickplace.sh b5a-moveit
#        ./tools/record_pickplace.sh b5b-direct
#        Ctrl+C để dừng.
#
# Khác với rx150_fuzzy_controller/scripts/rx150_fuzzy_record.sh (chỉ 5 topic của
# tầng điều khiển): bản này thêm tầng nhận diện + tầng quyết định + TF, nên một
# bag đủ trả lời "tay đi sai vì planner, vì bám kém, hay vì detect sai?".
#
# Mở lại:
#   ros2 run plotjuggler plotjuggler -l ~/interbotix_ws/data_analysis/layouts/fuzzy_plotjuggler_layout.xml
#   → Data → Load → ROS2 bag. So /rx150/fuzzy/reference (đặt) với
#     /rx150/joint_states (thực): lệch = bám kém; cả hai cùng sai = kế hoạch sai.
#
# Bag tự thêm timestamp nên không bao giờ ghi đè lần chạy trước.
set -euo pipefail

PREFIX="${1:-pickplace}"
TS="$(date +%Y%m%d_%H%M%S)"
NAME="${PREFIX}_${TS}"

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="$WS/bags"
mkdir -p "$DATA_DIR"

# TF phải có: pose gắp nằm trong rx150/base_link, muốn dựng lại cảnh thì cần cây TF.
# /tf_static ghi transient_local nên ros2 bag bắt được bản latch lúc bắt đầu.
TOPICS=(
  # tầng phần cứng / điều khiển
  /rx150/joint_states
  /rx150/fuzzy/reference
  /rx150/fuzzy/error
  /rx150/fuzzy/edot
  /rx150/fuzzy/effort
  /rx150/commands/joint_group
  /rx150/commands/joint_single
  # tầng nhận diện
  /yolo/detected_tubes
  /yolo/tube_classes
  # tầng quyết định
  /tube_rack/status
  /pick_place_moveit/status
  /hand_gesture/selected_target
  /hand_gesture/event
  # hình học
  /tf
  /tf_static
)

echo "📦 Ghi bag: $DATA_DIR/$NAME"
echo "   ${#TOPICS[@]} topic — Ctrl+C để dừng."
echo
echo "   ⚠️  KHÔNG ghi ảnh/pointcloud (quá lớn). Cần ảnh thì thêm tay:"
echo "       ros2 bag record /camera/camera/color/image_raw -o <tên>"
echo

# Topic chưa tồn tại lúc bắt đầu vẫn được bắt khi nó xuất hiện.
exec ros2 bag record -o "$DATA_DIR/$NAME" "${TOPICS[@]}"
