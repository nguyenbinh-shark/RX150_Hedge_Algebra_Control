#!/usr/bin/env bash
# Record ros2 bag 5 topic fuzzy để so sánh A/B (step vs profile) và replay trong PlotJuggler.
# Dùng:  ./config/fuzzy_record.sh [tên_prefix]   (mặc định: fuzzy_run)
# VD:    ./config/fuzzy_record.sh step           (chạy enable_profile:=false)
#        ./config/fuzzy_record.sh profile        (chạy enable_profile:=true)
# Dừng: Ctrl+C -> PJ: Data -> Load -> ROS2 bag mở cả 'step' và 'profile' để overlay.
#
# Bag tự thêm timestamp vào tên để không ghi đè lần chạy trước.
# VD: fuzzy_run_20260813_133000/
set -euo pipefail
PREFIX="${1:-fuzzy_run}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
NAME="${PREFIX}_${TIMESTAMP}"

# Tạo thư mục data/ nếu chưa có (gom bag về 1 chỗ)
DATA_DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
mkdir -p "$DATA_DIR"

echo "📦 Recording to: $DATA_DIR/$NAME"
ros2 bag record -o "$DATA_DIR/$NAME" \
  /rx150/joint_states \
  /rx150/fuzzy/reference \
  /rx150/fuzzy/effort \
  /rx150/fuzzy/error \
  /rx150/fuzzy/edot
