#!/usr/bin/env bash
# setup_vendor.sh — kéo vendor Interbotix + third-party về src/vendor/.
#
# Chạy sau mỗi lần clone mới, TRƯỚC khi colcon build:
#     ./tools/setup_vendor.sh
#
# Vì sao cần script chứ không chỉ `vcs import`: interbotix_ros_core và
# interbotix_ros_toolboxes có SUBMODULE LỒNG (interbotix_xs_driver,
# dynamixel_workbench_toolbox, trossen_slate, ModernRobotics). `vcs import` clone
# repo cha nhưng để trống các thư mục đó → interbotix_xs_sdk build fail vì thiếu
# header. Bước `submodule update --init --recursive` checkout đúng SHA mà commit
# cha đã pin, nên vẫn tái lập được y hệt.
set -euo pipefail
cd "$(dirname "$0")/.."

command -v vcs >/dev/null || {
  echo "Thiếu vcstool. Cài: sudo apt install python3-vcstool" >&2; exit 1; }

mkdir -p src/vendor
echo "[1/2] vcs import src/vendor < rx150.repos"
vcs import src/vendor < rx150.repos

echo "[2/2] submodule update --init --recursive"
vcs custom src/vendor --git --args submodule update --init --recursive

echo
vcs status src/vendor --nested
echo "✅ vendor sẵn sàng. Tiếp: colcon build --symlink-install"
