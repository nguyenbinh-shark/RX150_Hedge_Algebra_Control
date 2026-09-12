#!/usr/bin/env bash
# build.sh — colcon build cho workspace này, đã gỡ sẵn hai cái bẫy của máy.
#
# Dùng:  ./tools/build.sh                       # build tất cả
#        ./tools/build.sh --packages-select rx150_modules
#        ./tools/build.sh --packages-up-to rx150_pick_place
# (mọi tham số truyền vào đều được chuyển thẳng cho colcon)
#
# Bẫy 1 — setuptools vs packaging:
#   ~/.local có setuptools 84, apt có packaging 21.3. Từ setuptools 71 trở đi nó
#   không vendor packaging nữa mà gọi thẳng canonicalize_version(strip_trailing_zero=…),
#   kwarg chỉ có từ packaging 24 → MỌI package dùng ament_python_install_package
#   build hỏng (interbotix_xs_msgs, interbotix_common_modules, …).
#   PYTHONNOUSERSITE=1 làm Python bỏ qua ~/.local, dùng cặp setuptools 59.6.0 +
#   packaging 21.3 của apt vốn khớp nhau. Không phải cài/gỡ gì trên máy.
#
# Bẫy 2 — overlay:
#   Phải source đủ 4 overlay TRƯỚC khi build, nếu không apriltag_ros và
#   easy_handeye2 sẽ không tìm thấy.
set -eo pipefail
cd "$(dirname "$0")/.."

# KHÔNG dùng `set -u`: /opt/ros/humble/setup.bash tham chiếu AMENT_TRACE_SETUP_FILES
# khi biến này chưa được đặt, gặp `set -u` là thoát ngay.
source /opt/ros/humble/setup.bash
[ -f "$HOME/apriltag_ws/install/setup.bash" ]      && source "$HOME/apriltag_ws/install/setup.bash"
[ -f "$HOME/easy_handeye2_ws/install/setup.bash" ] && source "$HOME/easy_handeye2_ws/install/setup.bash"

[ -d src/vendor/interbotix_ros_core ] || {
  echo "❌ Chưa có src/vendor/. Chạy ./tools/setup_vendor.sh trước." >&2; exit 1; }

./tools/check_vendor_patches.sh

export PYTHONNOUSERSITE=1
exec colcon build --symlink-install "$@"
