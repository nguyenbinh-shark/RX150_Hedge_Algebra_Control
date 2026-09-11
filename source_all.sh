#!/usr/bin/env bash
# Source đủ 4 layer để build/run stack rx150_fuzzy_controller + camera + hand-eye.
# Dùng: source source_all.sh
#
# Thứ tự: ROS underlay -> apriltag_ws -> easy_handeye2_ws -> rx150_ws
set -e
ROS_DISTRO="${ROS_DISTRO:-humble}"
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f "$HOME/apriltag_ws/install/setup.bash" ] && source "$HOME/apriltag_ws/install/setup.bash"
[ -f "$HOME/easy_handeye2_ws/install/setup.bash" ] && source "$HOME/easy_handeye2_ws/install/setup.bash"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
elif [ -f "$HOME/RX150_Hedge_Algebra_Control/install/setup.bash" ]; then
    source "$HOME/RX150_Hedge_Algebra_Control/install/setup.bash"
elif [ -f "$HOME/interbotix_ws/install/setup.bash" ]; then
    source "$HOME/interbotix_ws/install/setup.bash"
fi
echo "[source_all] ros=${ROS_DISTRO} + apriltag_ws + easy_handeye2_ws + rx150_ws"
