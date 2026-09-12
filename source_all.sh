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
fi

# ── Cảnh báo overlay ────────────────────────────────────────────────────────
# ~/interbotix_ws chứa CÙNG TÊN các package mà repo này vá (xs_driver, toolbox, xs_sdk).
# Nếu prefix của nó đứng TRƯỚC prefix của repo trong AMENT_PREFIX_PATH thì ros2 sẽ nạp
# bản CHƯA patch: build sạch, log không báo gì, nhưng không thấy dòng 'SyncRead mode:'.
# Chỉ so hai đường dẫn thư mục là vô nghĩa (chúng vốn khác nhau) — phải xem package nào
# thắng trong AMENT_PREFIX_PATH, nếu không cảnh báo sẽ nổ mỗi lần source rồi bị bỏ qua.
_shadowed=""
for _pkg in interbotix_xs_sdk interbotix_xs_driver dynamixel_workbench_toolbox; do
    _winner=""
    IFS=':' read -ra _prefixes <<< "${AMENT_PREFIX_PATH:-}"
    for _p in "${_prefixes[@]}"; do
        if [ "$(basename "$_p")" = "$_pkg" ]; then _winner="$_p"; break; fi
    done
    case "$_winner" in
        "$WS_DIR"/*|"") ;;
        *) _shadowed="$_shadowed\n     $_pkg  <-  $_winner" ;;
    esac
done
if [ -n "$_shadowed" ]; then
    echo "⚠️  CẢNH BÁO OVERLAY: package sau được nạp từ NGOÀI repo này:"
    printf "%b\n" "$_shadowed"
    echo "     → vendor patch sẽ IM LẶNG không có tác dụng."
    echo "     Gỡ dòng source ~/interbotix_ws trong ~/.bashrc, mở shell mới, rồi source lại."
fi
unset _shadowed _pkg _winner _p _prefixes
echo "[source_all] ros=${ROS_DISTRO} + apriltag_ws + easy_handeye2_ws + rx150_ws"
