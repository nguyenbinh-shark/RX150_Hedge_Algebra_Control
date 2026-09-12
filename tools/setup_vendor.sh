#!/usr/bin/env bash
# setup_vendor.sh — kéo vendor Interbotix + third-party về src/vendor/ rồi áp patch của repo.
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
#
# ── SINH LẠI PATCH sau khi sửa vendor ────────────────────────────────────────
# src/vendor/ bị gitignore, nên mọi sửa đổi vendor CHỈ tồn tại trong patches/vendor/.
# Sau khi sửa, sinh lại patch từ chính submodule (đang detached HEAD ở đúng SHA pin
# nên diff sạch) — ĐỪNG viết patch bằng tay, sai số đếm dòng trong header @@ sẽ làm
# `git apply` báo "corrupt patch" trên máy người khác mà máy bạn không hề thấy:
#
#   CX=src/vendor/interbotix_ros_core/interbotix_ros_xseries
#   git -C $CX/dynamixel_workbench_toolbox add -A
#   git -C $CX/dynamixel_workbench_toolbox diff --cached \
#       > patches/vendor/0001-dynamixel-workbench-toolbox-fast-sync-read.patch
#   git -C $CX/interbotix_xs_driver add -A
#   git -C $CX/interbotix_xs_driver diff --cached \
#       > patches/vendor/0002-interbotix-xs-driver-baudrate-and-fast-sync.patch
#   ./tools/check_vendor_patches.sh          # phải PASS trước khi commit
set -euo pipefail
cd "$(dirname "$0")/.."

command -v vcs >/dev/null || {
  echo "Thiếu vcstool. Cài: sudo apt install python3-vcstool" >&2; exit 1; }

PATCH_DIR="patches/vendor"
CX="src/vendor/interbotix_ros_core/interbotix_ros_xseries"

mkdir -p src/vendor
echo "[1/4] vcs import src/vendor < rx150.repos"
vcs import src/vendor < rx150.repos

echo "[2/4] submodule update --init --recursive"
vcs custom src/vendor --git --args submodule update --init --recursive

echo "[3/4] kiểm SHA interbotix_ros_core khớp bản đã pin"
PIN_FILE="$PATCH_DIR/interbotix_ros_core.sha"
if [ -f "$PIN_FILE" ]; then
  PIN="$(tr -d '[:space:]' < "$PIN_FILE")"
  ACTUAL="$(git -C src/vendor/interbotix_ros_core rev-parse HEAD 2>/dev/null || echo "")"
  if [ -z "$ACTUAL" ]; then
    echo "  ⚠️  src/vendor/interbotix_ros_core không phải git repo — bỏ qua kiểm SHA."
    echo "     (Cây được chép tay? Patch vẫn áp được, nhưng không xác minh được bản gốc.)"
  elif [ "$ACTUAL" != "$PIN" ]; then
    echo "  ❌ SHA lệch: có $ACTUAL, cần $PIN" >&2
    echo "     Patch trong $PATCH_DIR/ sinh ra từ $PIN; áp lên bản khác dễ trượt hoặc" >&2
    echo "     áp sai chỗ. Sửa 'version:' trong rx150.repos, hoặc sinh lại patch." >&2
    exit 1
  else
    echo "  ✓ SHA khớp: $PIN"
  fi
else
  echo "  ⚠️  Không có $PIN_FILE — bỏ qua kiểm SHA."
fi

echo "[4/4] áp vendor patch (idempotent)"
apply_patch() {  # $1 = tên file patch, $2 = thư mục gốc của patch
  local patch="$1" dir="$2"
  if git apply --reverse --check --directory="$dir" "$PATCH_DIR/$patch" 2>/dev/null; then
    echo "  ✓ $patch — đã áp từ trước, bỏ qua."
  elif git apply --directory="$dir" "$PATCH_DIR/$patch"; then
    echo "  ✓ $patch — áp xong."
  else
    echo "  ❌ $patch — áp thất bại." >&2
    return 1
  fi
}
apply_patch "0001-dynamixel-workbench-toolbox-fast-sync-read.patch" "$CX/dynamixel_workbench_toolbox"
apply_patch "0002-interbotix-xs-driver-baudrate-and-fast-sync.patch" "$CX/interbotix_xs_driver"

./tools/check_vendor_patches.sh

echo
vcs status src/vendor --nested || true
echo "✅ vendor sẵn sàng và đã áp đủ patch. Tiếp: ./tools/build.sh"
