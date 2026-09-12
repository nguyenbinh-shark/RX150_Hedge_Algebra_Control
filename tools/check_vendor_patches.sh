#!/usr/bin/env bash
# check_vendor_patches.sh — cây vendor có ĐÚNG bằng bản gốc + patch không?
#
# Phép kiểm duy nhất đáng tin là `git apply --reverse --check`: nó đối chiếu TỪNG DÒNG
# của patch với file trên đĩa. Kiểm bằng grep tên hàm thì một patch áp thiếu, áp sai chỗ,
# hay bị sửa tay sau đó vẫn "đạt" — mà đó chính là kiểu hỏng im lặng nguy hiểm nhất:
# build sạch, log không báo gì, nhưng chạy bản mã khác với bản trong patch.
#
# Trả 0 nếu mọi patch đã áp và khớp từng dòng; 1 nếu thiếu hoặc lệch.
set -eo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PATCH_DIR="patches/vendor"
CX="src/vendor/interbotix_ros_core/interbotix_ros_xseries"

# patch : thư mục gốc mà đường dẫn trong patch tính tương đối theo
PATCHES=(
  "0001-dynamixel-workbench-toolbox-fast-sync-read.patch:$CX/dynamixel_workbench_toolbox"
  "0002-interbotix-xs-driver-baudrate-and-fast-sync.patch:$CX/interbotix_xs_driver"
)

rc=0
for entry in "${PATCHES[@]}"; do
  patch="${entry%%:*}"
  dir="${entry##*:}"

  if [ ! -d "$dir" ]; then
    echo "❌ Thiếu $dir — chạy ./tools/setup_vendor.sh trước." >&2
    rc=1; continue
  fi

  if git apply --reverse --check --directory="$dir" "$PATCH_DIR/$patch" 2>/dev/null; then
    echo "✅ $patch — đã áp, khớp từng dòng."
  elif git apply --check --directory="$dir" "$PATCH_DIR/$patch" 2>/dev/null; then
    # áp thuận được sạch ⟹ cây đang ở bản gốc, chưa hề áp patch
    echo "❌ $patch — CHƯA áp. Chạy ./tools/setup_vendor.sh." >&2
    rc=1
  else
    # không reverse được mà cũng không forward được ⟹ đã bị sửa tay / áp dở
    echo "❌ $patch — cây vendor LỆCH khỏi patch (áp dở, hoặc bị sửa tay sau khi áp)." >&2
    echo "   Khôi phục: rm -rf src/vendor && ./tools/setup_vendor.sh" >&2
    echo "   Nếu bạn CỐ Ý sửa vendor, hãy sinh lại patch rồi commit — xem đầu tools/setup_vendor.sh." >&2
    rc=1
  fi
done

[ $rc -eq 0 ] || exit 1
echo "✅ Toàn bộ vendor patch khớp."
