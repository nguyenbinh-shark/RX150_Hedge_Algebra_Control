#!/usr/bin/env bash
set -euo pipefail
# Workspace root = 6 cấp lên từ src/rx150/controllers/rx150_ff_controller/src/fuzzy/.
# (Trước đợt dựng cây IRROS package nằm ở src/rx150_ff_controller/ nên chỉ 4 cấp;
#  để nguyên 4 cấp thì WS_ROOT trỏ vào src/rx150 và GEN_DIR không tồn tại.)
WS_ROOT="$(cd "$(dirname "$0")/../../../../../.." && pwd)"
GEN_DIR="${GEN_DIR:-${WS_ROOT}/fuzzy_codegen}"
SRC_FIS="$(dirname "$0")/fuzzy_type1.fis"
# Verify .fis tồn tại (source of truth cho codegen)
if [ ! -f "$SRC_FIS" ]; then
    echo "LỖI: $SRC_FIS không tồn tại. Engine ff cần .fis riêng để regen."
    exit 1
fi
cp "$SRC_FIS" "$GEN_DIR/fuzzy_type1.fis"
( cd "$GEN_DIR" && python3 fis2c.py fuzzy_type1.fis )
cp "$GEN_DIR/fuzzy_type1.c" "$(dirname "$0")/fuzzy_type1.c"
cp "$GEN_DIR/fuzzy_type1.h" "$(dirname "$0")/fuzzy_type1.h"
echo "Regenerated ff fuzzy_type1.{c,h} from $SRC_FIS"
