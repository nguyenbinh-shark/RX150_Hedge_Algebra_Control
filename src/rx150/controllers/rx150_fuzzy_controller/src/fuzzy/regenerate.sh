#!/usr/bin/env bash
set -euo pipefail
# Workspace root = 6 cấp lên từ src/rx150/controllers/rx150_fuzzy_controller/src/fuzzy/.
# (Trước đợt dựng cây IRROS package nằm ở src/rx150_fuzzy_controller/ nên chỉ 4 cấp; để
#  nguyên 4 cấp thì WS_ROOT trỏ vào src/rx150 và GEN_DIR không tồn tại.)
WS_ROOT="$(cd "$(dirname "$0")/../../../../../.." && pwd)"
GEN_DIR="${GEN_DIR:-${WS_ROOT}/fuzzy_codegen}"
SRC_FIS="$(dirname "$0")/fuzzy_type1.fis"
cp "$SRC_FIS" "$GEN_DIR/fuzzy_type1.fis"
( cd "$GEN_DIR" && python3 fis2c.py fuzzy_type1.fis )
cp "$GEN_DIR/fuzzy_type1.c" "$(dirname "$0")/fuzzy_type1.c"
# Note: .h được install vào include/..., nhưng codegen sinh cả hai.
# fuzzy_type1.h cũng ở src/fuzzy/ để #include "fuzzy_type1.h" hoạt động trong quá trình dev.
cp "$GEN_DIR/fuzzy_type1.h" "$(dirname "$0")/fuzzy_type1.h"
echo "Regenerated fuzzy_type1.{c,h} from $SRC_FIS"
