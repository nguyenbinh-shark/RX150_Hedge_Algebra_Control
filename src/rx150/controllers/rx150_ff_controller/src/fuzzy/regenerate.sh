#!/usr/bin/env bash
set -euo pipefail
# Workspace root = 4 levels up from src/rx150_ff_controller/src/fuzzy/
WS_ROOT="$(cd "$(dirname "$0")/../../../.." && pwd)"
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
