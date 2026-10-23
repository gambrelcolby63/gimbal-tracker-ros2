#!/usr/bin/env bash
# Rebuild the maneuver trace and render docs/demo/maneuver.mp4 and maneuver.gif.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build}"
PYTHON="${PYTHON:-python3}"
CFG="${CFG:-$ROOT/config/gimbal_tracker.yaml}"

if [[ ! -x "$BUILD/imm_comparison" ]]; then
  cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "$BUILD" --target imm_comparison -j

mkdir -p "$ROOT/docs/demo" "$ROOT/docs"
"$BUILD/imm_comparison" "$CFG" \
  "$ROOT/docs/imm_comparison.csv" \
  "$ROOT/docs/imm_comparison.json" \
  "$ROOT/docs/demo/maneuver_trace.csv"
"$PYTHON" "$ROOT/scripts/render_imm_demo.py" \
  --csv "$ROOT/docs/demo/maneuver_trace.csv" \
  --out-dir "$ROOT/docs/demo"
