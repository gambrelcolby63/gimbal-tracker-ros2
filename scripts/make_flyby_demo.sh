#!/usr/bin/env bash
# Rebuild the flyby log with the C++ library and render docs/demo/flyby.mp4 and flyby.gif.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build}"
PYTHON="${PYTHON:-python3}"
CFG="${CFG:-$ROOT/config/gimbal_tracker.yaml}"

if [[ ! -x "$BUILD/tracking_demo" ]]; then
  cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "$BUILD" --target tracking_demo -j

mkdir -p "$ROOT/docs/demo"
"$BUILD/tracking_demo" "$CFG" "$ROOT/docs/demo/flyby.csv"
"$PYTHON" "$ROOT/scripts/render_flyby_demo.py" \
  --csv "$ROOT/docs/demo/flyby.csv" \
  --out-dir "$ROOT/docs/demo"
