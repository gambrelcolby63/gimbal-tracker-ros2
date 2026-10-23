#!/usr/bin/env bash
# Regenerate docs/*.csv, docs/*.json, and docs/*.png from the C++ tools.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build}"
PYTHON="${PYTHON:-python3}"
CFG="${CFG:-$ROOT/config/gimbal_tracker.yaml}"
mkdir -p "$ROOT/docs"

"$BUILD/step_response" "$CFG" "$ROOT/docs/step_response.csv"
"$BUILD/step_response" "$CFG" "$ROOT/docs/step_response_friction.csv" --friction
"$BUILD/estimator_mc" "$CFG" "$ROOT/docs/estimator_mc.csv" "$ROOT/docs/estimator_mc.json"
"$BUILD/tracking_demo" "$CFG" "$ROOT/docs/tracking.csv"

"$PYTHON" "$ROOT/scripts/analyze_control.py" \
  --config "$CFG" \
  --csv "$ROOT/docs/step_response.csv" \
  --friction-csv "$ROOT/docs/step_response_friction.csv" \
  --out-dir "$ROOT/docs"
"$PYTHON" "$ROOT/scripts/analyze_estimator.py" \
  --csv "$ROOT/docs/estimator_mc.csv" \
  --summary "$ROOT/docs/estimator_mc.json" \
  --out-dir "$ROOT/docs"
"$PYTHON" "$ROOT/scripts/analyze_tracking.py" \
  --csv "$ROOT/docs/tracking.csv" \
  --out-dir "$ROOT/docs"
