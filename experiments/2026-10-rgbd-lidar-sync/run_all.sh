#!/usr/bin/env bash
# Run stages 00-06 in order (07 with --with-lidar). Any failure stops the pipeline.
# Usage: ./run_all.sh [--config configs/default.yaml] [--with-lidar] [--from N]
set -euo pipefail
cd "$(dirname "$0")"

CONFIG=configs/default.yaml
WITH_LIDAR=0
FROM=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --with-lidar) WITH_LIDAR=1; shift ;;
    --from) FROM="$2"; shift 2 ;;
    *) echo "unknown arg $1"; exit 2 ;;
  esac
done

PY=${PYTHON:-.venv/bin/python}
[[ -x "$PY" ]] || { echo "python venv not found at $PY (see README: python3.12 -m venv .venv)"; exit 1; }
export MPLBACKEND=Agg
mkdir -p outputs/logs

STAGES=(00_download 01_inspect_bag 02_extract_frames 03_estimate_poses 04_integrate_tsdf 05_postprocess_mesh 06_export_usd)
[[ $WITH_LIDAR == 1 ]] && STAGES+=(07_lidar_check)

for s in "${STAGES[@]}"; do
  n=$((10#${s:0:2}))
  (( n < FROM )) && continue
  echo "==================== $s ===================="
  start=$(date +%s)
  "$PY" "scripts/$s.py" --config "$CONFIG" 2>&1 | tee "outputs/logs/$s.log"
  echo "[run_all] $s finished in $(( $(date +%s) - start ))s"
done
"$PY" scripts/verify_usd.py outputs/usd/scene_collision.usdc | tee outputs/logs/verify_usd.log
du -sh outputs/* 2>/dev/null || true
