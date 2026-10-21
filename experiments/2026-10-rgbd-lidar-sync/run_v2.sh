#!/usr/bin/env bash
# v2: LiDAR-constrained poses on top of the v1 outputs (run ./run_all.sh first).
# 03b -> 04 -> 05 -> 06 -> 07 (outputs/v2, results/v2) -> 08 ablation -> 09 Isaac test scenes.
# Any failure stops the pipeline.  Usage: ./run_v2.sh [--config configs/v2.yaml]
set -euo pipefail
cd "$(dirname "$0")"

CONFIG=configs/v2.yaml
[[ "${1:-}" == "--config" ]] && CONFIG="$2"
PY=${PYTHON:-.venv/bin/python}
[[ -x "$PY" ]] || { echo "python venv not found at $PY"; exit 1; }
export MPLBACKEND=Agg
mkdir -p outputs/v2/logs

for s in 03b_lidar_refine 04_integrate_tsdf 05_postprocess_mesh 06_export_usd 07_lidar_check 08_evaluate_v2 09_test_scene; do
  echo "==================== v2 $s ===================="
  start=$(date +%s)
  "$PY" "scripts/$s.py" --config "$CONFIG" 2>&1 | tee "outputs/v2/logs/$s.log"
  echo "[run_v2] $s finished in $(( $(date +%s) - start ))s"
done
"$PY" scripts/verify_usd.py results/v2/scene_collision.usdc | tee outputs/v2/logs/verify_usd.log
