#!/usr/bin/env bash
# v3: pose-free loop edges + cutoff on top of v2 (needs the v1 outputs: ./run_all.sh --from 3, i.e. 03-05).
# pf_loop_edges -> 03b (ablation) -> eval_poses -> 04 -> 05 -> 06 -> 07 -> 09 (outputs/v3, results/v3).
# Any failure stops the pipeline.  Usage: ./run_v3.sh [--config configs/v3.yaml]
set -euo pipefail
cd "$(dirname "$0")"

CONFIG=configs/v3.yaml
[[ "${1:-}" == "--config" ]] && CONFIG="$2"
PY=${PYTHON:-.venv/bin/python}
[[ -x "$PY" ]] || { echo "python venv not found at $PY"; exit 1; }
export MPLBACKEND=Agg
mkdir -p outputs/v3/logs

for s in pf_loop_edges 03b_lidar_refine eval_poses 04_integrate_tsdf 05_postprocess_mesh 06_export_usd 07_lidar_check 09_test_scene; do
  echo "==================== v3 $s ===================="
  start=$(date +%s)
  "$PY" "scripts/$s.py" --config "$CONFIG" 2>&1 | tee "outputs/v3/logs/$s.log"
  echo "[run_v3] $s finished in $(( $(date +%s) - start ))s"
done
"$PY" scripts/verify_usd.py results/v3/scene_collision.usdc | tee outputs/v3/logs/verify_usd.log
