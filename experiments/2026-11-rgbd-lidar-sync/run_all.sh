#!/usr/bin/env bash
# Run stages 00-06 in order (07 with --with-lidar). Any failure stops the pipeline.
# Usage: ./run_all.sh (--dataset NAME | --config configs/NAME.yaml) [--with-lidar] [--from N] [--set key=value ...]
#   --dataset NAME is short for --config configs/NAME.yaml; every dataset writes to its own
#   paths.outputs_dir / paths.results_dir (outputs/<dataset>/, results/<dataset>/).
set -euo pipefail
cd "$(dirname "$0")"

CONFIG=""
WITH_LIDAR=0
FROM=0
SETS=()   # --set key=value pairs for the stage scripts
OVR=()    # the same key=value strings, for load_config below
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --dataset) CONFIG="configs/$2.yaml"; shift 2 ;;
    --with-lidar) WITH_LIDAR=1; shift ;;
    --from) FROM="$2"; shift 2 ;;
    --set) SETS+=(--set "$2"); OVR+=("$2"); shift 2 ;;
    *) echo "unknown arg $1"; exit 2 ;;
  esac
done
if [[ -z "$CONFIG" ]]; then
  echo "choose a dataset: --dataset NAME (one of: $(cd configs && ls *.yaml | grep -v '^default.yaml$' | sed 's/\.yaml$//' | tr '\n' ' '))"
  exit 2
fi
[[ -f "$CONFIG" ]] || { echo "config not found: $CONFIG"; exit 2; }

PY=${PYTHON:-.venv/bin/python}
[[ -x "$PY" ]] || { echo "python venv not found at $PY (see README: uv venv -p 3.12 .venv)"; exit 1; }
export MPLBACKEND=Agg
OUT=$("$PY" -c 'import sys; sys.path.insert(0, "scripts")
from common import exp_path, load_config
print(exp_path(load_config(sys.argv[1], sys.argv[2:])["paths"]["outputs_dir"]))' "$CONFIG" "${OVR[@]}")
USD=$("$PY" -c 'import sys; sys.path.insert(0, "scripts")
from common import load_config
print(load_config(sys.argv[1], sys.argv[2:])["usd"]["filename"])' "$CONFIG" "${OVR[@]}")
mkdir -p "$OUT/logs"
echo "[run_all] config $CONFIG -> $OUT"

STAGES=(00_download 01_inspect_bag 02_extract_frames 03_estimate_poses 04_integrate_tsdf 05_postprocess_mesh 06_export_usd)
[[ $WITH_LIDAR == 1 ]] && STAGES+=(07_lidar_check)

for s in "${STAGES[@]}"; do
  n=$((10#${s:0:2}))
  (( n < FROM )) && continue
  echo "==================== $s ===================="
  start=$(date +%s)
  "$PY" "scripts/$s.py" --config "$CONFIG" "${SETS[@]}" 2>&1 | tee "$OUT/logs/$s.log"
  echo "[run_all] $s finished in $(( $(date +%s) - start ))s"
done
"$PY" scripts/verify_usd.py "$OUT/usd/$USD" | tee "$OUT/logs/verify_usd.log"
du -sh "$OUT"/* 2>/dev/null || true
