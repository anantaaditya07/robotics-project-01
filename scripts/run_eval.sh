#!/usr/bin/env bash
# SemNav A/B evaluation (Phase 8): semantic layer on vs off, same waypoints, headless.
#
#   scripts/run_eval.sh [runs_per_config]        # default 5
#
# For each configuration: write a nav2_params variant (semantic_layer.enabled true/false) under
# data/eval/, start semnav.launch.py headless in its own process group, wait for Nav2, run
# scripts/eval_run.py, stop the group. Then scripts/eval_summary.py writes docs/results.md.
# Prerequisites: workspace built; tf2 underlay (D-20) in ~/semnav_underlay (scripts/setup_underlay.sh).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
RUNS="${1:-5}"
OUT="$REPO/data/eval"
UNDERLAY="${SEMNAV_UNDERLAY:-$HOME/semnav_underlay}"
READY_TIMEOUT=120   # s to wait for Nav2 "Managed nodes are active"
STOP_GRACE=10       # s between SIGINT and SIGKILL when stopping the stack

set +u
source /opt/ros/humble/setup.bash
source "$UNDERLAY/install/setup.bash"
source "$REPO/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$REPO/install/semnav_bringup/share/semnav_bringup/config/cyclonedds.xml"

mkdir -p "$OUT"
rm -f "$OUT/goals.csv" "$OUT/runs.csv"
PARAMS="$REPO/install/semnav_bringup/share/semnav_bringup/config/nav2_params.yaml"

stop_stack() {
  local pgid="$1"
  kill -INT -- "-$pgid" 2>/dev/null || true
  sleep "$STOP_GRACE"
  kill -KILL -- "-$pgid" 2>/dev/null || true
  pkill -KILL -x gzserver 2>/dev/null || true
}

for label in semantic_on semantic_off; do
  enabled=$([[ "$label" == semantic_on ]] && echo True || echo False)
  variant="$OUT/nav2_params_$label.yaml"
  # Only semantic_layer.enabled changes; every other parameter stays as in nav2_params.yaml.
  python3 - "$PARAMS" "$variant" "$enabled" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1])); want = sys.argv[3] == 'True'
for cm in ('global_costmap', 'local_costmap'):
    d[cm][cm]['ros__parameters']['semantic_layer']['enabled'] = want
yaml.safe_dump(d, open(sys.argv[2], 'w'), sort_keys=False)
PY
  log="$OUT/$label.launch.log"
  echo "== $label: launching (log $log)"
  setsid ros2 launch semnav_bringup semnav.launch.py gui:=false rviz:=false \
    nav2_params:="$variant" > "$log" 2>&1 &
  pgid=$!
  for _ in $(seq 1 "$READY_TIMEOUT"); do
    [[ $(grep -c "Managed nodes are active" "$log") -ge 2 ]] && break
    sleep 1
  done
  if [[ $(grep -c "Managed nodes are active" "$log") -lt 2 ]]; then
    echo "error: stack not ready for $label" >&2; stop_stack "$pgid"; exit 1
  fi
  /usr/bin/python3 "$REPO/scripts/eval_run.py" --label "$label" --runs "$RUNS" \
    --csv "$OUT/goals.csv" --runs-csv "$OUT/runs.csv" || true
  echo "$label stale-TF errors: $(grep -cE 'Transform data too old' "$log" || true)"
  stop_stack "$pgid"
done

python3 "$REPO/scripts/eval_summary.py" --goals "$OUT/goals.csv" --runs "$OUT/runs.csv" \
  --out "$REPO/docs/results.md"
