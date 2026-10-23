#!/usr/bin/env bash
# Mode B bringup probe + single /ee_goal publish attempt.
# Does not replace Mode A as the default perception/FKIE path.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REPORT="${1:-$ROOT/reports/mode_b_remani_ee_goal.md}"
METRICS="$ROOT/data/metrics/nbv_remani"
mkdir -p "$METRICS" "$(dirname "$REPORT")"

AGX_MODE="${AGX_MODE:-dynamic}"
SPAWN_PROFILE="${SPAWN_PROFILE:-aisle_safe}"
CONTAINER="${FKIE_CONTAINER:-nbv_fkie}"

{
  echo "# Mode B + REMANI /ee_goal probe"
  echo
  echo "- date: $(date -Is)"
  echo "- AGX_MODE=$AGX_MODE"
  echo "- SPAWN_PROFILE=$SPAWN_PROFILE"
  echo "- container=$CONTAINER"
  echo
} > "$REPORT"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "- status: **blocked** — container $CONTAINER not running" | tee -a "$REPORT"
  echo "- note: start with \`AGX_MODE=dynamic SPAWN_PROFILE=aisle_safe ./scripts/fkie_agx_warehouse_nbv.sh\` then re-run this probe." | tee -a "$REPORT"
  echo "MODE_B_PROBE_BLOCKED"
  exit 0
fi

echo "- container: running" | tee -a "$REPORT"

# Convert last FKIE goal or fallback demo goal -> /ee_goal
GOAL_XYZ="${GOAL_XYZ:--3.020 -5.294 1.609}"
OUT_JSON="$METRICS/ee_goal_mode_b_probe.json"
set +e
docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; source /workspace/ws_fkie/devel/setup.bash; \
  python3 /workspace/scripts/nbv_to_remani_ee_goal.py \
    --cam-xyz $GOAL_XYZ --cam-rpy 0 0.35 0 \
    --camera-frame d455_depth_optical_frame \
    --publish --out /workspace/data/metrics/nbv_remani/ee_goal_mode_b_probe.json"
RC=$?
set -e

if [[ -f "$OUT_JSON" ]]; then
  echo "- ee_goal_json: \`$OUT_JSON\`" | tee -a "$REPORT"
  echo '```json' >> "$REPORT"
  cat "$OUT_JSON" >> "$REPORT"
  echo '```' >> "$REPORT"
fi

# REMANI stack presence
if [[ -d "$ROOT/third_party/wbc" ]]; then
  echo "- wbc present: yes (branch remani-real-implementation expected)" | tee -a "$REPORT"
else
  echo "- wbc present: **no**" | tee -a "$REPORT"
fi

# Check if anything is subscribed / advertising ee_goal
set +e
HAS_EE=$(docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; rostopic info /ee_goal 2>/dev/null | head -40")
set -e
echo "- rostopic info /ee_goal:" | tee -a "$REPORT"
echo '```' >> "$REPORT"
echo "$HAS_EE" >> "$REPORT"
echo '```' >> "$REPORT"

if [[ "$RC" == "0" ]]; then
  echo "- bridge publish: OK (rc=0)" | tee -a "$REPORT"
  if echo "$HAS_EE" | grep -qi 'Subscribers:.*None\|Subscribers: *$'; then
    echo "- REMANI exec: **not verified** (no /ee_goal subscribers). Mode A retained." | tee -a "$REPORT"
    echo "MODE_B_BRIDGE_OK_REMANI_PENDING"
  else
    echo "- REMANI exec: subscriber seen (execution success still requires remani planner logs)." | tee -a "$REPORT"
    echo "MODE_B_PROBE_OK"
  fi
else
  echo "- bridge publish: **failed** rc=$RC" | tee -a "$REPORT"
  echo "MODE_B_PROBE_FAIL"
fi
