#!/usr/bin/env bash
# Restart REMANI only (keep Gazebo). Default environment_mode=simulated.
# Isolation: ENV_MODE=static_empty (motion pipeline only — not warehouse proof).
set -euo pipefail
ENV_MODE="${ENV_MODE:-simulated}"
source /opt/ros/noetic/setup.bash
source /workspace/ws_fkie/devel/setup.bash 2>/dev/null || true
# remani overlay last so $(find remani_planner) resolves
source /workspace/third_party/wbc/remani_planner/devel/setup.bash
export ROS_PACKAGE_PATH="/workspace/third_party/wbc/remani_planner/src/REMANI-Planner:${ROS_PACKAGE_PATH:-}"
rospack find remani_planner
rospack find mm_controller

LOG=/workspace/logs/fkie_agx_warehouse_mode_b/remani_${ENV_MODE}.log
mkdir -p /workspace/logs/fkie_agx_warehouse_mode_b
echo "[restart] ENV_MODE=${ENV_MODE}"

echo "[restart] stopping prior remani binaries..."
# Match executables only (avoid matching this script's argv)
for pat in 'remani_planner/remani_planner_node' 'mm_controller/mm_controller_node'; do
  pids=$(ps -eo pid,cmd | awk -v p="$pat" '$0 ~ p && $0 !~ /awk/ && $0 !~ /restart_remani/ {print $1}')
  if [ -n "${pids}" ]; then
    echo "kill $pat -> $pids"
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
  fi
done
# roslaunch parents for warehouse remani
ps -eo pid,cmd | awk '/remani_agx_warehouse.launch/ && !/awk/ && !/restart_remani/ {print $1}' | while read -r pid; do
  echo "kill launch $pid"
  kill "$pid" 2>/dev/null || true
done
sleep 3

if ! timeout 5 rosservice list 2>/dev/null | grep -q /gazebo/spawn_sdf_model; then
  echo "FAIL: Gazebo spawn service missing — abort (will not recreate world)"
  exit 3
fi
echo "[restart] Gazebo still up"

if [ "${ENV_MODE}" != "static_empty" ]; then
  # Always bounce feeder so VoxelAccum is rebuilt with current self-filter/purge.
  echo "[restart] bouncing warehouse cloud feeder"
  pkill -f 'python3 /workspace/scripts/warehouse_cloud_to_remani.py' 2>/dev/null || true
  sleep 2
  python3 /workspace/scripts/warehouse_cloud_to_remani.py --skip 6 --z-min 0.12 \
    --self-inflate 0.06 --accum-purge-extra 0.18 \
    >/workspace/logs/fkie_agx_warehouse_mode_b/warehouse_cloud_feeder.log 2>&1 &
  sleep 5

  python3 - <<'PY'
import json, rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
rospy.init_node("wait_feeder_restart", anonymous=True)
rospy.wait_for_message("/mm/car/odom", Odometry, timeout=15)
print("odom_ok")
last_ok = -1
t0 = rospy.Time.now()
while (rospy.Time.now() - t0).to_sec() < 90.0 and not rospy.is_shutdown():
    s = json.loads(rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=5).data)
    fok = int(s.get("frames_ok") or 0)
    if fok != last_ok:
        print(
            "feeder",
            s.get("ready"),
            "accum",
            s.get("accum_n"),
            "frames_ok",
            fok,
            "purged",
            s.get("last_accum_purged"),
        )
        last_ok = fok
    # Voxel scenes can plateau below 500; ready flag uses same threshold in feeder.
    if (s.get("ready") and int(s.get("accum_n") or 0) >= 500 and fok >= 2) or (
        int(s.get("accum_n") or 0) >= 250 and fok >= 5
    ):
        break
else:
    raise SystemExit("feeder not ready")
rospy.sleep(3.0)
s = json.loads(rospy.wait_for_message("/warehouse_map/self_filter_stats", String, timeout=5).data)
print("feeder_settled", s.get("accum_n"), s.get("last_accum_purged"), "frames", s.get("frames_ok"))
PY
else
  echo "[restart] static_empty — skipping warehouse feeder (isolation only)"
fi

echo "[restart] launching environment_mode:=${ENV_MODE}"
: > "$LOG"
roslaunch /workspace/configs/fkie/agx_adapt/launch/remani_agx_warehouse.launch \
  environment_mode:="${ENV_MODE}" >>"$LOG" 2>&1 &
LPID=$!
echo "roslaunch_pid=$LPID"
sleep 8
if [ "${ENV_MODE}" != "static_empty" ]; then
  python3 - <<'PY'
import rospy, time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
rospy.init_node("wait_remani_map", anonymous=True)
rospy.wait_for_message("/mm/car/odom", Odometry, timeout=10)
t0 = time.time()
while time.time() - t0 < 40:
    try:
        msg = rospy.wait_for_message(
            "/remani_planner_node/grid_map/occupancy_inflate", PointCloud2, timeout=3
        )
        n = sum(1 for _ in pc2.read_points(msg, field_names=("x",), skip_nans=True))
        print("inflate_n", n)
        if n > 1000:
            break
    except Exception as e:
        print("wait_inflate", e)
else:
    print("WARN: inflate still empty/sparse after wait")
PY
fi
sleep 2

ps -eo pid,cmd | awk '/remani_planner_node|mm_controller_node/ && !/awk/ {print}'
echo "--- log head ---"
head -80 "$LOG" || true
echo "--- collisions ---"
grep -E "BACKEND COLLISION|environment_mode|ERROR|use_global" "$LOG" | head -40 || true
# param
rosparam get /environment_mode 2>/dev/null || rosparam get /remani_planner_node/environment_mode 2>/dev/null || true
echo "REMANI_SIM_RESTART_DONE"
