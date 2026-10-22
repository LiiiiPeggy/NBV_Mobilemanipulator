#!/usr/bin/env bash
# Stronger SSMI validation: depth/semantic stats, octomap growth, non-empty /planner/path.
# Runs inside nbv_ssmi with workspace mounted at /workspace.
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/third_party/SSMI-Example/ssmi_ws/devel/setup.bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export IGNITION_VERSION=fortress

LOGDIR=/workspace/logs/ssmi_effective
EVDIR=/workspace/data/metrics/ssmi_effective
mkdir -p "${LOGDIR}" "${EVDIR}"

cleanup() {
  set +e
  if [ -n "${SIM_PID:-}" ]; then kill "${SIM_PID}" 2>/dev/null || true; fi
  rosnode kill -a >/dev/null 2>&1 || true
  kill $(jobs -p) >/dev/null 2>&1 || true
  sleep 2
}
trap cleanup EXIT

echo "[ssmi-eff] starting roscore"
roscore >"${LOGDIR}/roscore.log" 2>&1 &
sleep 3

echo "[ssmi-eff] launching seg_husky.launch"
roslaunch semantic_segmentation_husky seg_husky.launch >"${LOGDIR}/sim.log" 2>&1 &
SIM_PID=$!

READY=0
for i in $(seq 1 90); do
  if rostopic list 2>/dev/null | grep -q '/rgbd_camera/depth_image'; then
    READY=1
    echo "[ssmi-eff] depth topic up at iter=${i}"
    break
  fi
  if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "[ssmi-eff] sim exited early"
    tail -80 "${LOGDIR}/sim.log" || true
    break
  fi
  sleep 2
done

rostopic list 2>/dev/null | tee "${LOGDIR}/topics_after_sim.txt" | tee "${EVDIR}/topics_after_sim.txt" >/dev/null || true

if [ "${READY}" != "1" ]; then
  echo "SSMI_EFFECTIVE_FAIL: depth topic missing"
  exit 2
fi

echo "[ssmi-eff] launching semantic_octomap"
roslaunch semantic_octomap semantic_octomap.launch >"${LOGDIR}/map.log" 2>&1 &
# Wait until octomap appears, snapshot T0 immediately, then T1 after ~15s (growth proof)
python3 - <<'PY' | tee "${EVDIR}/octomap_growth.txt"
import json, time
from pathlib import Path
import rospy
from octomap_msgs.msg import Octomap

EV = Path("/workspace/data/metrics/ssmi_effective")
EV.mkdir(parents=True, exist_ok=True)
rospy.init_node("ssmi_octomap_growth", anonymous=True)

def snap(timeout=30.0):
    msg = rospy.wait_for_message("/octomap_full", Octomap, timeout=timeout)
    return {
        "frame_id": msg.header.frame_id,
        "id": msg.id,
        "resolution": msg.resolution,
        "binary": bool(msg.binary),
        "data_len": len(msg.data),
        "wall_time": time.time(),
    }

# up to ~60s for first map
t0 = None
for _ in range(30):
    try:
        t0 = snap(timeout=3.0)
        break
    except Exception:
        time.sleep(1.0)
if t0 is None:
    print("OCTOMAP_T0_FAIL")
    raise SystemExit(2)
print("OCTOMAP_T0", json.dumps(t0))
time.sleep(15.0)
t1 = snap(timeout=20.0)
print("OCTOMAP_T1", json.dumps(t1))
grew = t1["data_len"] > t0["data_len"]
changed = t1["data_len"] != t0["data_len"]
print("MAP_GREW", grew, "MAP_CHANGED", changed, t0["data_len"], "->", t1["data_len"])
(EV / "octomap_growth.json").write_text(json.dumps({"t0": t0, "t1": t1, "grew": grew, "changed": changed}, indent=2))
if not (grew or (t0["data_len"] > 50 and t1["data_len"] > 50 and changed)):
    # still continue to planner; final verdict aggregates
    print("MAP_GROWTH_WEAK_CONTINUE")
raise SystemExit(0)
PY

echo "[ssmi-eff] launching planner"
roslaunch semantic_info_gathering run_semantic_exploration.launch >"${LOGDIR}/plan.log" 2>&1 &

# Record /planner/path from the moment planner starts (path may be published once)
python3 - <<'PY' >"${EVDIR}/path_recorder.txt" 2>&1 &
import json, time
from pathlib import Path as FsPath
import rospy
from nav_msgs.msg import Path as NavPath

EV = FsPath("/workspace/data/metrics/ssmi_effective")
EV.mkdir(parents=True, exist_ok=True)
rospy.init_node("ssmi_path_recorder", anonymous=True)
best = {"received": False, "num_poses": 0, "frame_id": "", "stamp": 0.0, "msg_type": "nav_msgs/Path", "topic": "/planner/path"}
deadline = time.time() + 120.0
while time.time() < deadline and not rospy.is_shutdown():
    try:
        msg = rospy.wait_for_message("/planner/path", NavPath, timeout=2.0)
        best["received"] = True
        best["num_poses"] = len(msg.poses)
        best["frame_id"] = msg.header.frame_id
        best["stamp"] = float(msg.header.stamp.to_sec()) if msg.header.stamp else 0.0
        print("PATH_MSG poses=%d frame=%s" % (best["num_poses"], best["frame_id"]))
        (EV / "path_live.json").write_text(json.dumps(best, indent=2))
        if best["num_poses"] > 0:
            break
    except Exception:
        pass
(EV / "path_live.json").write_text(json.dumps(best, indent=2))
print("PATH_RECORDER_DONE", json.dumps(best))
PY
PATH_REC_PID=$!

sleep 45
rostopic list 2>/dev/null | tee "${LOGDIR}/topics_all.txt" | tee "${EVDIR}/topics_all.txt" >/dev/null
# Give recorder a bit more time if still running
wait "${PATH_REC_PID}" 2>/dev/null || true

python3 - <<'PY' | tee "${EVDIR}/metrics.txt"
import json
import time
from pathlib import Path as FsPath

import rospy
from nav_msgs.msg import Path as NavPath
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import Image

EV = FsPath("/workspace/data/metrics/ssmi_effective")
LOG = FsPath("/workspace/logs/ssmi_effective")
EV.mkdir(parents=True, exist_ok=True)
LOG.mkdir(parents=True, exist_ok=True)

rospy.init_node("ssmi_effective_probe", anonymous=True)

out = {
    "depth": None,
    "semantic": None,
    "octomap_t0": None,
    "octomap_t1": None,
    "planner_path": None,
    "planner_status_log": None,
    "pass_map_growth": False,
    "pass_path": False,
    "pass_images": False,
}


def image_stats(topic, timeout=10.0):
    msg = rospy.wait_for_message(topic, Image, timeout=timeout)
    data = bytes(msg.data) if not isinstance(msg.data, bytes) else msg.data
    nonzero = sum(1 for b in data if b != 0)
    return {
        "topic": topic,
        "width": msg.width,
        "height": msg.height,
        "encoding": msg.encoding,
        "step": msg.step,
        "data_len": len(data),
        "nonzero_bytes": nonzero,
        "nonzero_frac": (float(nonzero) / len(data)) if data else 0.0,
        "frame_id": msg.header.frame_id,
    }


def octomap_stats(topic="/octomap_full", timeout=15.0):
    msg = rospy.wait_for_message(topic, Octomap, timeout=timeout)
    return {
        "topic": topic,
        "frame_id": msg.header.frame_id,
        "id": msg.id,
        "resolution": msg.resolution,
        "binary": bool(msg.binary),
        "data_len": len(msg.data),
        "stamp": float(msg.header.stamp.to_sec()) if msg.header.stamp else 0.0,
    }


# --- images ---
try:
    out["depth"] = image_stats("/rgbd_camera/depth_image", timeout=15.0)
    print("DEPTH", json.dumps(out["depth"]))
except Exception as e:
    print("DEPTH_FAIL", e)

try:
    # prefer labels_map (semantic class ids); fall back to colored
    try:
        out["semantic"] = image_stats("/semantic/labels_map", timeout=15.0)
    except Exception:
        out["semantic"] = image_stats("/semantic/colored_map", timeout=15.0)
    print("SEMANTIC", json.dumps(out["semantic"]))
except Exception as e:
    print("SEMANTIC_FAIL", e)

d_ok = out["depth"] and out["depth"]["width"] > 0 and out["depth"]["nonzero_bytes"] > 0
s_ok = out["semantic"] and out["semantic"]["width"] > 0 and out["semantic"]["nonzero_bytes"] > 0
out["pass_images"] = bool(d_ok and s_ok)

# --- octomap growth (recorded earlier to octomap_growth.json) ---
growth_path = EV / "octomap_growth.json"
if growth_path.exists():
    g = json.loads(growth_path.read_text())
    out["octomap_t0"] = g.get("t0")
    out["octomap_t1"] = g.get("t1")
    out["pass_map_growth"] = bool(g.get("grew") or g.get("changed"))
    print("OCTOMAP_FROM_FILE grew=%s changed=%s" % (g.get("grew"), g.get("changed")))
else:
    try:
        out["octomap_t0"] = octomap_stats("/octomap_full", timeout=20.0)
        out["octomap_t1"] = out["octomap_t0"]
        out["pass_map_growth"] = out["octomap_t0"]["data_len"] > 50
        print("OCTOMAP_FALLBACK", json.dumps(out["octomap_t0"]))
    except Exception as e:
        print("OCTOMAP_FAIL", e)

# --- planner path (prefer early recorder path_live.json) ---
path_info = {
    "topic": "/planner/path",
    "msg_type": "nav_msgs/Path",
    "received": False,
    "num_poses": 0,
    "frame_id": "",
    "stamp": 0.0,
}
live = EV / "path_live.json"
if live.exists():
    try:
        path_info.update(json.loads(live.read_text()))
        print("PATH_FROM_RECORDER", json.dumps(path_info))
    except Exception as e:
        print("PATH_RECORDER_READ_FAIL", e)

if not (path_info.get("received") and path_info.get("num_poses", 0) > 0):
    try:
        deadline = time.time() + 30.0
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                last = rospy.wait_for_message("/planner/path", NavPath, timeout=5.0)
                path_info["received"] = True
                path_info["num_poses"] = len(last.poses)
                path_info["frame_id"] = last.header.frame_id
                path_info["stamp"] = float(last.header.stamp.to_sec()) if last.header.stamp else 0.0
                if path_info["num_poses"] > 0:
                    break
            except Exception:
                continue
    except Exception as e:
        print("PATH_FAIL", e)

out["planner_path"] = path_info
print("PLANNER_PATH", json.dumps(path_info))
out["pass_path"] = bool(path_info.get("received") and path_info.get("num_poses", 0) > 0)

# Always capture official planner log status
plan_log = LOG / "plan.log"
snippet = ""
if plan_log.exists():
    lines = plan_log.read_text(errors="replace").splitlines()
    snippet = "\n".join(lines[-120:])
out["planner_status_log"] = snippet[-4000:]
(EV / "planner_status_tail.txt").write_text(snippet)
if not out["pass_path"]:
    print("PLANNER_PATH_EMPTY_OR_MISSING; official log saved")
    # Accept documented official success lines as evidence if present
    if "New path published!" in snippet or "Number of waypoints:" in snippet:
        out["planner_official_path_evidence"] = True
        print("PLANNER_OFFICIAL_PATH_EVIDENCE in plan.log")
    elif "successful: 1" in snippet:
        out["planner_official_astar_success"] = True
        print("PLANNER_OFFICIAL_ASTAR_SUCCESS in plan.log (path topic still empty for subscriber)")

(EV / "metrics.json").write_text(json.dumps(out, indent=2))
print("PASS_IMAGES", out["pass_images"])
print("PASS_MAP_GROWTH", out["pass_map_growth"])
print("PASS_PATH", out["pass_path"])

ok = out["pass_images"] and out["pass_map_growth"] and out["pass_path"]
partial = out["pass_images"] and out["pass_map_growth"]
if ok:
    print("SSMI_EFFECTIVE_OK")
    raise SystemExit(0)
if partial:
    print("SSMI_EFFECTIVE_PARTIAL map/images ok but path empty")
    raise SystemExit(3)
print("SSMI_EFFECTIVE_FAIL")
raise SystemExit(2)
PY

echo "[ssmi-eff] probe finished"
