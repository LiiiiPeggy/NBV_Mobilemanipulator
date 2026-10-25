#!/usr/bin/env python3
"""One online object-discovery step (or initial observation) using synchronized snapshot GSAM.

Does not invent EE goals when FKIE has no usable NBV.
Does not re-subscribe live images for GSAM — only snapshot dirs.
"""
from __future__ import print_function

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rospy


ROOT = Path("/workspace")
PY = sys.executable
CONDA_PY = Path("/workspace/.conda_envs/nbv_perception/bin/python")


def perception_env():
    """Conda nbv_perception + vendored GD/SAM2; no DovSG; no system cv_bridge clash."""
    env = os.environ.copy()
    env["NBV_ROOT"] = "/workspace"
    env["NBV_PERCEPTION_ROOT"] = "/workspace"
    env["TRANSFORMERS_OFFLINE"] = "1"
    env["HF_HOME"] = "/workspace/checkpoints"
    vendor = [
        "/workspace",
        "/workspace/.conda_envs/nbv_perception/lib/python3.9/site-packages",
        "/workspace/perception/vendor/GroundingDINO",
        "/workspace/perception/vendor/segment-anything-2",
        "/opt/ros/noetic/lib/python3/dist-packages",
    ]
    env["PYTHONPATH"] = ":".join(vendor)
    env["PATH"] = "/workspace/.conda_envs/nbv_perception/bin:" + env.get("PATH", "")
    return env


def run(cmd, log_path=None, check=True, env=None):
    print("+", " ".join(cmd))
    out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    text = out.stdout or ""
    print(text[-4000:] if len(text) > 4000 else text)
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(log_path).write_text(text)
    if check and out.returncode != 0:
        raise SystemExit("CMD_FAIL(%d): %s" % (out.returncode, " ".join(cmd[:4])))
    return out


def write_status(path, **kw):
    path = Path(path)
    cur = {}
    if path.is_file():
        try:
            cur = json.loads(path.read_text())
        except Exception:
            cur = {}
    cur.update(kw)
    cur["wall_time"] = time.time()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cur, indent=2))
    # best-effort HUD publish
    try:
        subprocess.run(
            [
                PY,
                str(ROOT / "scripts/publish_online_exploration_status.py"),
                "--status-json",
                str(path),
                "--memory-json",
                str(cur.get("memory_path") or ""),
                "--once",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=8,
        )
    except Exception:
        pass
    return cur


def capture_and_perceive(obs_dir, snap_id, classes, min_stamp_after, log_dir, device=""):
    obs_dir = Path(obs_dir)
    obs_dir.mkdir(parents=True, exist_ok=True)
    cap = run(
        [
            PY,
            str(ROOT / "scripts/capture_synchronized_rgbd_snapshot.py"),
            "--out-dir",
            str(obs_dir),
            "--snapshot-id",
            snap_id,
            "--min-stamp-after",
            str(min_stamp_after),
        ],
        log_path=str(Path(log_dir) / ("capture_%s.log" % snap_id)),
    )
    if "SNAPSHOT_SYNC_OK" not in (cap.stdout or ""):
        raise SystemExit("CAPTURE_FAIL")
    gsam_py = str(CONDA_PY) if CONDA_PY.is_file() else PY
    perc_cmd = [
        gsam_py,
        str(ROOT / "scripts/perceive_snapshot_gsam.py"),
        "--snapshot-dir",
        str(obs_dir),
        "--classes",
        classes,
    ]
    if device:
        perc_cmd += ["--device", device]
    perc = run(
        perc_cmd,
        log_path=str(Path(log_dir) / ("gsam_%s.log" % snap_id)),
        check=False,
        env=perception_env() if CONDA_PY.is_file() else None,
    )
    if "PERCEIVE_SNAPSHOT_GSAM_OK" not in (perc.stdout or ""):
        raise SystemExit("GSAM_FAIL")
    return obs_dir / "perception.json"


def update_memory(mem_path, perc_path, gt, view_id, out_eval):
    cmd = [
        PY,
        str(ROOT / "scripts/update_online_object_memory.py"),
        "--memory",
        str(mem_path),
        "--perception",
        str(perc_path),
        "--view-id",
        str(view_id),
        "--out-eval",
        str(out_eval),
    ]
    if gt:
        cmd += ["--gt", str(gt)]
    run(cmd, check=True)
    return json.loads(Path(out_eval).read_text())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--step", type=int, required=True, help="0=initial obs only; >=1 NBV+obs")
    ap.add_argument("--max-steps", type=int, default=5)
    ap.add_argument("--gt", default="/workspace/data/scenes/indoor_objects/ycb/gt/seed42_gt.json")
    ap.add_argument(
        "--classes",
        default="banana,apple,bottle,bowl,box,can,cup,cracker,mustard,mug,orange,peach,pear,plum,gelatin,meat,sugar,tomato,sponge",
    )
    ap.add_argument("--device", default="")
    ap.add_argument("--boundary-xy", default="[[-8.3603,-2.6852],[-1.80,-2.6852],[-1.80,-8.6852],[-8.3603,-8.6852]]")
    ap.add_argument("--skip-motion", action="store_true", help="observation-only (debug)")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    step_dir = run_dir / "steps" / ("%03d" % args.step)
    step_dir.mkdir(parents=True, exist_ok=True)
    log_dir = step_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    mem_path = run_dir / "object_memory.json"
    status_path = run_dir / "status.json"
    if args.gt.startswith("/"):
        gt = args.gt
    else:
        gt = str(ROOT / args.gt.lstrip("/"))

    rospy.init_node("run_fkie_online_gsam_step", anonymous=True)

    write_status(
        status_path,
        step=args.step,
        max_steps=args.max_steps,
        phase="INITIAL_OBSERVATION" if args.step == 0 else "REQUEST_FKIE_NBV",
        fkie="IDLE",
        remani="IDLE",
        camera="STABLE",
        gsam="WAITING",
        memory_path=str(mem_path),
        confirmed_objects=0,
    )

    motion_end_stamp = 0.0
    exec_ok = True
    fkie_path = step_dir / "fkie_action_result.json"
    ee_path = step_dir / "ee_goal.json"
    exec_path = step_dir / "exec.json"

    if args.step >= 1 and not args.skip_motion:
        write_status(status_path, phase="REQUEST_FKIE_NBV", fkie="CALLING", camera="STABLE", gsam="WAITING")
        fkie_j = {}
        # At most one settle-retry when planner just restarted (empty cache → false complete).
        for attempt in (1, 2):
            fkie = run(
                [
                    PY,
                    str(ROOT / "scripts/save_fkie_action_result.py"),
                    "--out",
                    str(fkie_path),
                    "--timeout",
                    "90",
                    "--boundary-xy",
                    args.boundary_xy,
                    "--allow-empty",
                ],
                log_path=str(log_dir / ("fkie_try%d.log" % attempt)),
                check=False,
            )
            fkie_j = json.loads(fkie_path.read_text()) if fkie_path.is_file() else {}
            usable = bool(fkie_j.get("usable_nbv"))
            empty_cls = fkie_j.get("empty_class")
            if usable:
                break
            if attempt == 1 and empty_cls in ("A_COMPLETE", "B_NO_FRONTIER", "C_NO_BRANCH"):
                print("FKIE_SETTLE_RETRY after", empty_cls)
                time.sleep(12.0)
                continue
            break
        usable = bool(fkie_j.get("usable_nbv"))
        write_status(
            status_path,
            fkie=fkie_j.get("empty_class") or fkie_j.get("action_state_name") or "FAIL",
            phase="CHECK_FEASIBILITY",
        )
        if not usable:
            term = {
                "terminated": True,
                "reason": "no_usable_nbv",
                "empty_class": fkie_j.get("empty_class"),
                "empty_reason": fkie_j.get("empty_reason"),
                "step": args.step,
            }
            (step_dir / "terminate.json").write_text(json.dumps(term, indent=2))
            write_status(status_path, phase="TERMINATED_NO_NBV", remani="SKIP", gsam="SKIP")
            print(json.dumps(term, indent=2))
            print("STEP_TERMINATED_NO_USABLE_NBV")
            raise SystemExit(10)

        write_status(status_path, phase="REMANI_PLAN", remani="CONVERT")
        run(
            [
                PY,
                str(ROOT / "scripts/nbv_to_remani_ee_goal.py"),
                "--from-fkie-json",
                str(fkie_path),
                "--align-remani-ee",
                "--publish",
                "--out",
                str(ee_path),
            ],
            log_path=str(log_dir / "convert.log"),
        )
        goal_id = "online_%s_step%03d" % (run_dir.name, args.step)
        ee = json.loads(ee_path.read_text())
        ee["goal_id"] = goal_id
        ee_path.write_text(json.dumps(ee, indent=2))

        write_status(status_path, phase="EXECUTE", remani="EXEC_TRAJ", camera="MOVING")
        ex = run(
            [
                PY,
                str(ROOT / "scripts/execute_single_ee_goal.py"),
                "--from-json",
                str(ee_path),
                "--monitor-only",
                "--goal-id",
                goal_id,
                "--timeout",
                "120",
                "--out",
                str(exec_path),
            ],
            log_path=str(log_dir / "exec.log"),
            check=False,
        )
        exec_j = json.loads(exec_path.read_text()) if exec_path.is_file() else {"ok": False}
        exec_ok = bool(exec_j.get("ok"))
        write_status(
            status_path,
            phase="VERIFY_ARRIVAL",
            remani="OK" if exec_ok else "FAIL",
            camera="SETTLING" if exec_ok else "FAIL",
        )
        if not exec_ok:
            (step_dir / "terminate.json").write_text(
                json.dumps({"terminated": True, "reason": "remani_exec_fail", "step": args.step}, indent=2)
            )
            print("STEP_TERMINATED_REMANI_FAIL")
            raise SystemExit(11)
        # settle + require fresher stamps than motion end
        time.sleep(2.0)
        motion_end_stamp = rospy.Time.now().to_sec()
        time.sleep(0.5)

    # Capture + GSAM on fixed snapshot
    obs_dir = step_dir / "observation"
    snap_id = "run_%s_obs_%03d" % (run_dir.name, args.step)
    write_status(status_path, phase="CAPTURE_SYNCHRONIZED_RGBD", camera="STABLE", gsam="WAITING")
    perc_path = capture_and_perceive(
        obs_dir, snap_id, args.classes, motion_end_stamp, log_dir, device=args.device
    )
    write_status(status_path, phase="RUN_GROUNDED_SAM", gsam="RUNNING")
    # capture_and_perceive already ran GSAM; mark done
    write_status(status_path, phase="FUSE_OBJECT_MEMORY", gsam="OK")
    eval_path = step_dir / "memory_update.json"
    report = update_memory(mem_path, perc_path, gt, args.step, eval_path)

    mem = json.loads(mem_path.read_text()) if mem_path.is_file() else {}
    write_status(
        status_path,
        phase="EVALUATE_AND_LOG",
        gsam="OK",
        confirmed_objects=int(mem.get("n_confirmed", 0)),
        memory_path=str(mem_path),
        last_frame_tp=(report.get("frame_eval") or {}).get("tp"),
        last_memory_recall=(report.get("memory_eval") or {}).get("unique_recall"),
    )

    summary = {
        "step": args.step,
        "snapshot_id": snap_id,
        "exec_ok": exec_ok,
        "fkie_usable": bool(json.loads(fkie_path.read_text()).get("usable_nbv")) if fkie_path.is_file() else None,
        "memory_update": report,
        "observation_dir": str(obs_dir),
        "wall_time": datetime.utcnow().isoformat() + "Z",
    }
    (step_dir / "step_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: summary[k] for k in ("step", "snapshot_id", "exec_ok")}, indent=2))
    print("ONLINE_GSAM_STEP_OK")


if __name__ == "__main__":
    main()
