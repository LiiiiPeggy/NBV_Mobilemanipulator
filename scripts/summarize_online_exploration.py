#!/usr/bin/env python3
"""Build summary.json + simple metric curves for an online exploration run."""
from __future__ import print_function

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--terminated", type=int, default=0)
    ap.add_argument("--terminate-reason", default="")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    steps = sorted((run_dir / "steps").glob("*"))
    rows = []
    for sd in steps:
        if not sd.is_dir():
            continue
        ss = sd / "step_summary.json"
        mu = sd / "memory_update.json"
        fk = sd / "fkie_action_result.json"
        ex = sd / "exec.json"
        row = {"step": int(sd.name), "dir": str(sd)}
        if ss.is_file():
            row.update(json.loads(ss.read_text()))
        if mu.is_file():
            m = json.loads(mu.read_text())
            row["memory_update"] = m
            row["n_confirmed"] = m.get("n_confirmed_after")
            row["frame_tp"] = (m.get("frame_eval") or {}).get("tp")
            row["memory_recall"] = (m.get("memory_eval") or {}).get("unique_recall")
            row["confirmed_tp"] = (m.get("confirmed_eval") or {}).get("tp")
        if fk.is_file():
            f = json.loads(fk.read_text())
            row["fkie_empty_class"] = f.get("empty_class")
            row["usable_nbv"] = f.get("usable_nbv")
            row["camera_goal_xyz"] = f.get("camera_goal_xyz")
        if ex.is_file():
            e = json.loads(ex.read_text())
            row["ee_err_m"] = (e.get("stages") or {}).get("ee_err_m")
            row["exec_ok"] = e.get("ok")
        if (sd / "terminate.json").is_file():
            row["terminate"] = json.loads((sd / "terminate.json").read_text())
        rows.append(row)

    mem = {}
    if (run_dir / "object_memory.json").is_file():
        mem = json.loads((run_dir / "object_memory.json").read_text())

    summary = {
        "run_dir": str(run_dir),
        "n_steps_dirs": len(rows),
        "terminated": bool(args.terminated),
        "terminate_reason": args.terminate_reason,
        "n_memory_objects": mem.get("n_objects"),
        "n_confirmed": mem.get("n_confirmed"),
        "steps": rows,
        "mode_label": "online_autonomous_nbv",
        "not_offline_multiview": True,
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    # Curves
    xs = [r["step"] for r in rows if "n_confirmed" in r]
    conf = [r.get("n_confirmed") or 0 for r in rows if "n_confirmed" in r]
    recall = [r.get("memory_recall") if r.get("memory_recall") is not None else 0 for r in rows if "n_confirmed" in r]
    err = [r.get("ee_err_m") for r in rows if r.get("ee_err_m") is not None]
    err_x = [r["step"] for r in rows if r.get("ee_err_m") is not None]

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes[0, 0].plot(xs, conf, "o-")
    axes[0, 0].set_title("Confirmed objects vs NBV step")
    axes[0, 0].set_xlabel("step")
    axes[0, 0].set_ylabel("n_confirmed")
    axes[0, 1].plot(xs, recall, "o-")
    axes[0, 1].set_title("Unique memory Recall vs NBV step")
    axes[0, 1].set_xlabel("step")
    axes[0, 1].set_ylabel("unique_recall")
    if err_x:
        axes[1, 0].plot(err_x, err, "o-")
    axes[1, 0].set_title("Camera EE arrival error vs step")
    axes[1, 0].set_xlabel("step")
    axes[1, 0].set_ylabel("ee_err_m")
    axes[1, 1].axis("off")
    axes[1, 1].text(
        0.05,
        0.5,
        "online_autonomous_nbv\nn_confirmed=%s\nterminate=%s\n%s"
        % (mem.get("n_confirmed"), args.terminated, args.terminate_reason),
        fontsize=11,
        family="monospace",
    )
    fig.tight_layout()
    fig.savefig(str(run_dir / "curves.png"), dpi=120)
    print(json.dumps({k: summary[k] for k in ("n_steps_dirs", "n_confirmed", "terminated", "terminate_reason")}, indent=2))
    print("SUMMARY_OK", run_dir)


if __name__ == "__main__":
    main()
