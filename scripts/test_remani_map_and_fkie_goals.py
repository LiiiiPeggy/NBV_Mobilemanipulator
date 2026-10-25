#!/usr/bin/env python3
"""Offline regression tests for C1 self-filter + C3 FKIE goal JSON plumbing.

Run: python3 scripts/test_remani_map_and_fkie_goals.py
"""
from __future__ import print_function

import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

# Stub rospy before importing converter (offline CI / no ROS master).
import types

_rospy = types.ModuleType("rospy")
sys.modules.setdefault("rospy", _rospy)
_tf2 = types.ModuleType("tf2_ros")
sys.modules.setdefault("tf2_ros", _tf2)
_geom = types.ModuleType("geometry_msgs.msg")
sys.modules.setdefault("geometry_msgs", types.ModuleType("geometry_msgs"))
sys.modules.setdefault("geometry_msgs.msg", _geom)
_geom.PoseStamped = object
_tft_mod = types.ModuleType("tf")
_tft_trans = types.ModuleType("tf.transformations")

def _qfe(r, p, y):
    # minimal xyzw for tests
    return [0.0, 0.0, 0.0, 1.0]

_tft_trans.quaternion_from_euler = _qfe
_tft_trans.quaternion_matrix = lambda q: __import__("numpy").eye(4)
_tft_trans.quaternion_from_matrix = lambda T: [0, 0, 0, 1]
sys.modules.setdefault("tf", _tft_mod)
sys.modules.setdefault("tf.transformations", _tft_trans)
_tft_mod.transformations = _tft_trans

from robot_self_filter import offline_self_filter_smoke  # noqa: E402
from nbv_to_remani_ee_goal import load_fkie_goal  # noqa: E402


def test_self_filter():
    r = offline_self_filter_smoke()
    assert r["ok"], r
    assert r["robot_removed"] > 0
    assert r["env_kept"] >= int(0.9 * r["env_in"])
    print("PASS self_filter", r)


def test_fkie_goal_requires_orientation():
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "bad.json"
        p.write_text(json.dumps({"camera_goal_xyz": [1.0, 2.0, 1.5]}))
        try:
            load_fkie_goal(str(p), require_orientation=True)
            raise AssertionError("expected SystemExit")
        except SystemExit:
            pass

        p2 = Path(td) / "good.json"
        p2.write_text(
            json.dumps(
                {
                    "goals": [
                        {
                            "pose": {
                                "position": {"x": 1.0, "y": 2.0, "z": 1.5},
                                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                            }
                        }
                    ]
                }
            )
        )
        xyz, q, src = load_fkie_goal(str(p2), require_orientation=True)
        assert xyz == [1.0, 2.0, 1.5]
        assert abs(q[3] - 1.0) < 1e-9
        assert src == "goals[0].pose"
    print("PASS fkie_goal_orientation")


def test_empty_goals_not_executable():
    # save_fkie_action_result contract: n_goals==0 must not produce camera_goal_*
    data = {"n_goals": 0, "goals": [], "action_state": 3}
    assert "camera_goal_xyz" not in data
    print("PASS empty_goals_guard")


def main():
    test_self_filter()
    test_fkie_goal_requires_orientation()
    test_empty_goals_not_executable()
    print("ALL_OFFLINE_TESTS_OK")


if __name__ == "__main__":
    main()
