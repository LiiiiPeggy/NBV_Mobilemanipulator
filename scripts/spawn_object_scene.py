#!/usr/bin/env python3
"""Spawn furniture/objects from a layout JSON into a running Gazebo Classic.

Fail-closed by default: any required object that fails spawn or post-check
exits non-zero. Use --allow-partial for debug only.
"""
from __future__ import print_function
import argparse
import json
import os
import re
import sys
from pathlib import Path

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, GetWorldProperties
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft


def quat_yaw(yaw):
    q = tft.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def model_paths():
    root = Path(os.environ.get("NBV_ROOT", "/workspace"))
    return {
        "ycb": root / "third_party/gazebo_objects/gazebo_ycb/models",
        "local": root / "configs/fkie/agx_adapt/models",
    }


def resolve_sdf(item, paths):
    model = item["model"]
    src = item.get("source", "ycb")
    if src == "ycb":
        model_dir = paths["ycb"] / model
    else:
        model_dir = paths["local"] / model
    p = model_dir / "model.sdf"
    if not p.is_file():
        raise FileNotFoundError("missing model.sdf: %s" % p)
    return p.read_text(), model, model_dir


def check_mesh_uris(xml, model_dir):
    """Validate mesh/texture files referenced after URI rewrite."""
    issues = []
    abs_model = model_dir.resolve()
    # After prepare_sdf, URIs become file://...
    for m in re.finditer(r"file://([^\s\"'<]+)", xml):
        path = Path(m.group(1))
        if not path.is_file():
            issues.append("missing asset: %s" % path)
    # Also check common texture next to textured.dae
    meshes = abs_model / "meshes"
    if meshes.is_dir():
        for dae in meshes.glob("*.dae"):
            txt = dae.read_text(errors="ignore")
            for im in re.finditer(r"<init_from>([^<]+)</init_from>", txt):
                name = im.group(1).strip()
                if name.endswith((".png", ".jpg", ".jpeg", ".tga")):
                    cand = dae.parent / name
                    if not cand.is_file():
                        # sometimes under materials/textures
                        alt = abs_model / "materials" / "textures" / name
                        alt2 = abs_model / "meshes" / name
                        if not alt.is_file() and not alt2.is_file():
                            issues.append("missing texture for %s: %s" % (dae.name, name))
    return issues


def prepare_sdf(xml, model_dir, force_static=True):
    """Rewrite model:// URIs to absolute file:// and optionally force static."""
    abs_model = model_dir.resolve().as_posix()
    parent = model_dir.parent.resolve().as_posix()
    model_name = model_dir.name
    xml = xml.replace("model://%s/" % model_name, "file://%s/" % abs_model)

    def repl(m):
        uri = m.group(1)
        if uri.startswith("/"):
            return "file://%s" % uri
        return "file://%s/%s" % (parent, uri)

    xml = re.sub(r"model://([^\s\"'<]+)", repl, xml)
    if force_static and "<static>" not in xml:
        xml = re.sub(
            r"(<model[^>]*>)",
            r"\1\n    <static>true</static>",
            xml,
            count=1,
        )
    return xml


def world_model_names(get_world):
    try:
        w = get_world()
        return set(w.model_names)
    except Exception:
        return set()


def spawn_one(
    spawn_srv,
    delete_srv,
    name,
    xml,
    pose_dict,
    get_state,
    idempotent=True,
    only_missing=False,
    tol_xy=0.15,
    tol_z=0.25,
):
    """Spawn layout-owned model. Never deletes models outside this layout name.

    only_missing: if present and pose within tol, skip; if present but drifted, replace;
                  if absent, spawn. Does not touch unrelated world models.
    """
    existed = False
    try:
        st = get_state(name, "world")
        if st.success:
            existed = True
            if only_missing:
                p = st.pose.position
                if (
                    abs(p.x - pose_dict["x"]) <= tol_xy
                    and abs(p.y - pose_dict["y"]) <= tol_xy
                    and abs(p.z - pose_dict["z"]) <= tol_z
                ):
                    return True, "already_present_pose_ok", True
                delete_srv(name)
                rospy.sleep(0.25)
            elif idempotent:
                delete_srv(name)
                rospy.sleep(0.25)
            else:
                return True, "already_exists_skipped", True
    except Exception:
        pass
    pose = Pose()
    pose.position = Point(x=pose_dict["x"], y=pose_dict["y"], z=pose_dict["z"])
    pose.orientation = quat_yaw(float(pose_dict.get("yaw", 0.0)))
    resp = spawn_srv(name, xml, "", pose, "world")
    return bool(resp.success), resp.status_message, existed


def post_check(get_state, name, pose_dict, tol_xy=0.15, tol_z=0.25):
    st = get_state(name, "world")
    if not st.success:
        return False, {"error": "get_model_state failed after spawn"}, None
    p = st.pose.position
    dx = abs(p.x - pose_dict["x"])
    dy = abs(p.y - pose_dict["y"])
    dz = abs(p.z - pose_dict["z"])
    ok = dx <= tol_xy and dy <= tol_xy and dz <= tol_z
    actual = {"x": float(p.x), "y": float(p.y), "z": float(p.z)}
    return ok, {"dx": dx, "dy": dy, "dz": dz, "tol_xy": tol_xy, "tol_z": tol_z}, actual


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", required=True)
    ap.add_argument("--prefix", default="mo_")
    ap.add_argument(
        "--allow-partial",
        action="store_true",
        help="Debug only: do not exit non-zero if some objects fail",
    )
    ap.add_argument("--skip-asset-check", action="store_true")
    ap.add_argument(
        "--only-missing",
        action="store_true",
        help="Fill missing/drifted layout models only; do not delete unrelated world models",
    )
    ap.add_argument("--out-json", default="")
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text())
    paths = model_paths()
    rospy.init_node("spawn_object_scene", anonymous=True)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=90)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    get_world = None
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=10)
        get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    except Exception:
        get_world = None

    items = (
        list(layout.get("furniture", []))
        + list(layout.get("objects", []))
        + list(layout.get("occluders", []))
    )
    required_targets = list(layout.get("objects", []))
    results = []
    n_fail = 0

    for item in items:
        name = args.prefix + item["instance_id"]
        entry = {
            "gazebo_name": name,
            "instance_id": item.get("instance_id"),
            "model": item.get("model"),
            "class_name": item.get("class_name"),
            "source": item.get("source"),
            "difficulty": item.get("difficulty"),
            "layout_spawn": item.get("spawn"),
            "ok": False,
            "msg": "",
            "asset_issues": [],
            "exists_after": False,
            "pose_ok": False,
            "actual_pose": None,
            "pose_delta": None,
        }
        try:
            xml, _, model_dir = resolve_sdf(item, paths)
            xml = prepare_sdf(xml, model_dir, force_static=True)
            if not args.skip_asset_check:
                issues = check_mesh_uris(xml, model_dir)
                entry["asset_issues"] = issues
                if issues:
                    entry["msg"] = "asset_check_failed"
                    n_fail += 1
                    print("FAIL_ASSET", name, issues)
                    results.append(entry)
                    continue
            ok, msg, existed = spawn_one(
                spawn_srv,
                delete_srv,
                name,
                xml,
                item["spawn"],
                get_state,
                only_missing=bool(args.only_missing),
            )
            entry["msg"] = msg
            entry["existed_before"] = bool(existed)
            entry["ok"] = bool(ok)
            if not ok:
                n_fail += 1
                print("spawn", name, False, msg)
                results.append(entry)
                continue
            if msg == "already_present_pose_ok":
                pose_ok, delta, actual = post_check(get_state, name, item["spawn"])
                entry["exists_after"] = True
                entry["pose_ok"] = bool(pose_ok)
                entry["actual_pose"] = actual
                entry["pose_delta"] = delta
                print("spawn", name, "SKIP_PRESENT", actual)
                results.append(entry)
                continue
            rospy.sleep(0.15)
            pose_ok, delta, actual = post_check(get_state, name, item["spawn"])
            entry["exists_after"] = actual is not None
            entry["pose_ok"] = bool(pose_ok)
            entry["actual_pose"] = actual
            entry["pose_delta"] = delta
            if not entry["exists_after"]:
                entry["ok"] = False
                n_fail += 1
                print("spawn", name, "OK_THEN_MISSING", msg)
            elif not pose_ok:
                # still count as spawned but warn; pose drift beyond tol fails formal mode
                entry["ok"] = False
                n_fail += 1
                print("spawn", name, "POSE_DRIFT", delta, "actual", actual)
            else:
                print("spawn", name, True, msg, "pose", actual)
        except Exception as e:
            entry["msg"] = str(e)
            n_fail += 1
            print("spawn", name, "EXCEPTION", e)
        results.append(entry)

    # World inventory after spawn
    world_names = sorted(world_model_names(get_world)) if get_world else []
    expected_names = [args.prefix + it["instance_id"] for it in items]
    missing_in_world = [n for n in expected_names if n not in world_names] if world_names else []

    out_path = Path(args.out_json) if args.out_json else Path(args.layout).with_suffix(".spawned.json")
    payload = {
        "layout": args.layout,
        "scene_id": layout.get("scene_id"),
        "scene_kind": layout.get("scene_kind"),
        "n_requested": len(items),
        "n_ok": sum(1 for r in results if r["ok"]),
        "n_fail": n_fail,
        "n_required_targets": len(required_targets),
        "n_required_ok": sum(
            1
            for r in results
            if r.get("class_name") not in ("table", "occluder") and r["ok"]
        ),
        "missing_in_world": missing_in_world,
        "world_model_names": world_names,
        "spawned": results,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2))
    print("Wrote", out_path)

    all_ok = n_fail == 0 and payload["n_required_ok"] == len(required_targets)
    if all_ok:
        print("SPAWN_OBJECT_SCENE_OK", payload["n_ok"], "/", len(results))
        return 0
    print("SPAWN_OBJECT_SCENE_FAIL", payload["n_ok"], "/", len(results), "fail", n_fail)
    if args.allow_partial:
        print("SPAWN_OBJECT_SCENE_PARTIAL (allow-partial)")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
