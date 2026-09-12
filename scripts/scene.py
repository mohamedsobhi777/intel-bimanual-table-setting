"""View, render and smoke-test the dual SO-101 scene (no manipulation policy yet)."""
import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
HOME_POSE = (0, -1.0, 0, 0, 0, 0.7)


def load(seed=0):
    model = mujoco.MjModel.from_xml_path(str(ROOT / "scenes/scene.xml"))
    data = mujoco.MjData(model)
    reset(model, data, seed)
    return model, data


def reset(model, data, seed=0):
    """Restore both joint positions and servo targets, never the crossing zero pose."""
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    rng = np.random.default_rng(seed)
    # Small, non-overlapping reset variation; not the full challenge randomization.
    for name in ("plate", "cup", "bottle", "fork", "spoon"):
        data.joint(f"{name}_free").qpos[:2] += rng.uniform(-0.008, 0.008, 2)
    mujoco.mj_forward(model, data)


def check_arm_clearance(model, data):
    left = data.site("left/gripperframe").xpos
    right = data.site("right/gripperframe").xpos
    assert right[0] - left[0] > 0.30, ("Home grippers too close", left, right)
    for contact in data.contact:
        a = model.body(int(model.geom_bodyid[contact.geom1])).name
        b = model.body(int(model.geom_bodyid[contact.geom2])).name
        assert not ((a.startswith("left/") and b.startswith("right/")) or
                    (a.startswith("right/") and b.startswith("left/"))), (a, b)


def settle(model, data, steps, verify_arms=False):
    for _ in range(steps):
        mujoco.mj_step(model, data)
        if verify_arms:
            check_arm_clearance(model, data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("Non-finite simulation state")
    if any(w.number for w in data.warning):
        raise RuntimeError("MuJoCo emitted simulation warnings")


def check(model, data):
    assert model.nu == 12, f"Expected 12 actuators, got {model.nu}"
    assert model.nq == 48 and model.nv == 43
    for name in ("overview", "overhead", "front", "left/wrist_cam", "right/wrist_cam"):
        assert model.camera(name).id >= 0
    for name in ("plate", "cup", "bottle", "fork", "spoon"):
        pos = data.body(name).xpos
        assert 0.70 < pos[2] < 0.90, (name, pos)
        assert abs(pos[0]) < 0.50 and abs(pos[1]) < 0.34, (name, pos)
    assert -0.002 <= data.joint("drawer_slide").qpos[0] <= 0.142


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--check", action="store_true", help="Check 10 seeds, five seconds each")
    parser.add_argument("--render", type=Path, help="Write a PNG and exit")
    parser.add_argument("--camera", default="overview")
    args = parser.parse_args()
    if args.check:
        for seed in range(10):
            model, data = load(seed)
            check_arm_clearance(model, data)
            settle(model, data, 2500, verify_arms=True)
            check(model, data)
            print(json.dumps({"seed": seed, "seconds": round(data.time, 3), "actuators": model.nu, "status": "passed"}))
        # Test the passive drawer mechanism directly, independent of a robot policy.
        dof = model.joint("drawer_slide").dofadr[0]
        data.qfrc_applied[dof] = 3.0
        settle(model, data, 1000)
        assert data.joint("drawer_slide").qpos[0] > 0.12, "Drawer cannot open"
        data.qfrc_applied[dof] = -3.0
        settle(model, data, 1000)
        assert data.joint("drawer_slide").qpos[0] < 0.02, "Drawer cannot close"
        data.qfrc_applied[dof] = 0
        print(json.dumps({"drawer_open_close": "passed", "applied_force_N": 3}))
        # Exercise the same recovery used after the viewer's built-in reset.
        mujoco.mj_resetData(model, data)
        reset(model, data)
        check_arm_clearance(model, data)
        settle(model, data, 5000, verify_arms=True)
        print(json.dumps({"home_reset_and_10s_hold": "passed"}))
        return
    model, data = load(args.seed)
    settle(model, data, 500)
    if args.render:
        from PIL import Image
        args.render.parent.mkdir(parents=True, exist_ok=True)
        with mujoco.Renderer(model, height=960, width=1280) as renderer:
            option = mujoco.MjvOption()
            option.geomgroup[3:5] = 0  # Hide collision proxies; keep physical collisions enabled.
            renderer.update_scene(data, camera=args.camera, scene_option=option)
            Image.fromarray(renderer.render()).save(args.render)
        print(args.render)
        return
    from mujoco import viewer as mj_viewer
    with mj_viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera(args.camera).id
        viewer.opt.geomgroup[3:5] = 0
        while viewer.is_running():
            start = time.monotonic()
            mujoco.mj_step(model, data)
            simulation_time = data.time
            viewer.sync()
            # Backspace / Reset in the native viewer restores qpos0 and zero
            # servo targets. Replace that state before the next physics step.
            if data.time < simulation_time:
                with viewer.lock():
                    reset(model, data, args.seed)
            time.sleep(max(0, model.opt.timestep - (time.monotonic() - start)))


if __name__ == "__main__":
    main()
