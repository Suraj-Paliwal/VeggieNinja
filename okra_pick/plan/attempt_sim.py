#!/usr/bin/env python3
"""
Simulate the grab attempt for a Look event, even when the planner refuses it: show how close the arm gets.
VM only, nothing is sent to the robot.

    MUJOCO_GL=egl python attempt_sim.py <event dir>        -> <event>/attempt_sim.mp4 (+ attempt_trajectory.json)

First the normal planner. If it refuses, a best-effort version is made for this picture only: every straight line
is followed with the planner's normal IK accuracy as far as the arm can, then the arm holds there (truncated, never
forced). The jaws never close in that case (no grasp), and the gap between the grasp point and the pod at the end of
the approach is measured and shown. Such a trajectory must never be played on the robot: it is written as
attempt_trajectory.json, which nothing else reads.
"""

import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "..", "robot")]
import okra_plan as P  # noqa: E402
import pick_config as C  # noqa: E402
from g1_chain import Chain  # noqa: E402

REAL_CARTESIAN = P.cartesian


STOPPED = {"now": False}                          # the current attempt's approach stopped short


def truncated_cartesian(arm, q0, R, p0, p1, speed, name, twist_deg=0.0, twist_frac=1.0):
    """Like okra_plan.cartesian, but stops where the IK can no longer follow the line and holds that pose.
    After the approach stopped short, pull and retreat just hold too (no grasp happened; no IK branch jumps)."""
    if name == "approach":
        STOPPED["now"] = False
    T = np.linalg.norm(p1 - p0) / speed
    if twist_deg:
        T = max(T, abs(twist_deg) / C.TWIST_SPEED_DEG / twist_frac)
    n = max(2, int(np.ceil(T / C.DT)) + 1)
    out, q = [], q0
    if name != "approach" and STOPPED["now"]:
        return np.array([q0] * n)
    for s in np.linspace(0, 1, n):
        Rs = R @ P.rot_x(twist_deg * min(1.0, s / twist_frac)) if twist_deg else R
        q_new, pe, re = arm.ik(P.pin.SE3(Rs, p0 + s * (p1 - p0)), q, iters=150)
        jump = np.abs(np.asarray(q_new) - np.asarray(q)).max() / C.DT > 1.2 if out else False   # planner limit 1.5
        if pe > C.IK_POS_TOL or re > C.IK_ROT_TOL_DEG or jump:
            print("  %s: arm can follow the line to %.0f%%, holds there%s" % (name, 100 * s, " (joint jump)" if jump else ""), flush=True)
            STOPPED["now"] = True
            out.extend([q] * (n - len(out)))
            break
        q = q_new
        out.append(q)
    return np.array(out)


def grasp_gap(traj, q29):
    """Distance grasp point -> pod at the end of the approach segment (m)."""
    ch = Chain(os.path.join(HERE, "..", "robot", "g1.urdf"))
    end = next(s["end"] for s in traj["segments"] if s["name"] == "approach") - 1
    joints = dict(zip(("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"), q29[12:15]))
    joints.update(zip(C.RIGHT_ARM, traj["q"][end]))
    T = ch.fk("pelvis", "right_wrist_yaw_link", joints)
    tcp = T[:3, :3] @ np.array(C.TCP_XYZ) + T[:3, 3]
    return float(np.linalg.norm(tcp - np.array(traj["pod_xyz"])))


def main():
    ev = sys.argv[1].rstrip("/")
    target = json.load(open(os.path.join(ev, "target.json")))
    best_effort = False
    try:
        traj = P.plan(target)
    except P.PlanError as e:
        print("normal planner: REFUSED (%s)" % str(e).splitlines()[1].strip() if "\n" in str(e) else e)
        P.cartesian = truncated_cartesian
        try:
            traj = P.plan(target)
        finally:
            P.cartesian = REAL_CARTESIAN
        best_effort = True
    gap = grasp_gap(traj, target["q"])
    if best_effort:
        traj["events"][0]["at"] = len(traj["q"]) + 1               # jaws never close: not a real grasp
        print("BEST EFFORT (picture only): the grasp point stops %.1f cm from the pod" % (100 * gap))
    else:
        print("the normal planner accepts this pod: grasp point reaches it (%.1f cm)" % (100 * gap))
    tj = os.path.join(ev, "attempt_trajectory.json")
    json.dump(traj, open(tj, "w"))
    out = os.path.join(ev, "attempt_sim.mp4")
    subprocess.run([sys.executable, os.path.join(HERE, "sim_view.py"), tj, "--video", out], check=True,
                   env=dict(os.environ, MUJOCO_GL=os.environ.get("MUJOCO_GL", "egl")))
    json.dump({"normal_planner_ok": not best_effort, "grasp_gap_m": gap, "video": out},
              open(os.path.join(ev, "attempt_result.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
