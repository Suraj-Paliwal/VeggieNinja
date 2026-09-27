#!/usr/bin/env python3
"""
Show a planned okra grasp on the meshed G1 (MuJoCo Menagerie), kinematically: the joints follow the plan
exactly, nothing is simulated physically. Runs on the VM (dimos venv, needs the desktop / DISPLAY).

    MUJOCO_GL=glfw python sim_view.py trajectory.json                  # live viewer window, loops
    MUJOCO_GL=glfw python sim_view.py trajectory.json --video out.mp4  # render a video (front + side)

What you see (also in Guide/11_okra_pick_pipeline.md, "Simulation view"):
  OKRA     green pod with a dark cap, hanging by a thin stem from the PLANT stake behind it
  GRIPPER  Dex1 stand-in on the right wrist: dark body + two light jaws (parallel gripper) that open
           and close along the jaw axis from pick_config (the rubber hand of the model is hidden)
  lines    gripper path: blue = approach, orange = pull, grey = retreat
After the jaws close the pod moves with the gripper and the stem is removed (= harvested).
"""

import argparse
import json
import os
import subprocess
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
import pick_config as C  # noqa: E402
from g1_chain import Chain  # noqa: E402

MENAGERIE = os.path.join(os.path.dirname(mujoco.__file__), "..", "mujoco_playground", "external_deps",
                         "mujoco_menagerie", "unitree_g1", "scene.xml")
POD_R = 0.011                      # okra pod radius (m)
FINGER_T = 0.006                  # finger thickness along the closing axis (m)
JAW_OPEN, JAW_CLOSED = 0.040 + FINGER_T / 2, POD_R + FINGER_T / 2   # finger centre from the centre line (m)
LEGEND = "green pod = OKRA | pole = PLANT | dark body + 2 orange fingers = Dex1 | red dot = grasp point | path: blue/orange/grey"


def quat_z_to(v):
    """Quaternion (w,x,y,z) rotating +z onto v."""
    v = np.asarray(v, float) / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])
    c = np.cross(z, v)
    if np.linalg.norm(c) < 1e-9:
        return np.array([1.0, 0, 0, 0]) if v[2] > 0 else np.array([0.0, 1, 0, 0])
    q = np.r_[1.0 + np.dot(z, v), c]
    return q / np.linalg.norm(q)


def add_line(body, a, b, rgba, r=0.0025):
    g = body.add_geom()
    g.type, g.size, g.fromto = mujoco.mjtGeom.mjGEOM_CAPSULE, [r, 0, 0], np.r_[a, b]
    g.rgba, g.contype, g.conaffinity, g.group = rgba, 0, 0, 1
    return g


def build_model(traj, base_z):
    spec = mujoco.MjSpec.from_file(os.path.abspath(MENAGERIE))
    for g in spec.geoms:                                    # hide the rubber hand: the robot has a Dex1
        if g.meshname == "right_rubber_hand":
            g.rgba, g.material, g.group = [0, 0, 0, 0], "", 5
    wb = spec.worldbody
    off = np.array([0, 0, base_z])
    tcp = np.array(C.TCP_XYZ)

    # ---- Dex1 stand-in on the right wrist: body + two jaws on slide joints along the jaw axis
    wrist = spec.body("right_wrist_yaw_link")
    axis = [0, 1, 0] if C.CLOSE_AXIS == "y" else [0, 0, 1]
    other = [0, 0, 1] if C.CLOSE_AXIS == "y" else [0, 1, 0]
    base = wrist.add_body(name="dex1_base", pos=[0.0415, -0.003, 0])
    g = base.add_geom()
    g.type, g.size, g.pos = mujoco.mjtGeom.mjGEOM_BOX, [0.03, 0.045, 0.025], [0.03, 0, 0]
    g.rgba, g.contype, g.conaffinity = [0.15, 0.15, 0.17, 1], 0, 0
    s = base.add_site(name="GRIPPER", pos=[0.03, 0, 0.0] + 0.045 * np.array(other), size=[0.004, 0, 0])
    s.rgba = [1, 1, 1, 1]
    for side in (+1, -1):
        jb = wrist.add_body(name="jaw_%s" % ("a" if side > 0 else "b"), pos=tcp)
        j = jb.add_joint(name="jaw_%s_slide" % ("a" if side > 0 else "b"))
        j.type, j.axis, j.range = mujoco.mjtJoint.mjJNT_SLIDE, np.array(axis) * side, [-1, 1]
        jg = jb.add_geom()                                   # one finger: 6 cm long, 6 mm thick, 2.4 cm tall
        jg.type = mujoco.mjtGeom.mjGEOM_BOX
        jg.size = np.array([0.03, 0, 0]) + FINGER_T / 2 * np.abs(axis) + 0.012 * np.abs(other)
        jg.pos = [-0.01, 0, 0]
        jg.rgba, jg.contype, jg.conaffinity = [1.0, 0.45, 0.05, 1], 0, 0
    gp = wrist.add_site(name="grasp_point", pos=tcp, size=[0.005, 0, 0])
    gp.rgba = [1, 0, 0, 1]

    # ---- okra pod (mocap: stays put, then follows the gripper once grasped)
    ax = np.array(traj["target"].get("axis_pelvis") or [0, 0, 1.0], float)
    ax /= np.linalg.norm(ax)
    L = max(0.06, min(0.2, traj["target"].get("length_m") or 0.10))
    pod_c = np.array(traj["pod_xyz"]) + off
    pod = wb.add_body(name="okra", mocap=True, pos=pod_c, quat=quat_z_to(ax))
    for gg, (typ, size, pos, rgba) in enumerate([
            (mujoco.mjtGeom.mjGEOM_CAPSULE, [POD_R, L / 2 - POD_R, 0], [0, 0, 0], [0.30, 0.62, 0.12, 1]),
            (mujoco.mjtGeom.mjGEOM_ELLIPSOID, [POD_R * 1.25, POD_R * 1.25, 0.012], [0, 0, L / 2 - 0.004], [0.12, 0.30, 0.06, 1])]):
        pg = pod.add_geom()
        pg.type, pg.size, pg.pos, pg.rgba, pg.contype, pg.conaffinity = typ, size, pos, rgba, 0, 0
    ps = pod.add_site(name="OKRA", pos=[0, 0, -L / 2 - 0.02], size=[0.003, 0, 0])
    ps.rgba = [1, 1, 1, 1]

    # ---- plant: stake behind the pod (away from the robot) and a stem to the pod's top
    a = np.array(traj["approach_dir"]); a[2] = 0; a /= np.linalg.norm(a)
    top = pod_c + ax * L / 2
    stake = top + 0.07 * a
    add_line(wb, [stake[0], stake[1], 0.0], [stake[0], stake[1], top[2] + 0.30], [0.25, 0.35, 0.15, 1], r=0.008)
    stem = add_line(wb, stake, top, [0.20, 0.45, 0.10, 1], r=0.002)
    stem.name = "stem"
    ss = wb.add_site(name="PLANT", pos=[stake[0], stake[1], top[2] + 0.12], size=[0.003, 0, 0])
    ss.rgba = [1, 1, 1, 1]

    # ---- gripper path as thin lines
    p = {k: np.array(v) + off for k, v in traj["tcp_path"].items()}
    add_line(wb, p["pregrasp"], p["grasp"], [0.2, 0.4, 1.0, 0.6])
    add_line(wb, p["grasp"], p["pull"], [1.0, 0.55, 0.1, 0.8])
    add_line(wb, p["pull"], p["retreat"], [0.55, 0.55, 0.55, 0.6])
    return spec.compile()


def pelvis_height(q29):
    c = Chain(os.path.join(HERE, "..", "robot", "g1.urdf"))
    names = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
             "left_ankle_pitch_joint", "left_ankle_roll_joint"]
    return -c.fk("pelvis", "left_ankle_roll_link", dict(zip(names, q29[:6])))[2, 3] + 0.035


class Playback:
    def __init__(self, m, d, traj, base_z):
        self.m, self.d, self.t, self.base_z = m, d, traj, base_z
        self.q29 = traj["target"]["q"]
        self.close_at = traj["events"][0]["at"]
        self.seg_of = {}
        for s in traj["segments"]:
            for i in range(s["start"], s["end"]):
                self.seg_of[i] = s["name"]
        self.body_joints = [m.joint(i).name for i in range(1, 30)]     # the robot's 29 joints
        self.pod0 = (d.mocap_pos[0].copy(), d.mocap_quat[0].copy())
        self.stem = m.geom("stem").id
        self.stem_rgba = m.geom_rgba[self.stem].copy()
        self.tcp_site = m.site("GRIPPER").id
        self.grip_rel = None

    def jaw(self, i):
        n = int(0.6 / C.DT)                                           # jaws take ~0.6 s to close
        k = min(1.0, max(0.0, (i - self.close_at) / n))
        return JAW_OPEN + (JAW_CLOSED - JAW_OPEN) * k

    def set(self, i):
        m, d = self.m, self.d
        d.qpos[:3], d.qpos[3:7] = [0, 0, self.base_z], [1, 0, 0, 0]
        for k, n in enumerate(self.body_joints):
            d.qpos[m.joint(n).qposadr[0]] = self.q29[k]
        for n, v in zip(C.RIGHT_ARM, self.t["q"][i]):
            d.qpos[m.joint(n).qposadr[0]] = v
        d.qpos[m.joint("jaw_a_slide").qposadr[0]] = d.qpos[m.joint("jaw_b_slide").qposadr[0]] = self.jaw(i)
        if i < self.close_at:
            d.mocap_pos[0], d.mocap_quat[0] = self.pod0
            m.geom_rgba[self.stem] = self.stem_rgba
            self.grip_rel = None
        mujoco.mj_forward(m, d)
        wb = m.body("right_wrist_yaw_link").id
        Rw, pw = d.xmat[wb].reshape(3, 3), d.xpos[wb]
        if i >= self.close_at:
            if self.grip_rel is None:                                 # grasped: remember pod in wrist frame
                Rp = np.zeros(9)
                mujoco.mju_quat2Mat(Rp, d.mocap_quat[0])
                self.grip_rel = (Rw.T @ (d.mocap_pos[0] - pw), Rw.T @ Rp.reshape(3, 3))
            rp, Rr = self.grip_rel
            d.mocap_pos[0] = pw + Rw @ rp
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, (Rw @ Rr).flatten())
            d.mocap_quat[0] = q
            if self.seg_of[i] != "approach":
                m.geom_rgba[self.stem] = [0, 0, 0, 0]                 # pulled off the plant
            mujoco.mj_forward(m, d)
        gap = np.linalg.norm(d.site_xpos[m.site("grasp_point").id] - d.mocap_pos[0])
        return "%s | gripper %s | grasp point to pod %.1f cm | t=%.1fs" % (
            self.seg_of[i], "CLOSED" if i >= self.close_at else "open", 100 * gap, i * C.DT)


def camera(azimuth, lookat):
    cam = mujoco.MjvCamera()
    cam.type, cam.lookat, cam.distance, cam.azimuth, cam.elevation = (
        mujoco.mjtCamera.mjCAMERA_FREE, lookat, 1.3, azimuth, -15)
    return cam


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trajectory")
    ap.add_argument("--video")
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()
    traj = json.load(open(args.trajectory))
    base_z = pelvis_height(traj["target"]["q"])
    m = build_model(traj, base_z)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    pb = Playback(m, d, traj, base_z)
    lookat = np.array(traj["pod_xyz"]) * 0.7 + [0, 0, base_z]
    opt = mujoco.MjvOption()
    opt.label = mujoco.mjtLabel.mjLABEL_SITE

    if args.video:
        import cv2
        W, H = 560, 480
        r = mujoco.Renderer(m, H, W)
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                               "-s", "%dx%d" % (2 * W, H + 30), "-r", "25", "-i", "-", "-c:v", "libx264",
                               "-pix_fmt", "yuv420p", args.video], stdin=subprocess.PIPE)
        for i in range(0, len(traj["q"]), 2):                        # 50 Hz plan -> 25 fps video
            label = pb.set(i)
            views = []
            for az in (200, 95):                                      # front and right side
                r.update_scene(d, camera(az, lookat), scene_option=opt)
                views.append(r.render().copy())
            img = np.vstack([np.hstack(views), np.full((30, 2 * W, 3), 255, np.uint8)])
            cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            cv2.putText(img, LEGEND, (10, H + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
            ff.stdin.write(img.tobytes())
        ff.stdin.close()
        ff.wait()
        print("video: %s (%.1f s)" % (args.video, len(traj["q"]) * C.DT))
        return

    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat, v.cam.distance, v.cam.azimuth, v.cam.elevation = lookat, 1.3, 200, -15
        v.opt.label = mujoco.mjtLabel.mjLABEL_SITE
        while v.is_running():
            for i in range(len(traj["q"])):
                if not v.is_running():
                    break
                pb.set(i)
                v.sync()
                time.sleep(C.DT / args.speed)
            time.sleep(1.0)


if __name__ == "__main__":
    main()
