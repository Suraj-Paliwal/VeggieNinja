#!/usr/bin/env python3
"""
Replay a recorded g1_record episode on the meshed G1 (MuJoCo Menagerie), side by side with the head-camera video.
Kinematic replay of what the real robot did: its 29 logged joint angles (rt/lowstate) and IMU orientation; nothing
is simulated physically. Runs on the VM (dimos venv).

    MUJOCO_GL=egl python replay_episode.py ../../g1_record/data/<episode> [-o replay.mp4] [--fps 10] [--from S --to S]

Left: head camera (color.mkv). Right: the model in the logged pose. Both use the robot's own clock (video
timestamps color_ts.txt, lowstate t), so they are in sync without any clock correction.
Limits: there is no odometry, so the pelvis stays at the origin (a step shows as leg motion, not as travel); height
comes from the leg kinematics (feet assumed flat), heading is relative to the start of the clip.
"""

import argparse
import os
import subprocess
import sys

import cv2
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "..", "..", "g1_record")]
from inspect_episode import load_state, load_ts  # noqa: E402
from sim_view import MENAGERIE, pelvis_height  # noqa: E402

W, H = 640, 480


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--from", dest="t_from", type=float, default=0.0, help="seconds from the start of the video")
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    a = ap.parse_args()
    ep = a.episode.rstrip("/")
    out = a.out or os.path.join(ep, "replay_sim.mp4")
    ls = load_state(ep, "lowstate")
    ts = load_ts(ep, "color")
    if not ls or ts is None:
        sys.exit("episode needs state/lowstate_*.npz and color_ts.txt")

    m = mujoco.MjModel.from_xml_path(os.path.abspath(MENAGERIE))
    d = mujoco.MjData(m)
    joints = [m.joint(i).name for i in range(1, 30)]            # the robot's 29 joints, lowstate order
    adr = [m.joint(n).qposadr[0] for n in joints]
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.type, cam.distance, cam.azimuth, cam.elevation = mujoco.mjtCamera.mjCAMERA_FREE, 1.7, 150, -12

    yaw0 = ls["imu_rpy"][0][2]
    q_yaw0 = np.array([np.cos(-yaw0 / 2), 0, 0, np.sin(-yaw0 / 2)])  # heading relative to the clip start
    vid = cv2.VideoCapture(os.path.join(ep, "color.mkv"))
    t_end = ts[-1] - ts[0] if a.t_to is None else a.t_to
    ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s",
                           "%dx%d" % (2 * W, H + 34), "-r", str(a.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt",
                           "yuv420p", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    fi, frame, n = -1, None, 0
    q_start = ls["q"][0, :29]
    for t_rel in np.arange(a.t_from, t_end, 1.0 / a.fps):
        t = ts[0] + t_rel
        while fi + 1 < len(ts) and ts[fi + 1] <= t:                # newest camera frame at or before t
            ok, img = vid.read()
            fi += 1
            if ok:
                frame = img
        k = min(len(ls["t"]) - 1, int(np.searchsorted(ls["t"], t)))
        q29 = ls["q"][k, :29]
        d.qpos[:3] = [0, 0, pelvis_height(q29)]
        d.qpos[3:7] = quat_mul(q_yaw0, ls["imu_quat"][k])
        d.qpos[adr] = q29
        mujoco.mj_forward(m, d)
        cam.lookat = [0.1, 0, 0.75]
        r.update_scene(d, cam)
        sim = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
        cam_img = frame if frame is not None else np.zeros((H, W, 3), np.uint8)
        img = np.vstack([np.hstack([cv2.resize(cam_img, (W, H)), sim]), np.full((34, 2 * W, 3), 255, np.uint8)])
        rpy = np.degrees(ls["imu_rpy"][k])
        legs = np.degrees(np.abs(q29[:12] - q_start[:12]).max())
        right_arm = np.degrees(np.abs(q29[22:29] - q_start[22:29]).max())
        txt = ("t=%5.1f s | roll %+.1f pitch %+.1f yaw %+.1f deg | legs moved %4.1f deg | right arm moved %4.1f deg"
               % (t_rel, rpy[0], rpy[1], np.degrees(ls["imu_rpy"][k][2] - yaw0), legs, right_arm))
        cv2.putText(img, txt, (10, H + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
        cv2.putText(img, "head camera", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, "simulation (logged joints + IMU)", (W + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        ff.stdin.write(img.tobytes())
        n += 1
    ff.stdin.close()
    ff.wait()
    print("replay video: %s (%d frames at %g fps = %.1f s)" % (out, n, a.fps, n / a.fps))


if __name__ == "__main__":
    main()
