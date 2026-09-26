#!/usr/bin/env python3
"""
Find one okra pod and write its position in the robot's pelvis frame. Runs ON THE ROBOT (Orin),
conda env g1brainco (Python 3.8: torch+CUDA, ultralytics, pyrealsense2, unitree_sdk2py).

    python okra_perceive.py --out target.json [--frames 15] [--timeout 25]
    python okra_perceive.py --floor-check          # camera extrinsic sanity check, no detection

Needs the head RealSense free (Unitree videohub stopped: okra_pick.sh does that).

Per frame: colour + depth aligned to colour (pyrealsense2) -> OkraDetector (GPU) -> accepted pods
with xyz_m (camera optical frame) -> pod long axis from the mask -> pelvis frame via the URDF chain and
the live waist angles from rt/lowstate. The first pod seen inside the reach zone is tracked; after
--frames consistent sightings (within MATCH_M) the median is written as target.json.
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "okra_robot"))       # detector copied next to this folder
import pick_config as C                                            # noqa: E402
from g1_chain import Chain, T_pelvis_optical                       # noqa: E402

POD_RADIUS = 0.01      # m: the mask depth is the pod's front surface; its axis is ~1 cm further
MATCH_M = 0.03         # m: sightings closer than this to the tracked pod count as the same pod
MAX_SPREAD = 0.02      # m: refuse if the sightings disagree more than this


class LowState:
    def __init__(self, iface):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        ChannelFactoryInitialize(0, iface)
        self.msg, self.t = None, 0.0
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._cb, 10)

    def _cb(self, m):
        self.msg, self.t = m, time.time()

    def q(self):
        if self.msg is None or time.time() - self.t > 0.5:
            return None
        return [s.q for s in self.msg.motor_state[:29]]

    def rpy(self):
        return list(self.msg.imu_state.rpy) if self.msg is not None else None


def start_camera():
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    for _ in range(15):                                   # let auto-exposure settle
        pipe.wait_for_frames()
    return pipe, rs.align(rs.stream.color), scale, intr


def grab(pipe, align, scale):
    fs = align.process(pipe.wait_for_frames())
    c, d = fs.get_color_frame(), fs.get_depth_frame()
    return np.asanyarray(c.get_data()).copy(), np.asanyarray(d.get_data()).astype(np.float32) * scale


def pod_axis_cam(poly, depth_img, K, fallback_depth):
    """Unit vector along the pod (optical frame): the mask's long axis, with the real depth sampled at
    points 35 % of the length either side of the centre (5x5 median; falls back to the mask depth)."""
    poly = np.asarray(poly, np.float32)
    x0, y0 = np.floor(poly.min(0)).astype(int)
    w, h = np.ceil(poly.max(0)).astype(int) - (x0, y0) + 1
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [np.round(poly - (x0, y0)).astype(np.int32)], 1)
    ys, xs = np.nonzero(m)
    P = np.c_[xs + x0, ys + y0].astype(np.float64)              # mask pixels (PCA: long axis)
    c = P.mean(0)
    evals, evecs = np.linalg.eigh(np.cov((P - c).T))
    d = evecs[:, 1]
    proj = (P - c) @ d
    fx, fy, ppx, ppy = K
    pts = []
    for t in (np.percentile(proj, 15), np.percentile(proj, 85)):   # ~35 % either side, inside the ends
        u, v = c + t * d
        z = fallback_depth
        if depth_img is not None:
            ui, vi = int(round(u)), int(round(v))
            win = depth_img[max(0, vi - 2):vi + 3, max(0, ui - 2):ui + 3]
            win = win[(win > 0.05) & (win < 5.0)]
            if len(win):
                z = float(np.median(win))
        pts.append(np.array([(u - ppx) * z / fx, (v - ppy) * z / fy, z]))
    a = pts[1] - pts[0]
    return a / np.linalg.norm(a) if np.linalg.norm(a) > 1e-6 else None


def in_reach(p):
    return all(lo <= v <= hi for v, (lo, hi) in zip(p, (C.REACH_X, C.REACH_Y, C.REACH_Z)))


def floor_check(pipe, align, scale, intr, chain, ls):
    """Fit the floor plane in the pelvis frame; its height should be about -(pelvis height)."""
    _, depth = grab(pipe, align, scale)
    q = ls.q()
    if q is None:
        sys.exit("no rt/lowstate")
    T = T_pelvis_optical(chain, q[12:15])
    v, u = np.mgrid[240:480:4, 0:640:4]
    z = depth[v, u]
    ok = (z > 0.3) & (z < 3.0)
    pts = np.stack([(u[ok] - intr.ppx) * z[ok] / intr.fx, (v[ok] - intr.ppy) * z[ok] / intr.fy, z[ok]], 1)
    P = pts @ T[:3, :3].T + T[:3, 3]
    for _ in range(3):                                    # least squares z = a x + b y + c, drop outliers
        A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
        coef, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
        r = P[:, 2] - A @ coef
        P = P[np.abs(r) < max(0.01, 2.5 * r.std())]
    tilt = np.degrees(np.arctan(np.hypot(coef[0], coef[1])))
    names = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
             "left_ankle_pitch_joint", "left_ankle_roll_joint"]
    foot_z = chain.fk("pelvis", "left_ankle_roll_link", dict(zip(names, q[:6])))[2, 3] - 0.035
    print(json.dumps({"floor_height_below_camera_model": float(coef[2]), "floor_from_leg_kinematics": float(foot_z),
                      "difference_m": float(coef[2] - foot_z), "floor_tilt_deg": float(tilt),
                      "imu_rpy_deg": [float(np.degrees(x)) for x in ls.rpy()], "points": int(len(P))}, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="target.json")
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--weights", default=None)
    ap.add_argument("--floor-check", action="store_true")
    args = ap.parse_args()

    chain = Chain(os.path.join(HERE, "g1.urdf"))
    ls = LowState(args.iface)
    pipe, align, scale, intr = start_camera()
    K = (intr.fx, intr.fy, intr.ppx, intr.ppy)
    try:
        t0 = time.time()
        while ls.q() is None:
            if time.time() - t0 > 10:
                sys.exit("no rt/lowstate from the robot")
            time.sleep(0.1)
        if args.floor_check:
            floor_check(pipe, align, scale, intr, chain, ls)
            return

        from okra_detector import OkraDetector
        det = OkraDetector(args.weights, stream=True)
        print("detector %s, camera fx %.1f fy %.1f pp (%.1f, %.1f)" % (det.weights.name, *K), flush=True)
        track, seen, last = None, [], None
        t0 = time.time()
        while len(seen) < args.frames and time.time() - t0 < args.timeout:
            color, depth = grab(pipe, align, scale)
            q = ls.q()
            if q is None:
                continue
            T = T_pelvis_optical(chain, q[12:15])
            cands = []
            for d in det.detect(color, depth_m=depth, intrinsics=K):
                if "xyz_m" not in d:
                    continue
                ray = np.array(d["xyz_m"]) / np.linalg.norm(d["xyz_m"])
                pc = np.array(d["xyz_m"]) + POD_RADIUS * ray
                p = T[:3, :3] @ pc + T[:3, 3]
                ax_c = pod_axis_cam(d["mask"], depth, K, d["depth_m"])
                ax = T[:3, :3] @ ax_c if ax_c is not None else None
                if ax is not None and ax[2] < 0:
                    ax = -ax                                    # point the axis upwards, by convention
                cands.append(dict(d, p=p, axis=ax))
            if track is None:
                reach = [c for c in cands if in_reach(c["p"])]
                if reach:
                    track = max(reach, key=lambda c: c["conf"])["p"]
                elif cands:
                    print("pods seen, none in reach: %s" % [np.round(c["p"], 2).tolist() for c in cands], flush=True)
            if track is not None:
                near = [c for c in cands if np.linalg.norm(c["p"] - track) < MATCH_M]
                if near:
                    c = min(near, key=lambda c: np.linalg.norm(c["p"] - track))
                    seen.append((c, q))
                    last = (color, c)
                    print("sighting %d/%d  pelvis xyz %s  conf %.2f  len %.0f cm" % (
                        len(seen), args.frames, np.round(c["p"], 3).tolist(), c["conf"],
                        100 * (c.get("length_m") or 0)), flush=True)
    finally:
        pipe.stop()

    if len(seen) < args.frames:
        sys.exit("NO TARGET: %d/%d sightings in %.0f s" % (len(seen), args.frames, args.timeout))
    P = np.array([c["p"] for c, _ in seen])
    med = np.median(P, axis=0)
    spread = float(np.max(np.linalg.norm(P - med, axis=1)))
    axes = np.array([c["axis"] for c, _ in seen if c["axis"] is not None])
    axis = None
    if len(axes):
        axis = np.median(axes, axis=0)
        axis = (axis / np.linalg.norm(axis)).tolist()
    target = {"xyz_pelvis": med.tolist(), "axis_pelvis": axis, "spread_m": spread, "n": len(seen),
              "conf": float(np.median([c["conf"] for c, _ in seen])),
              "length_m": float(np.median([c.get("length_m") or 0 for c, _ in seen])),
              "q": seen[-1][1], "time": time.time(), "intrinsics": K, "source": "okra_perceive"}
    color, c = last
    cv2.polylines(color, [np.asarray(c["mask"], np.int32)], True, (0, 255, 0), 2)
    cv2.circle(color, (int(c["cx"]), int(c["cy"])), 5, (0, 0, 255), -1)
    cv2.putText(color, "okra %.2f  pelvis %s" % (c["conf"], np.round(med, 2).tolist()), (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(os.path.splitext(args.out)[0] + ".jpg", color)
    json.dump(target, open(args.out, "w"), indent=1)
    if spread > MAX_SPREAD:
        sys.exit("UNSTABLE TARGET: sightings spread %.1f cm (max %.0f)" % (spread * 100, MAX_SPREAD * 100))
    print("TARGET %s (spread %.1f cm, axis %s) -> %s" % (np.round(med, 3).tolist(), spread * 100,
                                                          None if axis is None else np.round(axis, 2).tolist(), args.out))


if __name__ == "__main__":
    main()
