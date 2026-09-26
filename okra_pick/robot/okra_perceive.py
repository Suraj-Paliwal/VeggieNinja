#!/usr/bin/env python3
"""
Find one okra pod and write its position in the robot's pelvis frame. Runs ON THE ROBOT (Orin),
conda env g1brainco (Python 3.8: torch+CUDA, ultralytics, pyrealsense2, unitree_sdk2py).

    python okra_perceive.py --event-dir DIR [--frames 15]   # one "look": evidence bundle for the HITL step
    python okra_perceive.py --floor-check                    # camera extrinsic sanity check, no detection

Needs the head RealSense free (Unitree videohub stopped: okra_pick.sh does that).

Per frame: colour + depth aligned to colour (pyrealsense2) -> OkraDetector (GPU) with a LOW confidence
floor and the filter's rejections kept -> each detection's 3D point and pod axis in the pelvis frame
(URDF chain + live waist angles). Observations are merged into candidates (candidates.py).

It does NOT choose a target: the VM decides with the human-in-the-loop policy (okra_pick/hitl).
Event bundle written to --event-dir:
  frames/NNN.jpg        every colour frame (JPEG 95)
  depth_key.npz         depth of the last frame, uint16 millimetres
  candidates.json       all candidates with statistics, key/best masks, pelvis xyz, axis, reach
  robot_state.json      29 joint angles, IMU rpy, waist, camera intrinsics and pelvis<-camera transform
  annotated.jpg         last frame with every candidate numbered
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
from candidates import Candidates                                  # noqa: E402
from g1_chain import Chain, T_pelvis_optical                       # noqa: E402

POD_RADIUS = 0.01      # m: the mask depth is the pod's front surface; its axis is ~1 cm further
CAND_CONF = 0.10       # detector confidence floor for candidates (the filter's own threshold is higher)


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


COLOR_TO_DEPTH = np.eye(4)   # set in start_camera from the camera's own factory calibration


def start_camera():
    """Colour + depth, with the RealSense's stored colour->depth extrinsics. Points are deprojected in the
    COLOUR optical frame (depth is aligned to colour); the URDF's d435_link is the depth module, so every
    point is moved by the factory colour->depth transform (~15 mm) before the URDF chain is applied."""
    global COLOR_TO_DEPTH
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    ex = prof.get_stream(rs.stream.color).get_extrinsics_to(prof.get_stream(rs.stream.depth))
    COLOR_TO_DEPTH = np.eye(4)
    COLOR_TO_DEPTH[:3, :3] = np.asarray(ex.rotation).reshape(3, 3).T      # librealsense stores column-major
    COLOR_TO_DEPTH[:3, 3] = ex.translation
    print("RealSense colour->depth offset from factory calibration: %s mm" % np.round(1000 * np.asarray(ex.translation), 1).tolist(), flush=True)
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
    T = T_pelvis_optical(chain, q[12:15]) @ COLOR_TO_DEPTH
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


def annotate(color, cands):
    img = color.copy()
    for c in cands:
        col = (0, 200, 0) if c["in_reach"] else (0, 200, 255)
        cv2.polylines(img, [np.asarray(c["key_mask"], np.int32)], True, col, 2)
        x, y = int(c["px"][0]), int(c["px"][1])
        cv2.putText(img, "%d" % c["id"], (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(img, "%d" % c["id"], (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event-dir", default="event")
    ap.add_argument("--frames", type=int, default=15)
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
        det = OkraDetector(args.weights, stream=True, conf=CAND_CONF)
        out = args.event_dir
        os.makedirs(os.path.join(out, "frames"), exist_ok=True)
        print("detector %s (floor %.2f), camera fx %.1f fy %.1f pp (%.1f, %.1f)" % (det.weights.name, CAND_CONF, *K), flush=True)
        cands = Candidates(args.frames)
        q = None
        for f in range(args.frames):
            color, depth = grab(pipe, align, scale)
            q = ls.q() or q
            T = T_pelvis_optical(chain, q[12:15]) @ COLOR_TO_DEPTH           # pelvis <- colour optical frame
            cv2.imwrite(os.path.join(out, "frames", "%03d.jpg" % f), color, [cv2.IMWRITE_JPEG_QUALITY, 95])
            obs = []
            for d in det.detect(color, depth_m=depth, intrinsics=K, return_rejected=True):
                o = {k: d[k] for k in ("cx", "cy", "conf", "accepted", "reason", "box")}
                o["mask"] = np.asarray(d["mask"]).tolist()
                if "xyz_m" in d:
                    ray = np.array(d["xyz_m"]) / np.linalg.norm(d["xyz_m"])
                    o["p"] = (T[:3, :3] @ (np.array(d["xyz_m"]) + POD_RADIUS * ray) + T[:3, 3]).tolist()
                    ax = pod_axis_cam(d["mask"], depth, K, d["depth_m"])
                    if ax is not None:
                        ax = T[:3, :3] @ ax
                        o["axis"] = (ax if ax[2] >= 0 else -ax).tolist()
                    o["length_m"] = d.get("length_m")
                obs.append(o)
            cands.add_frame(f, obs)
            print("frame %d/%d: %d detections" % (f + 1, args.frames, len(obs)), flush=True)
        key_color, key_depth = color, depth
    finally:
        pipe.stop()

    summary = cands.summary(in_reach)
    json.dump({"candidates": summary, "frames": args.frames, "conf_floor": CAND_CONF,
               "detector": det.weights.name, "time": time.time()},
              open(os.path.join(out, "candidates.json"), "w"))
    np.savez_compressed(os.path.join(out, "depth_key.npz"), depth_mm=np.round(key_depth * 1000).astype(np.uint16))
    T = T_pelvis_optical(chain, q[12:15]) @ COLOR_TO_DEPTH
    json.dump({"q": q, "imu_rpy": ls.rpy(), "waist": q[12:15], "color_to_depth": COLOR_TO_DEPTH.tolist(), "intrinsics": {"fx": K[0], "fy": K[1], "ppx": K[2], "ppy": K[3],
               "width": 640, "height": 480}, "T_pelvis_optical": T.tolist(), "time": time.time()},
              open(os.path.join(out, "robot_state.json"), "w"), indent=1)
    cv2.imwrite(os.path.join(out, "annotated.jpg"), annotate(key_color, summary))
    for c in summary:
        print("candidate %d: conf med %.2f max %.2f, seen %.0f%%, filter ok %.0f%% %s, xyz %s, reach %s" % (
            c["id"], c["conf_median"], c["conf_max"], 100 * c["seen_frac"], 100 * c["accepted_frac"],
            c["reject_reasons"] or "", None if c["xyz_pelvis"] is None else np.round(c["xyz_pelvis"], 3).tolist(),
            c["in_reach"]))
    print("EVENT %s: %d candidates" % (out, len(summary)))


if __name__ == "__main__":
    main()
