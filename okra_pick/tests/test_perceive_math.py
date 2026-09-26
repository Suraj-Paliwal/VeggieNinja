"""Offline check of okra_perceive's geometry (no camera): pelvis -> pixel -> back, and the pod axis."""
import os, sys, types
import numpy as np
sys.modules["pyrealsense2"] = types.ModuleType("pyrealsense2")          # not installed on the VM
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
import cv2  # noqa: E402
import okra_perceive as OP  # noqa: E402
from g1_chain import Chain, T_pelvis_optical  # noqa: E402

K = (615.0, 615.0, 320.0, 240.0)
c = Chain(os.path.join(HERE, "..", "robot", "g1.urdf"))
worst_p, worst_a = 0.0, 0.0
for waist in [(0, 0, 0), (0.2, 0, 0), (-0.15, 0.02, 0.1)]:
    T = T_pelvis_optical(c, waist); Ti = np.linalg.inv(T)
    for p in [(0.42, -0.15, 0.15), (0.38, -0.30, 0.12), (0.48, 0.0, 0.18)]:
        pc = Ti[:3, :3] @ np.array(p) + Ti[:3, 3]
        # pod: vertical 10 cm capsule at p; its front surface is POD_RADIUS closer along the ray
        ray = pc / np.linalg.norm(pc); surf = pc - OP.POD_RADIUS * ray
        u, v = K[0] * surf[0] / surf[2] + K[2], K[1] * surf[1] / surf[2] + K[3]
        # rebuild the detector's xyz_m from pixel + depth, then the perceive transform
        xyz = np.array([(u - K[2]) * surf[2] / K[0], (v - K[3]) * surf[2] / K[1], surf[2]])
        back = T[:3, :3] @ (xyz + OP.POD_RADIUS * xyz / np.linalg.norm(xyz)) + T[:3, 3]
        worst_p = max(worst_p, np.linalg.norm(back - p))
        # mask of the vertical pod: project its end points, make a thin polygon
        ends = [Ti[:3, :3] @ (np.array(p) + [0, 0, s]) + Ti[:3, 3] for s in (-0.05, 0.05)]
        px = [(K[0] * e[0] / e[2] + K[2], K[1] * e[1] / e[2] + K[3]) for e in ends]
        if not all(0 <= x < 640 and 0 <= y < 480 for x, y in px):
            print("  waist %s pod %s: outside the image, skipped (the robot would not see it)" % (waist, p))
            continue
        d = np.array(px[1]) - px[0]; n = np.array([-d[1], d[0]]) / np.linalg.norm(d) * 4
        poly = np.array([px[0] + n, px[1] + n, px[1] - n, px[0] - n], np.float32)
        depth = np.zeros((480, 640), np.float32)                     # synthetic depth of the pod surface
        for s_ in np.linspace(-0.05, 0.05, 60):
            q_ = Ti[:3, :3] @ (np.array(p) + [0, 0, s_]) + Ti[:3, 3]
            q_ -= OP.POD_RADIUS * q_ / np.linalg.norm(q_)
            cv2.circle(depth, (int(K[0] * q_[0] / q_[2] + K[2]), int(K[1] * q_[1] / q_[2] + K[3])), 4, float(q_[2]), -1)
        a = T[:3, :3] @ OP.pod_axis_cam(poly, depth, K, surf[2])
        a = a if a[2] > 0 else -a
        ang = np.degrees(np.arccos(np.clip(a[2], -1, 1)))
        print("  waist %s pod %s: axis %.1f deg from vertical" % (waist, p, ang))
        worst_a = max(worst_a, ang)
print("pelvis -> camera -> pelvis round trip: max error %.2e m" % worst_p)
print("vertical pod axis recovered within %.1f deg (depth sampled at the ends)" % worst_a)
sys.exit(0 if worst_p < 1e-9 and worst_a < 8 else 1)
