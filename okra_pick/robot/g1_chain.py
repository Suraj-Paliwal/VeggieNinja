"""Minimal URDF forward kinematics (numpy only, Python 3.8+), used on the robot and on the VM.

    chain = Chain("g1.urdf")
    T = chain.fk("pelvis", "d435_link", {"waist_yaw_joint": 0.1, ...})   # 4x4, pelvis <- d435_link

Also: T_PELVIS_OPTICAL(chain, waist) for the head camera's optical frame
(x right, y down, z forward), the frame pyrealsense2 deprojects into.
"""

import xml.etree.ElementTree as ET

import numpy as np

# d435_link (x forward, y left, z up) -> RealSense optical frame (x right, y down, z forward)
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])
WAIST_JOINTS = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_R(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def homog(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


class Chain:
    def __init__(self, urdf_path):
        self.joints = {}      # child link -> joint dict
        for j in ET.parse(urdf_path).getroot().iter("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            lim = j.find("limit")
            self.joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                T=homog(rpy_to_R(*rpy), xyz),
                axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
                lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None)

    def path(self, base, tip):
        """Joints from base to tip (base must be an ancestor of tip)."""
        out, link = [], tip
        while link != base:
            if link not in self.joints:
                raise ValueError("%s is not below %s" % (tip, base))
            out.append(self.joints[link])
            link = self.joints[link]["parent"]
        return out[::-1]

    def fk(self, base, tip, q=None):
        q = q or {}
        T = np.eye(4)
        for j in self.path(base, tip):
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                T = T @ homog(axis_angle_R(j["axis"], q.get(j["name"], 0.0)), [0, 0, 0])
            elif j["type"] == "prismatic":
                T = T @ homog(np.eye(3), np.asarray(j["axis"]) * q.get(j["name"], 0.0))
        return T


def T_pelvis_optical(chain, waist_q):
    """pelvis <- head camera optical frame, for waist (yaw, roll, pitch) in rad."""
    T = chain.fk("pelvis", "d435_link", dict(zip(WAIST_JOINTS, waist_q)))
    return T @ homog(R_LINK_OPTICAL, [0, 0, 0])
