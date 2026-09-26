"""Closed-loop arm correction: keep the planned grasp on the pod when the body moves (numpy, Python 3.8).

The plan (okra_plan.py) is computed in the pelvis frame of the perception moment. While the arm moves, the
balance controller can shift the pelvis relative to the (fixed) feet. Every 1/CORRECT_HZ s:

  pod_now   = where the pod is in the CURRENT pelvis frame (body_pose: legs + stance anchor)
  delta     = pod_now - pod_planned                     (how far the pod "moved" relative to the body)
  dq        = solve tcp(q_plan + dq) = tcp(q_plan) + delta   (7 right-arm joints; 2 Gauss-Newton steps on the
              grasp-point position Jacobian, warm-started from the previous dq)
  command   = planned q + weight * dq                   (lightly low-pass filtered, capped per joint)

Raises Moved when a foot moved (step/slip/tilt) or |delta| > MAX_CORRECTION: the target is no longer
trustworthy and the robot must look again.
"""

import numpy as np

import pick_config as C
from body_pose import BodyPose
from g1_chain import axis_angle_R, homog

WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]


class Moved(Exception):
    pass


class ArmKin:
    """Fast right-arm kinematics for a fixed waist: one pass gives the grasp point and its geometric
    Jacobian (z_k x (p_tcp - p_k)). ~20x faster than repeated generic FK."""

    def __init__(self, chain, waist):
        path = chain.path("pelvis", "right_wrist_yaw_link")
        k0 = [j["name"] for j in path].index(C.RIGHT_ARM[0])
        self.T0 = chain.fk("pelvis", path[k0 - 1 + 1]["parent"], dict(zip(WAIST, waist)))   # pelvis -> shoulder base
        self.joints = [(j["T"], np.asarray(j["axis"], float)) for j in path[k0:]]
        self.tcp_h = np.r_[np.asarray(C.TCP_XYZ, float), 1.0]

    def tcp_and_jac(self, q):
        T = self.T0.copy()
        ps, zs = [], []
        for (Tj, ax), qi in zip(self.joints, q):
            T = T @ Tj
            ps.append(T[:3, 3].copy())
            zs.append(T[:3, :3] @ ax)
            R = axis_angle_R(ax, qi)
            T[:3, :3] = T[:3, :3] @ R
        p = (T @ self.tcp_h)[:3]
        J = np.array([np.cross(z, p - pk) for z, pk in zip(zs, ps)]).T
        return p, J


class Corrector:
    def __init__(self, chain, traj):
        self.c = chain
        self.q_plan29 = list(traj["target"]["q"])
        self.bp = BodyPose(chain)
        self.bp.anchor(self.q_plan29)
        self.pod_plan = np.array(traj["pod_xyz"], float)
        self.pod_world = self.bp.to_world(self.pod_plan, self.q_plan29)
        self.waist = dict(zip(WAIST, self.q_plan29[12:15]))
        self.kin = ArmKin(chain, self.q_plan29[12:15])
        self.dq = np.zeros(7)
        self.delta = np.zeros(3)
        self.status = {}

    def tcp(self, q_arm):
        d = dict(self.waist)
        d.update(zip(C.RIGHT_ARM, q_arm))
        return (self.c.fk("pelvis", "right_wrist_yaw_link", d) @ homog(np.eye(3), C.TCP_XYZ))[:3, 3]

    def jacobian(self, q_arm, eps=1e-4):
        p0 = self.tcp(q_arm)
        J = np.zeros((3, 7))
        for k in range(7):
            q = np.array(q_arm, float)
            q[k] += eps
            J[:, k] = (self.tcp(q) - p0) / eps
        return J

    def update(self, q29_now, q_arm_plan):
        st = self.bp.status(q29_now)
        self.status = st
        if not st["feet_ok"]:
            raise Moved("a foot moved (feet disagree %.0f mm): look again" % (1000 * st["feet_disagree_m"]))
        pod_now = self.bp.world_to_now(self.pod_world, q29_now)
        delta = pod_now - self.pod_plan
        if np.linalg.norm(delta) > C.MAX_CORRECTION:
            raise Moved("body moved %.1f cm since perception (max %.0f): look again" % (
                100 * np.linalg.norm(delta), 100 * C.MAX_CORRECTION))
        q_plan = np.asarray(q_arm_plan, float)
        goal = self.kin.tcp_and_jac(q_plan)[0] + delta
        q = q_plan + self.dq
        for _ in range(2):
            p, J = self.kin.tcp_and_jac(q)
            q = q + J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(3), goal - p)
        self.dq = 0.5 * self.dq + 0.5 * np.clip(q - q_plan, -0.3, 0.3)
        self.delta = delta
        return delta

    def command(self, q_arm_plan, weight):
        return np.asarray(q_arm_plan) + weight * self.dq
