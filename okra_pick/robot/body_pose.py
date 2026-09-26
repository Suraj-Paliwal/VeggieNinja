"""Where is the robot's body in space? Stance-frame body pose from leg kinematics + IMU (numpy, Python 3.8).

While the robot stands, its feet do not move. The STANCE frame S is the left foot (left_ankle_roll_link)
frame at the anchor time. From the 12 leg joint angles (encoders, exact) the pelvis pose in S is

    T_S_pelvis(t) = inv(FK_pelvis->left_foot(q_t))

and any point measured in the pelvis frame at time a (e.g. the okra pod, perceived by the camera) can be
expressed in the pelvis frame at a later time b (the body swayed, the balance controller shifted):

    p_b = FK_L(q_b) @ inv(FK_L(q_a)) @ p_a                          (world_to_now / to_world below)

Checks: the right foot gives a second, independent estimate; if the two disagree, a foot moved (step,
slip, heel lift) and the anchor is no longer valid. The IMU gives the pelvis' absolute roll/pitch; if it
disagrees with the kinematic roll/pitch, the "flat foot on level ground" assumption is broken.
"""

import numpy as np

LEFT = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint"]
RIGHT = [n.replace("left", "right") for n in LEFT]
FOOT_SLIP_M = 0.01          # feet disagree more than this -> a foot moved
MAX_FOOT_TILT_DEG = 4.0     # IMU vs kinematic roll/pitch disagreement -> foot not flat


def rp_of(R):
    """roll, pitch (rad) of a rotation matrix (ZYX convention, like the Unitree IMU rpy)."""
    return np.arctan2(R[2, 1], R[2, 2]), np.arcsin(-np.clip(R[2, 0], -1, 1))


class BodyPose:
    def __init__(self, chain):
        self.c = chain
        self.anchor_q = None
        self.anchor_imu = None
        self.T_S_footR = None

    def _fk(self, q, names, off):
        return self.c.fk("pelvis", names[5].replace("_joint", "_link"), dict(zip(names, q[off:off + 6])))

    def anchor(self, q29, imu_rpy=None):
        """Fix the stance frame at the current left foot. Call when the robot stands still."""
        self.anchor_q = list(q29)
        self.anchor_imu = None if imu_rpy is None else list(imu_rpy)
        T_S_pelvis = np.linalg.inv(self._fk(q29, LEFT, 0))
        self.T_S_footR = T_S_pelvis @ self._fk(q29, RIGHT, 6)
        self.T0 = T_S_pelvis                                   # pelvis in the stance frame at the anchor

    def pelvis_in_stance(self, q29):
        """T_S_pelvis now (4x4, from the left foot)."""
        return np.linalg.inv(self._fk(q29, LEFT, 0))

    def status(self, q29, imu_rpy=None):
        if self.anchor_q is None:
            self.anchor(q29, imu_rpy)
        T_L = self.pelvis_in_stance(q29)
        T_R = self.T_S_footR @ np.linalg.inv(self._fk(q29, RIGHT, 6))
        T0 = self.T0
        slip = float(np.linalg.norm(T_L[:3, 3] - T_R[:3, 3]))
        out = {"pelvis_shift_m": (T_L[:3, 3] - T0[:3, 3]).tolist(),
               "pelvis_rot_deg": float(np.degrees(np.arccos(np.clip((np.trace(T0[:3, :3].T @ T_L[:3, :3]) - 1) / 2, -1, 1)))),
               "feet_disagree_m": slip, "feet_ok": slip < FOOT_SLIP_M}
        if imu_rpy is not None and self.anchor_imu is not None:
            # tilt change since the anchor, seen by the legs (foot assumed flat) and by the IMU
            r0, p0 = rp_of(T0[:3, :3])
            r1, p1 = rp_of(T_L[:3, :3])
            kin = np.degrees([r1 - r0, p1 - p0])
            imu = np.degrees([imu_rpy[0] - self.anchor_imu[0], imu_rpy[1] - self.anchor_imu[1]])
            out["tilt_change_deg"] = {"legs": kin.round(2).tolist(), "imu": imu.round(2).tolist()}
            out["foot_flat"] = bool(np.abs(imu - kin).max() < MAX_FOOT_TILT_DEG)
            out["feet_ok"] = out["feet_ok"] and out["foot_flat"]
        return out

    def to_world(self, p_pelvis, q29):
        """Point in the pelvis frame at pose q29 -> stance frame."""
        T = self.pelvis_in_stance(q29)
        return T[:3, :3] @ np.asarray(p_pelvis, float) + T[:3, 3]

    def world_to_now(self, p_stance, q29_now):
        """Stance-frame point -> pelvis frame now."""
        T = np.linalg.inv(self.pelvis_in_stance(q29_now))
        return T[:3, :3] @ np.asarray(p_stance, float) + T[:3, 3]
