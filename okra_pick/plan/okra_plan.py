#!/usr/bin/env python3
"""
Plan a right-arm okra grasp from a perceived target. Runs on the VM (dimos venv: pinocchio).

    python okra_plan.py target.json -o trajectory.json
    python okra_plan.py --synthetic 0.35 -0.20 0.20 -o trajectory.json     # pod at pelvis xyz, vertical

Input target.json (from robot/okra_perceive.py): xyz_pelvis, axis_pelvis (pod long axis, may be null),
q (29 measured joint angles, rt/lowstate order).

Grasp: approach horizontally (or tilted, see APPROACH_PITCHES_DEG) from the right shoulder towards the pod,
jaws closing ACROSS the pod. Segments:
  to_pregrasp (joint space) -> approach (straight line) -> [close gripper] -> pull (back/down with a wrist
  twist about the gripper axis in its first part: PULL_TWIST_DEG, TWIST_FRAC) -> retreat (straight, twist
  kept) -> home (joint space back to the start pose). The gripper stays closed at the end.

Refuses (exit 2) if the pod is outside PLAN_SANITY (bad depth; REACH_* is only a preference), any waypoint has no IK within tolerance, a joint limit
margin is violated, or the hand passes through the torso box. Writes nothing in that case.
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import pinocchio as pin

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
import pick_config as C  # noqa: E402

URDF = os.path.join(HERE, "..", "robot", "g1.urdf")
TORSO_BOX = ((-0.15, 0.12), (-0.16, 0.16), (-0.10, 0.60))   # pelvis frame; hand points must stay outside
# natural right-arm seeds for IK: dimos G1_READY_JOINTS and forward reaches (shoulder pitch -, elbow bent)
SEEDS = [(-0.4, -0.2, 0.0, 1.2, 0.0, 0.0, 0.0), (-0.8, -0.2, 0.0, 0.9, 0.0, 0.0, 0.0),
         (-1.1, -0.3, 0.2, 0.6, 0.0, 0.0, 0.0), (-0.6, -0.4, -0.3, 1.4, 0.0, 0.0, 0.0)]


# ---------------------------------------------------------------- model

class ArmModel:
    """Pinocchio model of the right arm with every other joint locked at the measured pose."""

    def __init__(self, q29):
        full = pin.buildModelFromUrdf(URDF)
        qfull = pin.neutral(full)
        names = [full.names[j] for j in range(full.njoints)]
        order = [n for n in names if n not in ("universe", "floating_base_joint")]  # = rt/lowstate order
        for i, n in enumerate(order[:len(q29)]):
            qfull[full.joints[full.getJointId(n)].idx_q] = q29[i]
        lock = [full.getJointId(n) for n in names[1:] if n not in C.RIGHT_ARM]
        self.model = pin.buildReducedModel(full, lock, qfull)
        self.data = self.model.createData()
        wrist = self.model.getFrameId("right_wrist_yaw_link")
        R_wt = tool_rotation_in_wrist()
        self.tcp = self.model.addFrame(pin.Frame("tcp", self.model.frames[wrist].parentJoint, wrist,
                                                 self.model.frames[wrist].placement *
                                                 pin.SE3(R_wt, np.array(C.TCP_XYZ)), pin.FrameType.OP_FRAME))
        self.data = self.model.createData()
        self.lo = self.model.lowerPositionLimit + C.LIMIT_MARGIN
        self.hi = self.model.upperPositionLimit - C.LIMIT_MARGIN
        self.check_frames = [self.model.getFrameId(n) for n in
                             ("right_elbow_link", "right_wrist_roll_link", "right_wrist_yaw_link")] + [self.tcp]
        self.q_start = np.array([q29[i] for i in C.RIGHT_ARM_IDX])

    def fk(self, q, frame=None):
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.tcp if frame is None else frame]

    def _ik1(self, goal, q0, iters):
        q = np.clip(q0.copy(), self.lo, self.hi)
        for _ in range(iters):
            err = pin.log6(self.fk(q).actInv(goal)).vector          # in the tcp frame
            if np.linalg.norm(err[:3]) < 2e-4 and np.linalg.norm(err[3:]) < 2e-3:
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, self.tcp, pin.ReferenceFrame.LOCAL)
            lam = 1e-3 + 0.05 * np.linalg.norm(err)                  # more damping far from the goal
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)
            q = np.clip(pin.integrate(self.model, q, np.clip(0.5 * dq, -0.15, 0.15)), self.lo, self.hi)
        e = self.fk(q).actInv(goal)
        return q, float(np.linalg.norm(e.translation)), float(np.degrees(np.linalg.norm(pin.log3(e.rotation))))

    def ik(self, goal, q0, iters=400, seeds=0):
        """Damped least squares on the 7 arm joints, optionally from extra random seeds.
        Returns the best (q, pos_err_m, rot_err_deg); among converged ones, the closest to q0."""
        cands = [self._ik1(goal, q0, iters)]
        if seeds:
            cands += [self._ik1(goal, np.array(sd), iters) for sd in SEEDS]
        rng = np.random.default_rng(0)
        for _ in range(max(0, seeds - len(SEEDS))):
            cands.append(self._ik1(goal, rng.uniform(self.lo, self.hi), iters))
        ok = [c for c in cands if c[1] <= C.IK_POS_TOL and c[2] <= C.IK_ROT_TOL_DEG and not self.in_torso(c[0])]
        if ok:
            return min(ok, key=lambda c: np.abs(c[0] - q0).max())
        return min(cands, key=lambda c: c[1] + 0.01 * c[2])

    def in_torso(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)
        for f in self.check_frames:
            p = self.data.oMf[f].translation
            if all(lo <= v <= hi for v, (lo, hi) in zip(p, TORSO_BOX)):
                return self.model.frames[f].name
        return None


def tool_rotation_in_wrist():
    """Tool frame: x = gripper pointing direction, y = jaw closing axis, z = x cross y."""
    if C.CLOSE_AXIS == "y":
        return np.eye(3)
    if C.CLOSE_AXIS == "z":
        return np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])  # columns: x_w, z_w, -y_w
    raise ValueError("CLOSE_AXIS must be 'y' or 'z'")


# ---------------------------------------------------------------- grasp geometry

def grasp_rotation(approach, pod_axis, flip):
    a = approach / np.linalg.norm(approach)
    u = np.array([0.0, 0.0, 1.0]) if pod_axis is None else np.asarray(pod_axis, float)
    c = np.cross(a, u)
    if np.linalg.norm(c) < 0.2:           # pod nearly along the approach: close horizontally
        c = np.cross(a, [0.0, 0.0, 1.0])
    c /= np.linalg.norm(c)
    if flip:
        c = -c
    z = np.cross(a, c)
    return np.column_stack([a, c, z])


def approach_dir(p, shoulder, pitch_deg):
    h = np.array([p[0] - shoulder[0], p[1] - shoulder[1], 0.0])
    h /= np.linalg.norm(h)
    t = np.radians(pitch_deg)             # + = pointing down
    return np.cos(t) * h + np.array([0.0, 0.0, -np.sin(t)])


def cosine_joint(q0, q1, speed):
    T = max(1.5, float(np.max(np.abs(q1 - q0))) * np.pi / 2 / speed)   # cosine peak speed = speed
    n = max(2, int(round(T / C.DT)))
    s = (1 - np.cos(np.linspace(0, np.pi, n))) / 2
    return q0 + s[:, None] * (q1 - q0)


# ---------------------------------------------------------------- planning

def plan(target):
    q29 = target["q"]
    p = np.asarray(target["xyz_pelvis"], float)
    # Only a sanity limit here (a bad depth reading must not send the arm somewhere absurd); whether the arm can
    # really get there is decided below by the IK, joint-limit margins and the torso check.
    for v, (lo, hi), ax in zip(p, C.PLAN_SANITY, "xyz"):
        if not lo <= v <= hi:
            raise PlanError("pod %s=%.3f m outside the sanity limits [%.2f, %.2f] (pelvis frame): bad depth?" % (ax, v, lo, hi))
    if not all(lo <= v <= hi for v, (lo, hi) in zip(p, (C.REACH_X, C.REACH_Y, C.REACH_Z))):
        print("note: pod %s is outside the comfortable reach box; trying the IK anyway" % np.round(p, 3).tolist(), flush=True)
    arm = ArmModel(q29)
    shoulder = arm.fk(arm.q_start, arm.model.getFrameId("right_shoulder_pitch_link")).translation

    best, notes = None, []
    for pitch in C.APPROACH_PITCHES_DEG:
        a = approach_dir(p, shoulder, pitch)
        for flip in (False, True):
            R = grasp_rotation(a, target.get("axis_pelvis"), flip)
            poses = {
                "pregrasp": p - C.PREGRASP_BACK * a,
                "grasp": p + C.GRASP_DEPTH * a,
                "pull": p - C.PULL_BACK * a - np.array([0, 0, C.PULL_DOWN]),
                "retreat": p - C.RETREAT_BACK * a - np.array([0, 0, C.PULL_DOWN]),
            }
            try:
                traj = build(arm, R, poses)
            except PlanError as e:
                notes.append("pitch %+d flip %d: %s" % (pitch, flip, e))
                continue
            cost = traj["min_limit_margin"]
            if best is None or cost > best[0]:
                best = (cost, traj, pitch, flip)
        if best is not None:
            break
    if best is None:
        raise PlanError("no feasible grasp:\n  " + "\n  ".join(notes[-6:]))
    _, traj, pitch, flip = best
    traj.update(approach_pitch_deg=pitch, close_axis_flipped=flip, target=target,
                perceived_at_robot=target.get("perceived_at_robot"),      # robot clock: the player checks the age with it
                pod_xyz=p.tolist(), approach_dir=approach_dir(p, shoulder, pitch).tolist())
    traj["tau_ff"] = gravity_ff(q29, traj["q"])
    return traj


_GRAV = {}


def gravity_ff(q29, Q_arm):
    """Gravity feed-forward per sample for arm_sdk motors 12..28 (waist, left arm, right arm): the torque that holds
    the waist and the right arm against gravity with the right arm at each planned pose (waist and left arm stay at
    q29). Model gravity (pelvis upright) times the per-joint scales fitted on the robot (pick_config.GRAVITY_SCALE);
    the left arm gets none (it stays at rest, not fitted)."""
    if not C.GRAVITY_FF_GAIN:
        return None
    if "m" not in _GRAV:
        m = pin.buildModelFromUrdf(URDF)
        order = [l.split('"')[1] for l in open(URDF) if "<joint " in l and 'type="revolute"' in l][:29]   # motor order
        _GRAV.update(m=m, d=m.createData(), order=order,
                     iq=[m.joints[m.getJointId(n)].idx_q for n in order], iv=[m.joints[m.getJointId(n)].idx_v for n in order])
    m, d, order, iq, iv = _GRAV["m"], _GRAV["d"], _GRAV["order"], _GRAV["iq"], _GRAV["iv"]
    scale = np.array([C.GRAVITY_SCALE.get(order[i], 1.0) if i in C.WAIST_IDX or i in C.RIGHT_ARM_IDX else 0.0
                      for i in C.UPPER_IDX])
    q = pin.neutral(m)
    for k in range(29):
        q[iq[k]] = q29[k]
    out = []
    for qa in Q_arm:
        for k, i in enumerate(C.RIGHT_ARM_IDX):
            q[iq[i]] = qa[k]
        g = pin.computeGeneralizedGravity(m, d, q)
        out.append(np.round(scale * np.array([g[iv[i]] for i in C.UPPER_IDX]), 3).tolist())
    return out


class PlanError(Exception):
    pass


def rot_x(deg):
    t = np.radians(deg)
    return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])


def cartesian(arm, q0, R, p0, p1, speed, name, twist_deg=0.0, twist_frac=1.0):
    """Straight line p0 -> p1; optionally twist the tool about its own x axis (the gripper axis) by
    twist_deg during the first twist_frac of the line (tool frame: R @ rot_x)."""
    T = np.linalg.norm(p1 - p0) / speed
    if twist_deg:
        T = max(T, abs(twist_deg) / C.TWIST_SPEED_DEG / twist_frac)
    n = max(2, int(np.ceil(T / C.DT)) + 1)
    out, q = [], q0
    for s in np.linspace(0, 1, n):
        Rs = R @ rot_x(twist_deg * min(1.0, s / twist_frac)) if twist_deg else R
        q, pe, re = arm.ik(pin.SE3(Rs, p0 + s * (p1 - p0)), q, iters=150)
        if pe > C.IK_POS_TOL or re > C.IK_ROT_TOL_DEG:
            raise PlanError("%s: IK error %.1f mm / %.1f deg at %.0f%%" % (name, pe * 1e3, re, s * 100))
        out.append(q)
    return np.array(out)


def build(arm, R, poses):
    """Tries both twist directions (if a twist is configured) and keeps the one with more joint margin."""
    best, err = None, None
    for sign in ((1, -1) if C.PULL_TWIST_DEG else (1,)):
        try:
            tr = build_one(arm, R, poses, sign * C.PULL_TWIST_DEG)
        except PlanError as e:
            err = e
            continue
        if best is None or tr["min_limit_margin"] > best["min_limit_margin"]:
            best = tr
    if best is None:
        raise err
    return best


def build_one(arm, R, poses, twist):
    q_pre, pe, re = arm.ik(pin.SE3(R, poses["pregrasp"]), arm.q_start, seeds=12)
    if pe > C.IK_POS_TOL or re > C.IK_ROT_TOL_DEG:
        raise PlanError("pregrasp IK error %.1f mm / %.1f deg" % (pe * 1e3, re))
    segs = [("to_pregrasp", cosine_joint(arm.q_start, q_pre, C.JOINT_SPEED))]
    segs.append(("approach", cartesian(arm, q_pre, R, poses["pregrasp"], poses["grasp"], C.APPROACH_SPEED, "approach")))
    segs.append(("pull", cartesian(arm, segs[-1][1][-1], R, poses["grasp"], poses["pull"], C.PULL_SPEED, "pull",
                                   twist_deg=twist, twist_frac=C.TWIST_FRAC)))
    R_tw = R @ rot_x(twist)
    segs.append(("retreat", cartesian(arm, segs[-1][1][-1], R_tw, poses["pull"], poses["retreat"], C.APPROACH_SPEED, "retreat")))
    segs.append(("home", cosine_joint(segs[-1][1][-1], arm.q_start, C.JOINT_SPEED)))

    q_all, segments = [], []
    for name, qs in segs:
        segments.append({"name": name, "start": len(q_all), "end": len(q_all) + len(qs)})
        q_all.extend(qs.tolist())
    q_all = np.array(q_all)

    step = np.abs(np.diff(q_all, axis=0)).max() / C.DT
    if step > 1.5:
        raise PlanError("joint speed %.2f rad/s too high (IK jump)" % step)
    for i in range(0, len(q_all), 5):
        hit = arm.in_torso(q_all[i])
        if hit:
            raise PlanError("%s inside torso box at sample %d" % (hit, i))
    margin = float(np.min(np.minimum(q_all - arm.lo, arm.hi - q_all)))
    return {"dt": C.DT, "joints": C.RIGHT_ARM, "motor_idx": C.RIGHT_ARM_IDX, "q": q_all.tolist(),
            "segments": segments,
            "events": [{"at": segments[1]["end"], "type": "close_gripper"}],
            "start_q": arm.q_start.tolist(), "duration_s": len(q_all) * C.DT, "pull_twist_deg": twist,
            "max_joint_speed": float(step), "min_limit_margin": margin + C.LIMIT_MARGIN,
            "tcp_path": {k: v.tolist() for k, v in poses.items()},
            "config": {k: getattr(C, k) for k in ("TCP_XYZ", "CLOSE_AXIS", "PREGRASP_BACK", "GRASP_DEPTH",
                                                    "PULL_BACK", "PULL_DOWN", "APPROACH_SPEED", "PULL_SPEED",
                                                    "PULL_TWIST_DEG", "TWIST_FRAC", "TWIST_SPEED_DEG")},
            "planned_at": time.time()}                                  # VM clock, information only


def synthetic_target(x, y, z):
    """Standing pose from the 2026-09-26 recordings (FSM 200): arms relaxed at the sides."""
    q = [0.0] * 29
    q[0] = q[6] = -0.33; q[3] = q[9] = 0.72; q[4] = q[10] = -0.36    # legs (irrelevant for the arm plan)
    q[15], q[16], q[18] = 0.29, 0.22, 0.97                          # left arm
    q[22], q[23], q[25] = 0.29, -0.22, 0.97                         # right arm
    return {"xyz_pelvis": [x, y, z], "axis_pelvis": [0.0, 0.0, 1.0], "q": q, "source": "synthetic"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", nargs="?")
    ap.add_argument("--synthetic", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("-o", "--out", default="trajectory.json")
    args = ap.parse_args()
    target = synthetic_target(*args.synthetic) if args.synthetic else json.load(open(args.target))
    try:
        traj = plan(target)
    except PlanError as e:
        print("REFUSED: %s" % e)
        sys.exit(2)
    json.dump(traj, open(args.out, "w"))
    seg = ", ".join("%s %.1fs" % (s["name"], (s["end"] - s["start"]) * C.DT) for s in traj["segments"])
    print("OK  pod %s  approach pitch %+d deg%s, pull twist %+.0f deg" % (
        np.round(traj["pod_xyz"], 3).tolist(), traj["approach_pitch_deg"],
        " (jaws flipped)" if traj["close_axis_flipped"] else "", traj["pull_twist_deg"]))
    print("    %d samples, %.1f s: %s" % (len(traj["q"]), traj["duration_s"], seg))
    print("    max joint speed %.2f rad/s, min joint-limit margin %.2f rad -> %s" % (
        traj["max_joint_speed"], traj["min_limit_margin"], args.out))


if __name__ == "__main__":
    main()
