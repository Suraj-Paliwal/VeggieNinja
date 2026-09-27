"""
Closed-loop grasp test (no robot): the fake robot's body sways during the reach (both ankles pitch, so the
pelvis moves relative to the fixed feet), and we measure where the grasp point really is relative to the
fixed pod at the moment the jaws close — with the correction and without it.

    ../dimos/.venv/bin/python tests/test_closed_loop.py <trajectory.json>
"""

import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
import test_player_sim as TP  # noqa: E402  (sets C.DT = 0.005: 4x faster)
import arm_player as AP  # noqa: E402
import pick_config as C  # noqa: E402
from body_pose import BodyPose  # noqa: E402
from closed_loop import Corrector  # noqa: E402
from g1_chain import Chain  # noqa: E402

CHAIN = Chain(os.path.join(HERE, "..", "robot", "g1.urdf"))


class SwayRobot(TP.FakeRobot):
    """FakeRobot + legs: ankle pitch ramps to `amp` rad during to_pregrasp/approach (pelvis tips forward)."""

    def __init__(self, traj, scenario, amp=0.0, step=False):
        super().__init__(traj, scenario)
        self.q_plan = list(traj["target"]["q"])
        self.amp, self.step, self.k = amp, step, 0
        self.at_close = None

    def legs_offset(self):
        f = min(1.0, self.k / 400.0)                       # ramps over ~400 ticks (2 s at 4x speed)
        return f

    def q29(self):
        q = list(self.q_plan)
        f = self.legs_offset()
        q[4] += self.amp * f                               # left ankle pitch
        q[10] += self.amp * f                              # right ankle pitch
        if self.step:
            q[6] += 0.4 * f                                # right hip pitch: the right foot lifts/moves
        for n, i in enumerate(C.RIGHT_ARM_IDX):
            q[i] = self.qarm[n]
        return q

    def send_arm(self, q_upper, weight, tau_upper=None):
        self.k += 1
        super().send_arm(q_upper, weight, tau_upper)

    def send_grip(self, q):
        if q == C.GRIP_CLOSE_Q and self.at_close is None:
            self.at_close = (self.qarm.copy(), self.q29())
        super().send_grip(q)


def grasp_error_mm(traj, robot):
    """Distance between the real grasp point and the (fixed) pod, both in the stance frame."""
    qarm, q29 = robot.at_close
    corr = Corrector(CHAIN, traj)                          # only used for FK helpers + pod in the world
    bp = BodyPose(CHAIN)
    bp.anchor(traj["target"]["q"])
    tcp_world = bp.to_world(corr.tcp(qarm), q29)
    target_world = bp.to_world(np.array(traj["pod_xyz"]) + C.GRASP_DEPTH * np.array(traj["approach_dir"]), traj["target"]["q"])
    return 1000 * np.linalg.norm(tcp_world - target_world)


def run(traj_path, amp, closed, step=False):
    traj = json.load(open(traj_path))
    traj["planned_at"] = time.time()
    r = SwayRobot(traj, "nominal", amp=amp, step=step)
    p = AP.Player(r, traj, "approach", closed_loop=closed)   # stop after the approach: close + measure
    orig = p.check

    def check(i, seg, *a):
        r.seg_now = seg
        return orig(i, seg, *a)
    p.check = check
    # close the gripper at the end of the approach to capture the grasp pose
    traj["events"] = [{"at": next(s["end"] for s in traj["segments"] if s["name"] == "approach") - 1,
                       "type": "close_gripper"}]
    p.t = traj
    aborted = p.run()
    shift = BodyPose(CHAIN)
    shift.anchor(traj["target"]["q"])
    body_mm = 1000 * np.linalg.norm(shift.status(r.q29())["pelvis_shift_m"])
    err = grasp_error_mm(traj, r) if r.at_close is not None else None
    return aborted, err, body_mm


def main():
    tp = sys.argv[1]
    rows, ok = [], True
    for name, amp, step, expect in [("no sway", 0.0, False, "ok"), ("sway 0.02 rad", 0.02, False, "ok"),
                                    ("sway 0.03 rad", 0.03, False, "ok"), ("sway 0.06 rad", 0.06, False, "abort"),
                                    ("right foot steps", 0.0, True, "abort")]:
        res = {}
        for closed in (False, True):
            aborted, err, body = run(tp, amp, closed, step)
            res[closed] = (aborted, err, body)
        a_c, e_c, body = res[True]
        a_o, e_o, _ = res[False]
        if expect == "ok":
            passed = a_c is None and e_c is not None and e_c < 5.0
        else:
            passed = a_c is not None
        ok &= passed
        rows.append("%s  %-17s body moved %5.1f mm | open loop: %s | closed loop: %s" % (
            "PASS" if passed else "FAIL", name, body,
            "grasp error %.1f mm" % e_o if e_o is not None else "aborted",
            ("grasp error %.1f mm" % e_c) if a_c is None else "ABORT (%s)" % a_c))
    print("\n".join(rows))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
