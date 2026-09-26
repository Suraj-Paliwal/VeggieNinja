"""
Offline test of robot/arm_player.py against a simulated robot (no DDS). Runs on the VM:

    ../dimos/.venv/bin/python tests/test_player_sim.py <trajectory.json>

The fake robot's arm follows commands with a first-order lag, the gripper stops at q=1.0 on a pod
(or closes to 0 when --empty), and faults are injected by scenario. Time runs 4x faster than real.
"""

import json
import os
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
import pick_config as C  # noqa: E402

C.DT = 0.005                                   # 4x faster than real time
import arm_player as AP  # noqa: E402


class FakeRobot:
    def __init__(self, traj, scenario):
        self.scenario, self.t0 = scenario, time.time()
        self.qarm = np.array(traj["start_q"], float)
        self.upper = np.zeros(17)
        self.upper[[k for k, i in enumerate(C.UPPER_IDX) if i in C.RIGHT_ARM_IDX]] = self.qarm
        self.grip_q, self.grip_target, self.ls_t = 5.3, None, time.time()
        self.ls = True
        self.seg_now = "?"
        self.q_plan29 = list(traj.get("target", {}).get("q", [0.0] * 29))
        self.sent_arm = 0
        self.sent_grip = []

    # --- inputs
    def q(self, idx):
        if idx == C.RIGHT_ARM_IDX:
            return self.qarm.copy()
        return self.upper.copy()

    def q29(self):
        q = list(self.q_plan29)
        for n, i in enumerate(C.RIGHT_ARM_IDX):
            q[i] = float(self.qarm[n])
        return q

    def tau(self, idx):
        return np.full(7, 20.0 if (self.scenario == "stuck_pod" and self.seg_now == "pull") else 2.0)

    def tilt(self):
        bad = (self.scenario == "tilt_approach" and self.seg_now == "approach") or \
              (self.scenario == "tilt_retreat" and self.seg_now == "retreat")
        return (0.0, 0.12 if bad else 0.01)

    def age(self):
        if self.scenario == "stale" and self.seg_now == "approach":
            return 1.0
        return 0.0

    # --- outputs
    def send_arm(self, q_upper, weight):
        k = [k for k, i in enumerate(C.UPPER_IDX) if i in C.RIGHT_ARM_IDX]
        target = np.asarray(q_upper)[k]
        self.qarm += 0.6 * weight * (target - self.qarm)          # lagging tracker
        self.sent_arm += 1

    def send_grip(self, q):
        self.grip_target = q
        if q is None:
            return
        stop_at = 0.0 if self.scenario == "empty" else 1.0
        goal = max(q, stop_at) if q < self.grip_q else q
        self.grip_q += float(np.clip(goal - self.grip_q, -0.3, 0.3))
        if not self.sent_grip or self.sent_grip[-1] != q:
            self.sent_grip.append(q)


def run(traj_path, scenario):
    traj = json.load(open(traj_path))
    traj["planned_at"] = time.time()
    r = FakeRobot(traj, scenario)
    p = AP.Player(r, traj, None)
    seg_of = {}
    for s in traj["segments"]:
        for i in range(s["start"], s["end"]):
            seg_of[i] = s["name"]
    orig_check = p.check

    def check(i, seg, *a):
        r.seg_now = seg
        return orig_check(i, seg, *a)
    p.check = check
    aborted = p.run()
    closed_at_end = r.grip_target == C.GRIP_CLOSE_Q
    back_home = np.abs(r.qarm - np.array(traj["start_q"])).max() < 0.05
    return aborted, closed_at_end, back_home, r.sent_grip


EXPECT = {  # scenario: (aborted?, gripper closed at end?, arm back at start?)
    "nominal": (False, True, True),
    "empty": (True, False, True),
    "stuck_pod": (True, False, True),
    "tilt_approach": (True, False, True),
    "tilt_retreat": (True, True, False),
    "stale": (True, False, True),
}

if __name__ == "__main__":
    ok_all = True
    for sc, exp in EXPECT.items():
        aborted, closed, home, grips = run(sys.argv[1], sc)
        got = (bool(aborted), closed, home)
        ok = got == exp
        ok_all &= ok
        print("%-14s %s  aborted=%-5s closed_at_end=%-5s back_at_start=%-5s gripper cmds=%s  %s" % (
            sc, "PASS" if ok else "FAIL", got[0], got[1], got[2], grips, ("(" + aborted + ")") if aborted else ""))
    sys.exit(0 if ok_all else 1)
