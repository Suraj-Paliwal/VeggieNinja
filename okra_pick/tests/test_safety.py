#!/usr/bin/env python3
"""
Safety gate logic (safety/safety_check.py evaluate) on synthetic robot probes. No robot needed.

    ../dimos/.venv/bin/python tests/test_safety.py
"""

import copy
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "safety"))
import safety_check as S  # noqa: E402

T_ROBOT = 1_000_000.0          # robot clock "now"
Q = [0.0] * 29
Q[12:15] = [-0.61, 0.0, 0.0]
Q[22:29] = [0.29, -0.22, 0.0, 0.97, 0.0, 0.0, 0.0]

HEALTHY = {
    "errors": [], "iface": "eth0", "robot_time": T_ROBOT, "fsm": 200, "switcher_mode": "ai",
    "lowstate": {"hz": 990.0, "max_gap_s": 0.05, "mode_machine": 5, "rpy_rad": [0.01, 0.005, 0.3],
                 "knee_load_nm": 40.0, "motor_temp_c": [40] * 29, "motor_error": [0] * 29, "q": Q},
    "grip_state": {"hz": 490.0, "q": 5.3}, "arm_sdk": {"hz": 0.0}, "grip_cmd": {"hz": 0.0},
    "motion_processes": [], "recorder_running": False, "realsense_usb": ["Bus 001 ID 8086:0b3a Intel RealSense"],
    "disk_free_gb": 40.0, "orin_thermal_c": {"CPU-therm": 55.0},
    "battery": {"topic": "rt/lf/bmsstate", "soc": 80},
}
LINK = {"ssh_s": 3.1, "clock_offset_s": -690.0}          # the Orin ~11.5 min behind the VM (measured 2026-09-26)
VM = {"vm_disk_free_gb": 60.0, "vm_load1": 1.0, "vm_cpus": 8}
EVENT = {"time": T_ROBOT - 60.0, "q": Q}                 # perceived 60 s ago on the robot clock
TRAJ = {"start_q": Q[22:29]}
BASE = {"mode_machine": 5, "motor_error": [0] * 29}


def probe(**changes):
    p = copy.deepcopy(HEALTHY)
    for k, v in changes.items():
        if "." in k:
            a, b = k.split(".")
            p[a][b] = v
        else:
            p[k] = v
    return p


def levels(res):
    return {n: lv for n, lv, _ in res}


CASES = [
    # name, step, probe, event, expect overall, checks that must have the given level
    ("healthy reach, clock 11.5 min off", "reach", probe(), EVENT, "PASS", {}),
    ("robot unreachable", "reach", None, EVENT, "FAIL", {"link": "FAIL"}),
    ("stale perception (robot clock)", "reach", probe(), dict(EVENT, time=T_ROBOT - 400), "FAIL", {"event_age": "FAIL"}),
    ("perception in the robot's future", "reach", probe(), dict(EVENT, time=T_ROBOT + 30), "FAIL", {"event_age": "FAIL"}),
    ("no perception time", "plan", probe(), {"q": Q}, "FAIL", {"event_age": "FAIL"}),
    ("another program on rt/arm_sdk", "pick", probe(**{"arm_sdk.hz": 36.0}), EVENT, "FAIL", {"arm_sdk_free": "FAIL"}),
    ("waist teleop process running", "reach", probe(motion_processes=["123 python3 head_teleop.py"]), EVENT, "FAIL",
     {"motion_processes": "FAIL"}),
    ("not in a balance FSM", "reach", probe(fsm=4), EVENT, "FAIL", {"fsm": "FAIL"}),
    ("legs not loaded (on gantry, feet up)", "reach", probe(**{"lowstate.knee_load_nm": 3.0}), EVENT, "FAIL",
     {"knee_load": "FAIL"}),
    ("tilted", "reach", probe(**{"lowstate.rpy_rad": [0.0, 0.09, 0.0]}), EVENT, "FAIL", {"tilt": "FAIL"}),
    ("tilted, look only warns", "look", probe(**{"lowstate.rpy_rad": [0.0, 0.09, 0.0]}), None, "WARN", {"tilt": "WARN"}),
    ("lowstate slow", "reach", probe(**{"lowstate.hz": 40.0}), EVENT, "FAIL", {"lowstate": "FAIL"}),
    ("battery unknown -> warn, not assumed", "reach", probe(battery=None), EVENT, "WARN", {"battery": "WARN"}),
    ("battery low", "pick", probe(battery={"topic": "rt/lf/bmsstate", "soc": 12}), EVENT, "FAIL", {"battery": "FAIL"}),
    ("hot motor warns", "reach", probe(**{"lowstate.motor_temp_c": [40] * 25 + [75] + [40] * 3}), EVENT, "WARN",
     {"motor_temp": "WARN"}),
    ("right-arm error bits", "pick", probe(**{"lowstate.motor_error": [0] * 24 + [4] + [0] * 4}), EVENT, "FAIL",
     {"arm_motor_error": "FAIL"}),
    ("mode_machine changed", "reach", probe(**{"lowstate.mode_machine": 6}), EVENT, "FAIL", {"mode_machine": "FAIL"}),
    ("waist moved since perception", "reach", probe(**{"lowstate.q": Q[:12] + [-0.5, 0.0, 0.0] + Q[15:]}), EVENT,
     "FAIL", {"waist_same": "FAIL"}),
    ("no camera for look", "look", probe(realsense_usb=[]), None, "FAIL", {"camera": "FAIL"}),
    ("no iface route", "look", probe(iface=None), None, "FAIL", {"iface": "FAIL"}),
    ("sdk missing on robot", "reach", {"errors": ["unitree_sdk2py import failed: x"], "iface": "eth0",
                                        "robot_time": T_ROBOT}, EVENT, "FAIL", {"probe": "FAIL"}),
]


def main():
    ok_all = True
    for name, step, p, ev, exp, must in CASES:
        res = S.evaluate(step, p, LINK if p else {"ssh_error": "timeout"}, VM, ev, TRAJ, BASE)
        got, lv = S.overall(res), levels(res)
        ok = got == exp and all(lv.get(k) == v for k, v in must.items())
        ok_all &= ok
        print("%-40s %-6s %s %s" % (name, step, "PASS" if ok else "FAIL", got if ok else (got, lv)))
    # operator accept: downgrades only the named FAIL
    res = S.apply_accept(S.evaluate("pick", probe(battery=None, **{"arm_sdk.hz": 36.0}), LINK, VM, EVENT, TRAJ, BASE),
                         {"battery"})
    ok = S.overall(res) == "FAIL" and levels(res)["arm_sdk_free"] == "FAIL"
    ok_all &= ok
    print("%-40s %-6s %s" % ("accept list only downgrades named checks", "pick", "PASS" if ok else "FAIL"))
    # player preflight: perception age on the robot clock only
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "robot"))
    import time
    import numpy as np
    import arm_player as AP
    AP.fsm_state = lambda: 200

    class R:
        ls = True
        def tilt(self): return (0.0, 0.0)
        def q(self, idx): return np.array([Q[i] for i in idx])
    now = time.time()
    base_traj = {"start_q": Q[22:29], "target": {"q": Q}}
    for name, t_perc, want_ok in (("player: no robot-clock perception time", None, False),
                                  ("player: perceived 60 s ago", now - 60, True),
                                  ("player: perceived 400 s ago", now - 400, False),
                                  ("player: perception in the future", now + 30, False)):
        why = AP.preflight(R(), dict(base_traj, perceived_at_robot=t_perc))
        ok = (why is None) == want_ok
        ok_all &= ok
        print("%-40s %-6s %s %s" % (name, "dry", "PASS" if ok else "FAIL", why or "go"))
    print("ALL PASS" if ok_all else "SOME FAILED")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
