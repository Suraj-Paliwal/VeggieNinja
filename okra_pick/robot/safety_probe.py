#!/usr/bin/env python3
"""
Read-only snapshot of the robot, measured live (nothing is assumed, nothing is commanded).
Runs ON THE ROBOT (Orin) with ~/g1_rec/pylib (Python 3.8); prints ONE JSON line. Used by
okra_pick/safety/safety_check.py before every okra_pick.sh step.

    PYTHONPATH=~/g1_rec/pylib python3 safety_probe.py [--iface auto] [--window 1.0]

Reports: robot clock, network interface (from the route to the motion controller), rt/lowstate rate/gaps,
mode_machine, IMU tilt, knee load, per-motor temperature and error bits, Dex1 right state, whether anybody
is already publishing rt/arm_sdk or rt/dex1/right/cmd, battery (if a BMS topic answers), FSM id and
MotionSwitcher mode, motion-related processes, RealSense on USB, disk free and Orin thermal zones.
A value that cannot be read is reported as null with the reason, never filled in.
"""

import argparse
import glob
import json
import os
import shutil
import subprocess
import threading
import time

MOTION_CONTROLLER = "192.168.123.161"
BMS_TOPICS = ["rt/lf/bmsstate", "rt/bmsstate"]          # tried; whichever answers is reported
MOTION_PROCS = ["arm_player.py", "head_teleop.py", "arm_move.py", "waist.py", "g1_fsm.py", "record.py --head"]
L_KNEE, R_KNEE = 3, 9


def sh(cmd, timeout=3):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return "error: %s" % e


def detect_iface():
    out = sh("ip -o route get %s" % MOTION_CONTROLLER)
    parts = out.split()
    return parts[parts.index("dev") + 1] if "dev" in parts else None


class Topic:
    """Counts messages and inter-arrival gaps; keeps the latest message."""

    def __init__(self):
        self.n, self.last, self.t_first, self.t_last, self.max_gap = 0, None, None, None, 0.0
        self.lock = threading.Lock()

    def cb(self, m):
        t = time.time()
        with self.lock:
            if self.t_last is not None:
                self.max_gap = max(self.max_gap, t - self.t_last)
            if self.t_first is None:
                self.t_first = t
            self.n, self.last, self.t_last = self.n + 1, m, t

    def reset(self):
        with self.lock:
            self.n, self.t_first, self.t_last, self.max_gap = 0, None, None, 0.0



def max_temp(t):
    """Motor temperature as one int: unitree_hg motors report a list (winding, case), Dex1 (unitree_go) one int."""
    if t is None:
        return None
    return max(int(x) for x in t) if hasattr(t, "__iter__") else int(t)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="auto")
    ap.add_argument("--window", type=float, default=1.0, help="measurement window, s")
    args = ap.parse_args()
    t_start = time.time()
    rep = {"probe_version": 1, "errors": []}

    iface = detect_iface() if args.iface == "auto" else args.iface
    rep["iface"] = iface
    if not iface:
        rep["errors"].append("no route to the motion controller %s" % MOTION_CONTROLLER)

    # ---- processes / devices / disk / thermal (no DDS needed)
    ps = sh("ps -eo pid,args")
    rep["motion_processes"] = [l.strip() for l in ps.splitlines()
                               if any(p in l for p in MOTION_PROCS) and "safety_probe" not in l]
    rep["recorder_running"] = any("recorder.py" in l for l in ps.splitlines())
    rep["agent_running"] = any("agent.py" in l and "python" in l for l in ps.splitlines())
    rep["realsense_usb"] = [l for l in sh("lsusb").splitlines() if "8086:" in l]
    du = shutil.disk_usage(os.path.expanduser("~"))
    rep["disk_free_gb"] = round(du.free / 1e9, 1)
    zones = {}
    for z in glob.glob("/sys/class/thermal/thermal_zone*/"):
        try:
            zones[open(z + "type").read().strip()] = int(open(z + "temp").read()) / 1000.0
        except Exception:  # noqa: BLE001
            pass
    rep["orin_thermal_c"] = zones
    rep["videohub"] = sh("/unitree/sbin/mscli getservice video_hub_pc4 2>/dev/null | grep -o 'status:[0-9]*'") or None

    # ---- DDS
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowCmd_, LowState_
    except Exception as e:  # noqa: BLE001
        rep["errors"].append("unitree_sdk2py import failed: %s" % e)
        rep["robot_time"] = time.time()
        print(json.dumps(rep))
        return
    ChannelFactoryInitialize(0, iface or "")
    tp = {k: Topic() for k in ("lowstate", "grip_state", "arm_sdk", "grip_cmd")}
    subs = [ChannelSubscriber("rt/lowstate", LowState_), ChannelSubscriber("rt/dex1/right/state", MotorStates_),
            ChannelSubscriber("rt/arm_sdk", LowCmd_), ChannelSubscriber("rt/dex1/right/cmd", MotorCmds_)]
    for s, k in zip(subs, ("lowstate", "grip_state", "arm_sdk", "grip_cmd")):
        s.Init(tp[k].cb, 10)
    bms = {}
    for name in BMS_TOPICS:
        bms[name] = Topic()
        s = ChannelSubscriber(name, BmsState_)
        s.Init(bms[name].cb, 10)
        subs.append(s)

    t0 = time.time()                                      # discovery: wait for the first lowstate
    while tp["lowstate"].n == 0 and time.time() - t0 < 5.0:
        time.sleep(0.05)
    rep["discovery_s"] = round(time.time() - t0, 2)
    for t in tp.values():
        t.reset()
    time.sleep(args.window)
    w = args.window

    for k, t in tp.items():
        rep[k] = {"hz": round(t.n / w, 1), "max_gap_s": round(t.max_gap, 3) if t.n > 1 else None,
                  "age_s": None if t.t_last is None else round(time.time() - t.t_last, 3)}
    m = tp["lowstate"].last
    if m is not None:
        ms = m.motor_state
        rep["lowstate"].update(
            mode_machine=int(m.mode_machine), tick=int(m.tick),
            rpy_rad=[round(float(x), 4) for x in m.imu_state.rpy],
            knee_load_nm=round(abs(ms[L_KNEE].tau_est) + abs(ms[R_KNEE].tau_est), 1),
            motor_temp_c=[max_temp(ms[i].temperature) for i in range(29)],
            motor_error=[int(ms[i].motorstate) for i in range(29)],
            q=[round(float(ms[i].q), 4) for i in range(29)])
    g = tp["grip_state"].last
    if g is not None and len(g.states):
        rep["grip_state"].update(q=round(float(g.states[0].q), 3),
                                 temp_c=max_temp(getattr(g.states[0], "temperature", None)))
    for name, t in bms.items():
        if t.last is not None:
            b = t.last
            rep["battery"] = {"topic": name, "soc": int(b.soc), "current": int(b.current),
                              "voltage_mv": [int(x) for x in b.bmsvoltage], "temp_c": [int(x) for x in b.temperature]}
            break
    else:
        rep["battery"] = None
        rep["errors"].append("battery unknown: no message on %s" % ", ".join(BMS_TOPICS))

    # ---- FSM + MotionSwitcher mode (RPC)
    try:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        c = LocoClient()
        c.SetTimeout(3.0)
        c.Init()
        fsm = None
        for _ in range(3):
            code, data = c._Call(7001, "{}")
            if code == 0:
                fsm = json.loads(data)["data"]
                break
            time.sleep(0.3)
        rep["fsm"] = fsm
        if fsm is None:
            rep["errors"].append("FSM query failed (last code %s)" % code)
    except Exception as e:  # noqa: BLE001
        rep["fsm"] = None
        rep["errors"].append("FSM query error: %s" % e)
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        msc = MotionSwitcherClient()
        msc.SetTimeout(3.0)
        msc.Init()
        code, data = msc.CheckMode()
        rep["switcher_mode"] = data.get("name") if code == 0 and isinstance(data, dict) else None
    except Exception as e:  # noqa: BLE001
        rep["switcher_mode"] = None
        rep["errors"].append("MotionSwitcher error: %s" % e)

    rep["probe_s"] = round(time.time() - t_start, 2)
    rep["robot_time"] = time.time()                       # sampled last: the VM pairs it with the ssh round trip
    print(json.dumps(rep))


if __name__ == "__main__":
    main()
