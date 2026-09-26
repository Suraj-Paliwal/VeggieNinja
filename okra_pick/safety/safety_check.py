#!/usr/bin/env python3
"""
Safety gate run before EVERY okra_pick.sh step. It measures the robot live (robot/safety_probe.py over ssh),
checks the VM, compares against the perception event / plan when there is one, and prints PASS/WARN/FAIL.

    ../dimos/.venv/bin/python safety/safety_check.py STEP [--event DIR] [--traj trajectory.json]

STEP: gripper | floor | look | perceive | plan | dry | reach | pick | status
Exit 0 = go (PASS/WARN), 3 = FAIL (the step must not run). Prints IFACE=<robot iface> for the caller.
Every result is saved: <event>/safety/<step>_<time>.json, or okra_data/safety/<day>/ without an event.

Nothing about the robot is assumed: clock, network interface, FSM, mode_machine, tilt, battery, temperatures,
other programs commanding the arm - all come from the probe at this moment. Ages are computed on the robot
clock only (perception time and "now" both from the Orin), so a VM/robot clock difference cannot hide a stale
plan. OKRA_SAFETY_ACCEPT=check1,check2 downgrades named FAILs to WARN for one run (logged with the result).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OKRA = os.path.dirname(HERE)
J = os.path.dirname(OKRA)
sys.path.insert(0, HERE)
import safety_limits as L  # noqa: E402

HOST = os.environ.get("G1_HOST", "unitree@192.168.123.164")
PROBE = "cd okra_pick/robot && PYTHONPATH=~/g1_rec/pylib timeout 30 python3 safety_probe.py"
BASELINE = os.path.join(J, "okra_data", "safety", "robot_baseline.json")
PASS, WARN, FAIL = "PASS", "WARN", "FAIL"


def run_probe():
    t0 = time.time()
    try:
        p = subprocess.run(["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", HOST, PROBE],
                           capture_output=True, text=True, timeout=45)
    except Exception as e:  # noqa: BLE001
        return None, {"ssh_error": str(e)}
    t1 = time.time()
    line = next((l for l in reversed(p.stdout.splitlines()) if l.startswith("{")), None)
    if line is None:
        return None, {"ssh_error": (p.stderr or p.stdout).strip()[-400:] or "rc %d" % p.returncode}
    probe = json.loads(line)
    # robot_time is sampled just before the probe exits: pair it with the receive time
    return probe, {"ssh_s": round(t1 - t0, 2), "clock_offset_s": round(probe["robot_time"] - t1, 2)}


def vm_state():
    du = shutil.disk_usage(J)
    return {"vm_disk_free_gb": round(du.free / 1e9, 1), "vm_load1": os.getloadavg()[0], "vm_cpus": os.cpu_count(),
            "vm_time": time.time()}


def load_json(path):
    try:
        return json.load(open(path))
    except Exception:  # noqa: BLE001
        return None


def evaluate(step, probe, link, vm, event_state=None, traj=None, baseline=None):
    """Pure function (tested offline). Returns a list of (check, level, message)."""
    R = []

    def add(name, ok, msg, level_if_bad=FAIL):
        R.append((name, PASS if ok else level_if_bad, msg))

    motion = step in L.MOTION_STEPS
    if probe is None:
        R.append(("link", FAIL, "robot probe failed: %s" % link.get("ssh_error")))
        return R
    add("link", True, "ssh round trip %.1f s, robot clock %+.1f s vs VM (measured; ages use the robot clock)"
        % (link["ssh_s"], link["clock_offset_s"]))
    for e in probe.get("errors", []):
        if e.startswith("battery unknown"):
            continue                                      # handled below
        R.append(("probe", FAIL if ("import" in e or "route" in e) else WARN, e))
    add("iface", bool(probe.get("iface")), "robot interface %s (route to the motion controller)" % probe.get("iface"))

    ls = probe.get("lowstate") or {}
    add("lowstate", ls.get("hz", 0) >= L.MIN_LOWSTATE_HZ and (ls.get("max_gap_s") or 99) <= L.MAX_LOWSTATE_GAP_S,
        "rt/lowstate %.0f Hz, max gap %s s (need >= %.0f Hz, <= %.2f s)"
        % (ls.get("hz", 0), ls.get("max_gap_s"), L.MIN_LOWSTATE_HZ, L.MAX_LOWSTATE_GAP_S))
    if "mode_machine" not in ls:
        return R

    mm, base_mm = ls["mode_machine"], (baseline or {}).get("mode_machine")
    if base_mm is None:
        R.append(("mode_machine", WARN, "mode_machine %d, no baseline yet (recorded now)" % mm))
    else:
        add("mode_machine", mm == base_mm, "mode_machine %d (baseline %d: joint indices depend on it)" % (mm, base_mm),
            FAIL if motion else WARN)

    roll, pitch = ls["rpy_rad"][0], ls["rpy_rad"][1]
    tilt = max(abs(roll), abs(pitch))
    add("tilt", tilt <= L.MAX_TILT_RAD, "roll %+.1f pitch %+.1f deg (max %.1f)" % (
        roll * 57.3, pitch * 57.3, L.MAX_TILT_RAD * 57.3), FAIL if motion else WARN)

    fsm = probe.get("fsm")
    if motion:
        add("fsm", fsm in L.MAIN_FSM, "FSM %s, switcher mode %r (need %s)" % (fsm, probe.get("switcher_mode"), L.MAIN_FSM))
        add("knee_load", ls["knee_load_nm"] >= L.MIN_KNEE_LOAD_NM,
            "knee load %.1f Nm (need >= %.0f: robot standing on its legs)" % (ls["knee_load_nm"], L.MIN_KNEE_LOAD_NM))
    else:
        R.append(("fsm", PASS, "FSM %s, switcher mode %r" % (fsm, probe.get("switcher_mode"))))

    # who else is commanding the arm / gripper right now
    if step in L.GRIPPER_STEPS:
        arm_hz, grip_hz = probe["arm_sdk"]["hz"], probe["grip_cmd"]["hz"]
        add("arm_sdk_free", arm_hz == 0, "rt/arm_sdk: %.0f Hz from another program" % arm_hz if arm_hz else
            "rt/arm_sdk: nobody publishing")
        add("gripper_cmd_free", grip_hz == 0, "rt/dex1/right/cmd: %.0f Hz from another program" % grip_hz if grip_hz else
            "rt/dex1/right/cmd: nobody publishing")
        procs = probe.get("motion_processes", [])
        add("motion_processes", not procs, "running: %s" % procs if procs else "no other motion program running")
        g = probe["grip_state"]
        add("gripper_state", g["hz"] >= L.MIN_GRIP_STATE_HZ, "Dex1 right state %.0f Hz, q %s" % (g["hz"], g.get("q")))

    # motors
    temps, errs = ls["motor_temp_c"], ls["motor_error"]
    hot = {i: t for i, t in enumerate(temps) if t >= L.MOTOR_TEMP_WARN_C}
    add("motor_temp", not hot, "max %d C on motor %d%s" % (max(temps), temps.index(max(temps)),
        (", >= %d C: %s" % (L.MOTOR_TEMP_WARN_C, hot)) if hot else ""), WARN)
    arm_err = {i: errs[i] for i in L.RIGHT_ARM_IDX if errs[i]}
    base_err = (baseline or {}).get("motor_error")
    other_err = {i: e for i, e in enumerate(errs) if e and i not in L.RIGHT_ARM_IDX
                 and not (base_err and base_err[i] == e)}
    add("arm_motor_error", not arm_err, "right-arm motor error bits %s" % arm_err if arm_err else
        "right-arm motors report no error bits", FAIL if motion else WARN)
    if other_err:
        R.append(("motor_error", WARN, "error bits (not in baseline) on motors %s" % other_err))

    # battery: measured or unknown, never assumed
    b = probe.get("battery")
    if b is None:
        R.append(("battery", WARN, "battery unknown (no BMS message) - check the robot's battery display"))
    else:
        add("battery", b["soc"] >= L.MIN_BATTERY_SOC, "battery %d %% (min %d, topic %s)" % (
            b["soc"], L.MIN_BATTERY_SOC, b["topic"]), FAIL if motion else WARN)

    # camera
    if step in L.CAMERA_STEPS:
        add("camera", bool(probe.get("realsense_usb")), "RealSense on USB: %s" % (probe.get("realsense_usb") or "none"))

    # resources
    add("robot_disk", probe["disk_free_gb"] >= L.MIN_ROBOT_DISK_GB, "robot disk %.1f GB free" % probe["disk_free_gb"])
    add("vm_disk", vm["vm_disk_free_gb"] >= L.MIN_VM_DISK_GB, "VM disk %.1f GB free" % vm["vm_disk_free_gb"])
    th = probe.get("orin_thermal_c") or {}
    if th:
        z = max(th, key=th.get)
        add("orin_temp", th[z] < L.ORIN_TEMP_WARN_C, "Orin %s %.0f C" % (z, th[z]), WARN)
    add("vm_load", vm["vm_load1"] < vm["vm_cpus"], "VM load %.1f on %d CPUs (keep the VM idle during motion)" % (
        vm["vm_load1"], vm["vm_cpus"]), WARN)
    if motion and probe.get("recorder_running"):
        R.append(("recorder", WARN, "a recorder is already running on the robot"))

    # against the perception event (robot clock only)
    if step in L.EVENT_STEPS:
        if not event_state or "time" not in event_state:
            R.append(("event_age", FAIL, "no robot_state.json with a robot-clock time in the event"))
        else:
            age = probe["robot_time"] - event_state["time"]
            add("event_age", 0 <= age <= L.MAX_EVENT_AGE_S, "perceived %.0f s ago on the robot clock (max %.0f)" % (
                age, L.MAX_EVENT_AGE_S))
            q_now, q_ev = ls["q"], event_state["q"]
            wd = max(abs(q_now[i] - q_ev[i]) for i in (12, 13, 14))
            add("waist_same", wd <= L.MAX_WAIST_DIFF_RAD, "waist moved %.3f rad since perception (max %.2f)" % (
                wd, L.MAX_WAIST_DIFF_RAD))
    if motion and traj is not None:
        d = max(abs(ls["q"][i] - s) for i, s in zip(L.RIGHT_ARM_IDX, traj["start_q"]))
        add("arm_start", d <= L.MAX_START_DIFF_RAD, "right arm %.3f rad from the plan's start pose (max %.2f)" % (
            d, L.MAX_START_DIFF_RAD))
    return R


def apply_accept(res, accept):
    """Operator-named FAILs become WARN for this run; the note stays in the saved result."""
    return [(n, WARN, m + " [FAIL accepted by operator]") if (lv == FAIL and n in accept) else (n, lv, m)
            for n, lv, m in res]


def overall(res):
    lv = {x[1] for x in res}
    return FAIL if FAIL in lv else WARN if WARN in lv else PASS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("step")
    ap.add_argument("--event")
    ap.add_argument("--traj")
    args = ap.parse_args()
    accept = {x for x in os.environ.get("OKRA_SAFETY_ACCEPT", "").split(",") if x}

    probe, link = run_probe()
    vm = vm_state()
    ev_state = load_json(os.path.join(args.event, "robot_state.json")) if args.event else None
    traj = load_json(args.traj) if args.traj else None
    baseline = load_json(BASELINE)
    res = evaluate(args.step, probe, link, vm, ev_state, traj, baseline)
    res = apply_accept(res, accept)
    worst = overall(res)
    print("safety check before '%s': %s" % (args.step, worst))
    for n, lv, m in res:
        print("  %-4s %-18s %s" % (lv, n, m))

    if probe and "mode_machine" in (probe.get("lowstate") or {}) and baseline is None:
        os.makedirs(os.path.dirname(BASELINE), exist_ok=True)
        json.dump({"mode_machine": probe["lowstate"]["mode_machine"], "motor_error": probe["lowstate"]["motor_error"],
                   "recorded": time.strftime("%Y-%m-%d %H:%M:%S")}, open(BASELINE, "w"), indent=1)

    out_dir = os.path.join(args.event, "safety") if args.event else \
        os.path.join(J, "okra_data", "safety", time.strftime("%Y-%m-%d"))
    os.makedirs(out_dir, exist_ok=True)
    json.dump({"step": args.step, "result": worst, "checks": res, "accepted": sorted(accept), "link": link, "vm": vm,
               "probe": probe}, open(os.path.join(out_dir, "%s_%s.json" % (args.step, time.strftime("%H%M%S"))), "w"),
              indent=1)
    if probe and probe.get("iface"):
        print("IFACE=%s" % probe["iface"])
    sys.exit(3 if worst == FAIL else 0)


if __name__ == "__main__":
    main()
