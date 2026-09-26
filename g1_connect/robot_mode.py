#!/usr/bin/env python3
"""Switch the G1 locomotion state (LocoClient FSM). Runs ON THE ROBOT (Loco RPC is unreliable from the VM).

From the VM:   ./robot_mode.sh status | damp | ready | start | zero

  status  read-only: FSM id/mode, MotionSwitcher mode, leg/waist torques
  ai      MotionSwitcher SelectMode("ai"): starts Unitree's motion service (needed after a reboot
          if CheckMode shows name ''); nothing moves
  damp    FSM 1   damping: joints resist motion, nothing moves by itself
  ready   FSM 4   stand up / locked stand: legs move to the standing pose (robot must hang on the gantry)
  start   FSM 200 AI balance: balance controller on (feet must be flat on the ground, gantry slack).
          On this firmware 500 is acknowledged but ignored; 200 worked (2026-09-26) and the remote gives 501.
  zero    FSM 0   zero torque: robot goes limp (only when it is supported)
  step VY [VX] [DUR]  walk with body velocity (m/s, +VY = left, +VX = forward) for DUR s (default 1)
                      only in a balance state (FSM 200/500/501); |v| <= 0.2 m/s, DUR <= 2 s

Normal bring-up: damp -> ready -> (lower gantry until feet are flat) -> start.
Every step prints the state afterwards. Nothing here commands arms or the waist.

Balance condition: `start` is refused unless the torso is upright (|roll|, |pitch| < MAX_TILT)
and the knees carry load (|L| + |R| knee torque > MIN_KNEE_LOAD). `start --ignore-load` skips
only the load part (tau_est may be unreliable in locked stand); the tilt part is never skipped.
After `start` the tilt is watched for WATCH_S seconds and reported.
"""

import json
import sys
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID, ROBOT_API_ID_LOCO_GET_FSM_MODE
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

FSM = {"zero": 0, "damp": 1, "ready": 4, "start": 200}
MAIN = (200, 500, 501)  # states in which the balance controller runs
NAMES = {0: "zero torque", 1: "damp", 2: "squat", 3: "sit", 4: "stand up / locked stand", 200: "AI balance",
         500: "main control (balance)", 501: "main control", 702: "lie->stand", 706: "squat<->stand"}
JOINTS = {0: "L hip pitch", 3: "L knee", 6: "R hip pitch", 9: "R knee", 12: "waist yaw", 14: "waist pitch"}

MAX_TILT = 0.05       # rad (~3 deg) roll/pitch allowed before start
MIN_KNEE_LOAD = 15.0  # Nm, |L knee| + |R knee| torque before start
WATCH_S = 5.0         # s of tilt monitoring after start
MAX_STEP_V = 0.2      # m/s, step command speed limit
MAX_STEP_DUR = 2.0    # s, step command duration limit
STEP_ABORT_TILT = 0.14  # rad (~8 deg): stop walking at once above this
L_KNEE, R_KNEE = 3, 9

state = {}


def call(client, api, tries=3):
    for _ in range(tries):
        code, data = client._Call(api, json.dumps({}))
        if code == 0:
            return json.loads(data)["data"]
        time.sleep(0.5)
    return "error %d" % code


def status(client, ms):
    fsm = call(client, ROBOT_API_ID_LOCO_GET_FSM_ID)
    print("FSM id   : %s (%s)" % (fsm, NAMES.get(fsm, "?")))
    print("FSM mode : %s" % call(client, ROBOT_API_ID_LOCO_GET_FSM_MODE))
    print("switcher : %s" % (ms.CheckMode(),))
    m = state.get("m")
    if m is None:
        print("no rt/lowstate")
        return fsm
    print("imu rpy  : %s rad" % [round(x, 3) for x in m.imu_state.rpy])
    for i, n in JOINTS.items():
        s = m.motor_state[i]
        print("  %-12s q %+.3f  tau %+6.2f Nm" % (n, s.q, s.tau_est))
    print("start ok?: %s %s" % balance_ok())
    return fsm


def balance_ok(ignore_load=False):
    """Returns (ok, reason) for the start condition, from the latest rt/lowstate."""
    m = state.get("m")
    if m is None:
        return False, "no rt/lowstate"
    roll, pitch = m.imu_state.rpy[0], m.imu_state.rpy[1]
    load = abs(m.motor_state[L_KNEE].tau_est) + abs(m.motor_state[R_KNEE].tau_est)
    msg = "roll %+.1f deg, pitch %+.1f deg (limit %.0f), knee load %.1f Nm (min %.0f)" % (
        roll * 57.3, pitch * 57.3, MAX_TILT * 57.3, load, MIN_KNEE_LOAD)
    if max(abs(roll), abs(pitch)) > MAX_TILT:
        return False, "NOT UPRIGHT: " + msg
    if load < MIN_KNEE_LOAD and not ignore_load:
        return False, "LEGS NOT LOADED: " + msg
    return True, msg


def watch_tilt(seconds):
    t0, worst = time.time(), 0.0
    while time.time() - t0 < seconds:
        m = state.get("m")
        if m is not None:
            worst = max(worst, abs(m.imu_state.rpy[0]), abs(m.imu_state.rpy[1]))
        time.sleep(0.05)
    print("   max tilt in %.0f s after start: %.1f deg" % (seconds, worst * 57.3), flush=True)


def step(client, vy, vx, dur):
    """Walk briefly with the balance gate before and a tilt watch during; always ends with a stop."""
    fsm = call(client, ROBOT_API_ID_LOCO_GET_FSM_ID)
    if fsm not in MAIN:
        print("REFUSED: FSM %s is not a balance state %s" % (fsm, MAIN))
        sys.exit(1)
    ok, why = balance_ok()
    print("balance check: %s" % why, flush=True)
    if not ok:
        print("REFUSED: not balanced, nothing sent")
        sys.exit(1)
    if abs(vx) > MAX_STEP_V or abs(vy) > MAX_STEP_V or not 0 < dur <= MAX_STEP_DUR:
        print("REFUSED: limits are |v| <= %.1f m/s, 0 < DUR <= %.0f s" % (MAX_STEP_V, MAX_STEP_DUR))
        sys.exit(1)
    print("-> SetVelocity(vx=%.2f, vy=%.2f, omega=0, %.1f s)  ~%.0f cm" % (
        vx, vy, dur, 100 * dur * max(abs(vx), abs(vy))), flush=True)
    worst, aborted = 0.0, False
    try:
        print("   reply code %s" % client.SetVelocity(vx, vy, 0.0, dur), flush=True)
        t0 = time.time()
        while time.time() - t0 < dur + 1.5:  # walking + settling
            m = state["m"]
            tilt = max(abs(m.imu_state.rpy[0]), abs(m.imu_state.rpy[1]))
            worst = max(worst, tilt)
            if tilt > STEP_ABORT_TILT:
                print("TILT ABORT at %.1f deg" % (tilt * 57.3), flush=True)
                aborted = True
                break
            time.sleep(0.005)
    finally:
        print("-> StopMove  reply code %s" % client.SetVelocity(0.0, 0.0, 0.0, 1.0), flush=True)
    print("   max tilt during step: %.1f deg%s" % (worst * 57.3, "  (ABORTED)" if aborted else ""), flush=True)


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    ignore_load = "--ignore-load" in sys.argv
    if cmd not in FSM and cmd not in ("status", "step", "ai"):
        sys.exit(__doc__)
    ChannelFactoryInitialize(0, "eth0")
    ChannelSubscriber("rt/lowstate", LowState_).Init(lambda m: state.__setitem__("m", m), 10)
    ms = MotionSwitcherClient()
    ms.SetTimeout(3.0)
    ms.Init()
    client = LocoClient()
    client.SetTimeout(5.0)
    client.Init()
    time.sleep(1.5)
    if cmd == "status":
        status(client, ms)
        return
    if cmd == "ai":
        print("-> SelectMode('ai')  reply %s" % (ms.SelectMode("ai"),), flush=True)
        time.sleep(5.0)  # the motion service needs a few seconds to come up
        status(client, ms)
        return
    if cmd == "step":
        a = [float(x) for x in sys.argv[2:] if not x.startswith("--")]
        if not a:
            sys.exit("usage: step VY [VX] [DUR]")
        step(client, a[0], a[1] if len(a) > 1 else 0.0, a[2] if len(a) > 2 else 1.0)
        time.sleep(1.0)
        status(client, ms)
        return
    if cmd == "start":
        ok, why = balance_ok(ignore_load)
        print("balance check: %s" % why, flush=True)
        if not ok:
            print("REFUSED: start not sent. Straighten the robot / load the feet, then retry.")
            sys.exit(1)
    print("-> SetFsmId(%d) %s" % (FSM[cmd], NAMES[FSM[cmd]]), flush=True)
    code = client.SetFsmId(FSM[cmd])
    print("   reply code %s" % code, flush=True)
    if cmd == "start":
        watch_tilt(WATCH_S)
    else:
        time.sleep(3.0)
    status(client, ms)


if __name__ == "__main__":
    main()
