#!/usr/bin/env python3
"""Aim the G1 head camera (waist yaw/pitch via rt/arm_sdk) from a terminal while rec.sh records.

Runs on the VM. Uses g1_video/waist.py (limits: yaw ±0.6 rad, pitch ±0.3 rad, 0.4 rad/s,
2 s blend in/out; arms are held where they are). Nothing moves until the first key / sweep step.

  ../g1_video/.venv/bin/python head_teleop.py            # keys (needs a real terminal)
  ../g1_video/.venv/bin/python head_teleop.py --sweep    # scripted: left, right, up, down, center
  ../g1_video/.venv/bin/python head_teleop.py --yaw -0.15 # one move (here ~9 deg right), hold, center

Keys: a/← left   d/→ right   w/↑ up   s/↓ down   c center   q quit (centers and releases)

Balance condition: refuses to engage if |roll| or |pitch| > START_TILT; while engaged, if the
tilt drifts more than ABORT_DRIFT from the start, it stops, centers and releases at once.
"""

import argparse
import os
import select
import sys
import termios
import time
import threading
import tty

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, "..", "g1_video"))  # VM layout
sys.path.insert(0, here)                                 # robot: waist.py copied next to it
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from waist import WaistController  # noqa: E402

STEP = 0.1  # rad per key press (~6 deg)
KEYS = {"a": (STEP, 0), "\x1b[D": (STEP, 0), "d": (-STEP, 0), "\x1b[C": (-STEP, 0),
        "w": (0, -STEP), "\x1b[A": (0, -STEP), "s": (0, STEP), "\x1b[B": (0, STEP)}
START_TILT = 0.10   # rad (~6 deg): max |roll|/|pitch| to engage
ABORT_DRIFT = 0.08  # rad (~5 deg): tilt change that aborts the move
STALE_S = 0.5       # s without a new rt/lowstate (lossy link) also aborts
abort = threading.Event()

# (label, yaw, pitch, hold s) — yaw + = left, pitch + = down
SWEEP = [("left", 0.3, 0.0, 2.0), ("right", -0.3, 0.0, 2.0), ("center", 0.0, 0.0, 1.5),
         ("up", 0.0, -0.15, 2.0), ("down", 0.0, 0.15, 2.0), ("center", 0.0, 0.0, 1.5)]


def wait_state(w, timeout=20.0):
    t0 = time.time()
    while not w.ready():
        if time.time() - t0 > timeout:
            sys.exit("no rt/lowstate from the robot after %.0f s (run g1_connect/check.sh)" % timeout)
        time.sleep(0.1)


def tilt(w):
    rpy = w.state.imu_state.rpy
    return rpy[0], rpy[1]


def balance_watch(w, r0, p0):
    """Abort (center + release) if the torso tilts ABORT_DRIFT away from where it started,
    or if the state stops arriving for STALE_S (we can no longer see the balance)."""
    last_msg, last_t = w.state, time.monotonic()
    while not abort.is_set() and not w.released.is_set():
        if w.state is not last_msg:
            last_msg, last_t = w.state, time.monotonic()
        r, p = tilt(w)
        why = None
        if max(abs(r - r0), abs(p - p0)) > ABORT_DRIFT:
            why = "roll %+.1f pitch %+.1f deg (start %+.1f %+.1f)" % (r * 57.3, p * 57.3, r0 * 57.3, p0 * 57.3)
        elif time.monotonic() - last_t > STALE_S:
            why = "no rt/lowstate for %.1f s" % (time.monotonic() - last_t)
        if why:
            print("\nBALANCE ABORT: " + why, flush=True)
            abort.set()
            w.center()
            return
        time.sleep(0.01)


def sleep_or_abort(seconds):
    if abort.wait(seconds):
        raise KeyboardInterrupt("balance abort")


def goto(w, yaw, pitch):
    gy, gp = w.goal()
    w.nudge(yaw - gy, pitch - gp)


def run_sweep(w):
    for label, yaw, pitch, hold in SWEEP:
        goto(w, yaw, pitch)
        gy, gp = w.goal()
        print("-> %-6s goal yaw %+.2f pitch %+.2f" % (label, gy, gp), flush=True)
        sleep_or_abort(abs(yaw) / w.speed + abs(pitch) / w.speed + hold)


def run_once(w, yaw, pitch, hold):
    goto(w, yaw, pitch)
    gy, gp = w.goal()
    print("-> goal yaw %+.2f pitch %+.2f, hold %.0f s" % (gy, gp, hold), flush=True)
    sleep_or_abort((abs(gy) + abs(gp)) / w.speed + hold)


def read_key():
    k = os.read(sys.stdin.fileno(), 3).decode(errors="ignore")
    return k if k.startswith("\x1b") else k[:1].lower()


def run_keys(w):
    if not sys.stdin.isatty():
        sys.exit("keyboard mode needs a real terminal; use --sweep here")
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(__doc__.split("Keys:")[1].strip(), flush=True)
    try:
        while not abort.is_set():
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            k = read_key()
            if k == "q":
                break
            if k == "c":
                w.center()
            elif k in KEYS:
                w.nudge(*KEYS[k])
            else:
                continue
            gy, gp = w.goal()
            print("\rgoal yaw %+.2f  pitch %+.2f   " % (gy, gp), end="", flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="enp0s8")
    ap.add_argument("--sweep", action="store_true", help="scripted small moves, then release")
    ap.add_argument("--yaw", type=float, help="single move: yaw offset from the start pose, rad (+ = left, - = right)")
    ap.add_argument("--pitch", type=float, default=0.0, help="with --yaw: pitch offset from the start pose (+ = down)")
    ap.add_argument("--hold", type=float, default=3.0, help="with --yaw: seconds to hold")
    ap.add_argument("--speed", type=float, default=0.4, help="waist speed rad/s (max 0.4)")
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    w = WaistController(speed=args.speed)
    print("waiting for rt/lowstate...", flush=True)
    wait_state(w)
    r0, p0 = tilt(w)
    print("balance check: roll %+.1f pitch %+.1f deg (limit %.0f)" % (r0 * 57.3, p0 * 57.3, START_TILT * 57.3))
    if max(abs(r0), abs(p0)) > START_TILT:
        sys.exit("REFUSED: robot is not upright; nothing was sent")
    try:
        if not w.engage():
            sys.exit("could not engage")
        threading.Thread(target=balance_watch, args=(w, r0, p0), daemon=True).start()
        print("engaged: blending in over 2 s (holding current pose), speed %.2f rad/s" % w.speed, flush=True)
        sleep_or_abort(2.0)
        if args.yaw is not None:
            run_once(w, args.yaw, args.pitch, args.hold)
        elif args.sweep:
            run_sweep(w)
        else:
            run_keys(w)
    except KeyboardInterrupt:
        pass
    finally:
        print("centering and releasing arm_sdk...", flush=True)
        w.release()
        print("released" if w.released.is_set() else "release timed out!", flush=True)


if __name__ == "__main__":
    main()
