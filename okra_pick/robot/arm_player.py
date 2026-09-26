#!/usr/bin/env python3
"""
Execute a planned okra grasp on the G1. Runs ON THE ROBOT (Orin) with ~/g1_rec/pylib (Python 3.8).

    PYTHONPATH=~/g1_rec/pylib python3 arm_player.py trajectory.json            # real run
    PYTHONPATH=~/g1_rec/pylib python3 arm_player.py trajectory.json --dry      # all checks, nothing sent
    PYTHONPATH=~/g1_rec/pylib python3 arm_player.py --gripper open|close       # gripper only
    ... arm_player.py trajectory.json --until approach   # stop after a segment (e.g. reach test, no grasp)

Before moving: balance state (FSM 200/500/501), upright, fresh rt/lowstate, and the plan's start pose
matches the arm now (else the plan is stale). Then: open gripper -> blend arm_sdk in over 2 s holding
the current upper body -> play the right-arm trajectory at 50 Hz -> close the gripper at the event and
check it holds something -> finish -> blend out over 2 s.

Monitors every tick (abort): tilt drift, stale state, tracking error, and joint torque during the pull.
Abort = stop advancing; if the pod is held during the pull, open the gripper; then play the executed path
backwards to the start pose and blend out. Ctrl-C does the same.
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pick_config as C  # noqa: E402

MAIN_FSM = (200, 500, 501)


class Robot:
    """rt/lowstate + rt/dex1/right/state in, rt/arm_sdk + rt/dex1/right/cmd out."""

    def __init__(self, iface, dry):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorCmd_, MotorStates_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
        ChannelFactoryInitialize(0, iface)
        self.dry = dry
        self.ls, self.ls_t, self.grip_q = None, 0.0, None
        ChannelSubscriber("rt/lowstate", LowState_).Init(self._on_ls, 10)
        ChannelSubscriber(C.GRIP_STATE_TOPIC, MotorStates_).Init(self._on_grip, 10)
        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.grip_pub = ChannelPublisher(C.GRIP_CMD_TOPIC, MotorCmds_)
        if not dry:
            self.arm_pub.Init()
            self.grip_pub.Init()
        self.cmd, self.crc = unitree_hg_msg_dds__LowCmd_(), CRC()
        self.MotorCmds_, self.MotorCmd_ = MotorCmds_, MotorCmd_
        self.grip_target = None

    def _on_ls(self, m):
        self.ls, self.ls_t = m, time.time()

    def _on_grip(self, m):
        if len(m.states):
            self.grip_q = m.states[0].q

    def q(self, idx):
        return np.array([self.ls.motor_state[i].q for i in idx])

    def q29(self):
        return [self.ls.motor_state[i].q for i in range(29)]

    def tau(self, idx):
        return np.array([self.ls.motor_state[i].tau_est for i in idx])

    def tilt(self):
        r = self.ls.imu_state.rpy
        return r[0], r[1]

    def age(self):
        return time.time() - self.ls_t

    def send_arm(self, q_upper, weight):
        """q_upper: 17 targets for motors 12..28; weight: arm_sdk blend 0..1."""
        for k, i in enumerate(C.UPPER_IDX):
            m = self.cmd.motor_cmd[i]
            m.q, m.dq, m.tau = float(q_upper[k]), 0.0, 0.0
            m.kp, m.kd = (C.KP_WAIST, C.KD_WAIST) if i in C.WAIST_IDX else (C.KP_ARM, C.KD_ARM)
        self.cmd.motor_cmd[C.WEIGHT_IDX].q = float(weight)
        self.cmd.crc = self.crc.Crc(self.cmd)
        if not self.dry:
            self.arm_pub.Write(self.cmd)

    def send_grip(self, q):
        self.grip_target = q
        msg = self.MotorCmds_([self.MotorCmd_(mode=1, q=float(q), dq=0.0, tau=0.0, kp=C.GRIP_KP, kd=C.GRIP_KD,
                                              reserve=[0, 0, 0])])
        if not self.dry:
            self.grip_pub.Write(msg)


def fsm_state():
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    c = LocoClient()
    c.SetTimeout(3.0)
    c.Init()
    for _ in range(3):
        code, data = c._Call(7001, "{}")
        if code == 0:
            return json.loads(data)["data"]
        time.sleep(0.3)
    return None


class Abort(Exception):
    pass


class Player:
    def __init__(self, robot, traj, until, closed_loop=True):
        self.r, self.t = robot, traj
        self.Q = np.array(traj["q"])
        self.corr = None
        if closed_loop and traj.get("target", {}).get("q") and traj.get("pod_xyz"):
            from closed_loop import Corrector
            from g1_chain import Chain
            self.corr = Corrector(Chain(os.path.join(HERE, "g1.urdf")), traj)
        self.cmd_q = self.Q[0].copy()
        self.corr_log = []
        self.until = until
        self.log = []
        self.stop = threading.Event()
        self.holding = False

    def wait_grip(self, timeout):
        """Keep commanding the gripper target until it stops moving. Returns the final q (or None)."""
        t0, last, still = time.time(), None, 0.0
        while time.time() - t0 < timeout:
            self.r.send_grip(self.r.grip_target)
            q = self.r.grip_q
            if q is not None and last is not None and abs(q - last) < 0.005:
                still += C.DT
                if still > 0.3:
                    break
            else:
                still = 0.0
            last = q
            time.sleep(C.DT)
        return self.r.grip_q

    def check(self, i, seg, hold_upper, r0, p0):
        if self.stop.is_set():
            raise Abort("stopped by user")
        if self.r.age() > C.STALE_S:
            raise Abort("no rt/lowstate for %.2f s" % self.r.age())
        r, p = self.r.tilt()
        if max(abs(r - r0), abs(p - p0)) > C.ABORT_TILT_DRIFT:
            raise Abort("tilt drift: roll %+.1f pitch %+.1f deg" % (np.degrees(r - r0), np.degrees(p - p0)))
        err = np.abs(self.r.q(C.RIGHT_ARM_IDX) - self.cmd_q)
        if i > 0 and err.max() > C.MAX_TRACK_ERR:
            raise Abort("tracking error %.2f rad on %s (blocked?)" % (err.max(), C.RIGHT_ARM[int(err.argmax())]))
        if seg == "pull" and self.holding:
            tau = np.abs(self.r.tau(C.RIGHT_ARM_IDX))
            if tau.max() > C.MAX_PULL_TAU:
                raise Abort("pull torque %.1f Nm on %s (pod did not come off)" % (tau.max(), C.RIGHT_ARM[int(tau.argmax())]))

    def upper_with_arm(self, hold_upper, q_arm):
        u = hold_upper.copy()
        u[[k for k, i in enumerate(C.UPPER_IDX) if i in C.RIGHT_ARM_IDX]] = q_arm
        return u

    def blend(self, hold_upper, q_arm, w0, w1, secs=2.0):
        n = int(secs / C.DT)
        for k in range(n + 1):
            self.r.send_arm(self.upper_with_arm(hold_upper, q_arm), w0 + (w1 - w0) * k / n)
            if self.r.grip_target is not None:
                self.r.send_grip(self.r.grip_target)
            time.sleep(C.DT)

    def run(self):
        hold_upper = self.r.q(C.UPPER_IDX)                 # waist + left arm stay exactly here
        r0, p0 = self.r.tilt()
        seg_of = np.empty(len(self.Q), dtype=object)
        for s in self.t["segments"]:
            seg_of[s["start"]:s["end"]] = s["name"]
        end = len(self.Q)
        if self.until:
            end = next(s["end"] for s in self.t["segments"] if s["name"] == self.until)
        close_at = {e["at"] for e in self.t["events"] if e["type"] == "close_gripper"}

        print("gripper: open", flush=True)
        self.r.send_grip(C.GRIP_OPEN_Q)
        self.wait_grip(C.GRIP_SETTLE_S)
        print("blend in (2 s)", flush=True)
        self.blend(hold_upper, self.Q[0], 0.0, 1.0)
        i, done = 0, 0
        try:
            next_t = time.time()
            while i < end:
                seg = seg_of[i]
                if i in close_at:
                    print("gripper: close", flush=True)
                    self.r.send_grip(C.GRIP_CLOSE_Q)
                    gq = self.wait_grip(C.GRIP_SETTLE_S)
                    if gq is None or gq < C.GRIP_EMPTY_Q:
                        raise Abort("gripper closed on nothing (q=%s)" % (None if gq is None else round(gq, 2)))
                    self.holding = True
                    print("gripper holding (q=%.2f)" % gq, flush=True)
                    next_t = time.time()
                self.check(i, seg, hold_upper, r0, p0)
                if done == 0 or seg != seg_of[i - 1]:
                    print("segment: %s" % seg, flush=True)
                if self.corr is not None:
                    if i % max(1, int(round(1.0 / (C.CORRECT_HZ * C.DT)))) == 0:
                        try:
                            d = self.corr.update(self.r.q29(), self.Q[i])
                        except Exception as e:  # closed_loop.Moved
                            raise Abort(str(e))
                        self.corr_log.append([time.time(), i] + [float(x) for x in d])
                    s0 = next(x for x in self.t["segments"] if x["name"] == seg)
                    frac = (i - s0["start"]) / float(max(1, s0["end"] - s0["start"]))
                    w = frac if seg == "to_pregrasp" else (1.0 - frac) if seg == "home" else 1.0
                    self.cmd_q = self.corr.command(self.Q[i], w)
                else:
                    self.cmd_q = self.Q[i]
                self.r.send_arm(self.upper_with_arm(hold_upper, self.cmd_q), 1.0)
                self.r.send_grip(self.r.grip_target)
                self.log.append([time.time(), i] + self.r.q(C.RIGHT_ARM_IDX).tolist())
                i += 1
                done = i
                next_t += C.DT
                time.sleep(max(0.0, next_t - time.time()))
            q_final = self.Q[end - 1]
            print("done%s" % (" (stopped after %s)" % self.until if self.until else ""), flush=True)
        except Abort as e:
            self.aborted = str(e)
            seg = seg_of[min(i, len(self.Q) - 1)]          # the segment being executed when it failed
            print("ABORT during %s: %s" % (seg, e), flush=True)
            if seg in ("retreat", "home"):
                # the pod is off the plant and in the gripper: going back would push it into the plant
                print("past the pull: stopping here, keeping the gripper closed", flush=True)
                q_final = self.Q[max(0, i - 1)]
            else:
                if self.holding or self.r.grip_target == C.GRIP_CLOSE_Q:
                    print("gripper: open (let go of the plant)", flush=True)
                    self.r.send_grip(C.GRIP_OPEN_Q)
                    self.wait_grip(C.GRIP_SETTLE_S)
                print("returning along the executed path", flush=True)
                for j in range(max(0, i - 1), -1, -1):
                    if self.r.age() > 1.0:
                        break
                    self.r.send_arm(self.upper_with_arm(hold_upper, self.Q[j]), 1.0)
                    self.r.send_grip(self.r.grip_target)
                    time.sleep(C.DT)
                q_final = self.Q[0]
        if self.until and not getattr(self, "aborted", None):
            print("returning to start after --until", flush=True)
            for j in range(end - 1, -1, -1):
                self.r.send_arm(self.upper_with_arm(hold_upper, self.Q[j]), 1.0)
                time.sleep(C.DT)
            q_final = self.Q[0]
        print("blend out (2 s)", flush=True)
        self.blend(hold_upper, q_final, 1.0, 0.0)
        return getattr(self, "aborted", None)


def preflight(robot, traj):
    t0 = time.time()
    while robot.ls is None:
        if time.time() - t0 > 5:
            return "no rt/lowstate"
        time.sleep(0.05)
    fsm = fsm_state()
    if fsm not in MAIN_FSM:
        return "robot not in a balance state (FSM %s, need %s)" % (fsm, MAIN_FSM)
    r, p = robot.tilt()
    if max(abs(r), abs(p)) > C.MAX_TILT:
        return "not upright: roll %.1f pitch %.1f deg" % (np.degrees(r), np.degrees(p))
    if traj is not None:
        diff = np.abs(robot.q(C.RIGHT_ARM_IDX) - np.array(traj["start_q"])).max()
        if diff > C.MAX_START_DIFF:
            return "plan is stale: arm moved %.2f rad since planning (max %.2f)" % (diff, C.MAX_START_DIFF)
        tq = traj.get("target", {}).get("q")
        if tq:
            wd = np.abs(robot.q(C.WAIST_IDX) - np.array(tq[12:15])).max()
            if wd > C.MAX_WAIST_DIFF:
                return "waist moved %.2f rad since perception (max %.2f): look again" % (wd, C.MAX_WAIST_DIFF)
        age = time.time() - traj.get("planned_at", 0)
        if age > 300:
            return "plan is %.0f s old (max 300)" % age
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trajectory", nargs="?")
    ap.add_argument("--dry", action="store_true", help="run every check, send nothing")
    ap.add_argument("--until", choices=["to_pregrasp", "approach"], help="stop after this segment and go back")
    ap.add_argument("--gripper", choices=["open", "close"])
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--open-loop", action="store_true", help="disable the body-motion correction")
    args = ap.parse_args()

    robot = Robot(args.iface, args.dry)
    traj = json.load(open(args.trajectory)) if args.trajectory else None
    if args.gripper:
        time.sleep(1.0)
        robot.send_grip(C.GRIP_OPEN_Q if args.gripper == "open" else C.GRIP_CLOSE_Q)
        q = Player(robot, {"q": [[0] * 7], "segments": [], "events": []}, None).wait_grip(C.GRIP_SETTLE_S)
        print("gripper %s -> q=%s" % (args.gripper, None if q is None else round(q, 3)))
        return
    if traj is None:
        sys.exit("need a trajectory.json (or --gripper)")
    why = preflight(robot, traj)
    if why:
        print("REFUSED: %s" % why)
        sys.exit(1)
    print("preflight ok: %d samples, %.1f s%s" % (len(traj["q"]), traj["duration_s"], " (DRY RUN)" if args.dry else ""),
          flush=True)
    if args.dry:
        return
    player = Player(robot, traj, args.until, closed_loop=not args.open_loop)
    signal.signal(signal.SIGINT, lambda *_: player.stop.set())
    aborted = player.run()
    log = os.path.splitext(args.trajectory)[0] + "_executed.json"
    json.dump({"aborted": aborted, "log": player.log, "corrections": player.corr_log}, open(log, "w"))
    print("result: %s  (log %s)" % ("ABORTED: " + aborted if aborted else "OK", log))
    sys.exit(1 if aborted else 0)


if __name__ == "__main__":
    main()
