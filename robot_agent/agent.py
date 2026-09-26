#!/usr/bin/env python3
"""
G1 robot agent: one persistent process ON THE ROBOT (Orin) that owns the DDS connection and executes
commands sent from the VM over TCP. Replaces "ssh + start python + DDS discovery" per command (8-10 s)
with a single open connection (~ms), and keeps all real-time loops on the robot.

    PYTHONPATH=~/g1_rec/pylib python3 -u agent.py [--host 192.168.123.164] [--port 7777]
    python3 agent.py --sim --host 127.0.0.1          # simulated robot, for testing without hardware

Protocol: newline-delimited JSON over TCP.
  request   {"id": 1, "cmd": "step", "args": {"vy": 0.1, "dur": 1.5}}
  progress  {"id": 1, "log": "max tilt 1.2 deg"}            (zero or more)
  reply     {"id": 1, "ok": true, "result": {...}}  or  {"id": 1, "ok": false, "error": "..."}
  stream    {"state": {...}}                                  (after cmd "watch", until disconnect)

Commands: ping, status, watch, stop, ai, damp, ready, start, zero, step, head, gripper, say, led, anchor.
Status includes the body pose (okra_pick/robot/body_pose.py): pelvis shift relative to the feet since the
last anchor, and whether both feet still agree (a disagreement = a foot moved).
One motion command runs at a time (others get "busy"); "stop" is always accepted: it sends Ctrl-C (SIGINT) to a
running okra arm_player.py (it aborts: opens the gripper if needed, plays its path back), sends zero velocity
and cancels the running task (which then centres/releases the waist or finishes its own safe exit).
"""

import argparse
import json
import math
import os
import socket
import socketserver
import subprocess
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# ---- limits (same values as g1_connect/robot_mode.py and okra_pick/robot/pick_config.py)
MAX_TILT = 0.05            # rad, balance gate for start / step
MIN_KNEE_LOAD = 15.0       # Nm
MAX_STEP_V, MAX_STEP_DUR = 0.2, 2.0
STEP_ABORT_TILT = 0.14     # rad
HEAD_START_TILT, HEAD_ABORT_DRIFT = 0.10, 0.08
BALANCE_FSM = (200, 500, 501)
FSM = {"zero": 0, "damp": 1, "ready": 4, "start": 200}
FSM_NAMES = {0: "zero torque", 1: "damp", 2: "squat", 3: "sit", 4: "locked stand", 200: "AI balance",
             500: "main control", 501: "main control", 702: "lie->stand", 706: "squat<->stand"}
GRIP_OPEN_Q, GRIP_CLOSE_Q, GRIP_KP, GRIP_KD = 5.2, 0.0, 5.0, 0.05
SPEAKER_ID = 0             # G1 TtsMaker voice id (CALIBRATE: check which id speaks English)
L_KNEE, R_KNEE = 3, 9


# ======================================================================= robot interfaces


# the okra pick player (okra_pick/robot/arm_player.py) runs as its own process; SIGINT = its clean abort.
# Anchored on the interpreter so the "bash -c ... arm_player.py" wrapper is not hit.
ARM_PLAYER_PATTERN = r"^[^ ]*python[0-9.]* [^ ]*arm_player\.py"


def interrupt_arm_player():
    """SIGINT every running arm_player.py. Returns the pids signalled."""
    try:
        pids = subprocess.run(["pgrep", "-f", ARM_PLAYER_PATTERN], capture_output=True, text=True,
                              timeout=2).stdout.split()
        if pids:
            subprocess.run(["kill", "-INT"] + pids, timeout=2)
        return [int(p) for p in pids]
    except Exception as e:  # noqa: BLE001 - stop must still zero the velocity
        return "error: %s" % e

class RealRobot:
    def __init__(self, iface):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_, MotorCmds_, MotorStates_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        ChannelFactoryInitialize(0, iface)
        self.ls, self.ls_t, self.ls_n, self.grip_q = None, 0.0, 0, None
        ChannelSubscriber("rt/lowstate", LowState_).Init(self._on_ls, 10)
        ChannelSubscriber("rt/dex1/right/state", MotorStates_).Init(self._on_grip, 10)
        self.loco = LocoClient()
        self.loco.SetTimeout(3.0)
        self.loco.Init()
        self.ms = MotionSwitcherClient()
        self.ms.SetTimeout(3.0)
        self.ms.Init()
        self.grip_pub = ChannelPublisher("rt/dex1/right/cmd", MotorCmds_)
        self.grip_pub.Init()
        self._MotorCmds, self._MotorCmd = MotorCmds_, MotorCmd_

    def _on_ls(self, m):
        self.ls, self.ls_t = m, time.time()
        self.ls_n += 1

    def _on_grip(self, m):
        if len(m.states):
            self.grip_q = m.states[0].q

    # --- state
    def fresh(self):
        return self.ls is not None and time.time() - self.ls_t < 0.5

    def rpy(self):
        return list(self.ls.imu_state.rpy)

    def q(self, i):
        return self.ls.motor_state[i].q

    def q29(self):
        return [self.ls.motor_state[i].q for i in range(29)]

    def tau(self, i):
        return self.ls.motor_state[i].tau_est

    # --- loco / switcher (callers hold the agent's loco lock)
    def fsm(self):
        for _ in range(3):
            code, data = self.loco._Call(7001, "{}")
            if code == 0:
                return json.loads(data)["data"]
            time.sleep(0.2)
        return None

    def set_fsm(self, fsm_id):
        return self.loco.SetFsmId(fsm_id)

    def velocity(self, vx, vy, wz, dur):
        return self.loco.SetVelocity(vx, vy, wz, dur)

    def mode(self):
        code, d = self.ms.CheckMode()
        return (d or {}).get("name") if code == 0 else None

    def select_ai(self):
        return self.ms.SelectMode("ai")[0]

    def grip_cmd(self, q):
        self.grip_pub.Write(self._MotorCmds([self._MotorCmd(mode=1, q=float(q), dq=0.0, tau=0.0,
                                                            kp=GRIP_KP, kd=GRIP_KD, reserve=[0, 0, 0])]))

    def waist_controller(self, speed):
        from waist import WaistController
        return WaistController(speed=speed)

    def _audio(self):
        if not hasattr(self, "_audio_client"):
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
            self._audio_client = AudioClient()
            self._audio_client.SetTimeout(3.0)
            self._audio_client.Init()
        return self._audio_client

    def say(self, text):
        return self._audio().TtsMaker(text, SPEAKER_ID)

    def led(self, r, g, b):
        return self._audio().LedControl(int(r), int(g), int(b))


class SimRobot:
    """Minimal stand-in for tests without hardware: tilt noise, FSM transitions, gripper motion."""

    class _W:
        def __init__(self):
            self.goal_ = [0.0, 0.0]
            self.released = threading.Event()
            self.engaged = threading.Event()

        def engage(self):
            self.engaged.set()
            return True

        def nudge(self, dy=0.0, dp=0.0):
            self.goal_ = [self.goal_[0] + dy, self.goal_[1] + dp]

        def center(self):
            self.goal_ = [0.0, 0.0]

        def goal(self):
            return tuple(self.goal_)

        def release(self, timeout=8.0):
            self.center()
            time.sleep(0.2)
            self.released.set()

    def __init__(self):
        self._fsm, self._mode, self.grip_q, self.ls_n, self.ls_t = 200, "ai", 5.3, 0, time.time()
        self.tilt_bump = 0.0
        threading.Thread(target=self._tick, daemon=True).start()

    def _tick(self):
        while True:
            self.ls_n += 1
            self.ls_t = time.time()
            time.sleep(0.001)

    def fresh(self):
        return True

    def rpy(self):
        return [0.004, 0.012 + self.tilt_bump, 0.3]

    def q(self, i):
        return 0.6 if i in (L_KNEE, R_KNEE) else 0.0

    def q29(self):
        q = [0.0] * 29
        q[0] = q[6] = -0.33
        q[3] = q[9] = 0.72
        q[4] = q[10] = -0.36
        return q

    def tau(self, i):
        return -16.0 if i in (L_KNEE, R_KNEE) else 0.5

    def fsm(self):
        time.sleep(0.005)
        return self._fsm

    def set_fsm(self, f):
        self._fsm = f
        return 0

    def velocity(self, vx, vy, wz, dur):
        return 0

    def mode(self):
        return self._mode

    def select_ai(self):
        self._mode = "ai"
        return 0

    def grip_cmd(self, q):
        self.grip_q += max(-0.2, min(0.2, max(q, 1.0) - self.grip_q))

    def waist_controller(self, speed):
        return SimRobot._W()

    def say(self, text):
        self.last_said = text
        return 0

    def led(self, r, g, b):
        self.last_led = (r, g, b)
        return 0


# ======================================================================= agent

class Cancelled(Exception):
    pass


class Agent:
    def __init__(self, robot, log_path):
        self.r = robot
        self.loco_lock = threading.Lock()          # LocoClient / switcher calls are not thread-safe
        self.motion_lock = threading.Lock()        # one motion task at a time
        self.cancel = threading.Event()
        self.busy = None
        self.fsm_cache, self.mode_cache = None, None
        self.rate, self._last_n, self._last_t = 0.0, 0, time.time()
        self.log_f = open(log_path, "a")
        self.bp, self.anchor_t = None, None
        try:
            from body_pose import BodyPose
            from g1_chain import Chain
            self.bp = BodyPose(Chain(os.path.join(HERE, "g1.urdf")))
        except Exception as e:  # noqa: BLE001 - body pose is optional
            self.log("body pose unavailable: %s" % e)
        threading.Thread(target=self._poll, daemon=True).start()

    def log(self, msg):
        self.log_f.write("%s %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg))
        self.log_f.flush()

    def _poll(self):
        """Refresh FSM/mode every second and the lowstate rate, so status is instant."""
        while True:
            try:
                with self.loco_lock:
                    self.fsm_cache = self.r.fsm()
                    self.mode_cache = self.r.mode()
            except Exception:
                pass
            n, t = self.r.ls_n, time.time()
            self.rate = (n - self._last_n) / max(1e-3, t - self._last_t)
            self._last_n, self._last_t = n, t
            time.sleep(1.0)

    # ------------------------------------------------------------- state
    def balance(self, need_load=True):
        if not self.r.fresh():
            return False, "no fresh rt/lowstate"
        roll, pitch, _ = self.r.rpy()
        load = abs(self.r.tau(L_KNEE)) + abs(self.r.tau(R_KNEE))
        msg = "roll %+.1f pitch %+.1f deg, knee load %.1f Nm" % (math.degrees(roll), math.degrees(pitch), load)
        if max(abs(roll), abs(pitch)) > MAX_TILT:
            return False, "NOT UPRIGHT: " + msg
        if need_load and load < MIN_KNEE_LOAD:
            return False, "LEGS NOT LOADED: " + msg
        return True, msg

    def state(self):
        s = {"t": round(time.time(), 3), "fsm": self.fsm_cache, "fsm_name": FSM_NAMES.get(self.fsm_cache, "?"),
             "mode": self.mode_cache, "lowstate_hz": round(self.rate), "busy": self.busy,
             "gripper_q": None if self.r.grip_q is None else round(self.r.grip_q, 3)}
        if self.r.fresh():
            roll, pitch, yaw = self.r.rpy()
            s.update(roll_deg=round(math.degrees(roll), 2), pitch_deg=round(math.degrees(pitch), 2),
                     yaw_deg=round(math.degrees(yaw), 1),
                     knee_load=round(abs(self.r.tau(L_KNEE)) + abs(self.r.tau(R_KNEE)), 1),
                     waist=[round(self.r.q(i), 3) for i in (12, 13, 14)])
        s["balance_ok"], s["balance"] = self.balance()
        if self.bp is not None and self.r.fresh():
            q29, rpy = self.r.q29(), self.r.rpy()
            if self.bp.anchor_q is None:
                self.bp.anchor(q29, rpy)
                self.anchor_t = time.time()
            b = self.bp.status(q29, rpy)
            sh = b["pelvis_shift_m"]
            s["body"] = {"shift_mm": round(1000 * math.sqrt(sum(x * x for x in sh)), 1),
                         "shift_xyz_mm": [round(1000 * x, 1) for x in sh], "rot_deg": round(b["pelvis_rot_deg"], 2),
                         "feet_ok": b["feet_ok"], "feet_disagree_mm": round(1000 * b["feet_disagree_m"], 1),
                         "anchored_s_ago": round(time.time() - self.anchor_t, 1)}
        return s

    def reanchor(self):
        if self.bp is not None and self.r.fresh():
            self.bp.anchor(self.r.q29(), self.r.rpy())
            self.anchor_t = time.time()
            return True
        return False

    # ------------------------------------------------------------- helpers
    def _sleep(self, secs):
        if self.cancel.wait(secs):
            raise Cancelled()

    def _motion(self, name, fn, emit):
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError("busy with %s (send stop first)" % self.busy)
        self.busy, t0 = name, time.time()
        self.cancel.clear()
        try:
            return fn(emit)
        except Cancelled:
            emit("cancelled by stop")
            return {"cancelled": True}
        finally:
            self.busy = None
            self.motion_lock.release()
            self.log("%s finished in %.1f s" % (name, time.time() - t0))

    # ------------------------------------------------------------- commands
    def cmd_ping(self, a, emit):
        return {"pong": time.time()}

    def cmd_status(self, a, emit):
        return self.state()

    def cmd_stop(self, a, emit):
        task = self.busy
        self.cancel.set()
        players = interrupt_arm_player()
        replies = []
        for _ in range(3):
            with self.loco_lock:
                replies.append(self.r.velocity(0.0, 0.0, 0.0, 1.0))
            time.sleep(0.05)
        return {"velocity_zero_replies": replies, "cancelled_task": task, "arm_player_interrupted": players}

    def cmd_ai(self, a, emit):
        def run(emit):
            with self.loco_lock:
                code = self.r.select_ai()
            emit("SelectMode('ai') reply %s, waiting 5 s for the motion service" % code)
            self._sleep(5.0)
            with self.loco_lock:
                return {"mode": self.r.mode(), "fsm": self.r.fsm()}
        return self._motion("ai", run, emit)

    def _fsm_cmd(self, name, a, emit):
        target = FSM[name]

        def run(emit):
            if name == "zero" and not a.get("confirm"):
                raise RuntimeError("zero torque makes the robot limp: resend with confirm=true (robot supported!)")
            if name == "start":
                ok, why = self.balance()
                emit("balance check: " + why)
                if not ok:
                    raise RuntimeError("REFUSED: " + why)
            with self.loco_lock:
                code = self.r.set_fsm(target)
            emit("SetFsmId(%d) %s -> reply %s" % (target, FSM_NAMES[target], code))
            worst, t0 = 0.0, time.time()
            while time.time() - t0 < (5.0 if name == "start" else 3.0):
                if self.r.fresh():
                    worst = max(worst, max(abs(x) for x in self.r.rpy()[:2]))
                self._sleep(0.05)
            with self.loco_lock:
                fsm = self.r.fsm()
            return {"fsm": fsm, "fsm_name": FSM_NAMES.get(fsm, "?"), "max_tilt_deg": round(math.degrees(worst), 1),
                    "reached": fsm == target}
        return self._motion(name, run, emit)

    def cmd_damp(self, a, emit):
        return self._fsm_cmd("damp", a, emit)

    def cmd_ready(self, a, emit):
        return self._fsm_cmd("ready", a, emit)

    def cmd_start(self, a, emit):
        return self._fsm_cmd("start", a, emit)

    def cmd_zero(self, a, emit):
        return self._fsm_cmd("zero", a, emit)

    def cmd_step(self, a, emit):
        vx, vy, wz = float(a.get("vx", 0)), float(a.get("vy", 0)), float(a.get("wz", 0))
        dur = float(a.get("dur", 1.0))

        def run(emit):
            if self.fsm_cache not in BALANCE_FSM:
                raise RuntimeError("REFUSED: FSM %s is not a balance state %s" % (self.fsm_cache, BALANCE_FSM))
            if max(abs(vx), abs(vy)) > MAX_STEP_V or abs(wz) > 0.5 or not 0 < dur <= MAX_STEP_DUR:
                raise RuntimeError("REFUSED: limits |v| <= %.1f m/s, |wz| <= 0.5 rad/s, 0 < dur <= %.0f s"
                                   % (MAX_STEP_V, MAX_STEP_DUR))
            ok, why = self.balance()
            emit("balance check: " + why)
            if not ok:
                raise RuntimeError("REFUSED: " + why)
            worst, aborted = 0.0, None
            try:
                with self.loco_lock:
                    code = self.r.velocity(vx, vy, wz, dur)
                emit("SetVelocity(vx=%.2f vy=%.2f wz=%.2f, %.1f s) reply %s" % (vx, vy, wz, dur, code))
                t0 = time.time()
                while time.time() - t0 < dur + 1.0:
                    tilt = max(abs(x) for x in self.r.rpy()[:2])
                    worst = max(worst, tilt)
                    if tilt > STEP_ABORT_TILT:
                        aborted = "tilt %.1f deg" % math.degrees(tilt)
                        break
                    self._sleep(0.005)
            finally:
                with self.loco_lock:
                    self.r.velocity(0.0, 0.0, 0.0, 1.0)
                emit("stop sent")
                time.sleep(0.5)
                if self.reanchor():                            # the feet moved on purpose: new stance
                    emit("body pose re-anchored at the new stance")
            return {"max_tilt_deg": round(math.degrees(worst), 1), "aborted": aborted,
                    "distance_cm_est": round(100 * dur * math.hypot(vx, vy))}
        return self._motion("step", run, emit)

    def cmd_head(self, a, emit):
        yaw, pitch = float(a.get("yaw", 0)), float(a.get("pitch", 0))
        hold, speed = float(a.get("hold", 3.0)), float(a.get("speed", 0.3))

        def run(emit):
            r0, p0, _ = self.r.rpy()
            if max(abs(r0), abs(p0)) > HEAD_START_TILT:
                raise RuntimeError("REFUSED: not upright (roll %.1f pitch %.1f deg)" % (math.degrees(r0), math.degrees(p0)))
            w = self.r.waist_controller(speed)
            t_wait = time.time()
            while hasattr(w, "ready") and not w.ready():
                if time.time() - t_wait > 3:
                    raise RuntimeError("waist controller got no rt/lowstate")
                time.sleep(0.02)
            w.engage()
            emit("engaged (2 s blend in), moving to yaw %+.2f pitch %+.2f rad from the start pose" % (yaw, pitch))
            aborted = None
            try:
                t_end = time.time() + 2.0
                w.nudge(yaw, pitch)
                t_end += (abs(yaw) + abs(pitch)) / max(0.05, speed) + hold
                while time.time() < t_end:
                    r, p, _ = self.r.rpy()
                    if max(abs(r - r0), abs(p - p0)) > HEAD_ABORT_DRIFT:
                        aborted = "tilt drift roll %+.1f pitch %+.1f deg" % (math.degrees(r - r0), math.degrees(p - p0))
                        emit("BALANCE ABORT: " + aborted)
                        break
                    if not self.r.fresh():
                        aborted = "rt/lowstate stale"
                        break
                    self._sleep(0.01)
            finally:
                emit("centering and releasing")
                w.release()
            return {"released": w.released.is_set(), "aborted": aborted}
        return self._motion("head", run, emit)

    def cmd_gripper(self, a, emit):
        pos = a.get("pos", "open")
        q = GRIP_OPEN_Q if pos == "open" else GRIP_CLOSE_Q if pos == "close" else float(pos)

        def run(emit):
            last, still, t0 = None, 0.0, time.time()
            while time.time() - t0 < 3.0:
                self.r.grip_cmd(q)
                g = self.r.grip_q
                if g is not None and last is not None and abs(g - last) < 0.005:
                    still += 0.02
                    if still > 0.3:
                        break
                else:
                    still = 0.0
                last = g
                self._sleep(0.02)
            return {"target_q": q, "gripper_q": None if self.r.grip_q is None else round(self.r.grip_q, 3)}
        return self._motion("gripper", run, emit)


def _cmd_say(self, a, emit):
    """Speak through the G1 speaker (not a motion: allowed while busy)."""
    return {"reply": self.r.say(str(a.get("text", ""))[:200])}


def _cmd_led(self, a, emit):
    rgb = a.get("rgb", [0, 0, 0])
    return {"reply": self.r.led(*rgb)}


def _cmd_anchor(self, a, emit):
    """Take the current stance as the new reference for the body pose (after the robot moved on purpose)."""
    return {"anchored": self.reanchor()}


Agent.cmd_say, Agent.cmd_led, Agent.cmd_anchor = _cmd_say, _cmd_led, _cmd_anchor


# ======================================================================= TCP server

def make_handler(agent):
    class Handler(socketserver.StreamRequestHandler):
        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def handle(self):
            peer = "%s:%s" % self.client_address
            wlock = threading.Lock()

            def send(obj):
                with wlock:
                    self.wfile.write((json.dumps(obj) + "\n").encode())
                    self.wfile.flush()

            for line in self.rfile:
                try:
                    req = json.loads(line.decode())
                except ValueError:
                    send({"ok": False, "error": "bad json"})
                    continue
                rid, cmd, args = req.get("id"), req.get("cmd", ""), req.get("args") or {}
                if cmd == "watch":
                    hz = min(50.0, float(args.get("hz", 10)))
                    try:
                        while True:
                            send({"state": agent.state()})
                            time.sleep(1.0 / hz)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
                fn = getattr(agent, "cmd_" + cmd, None)
                agent.log("%s %s %s" % (peer, cmd, json.dumps(args)))
                if fn is None:
                    send({"id": rid, "ok": False, "error": "unknown command %r" % cmd})
                    continue
                try:
                    res = fn(args, lambda m: send({"id": rid, "log": m}))
                    send({"id": rid, "ok": True, "result": res})
                except Exception as e:  # noqa: BLE001 - every error goes back to the client
                    agent.log("error in %s: %s" % (cmd, traceback.format_exc(limit=2)))
                    send({"id": rid, "ok": False, "error": str(e)})
    return Handler


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.123.164", help="bind address (robot network only)")
    ap.add_argument("--port", type=int, default=7777)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--sim", action="store_true", help="simulated robot (tests)")
    ap.add_argument("--log", default=os.path.join(HERE, "agent_commands.log"))
    args = ap.parse_args()
    robot = SimRobot() if args.sim else RealRobot(args.iface)
    agent = Agent(robot, args.log)
    time.sleep(1.5)                                   # DDS discovery once, at start-up
    srv = Server((args.host, args.port), make_handler(agent))
    agent.log("agent started on %s:%d (%s)" % (args.host, args.port, "SIM" if args.sim else "robot"))
    print("agent listening on %s:%d%s" % (args.host, args.port, " (SIM)" if args.sim else ""), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
