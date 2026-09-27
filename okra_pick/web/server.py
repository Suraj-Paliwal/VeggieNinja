#!/usr/bin/env python3
"""
Operator web interface for the G1 okra pick (human-in-the-loop). Runs on the VM, standard library only
(plus the okra_pick/hitl modules). Open http://localhost:8080 in a browser.

    ../../dimos/.venv/bin/python server.py                        # this VM only
    ../../dimos/.venv/bin/python server.py --host 0.0.0.0 --token SECRET   # other devices on the network:
                                                                  # open http://<vm-ip>:8080/?token=SECRET

It talks to robot_agent (live state, STOP, bring-up, steps, waist, gripper, voice/LED) and runs the
pipeline steps (look / plan / reach / pick / record / dataset) as background jobs with a live log.
Live camera: /api/live.mjpg relays the robot's okra_pick/robot/live_cam.py (OKRA_LIVE, default <robot>:8091);
the JPEGs are made on the robot, the VM only forwards bytes.
"""

import argparse
import json
import mimetypes
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
OKRA = os.path.dirname(HERE)
J = os.path.dirname(OKRA)
sys.path.insert(0, os.path.join(OKRA, "hitl"))
import confirm  # noqa: E402
import store  # noqa: E402

AGENT = os.environ.get("G1_AGENT", "192.168.123.164:7777")
PY_VM = os.path.join(J, "dimos", ".venv", "bin", "python")
PICK_SH = os.environ.get("OKRA_PICK_SH") or os.path.join(OKRA, "okra_pick.sh")   # override: offline tests/demo only
ROBOT_CMDS = {"stop", "status", "ai", "damp", "ready", "start", "step", "head", "gripper", "say", "led", "ping", "anchor"}
TOKEN = None


# ------------------------------------------------------------------ robot agent

def agent_call(cmd, args=None, timeout=15.0):
    host, port = AGENT.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=1.5)
    s.settimeout(timeout)
    s.sendall((json.dumps({"id": 1, "cmd": cmd, "args": args or {}}) + "\n").encode())
    logs = []
    for line in s.makefile("r"):
        m = json.loads(line)
        if "log" in m:
            logs.append(m["log"])
            continue
        s.close()
        return {"ok": m.get("ok"), "result": m.get("result"), "error": m.get("error"), "log": logs}
    return {"ok": False, "error": "no reply", "log": logs}


ROBOT_SSH = "unitree@" + AGENT.rsplit(":", 1)[0]
# SIGINT = arm_player's clean abort; anchored on the interpreter so the "bash -c" wrapper is not hit
KILL_PLAYER = r"pkill -INT -e -f '^[^ ]*python[0-9.]* [^ ]*arm_player\.py' || echo 'no arm_player running'"


def stop_all():
    """STOP: interrupt the arm player over ssh directly (works without the agent) and, in parallel,
    agent 'stop' (zero velocity, cancel its task, and it also interrupts the player)."""
    out = {}

    def direct():
        try:
            p = subprocess.run(["ssh", "-o", "ConnectTimeout=3", "-o", "BatchMode=yes", ROBOT_SSH, KILL_PLAYER],
                               capture_output=True, text=True, timeout=6)
            out["direct"] = (p.stdout + p.stderr).strip() or "rc %d" % p.returncode
            out["direct_ok"] = p.returncode == 0
        except Exception as e:  # noqa: BLE001
            out["direct"], out["direct_ok"] = "ssh failed: %s" % e, False

    t = threading.Thread(target=direct)
    t.start()
    try:
        r = agent_call("stop", timeout=10.0)
    except Exception as e:  # noqa: BLE001 - agent may be offline; the direct path still counts
        r = {"ok": False, "error": "agent offline (%s)" % e.__class__.__name__, "log": []}
    t.join()
    r["log"] = r.get("log", []) + ["arm player (ssh): " + out["direct"]]
    if not r.get("ok") and out["direct_ok"]:
        r["ok"], r["result"] = True, {"arm_player": out["direct"], "agent": r.pop("error", None)}
    return r


class StatePoller(threading.Thread):
    """Robot state at ~4 Hz (cheap TCP call) and recording status every 5 s (ssh)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.robot, self.robot_err, self.recording, self.t_rec = None, "starting", None, 0

    def run(self):
        while True:
            try:
                r = agent_call("status", timeout=2.0)
                self.robot, self.robot_err = (r["result"], None) if r["ok"] else (None, r["error"])
            except OSError as e:
                self.robot, self.robot_err = None, "agent offline (%s)" % e.__class__.__name__
            if time.time() - self.t_rec > 5:
                self.t_rec = time.time()
                try:
                    out = subprocess.run([os.path.join(J, "g1_record", "rec.sh"), "status"], capture_output=True,
                                         text=True, timeout=8).stdout
                    self.recording = out.splitlines()[0] if out else "unknown"
                except (subprocess.SubprocessError, OSError):
                    self.recording = "unknown (robot unreachable)"
            time.sleep(0.25)


# ------------------------------------------------------------------ live camera relay

LIVE = os.environ.get("OKRA_LIVE", AGENT.rsplit(":", 1)[0] + ":8091")   # robot/live_cam.py


class LiveRelay:
    """One upstream MJPEG connection to the robot's live_cam.py, shared by every browser. It only runs while
    someone watches, keeps just the newest JPEG (a slow viewer skips frames instead of lagging), and never
    decodes anything: the VM only moves bytes."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.t_arrive, self.robot_t, self.source = None, 0, 0.0, None, None
        self.clients, self.fps, self.error, self.running = 0, 0.0, "not started", False

    def want(self):
        with self.cond:
            self.clients += 1
            if not self.running:
                self.running = True
                threading.Thread(target=self._run, daemon=True).start()

    def unwant(self):
        with self.cond:
            self.clients -= 1

    def _run(self):
        idle_since = None
        while True:
            with self.cond:
                if self.clients <= 0:
                    idle_since = idle_since or time.time()
                    if time.time() - idle_since > 5:
                        self.running = False
                        return
                else:
                    idle_since = None
            if idle_since:                               # nobody watching: wait, maybe someone comes back
                time.sleep(0.5)
                continue
            try:
                self._stream()
            except Exception as e:  # noqa: BLE001 - robot off, live_cam not started, link drop
                self.error = "%s: %s" % (e.__class__.__name__, e)
            time.sleep(1.0)

    def _stream(self):
        host, port = LIVE.rsplit(":", 1)
        s = socket.create_connection((host, int(port)), timeout=3)
        s.settimeout(5)                                  # no frame for 5 s = dead link, reconnect
        f = s.makefile("rb")
        try:
            s.sendall(b"GET /live.mjpg HTTP/1.0\r\nHost: robot\r\n\r\n")
            status = f.readline()
            if b" 200 " not in status:
                raise IOError("live_cam answered %r" % status.strip())
            while f.readline().strip():                  # response headers
                pass
            self.error, n, t0 = None, 0, time.time()
            while self.clients > 0:
                line = f.readline()
                if not line:
                    raise IOError("stream closed by the robot")
                if not line.startswith(b"--frame"):
                    continue
                hdr = {}
                for h in iter(f.readline, b"\r\n"):
                    if not h:
                        raise IOError("stream closed by the robot")
                    k, _, v = h.decode(errors="replace").partition(":")
                    hdr[k.strip().lower()] = v.strip()
                jpg = f.read(int(hdr["content-length"]))
                with self.cond:
                    self.jpeg, self.t_arrive = jpg, time.time()
                    self.robot_t, self.source = float(hdr.get("x-robot-time", 0)) or None, hdr.get("x-source")
                    self.seq += 1
                    self.cond.notify_all()
                n += 1
                if time.time() - t0 >= 2:
                    self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
        finally:
            s.close()

    def state(self):
        age = round(time.time() - self.t_arrive, 1) if self.t_arrive else None
        return {"url": LIVE, "watching": self.clients, "connected": self.running and self.error is None and age is not None
                and age < 2, "fps": round(self.fps, 1), "age_s": age, "source": self.source, "error": self.error}


def live_get(path, timeout=3.0):
    """Small JSON call to live_cam.py (status / control)."""
    return json.loads(live_get_bytes(path, timeout))


# ------------------------------------------------------------------ auto-look

class AutoLook(threading.Thread):
    """While armed: watch the live feed's detector. When a filter-accepted okra with conf >= min_conf is in
    `need` of the last 10 frames, keep a snapshot, run the normal Look step (15 frames + depth -> 3D candidates)
    and ALWAYS ask the operator. One okra at a time: after a YES the chain (after_yes) does the work for THAT okra
    (plan -> reach or pick, each behind okra_pick.sh's live safety gate), then waits for the outcome answer before
    it looks for the next one. The arm only moves when the YES itself was confirmed for motion (the page asks),
    and STOP cancels the chain and disarms auto-look."""

    def __init__(self, app):
        super().__init__(daemon=True)
        self.app, self.armed, self.min_conf, self.need, self.after_yes = app, False, 0.5, 6, "pick"
        self.cooldown_s, self.t_last, self.status, self.okra = 20.0, 0.0, "off", None
        self.chain = None                                 # {"event", "kind", "step", "cancelled"} for the current okra

    def settings(self):
        return {"armed": self.armed, "min_conf": self.min_conf, "need": self.need, "after_yes": self.after_yes,
                "status": self.status, "okra": self.okra, "chain": self.chain}

    # ------------------------------------------------ work for one confirmed okra
    def start_chain(self, ev, kind):
        """kind: plan | reach | pick. Plan first; motion only if the plan succeeded and nobody pressed STOP."""
        app, sh, eid = self.app, PICK_SH, os.path.basename(ev)
        ch = self.chain = {"event": eid, "kind": kind, "step": "plan", "cancelled": False}

        def after_plan(rc):
            if ch["cancelled"]:
                ch["step"] = "cancelled (STOP)"
            elif rc != 0:
                ch["step"] = "plan failed (see log): fix it or skip this okra"
            elif kind == "plan":
                ch["step"] = "planned: reach/pick by hand, or skip this okra"
            else:
                ch["step"] = kind
                app.jobs.start(kind, [sh, kind, eid, "--yes"], env={"OKRA_NONINTERACTIVE": "1"}, then=after_move,
                               keep_log=True)

        def after_move(rc):
            ch["step"] = ("cancelled (STOP)" if ch["cancelled"] else
                          "%s done: answer 'did it pick?'" % kind if rc == 0 else "%s ended with exit %s: answer the outcome" % (kind, rc))
        app.start_plan(ev, then=after_plan, keep_log=True)

    def cancel(self):
        """STOP: no further chain step starts, auto-look is disarmed (re-arm by hand)."""
        if self.chain:
            self.chain["cancelled"] = True
        self.armed = False

    def blocked(self):
        """Why a trigger must wait now (None = free). Measured live every time, nothing assumed."""
        app = self.app
        if app.jobs.running:
            return "busy: %s" % app.jobs.name
        ev = app.event_view()
        if ev and ev["pending"]:
            return "waiting for your answer"
        if ev and ev["target"] and not ev["outcome"]:
            return "okra of %s: finish it (answer 'did it pick?' or skip it) before the next one" % ev["id"]
        r = app.poll.robot
        if r and r.get("busy"):
            return "robot busy: %s" % r["busy"]
        if (app.poll.recording or "").startswith("RECORDING"):
            return "recording"
        left = self.cooldown_s - (time.time() - self.t_last)
        if left > 0:
            return "cooldown %.0f s" % left
        return None

    def run(self):
        while True:
            time.sleep(0.3)
            if not self.armed:
                self.status, self.okra = "off", None
                continue
            try:
                st = live_get("/status?min_conf=%.2f" % self.min_conf)
            except Exception as e:  # noqa: BLE001
                self.status, self.okra = "no live feed (%s)" % e.__class__.__name__, None
                continue
            self.okra = st.get("okra")
            if not self.okra or not self.okra["detecting"]:
                self.status = "live feed is not detecting (source %s, outlines %s)" % (st.get("source"),
                                                                                      "on" if st.get("overlay") else "off")
                continue
            why = self.blocked()
            if why:
                self.status = "paused: " + why
                continue
            if self.okra["hits"] < self.need:
                self.status = "watching (best %.2f, %d/%d frames)" % (self.okra["best_conf"], self.okra["hits"], self.need)
                continue
            self.fire()

    def fire(self):
        app, okra = self.app, dict(self.okra)
        self.t_last, self.status = time.time(), "okra seen (conf %.2f): looking" % okra["best_conf"]
        shots = {}
        for name, path in (("trigger.jpg", "/raw.jpg"), ("trigger_live.jpg", "/snap.jpg")):
            try:
                shots[name] = live_get_bytes(path)
            except Exception:  # noqa: BLE001 - the snapshot is evidence only; the look still runs
                pass

        def done(rc, log):
            d = next((l.split("=", 1)[1] for l in log if l.startswith("EVENT_DIR=")), None)
            if rc == 0 and d:
                for name, data in shots.items():
                    open(os.path.join(d, name), "wb").write(data)
                store.save(d, "trigger.json", {"source": "auto-look", "okra": okra, "time": time.time()})
                app.event = d
                dec = confirm.prepare(d, "always")                  # auto-look always asks the human
                log.append("auto-look: question %s (%s)" % (dec["action"].upper(), "; ".join(dec["reasons"])))
            self.t_last = time.time()                                # cooldown counts from the end of the look
        try:
            app.jobs.start("auto-look", look_argv(), on_done=done)
            app.jobs.log.append("auto-look: okra in %d/%d live frames, best conf %.2f >= %.2f"
                                % (okra["hits"], okra["frames"], okra["best_conf"], self.min_conf))
        except RuntimeError as e:
            self.status = "paused: %s" % e


def look_argv():
    """The Look step. OKRA_LOOK_CMD replaces it in offline tests/demos only (tests/fake_look.py)."""
    return os.environ["OKRA_LOOK_CMD"].split() if os.environ.get("OKRA_LOOK_CMD") else [PICK_SH, "look"]


def live_get_bytes(path, timeout=3.0):
    host, port = LIVE.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=timeout)
    try:
        s.sendall(("GET %s HTTP/1.0\r\nHost: robot\r\n\r\n" % path).encode())
        data = b"".join(iter(lambda: s.recv(65536), b""))
    finally:
        s.close()
    head, _, body = data.partition(b"\r\n\r\n")
    if b" 200 " not in head.split(b"\r\n")[0]:
        raise IOError("live_cam %s: %r" % (path, head[:60]))
    return body


# ------------------------------------------------------------------ record everything

CAMERA_JOBS = ("look", "auto-look", "floor")       # steps that need the camera itself (they pause the recording)


class RecordAll(threading.Thread):
    """While on: keep a recording ("all_<time>") running on the robot, so every step is on video with all joint/IMU
    data. Camera steps (Look, floor) stop it themselves (okra_pick.sh free_camera) and this starts the next one as
    soon as they are done. Every finished recording is copied to the VM (g1_record/data) for the page."""

    def __init__(self, app):
        super().__init__(daemon=True)
        self.app, self.on, self.status, self.pulled, self.last_try = app, False, "off", set(), 0.0

    def rec(self, *args, timeout=60):
        return subprocess.run([os.path.join(J, "g1_record", "rec.sh")] + list(args), capture_output=True, text=True,
                              timeout=timeout)

    def run(self):
        while True:
            time.sleep(3.0)
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001 - robot offline etc.: try again later
                self.status = "error: %s" % e

    def tick(self):
        if not self.on and time.time() - getattr(self, "t_idle", 0) < 30:   # off: only look for new recordings now and then
            return
        self.t_idle = time.time()
        st = self.rec("status", timeout=15).stdout
        recording = st.startswith("RECORDING")
        self.app.poll.recording = st.splitlines()[0] if st else self.app.poll.recording
        self.pull_finished(st if recording else "")
        if not self.on:
            self.status = "off"
            return
        job = self.app.jobs
        if recording:
            self.status = "recording: " + st.splitlines()[0].split("/")[-1]
            return
        if job.running and job.name in CAMERA_JOBS:
            self.status = "paused: %s needs the camera" % job.name
            return
        if time.time() - self.last_try < 10:                  # do not hammer a failing start
            return
        self.last_try = time.time()
        r = self.rec("start", "all_" + time.strftime("%Y%m%d_%H%M%S"), timeout=60)
        self.status = "started" if r.returncode == 0 else "could not start: %s" % (r.stdout + r.stderr).strip()[-120:]

    def pull_finished(self, current):
        """Copy finished recordings from the robot that the VM does not have yet (in the background, one at a time)."""
        if getattr(self, "pulling", False):
            return
        lst = self.rec("list", timeout=20).stdout.split()
        names = [n for n in lst if n.startswith(("all_", "okra_", "web_")) and n not in self.pulled
                 and n not in current and not os.path.isdir(os.path.join(REC, n))]
        if not names:
            return

        def pull(n):
            self.pulling = True
            try:
                self.rec("pull", n, timeout=900)
                d = os.path.join(REC, n)
                if os.path.isfile(os.path.join(d, "color.mkv")):
                    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", os.path.join(d, "color.mkv"), "-c", "copy",
                                    "-movflags", "+faststart", os.path.join(d, "color.mp4")], timeout=300)
                self.pulled.add(n)
            finally:
                self.pulling = False
        threading.Thread(target=pull, args=(names[0],), daemon=True).start()


# ------------------------------------------------------------------ recordings (g1_record/data)

REC = os.path.join(J, "g1_record", "data")


def episode_dir(name):
    d = os.path.realpath(os.path.join(REC, str(name or "")))
    if not d.startswith(os.path.realpath(REC) + os.sep) or not os.path.isdir(d):
        raise ValueError("no such recording: %s" % name)
    return d


def episodes():
    out = []
    for n in sorted(os.listdir(REC), reverse=True) if os.path.isdir(REC) else []:
        d = os.path.join(REC, n)
        if not os.path.isfile(os.path.join(d, "color.mkv")):
            continue
        meta = {}
        try:
            meta = json.load(open(os.path.join(d, "meta.json")))
        except (OSError, ValueError):
            pass
        ts = os.path.join(d, "color_ts.txt")
        dur = None
        try:
            v = [float(l) for l in open(ts) if l.strip() and not l.startswith("#")]
            dur = round((v[-1] - v[0]) / 1000.0, 1)
        except (OSError, ValueError, IndexError):
            pass
        out.append({"name": n, "duration_s": dur, "start_time": meta.get("start_time"),
                    "camera": "/rec/%s/color.mp4" % n if os.path.exists(os.path.join(d, "color.mp4")) else None,
                    "replay": "/rec/%s/replay_sim.mp4" % n if os.path.exists(os.path.join(d, "replay_sim.mp4")) else None})
    return out


# ------------------------------------------------------------------ background jobs

class Jobs:
    def __init__(self):
        self.lock = threading.Lock()
        self.name, self.running, self.rc, self.log, self.t0 = None, False, None, [], 0

    def start(self, name, argv, env=None, on_done=None, then=None, keep_log=False):
        """on_done(rc, log) runs before the job counts as finished; then(rc) after (it may start the next job)."""
        with self.lock:
            if self.running:
                raise RuntimeError("busy: %s is still running" % self.name)
            log = (self.log[-150:] if keep_log else []) + ["$ " + " ".join(argv)]
            self.name, self.running, self.rc, self.log, self.t0 = name, True, None, log, time.time()
        threading.Thread(target=self._run, args=(argv, env, on_done, then), daemon=True).start()

    def _run(self, argv, env, on_done, then=None):
        try:
            p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                 stdin=subprocess.DEVNULL, env=dict(os.environ, **(env or {})))
            for line in p.stdout:
                self.log.append(line.rstrip())
                self.log = self.log[-400:]
            self.rc = p.wait()
            if on_done:
                on_done(self.rc, self.log)
        except Exception as e:  # noqa: BLE001
            self.log.append("ERROR: %s" % e)
            self.rc = -1
        finally:
            self.log.append("[%s finished: exit %s, %.0f s]" % (self.name, self.rc, time.time() - self.t0))
            self.running = False
        if then:
            try:
                then(self.rc)
            except Exception as e:  # noqa: BLE001
                self.log.append("ERROR starting the next step: %s" % e)

    def state(self):
        return {"name": self.name, "running": self.running, "rc": self.rc, "log": self.log[-120:]}


# ------------------------------------------------------------------ app state

class App:
    def __init__(self):
        self.jobs = Jobs()
        self.poll = StatePoller()
        self.poll.start()
        self.live = LiveRelay()
        self.auto = AutoLook(self)
        self.auto.start()
        self.recall = RecordAll(self)
        self.recall.start()
        self.event = store.find()

    def start_plan(self, ev, then=None, keep_log=False):
        """Plan (okra_pick.sh plan: safety gate + planner + sim video). If refused: keep the reason in plan_error.json
        and make the best-effort attempt video (plan/attempt_sim.py, VM only, nothing moves), so the page can show
        why. then(rc) runs after the plan (before the attempt video)."""
        eid = os.path.basename(ev)

        def done(rc, log):
            if rc == 0:
                for f in ("plan_error.json",):
                    if os.path.exists(os.path.join(ev, f)):
                        os.remove(os.path.join(ev, f))
                return
            i = next((k for k, l in enumerate(log) if "REFUSED" in l or "SAFETY GATE" in l), None)
            store.save(ev, "plan_error.json", {"reason": "\n".join(log[i:i + 8]) if i is not None else "\n".join(log[-8:]),
                                               "safety_gate": any("SAFETY GATE" in l for l in log), "time": time.time()})

        def after(rc):
            if then:
                then(rc)
            err = store.load(ev, "plan_error.json")
            if rc != 0 and err and not err["safety_gate"] and not self.jobs.running:
                self.jobs.start("attempt video", [PY_VM, os.path.join(OKRA, "plan", "attempt_sim.py"), ev],
                                env={"MUJOCO_GL": "egl"}, keep_log=True)
        self.jobs.start("plan", [PICK_SH, "plan", eid], on_done=done, then=after, keep_log=keep_log)

    def rel(self, path):
        return os.path.relpath(path, store.ROOT)

    def event_view(self, ev=None):
        ev = ev or self.event
        if not ev or not os.path.isdir(ev):
            return None
        cj = store.load(ev, "candidates.json") or {}
        pend = store.load(ev, "decision_pending.json")
        dec = store.load(ev, "decision.json")
        tr = store.load(ev, "trajectory.json")
        files = {f: "/data/%s/%s" % (self.rel(ev), f) for f in ("question.jpg", "question_view.jpg", "annotated.jpg", "sim.mp4", "trigger_live.jpg", "attempt_sim.mp4")
                 if os.path.exists(os.path.join(ev, f))}
        cands = [{k: c[k] for k in ("id", "conf_median", "conf_max", "seen_frac", "accepted_frac", "reject_reasons",
                                    "in_reach", "px", "key_mask", "xyz_pelvis", "length_m")}
                 for c in cj.get("candidates", [])]
        return {"id": os.path.basename(ev), "path": self.rel(ev), "candidates": cands, "pending": pend, "decision": dec,
                "target": store.load(ev, "target.json") is not None,
                "plan": None if not tr else {"duration_s": round(tr["duration_s"], 1), "pitch": tr["approach_pitch_deg"],
                                             "segments": [s["name"] for s in tr["segments"]]},
                "executed": store.load(ev, "trajectory_executed.json"), "outcome": store.load(ev, "outcome.json"),
                "files": files, "detector": cj.get("detector"), "trigger": store.load(ev, "trigger.json"),
                "looked_at": cj.get("time"), "target_xyz": (store.load(ev, "target.json") or {}).get("xyz_pelvis"),
                "plan_error": store.load(ev, "plan_error.json"), "attempt": store.load(ev, "attempt_result.json")}

    def state(self):
        return {"robot": self.poll.robot, "robot_error": self.poll.robot_err, "recording": self.poll.recording,
                "job": self.jobs.state(), "event": self.event_view(), "agent": AGENT, "live": self.live.state(), "auto": self.auto.settings(),
                "record_all": {"on": self.recall.on, "status": self.recall.status},
                "time": time.time()}


APP = None
PLACE = {}


# ------------------------------------------------------------------ HTTP

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authorised(self):
        if not TOKEN:
            return True
        q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        return self.headers.get("X-Token") == TOKEN or q.get("token", [None])[0] == TOKEN

    def _file(self, path, ctype=None):
        if not os.path.isfile(path):
            return self._json({"error": "not found"}, 404)
        data = open(path, "rb").read()
        rng = self.headers.get("Range", "")                  # video seeking: "bytes=START-[END]"
        start, end, code = 0, len(data) - 1, 200
        if rng.startswith("bytes=") and data:
            a, _, b = rng[6:].split(",")[0].partition("-")
            try:
                start = int(a) if a else max(0, len(data) - int(b))
                end = min(len(data) - 1, int(b)) if a and b else len(data) - 1
                code = 206
            except ValueError:
                pass
        self.send_response(code)
        self.send_header("Content-Type", ctype or mimetypes.guess_type(path)[0] or "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Accept-Ranges", "bytes")
        if code == 206:
            self.send_header("Content-Range", "bytes %d-%d/%d" % (start, end, len(data)))
        self.send_header("Content-Length", str(end - start + 1))
        self.end_headers()
        try:
            self.wfile.write(data[start:end + 1])
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _bytes(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _live_stream(self):
        """Browser <img> MJPEG: the newest robot frame each time one arrives (frames are skipped, never queued)."""
        live = APP.live
        live.want()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            with live.cond:                              # never start with an old cached frame
                seen = live.seq if time.time() - live.t_arrive > 2 else -1
            while True:
                with live.cond:
                    live.cond.wait_for(lambda: live.seq != seen, timeout=5.0)
                    jpg, seen = live.jpeg, live.seq
                if jpg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                self.wfile.write(jpg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            live.unwant()

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return self._file(os.path.join(HERE, "index.html"), "text/html; charset=utf-8")
        if not self._authorised():
            return self._json({"error": "token required"}, 403)
        if url.path == "/api/state":
            return self._json(APP.state())
        if url.path == "/api/events":
            rows = store.reindex()
            return self._json({"events": rows[::-1][:60]})
        if url.path == "/api/live.mjpg":
            return self._live_stream()
        if url.path == "/api/live.jpg":
            if APP.live.jpeg is None:
                return self._json({"error": "no live frame yet"}, 503)
            return self._bytes(APP.live.jpeg, "image/jpeg")
        if url.path == "/api/place":                    # placement guide: one reading from the live detection
            try:
                if "pg" not in PLACE:
                    sys.path.insert(0, os.path.join(OKRA, "tools"))
                    import place_guide as pg
                    PLACE["pg"], PLACE["chain"] = pg, pg.Chain(os.path.join(OKRA, "robot", "g1.urdf"))
                pg = PLACE["pg"]
                rs, z_pod, eid = pg.camera()
                msg, inside = pg.reading(PLACE["chain"], rs, z_pod)
                h = ("pod height measured by Look %s" % eid) if z_pod is not None else "pod height ASSUMED ~90 cm"
                return self._json({"ok": True, "text": msg, "inside": inside, "height": h})
            except (Exception, SystemExit) as e:  # noqa: BLE001 - robot / feed offline
                return self._json({"ok": False, "text": "guide unavailable: %s" % e})
        if url.path == "/api/episodes":
            return self._json({"episodes": episodes()})
        if url.path.startswith("/rec/"):
            p = os.path.realpath(os.path.join(REC, urllib.parse.unquote(url.path[5:])))
            if not p.startswith(os.path.realpath(REC) + os.sep) or not p.endswith((".mp4", ".jpg")):
                return self._json({"error": "forbidden"}, 403)
            return self._file(p)
        if url.path.startswith("/data/"):
            p = os.path.realpath(os.path.join(store.ROOT, urllib.parse.unquote(url.path[6:])))
            if not p.startswith(os.path.realpath(store.ROOT) + os.sep):
                return self._json({"error": "forbidden"}, 403)
            return self._file(p)
        return self._json({"error": "not found"}, 404)

    def do_POST(self):
        if not self._authorised():
            return self._json({"error": "token required"}, 403)
        path = urllib.parse.urlparse(self.path).path
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")
        try:
            return self._json(self.route(path, body))
        except Exception as e:  # noqa: BLE001 - errors go to the page
            return self._json({"ok": False, "error": str(e)}, 400)

    def route(self, path, b):
        app = APP
        ev = store.find(b["event"]) if b.get("event") else app.event
        sh = PICK_SH
        if path == "/api/robot":
            cmd = b.get("cmd")
            if cmd not in ROBOT_CMDS:
                raise ValueError("command not allowed: %s" % cmd)
            if cmd == "stop":
                app.auto.cancel()
                return stop_all()
            return agent_call(cmd, b.get("args"), timeout=60.0)
        if path == "/api/look":
            mode = b.get("mode") or None

            def done(rc, log):
                d = next((l.split("=", 1)[1] for l in log if l.startswith("EVENT_DIR=")), None)
                if rc == 0 and d:
                    app.event = d
                    dec = confirm.prepare(d, mode)
                    log.append("decision: %s (%s)" % (dec["action"].upper(), "; ".join(dec["reasons"])))
            app.jobs.start("look", look_argv(), on_done=done)
            return {"ok": True}
        if path == "/api/ask_again":
            dec = confirm.prepare(ev, "always")
            return {"ok": True, "decision": dec}
        if path == "/api/answer":
            dec = confirm.apply_answer(ev, str(b["answer"]), b.get("operator") or "web")
            kind, note = app.auto.after_yes, None
            if dec.get("target") and kind != "ask":
                if kind in ("reach", "pick") and b.get("confirm") != kind.upper():
                    kind, note = "plan", "motion not confirmed with the answer: plan only"
                app.auto.start_chain(ev, kind)
            return {"ok": True, "decision": dec, "chain": app.auto.chain if dec.get("target") else None, "note": note}
        if path == "/api/plan":
            app.start_plan(ev)
            return {"ok": True}
        if path == "/api/attempt":                      # best-effort grab video for this event (VM only)
            app.jobs.start("attempt video", [PY_VM, os.path.join(OKRA, "plan", "attempt_sim.py"), ev], env={"MUJOCO_GL": "egl"})
            return {"ok": True}
        if path == "/api/replay":                       # camera + simulation replay of a recording (VM only)
            epd = episode_dir(b.get("episode"))
            cmd = ("cd %s && ( [ -f color.mp4 ] || ffmpeg -loglevel error -y -i color.mkv -c copy -movflags +faststart color.mp4 ) "
                   "&& %s %s %s" % (epd, PY_VM, os.path.join(OKRA, "plan", "replay_episode.py"), epd))
            app.jobs.start("replay " + os.path.basename(epd), ["bash", "-c", cmd], env={"MUJOCO_GL": "egl"})
            return {"ok": True}
        if path == "/api/execute":
            kind = b.get("kind")
            if kind not in ("dry", "reach", "pick"):
                raise ValueError("kind must be dry, reach or pick")
            if kind != "dry" and b.get("confirm") != kind.upper():
                raise ValueError("type %s to confirm" % kind.upper())
            env = {"OKRA_NONINTERACTIVE": "1"}
            if kind == "reach" and b.get("hold"):              # hold at the pod (place it / look), optional jaw test
                env["OKRA_HOLD"] = str(int(min(60, max(0, float(b["hold"])))))
                if b.get("jaw_cycle") and float(b["hold"]) >= 10:
                    env["OKRA_JAW_CYCLE"] = "1"
            app.jobs.start(kind, [sh, kind, os.path.basename(ev), "--yes"], env=env)
            return {"ok": True}
        if path == "/api/outcome":
            if b.get("result") not in ("success", "fail", "partial", "skipped"):
                raise ValueError("result must be success, fail, partial or skipped")
            if b["result"] == "skipped" and app.jobs.running:
                raise ValueError("wait until %s has finished (or press STOP)" % app.jobs.name)
            ex = store.load(ev, "trajectory_executed.json") or {}
            store.save(ev, "outcome.json", {"result": b["result"], "note": b.get("note", ""), "operator": b.get("operator") or "web",
                                            "player_aborted": ex.get("aborted"), "time": time.time()})
            store.reindex()
            if app.auto.chain and app.auto.chain["event"] == os.path.basename(ev):
                app.auto.chain["step"] = "finished: " + b["result"]
            app.auto.t_last = time.time()                   # short pause before looking for the next okra
            return {"ok": True}
        if path == "/api/select_event":
            if not ev:
                raise ValueError("event not found")
            app.event = ev
            return {"ok": True}
        if path == "/api/safety":   # live safety gate (robot probe); also runs automatically before every step
            step = b.get("step") or "status"
            if step not in ("status", "gripper", "floor", "look", "plan", "dry", "reach", "pick"):
                raise ValueError("unknown step: %s" % step)
            args = [sh, "safety", step] + ([os.path.basename(ev)] if ev and step in ("plan", "dry", "reach", "pick") else [])
            app.jobs.start("safety " + step, args)
            return {"ok": True}
        if path == "/api/setup":                        # link check, deploy code, camera floor check (nothing moves)
            act = b.get("action")
            cmds = {"check": [os.path.join(J, "g1_connect", "check.sh")],
                    "deploy": ["bash", "-c", "%s deploy && %s deploy" % (PICK_SH, os.path.join(J, "robot_agent", "agent.sh"))],
                    "floor": [PICK_SH, "floor"]}
            if act not in cmds:
                raise ValueError("action must be check, deploy or floor")
            app.jobs.start({"check": "link check", "deploy": "deploy", "floor": "floor check"}[act], cmds[act])
            return {"ok": True}
        if path == "/api/point":                        # fixed-point target from the live joint state, then plan
            def done(rc, log):
                d = next((l.split("=", 1)[1] for l in log if l.startswith("EVENT_DIR=")), None)
                if rc == 0 and d:
                    app.event = d

            def then(rc):
                if rc == 0 and app.event:
                    app.start_plan(app.event, keep_log=True)
            args = [sh, "point"] + ([str(float(v)) for v in b["xyz"]] if b.get("xyz") else [])
            app.jobs.start("fixed point", args, on_done=done, then=then)
            return {"ok": True}
        if path == "/api/record_all":
            app.recall.on = bool(b.get("on"))
            if not app.recall.on:
                app.jobs.start("record stop", [os.path.join(J, "g1_record", "rec.sh"), "stop"])
            return {"ok": True, "record_all": {"on": app.recall.on}}
        if path == "/api/record":
            act = b.get("action")
            if act == "start":
                app.jobs.start("record start", [os.path.join(J, "g1_record", "rec.sh"), "start",
                                                "web_" + time.strftime("%Y%m%d_%H%M%S")])
            elif act == "stop":
                app.jobs.start("record stop", [os.path.join(J, "g1_record", "rec.sh"), "stop"])
            else:
                raise ValueError("action must be start or stop")
            app.poll.t_rec = 0
            return {"ok": True}
        if path == "/api/agent":
            act = b.get("action")
            if act not in ("start", "restart", "status"):
                raise ValueError("action must be start, restart or status")
            app.jobs.start("agent " + act, [os.path.join(J, "robot_agent", "agent.sh"), act])
            return {"ok": True}
        if path == "/api/auto":
            a = app.auto
            if "min_conf" in b:
                a.min_conf = min(0.95, max(0.10, float(b["min_conf"])))
            if "after_yes" in b:
                if b["after_yes"] not in ("ask", "plan", "reach", "pick"):
                    raise ValueError("after_yes must be ask, plan, reach or pick")
                a.after_yes = b["after_yes"]
            if "armed" in b:
                a.armed = bool(b["armed"])
                if a.armed:
                    a.t_last = 0.0
                    try:                                  # the detector must run on the robot for this
                        live_get("/control?overlay=1")
                    except Exception as e:  # noqa: BLE001
                        app.jobs.log.append("auto-look: live feed not reachable (%s)" % e)
            return {"ok": True, "auto": a.settings()}
        if path == "/api/live":
            act = b.get("action")
            if act in ("start", "stop", "status"):
                app.jobs.start("live " + act, [sh, "live", act])
                return {"ok": True}
            if act in ("overlay_on", "overlay_off"):
                return {"ok": True, "result": live_get("/control?overlay=%d" % (act == "overlay_on"))}
            raise ValueError("action must be start, stop, status, overlay_on or overlay_off")
        if path == "/api/dataset":
            argv = [PY_VM, os.path.join(OKRA, "hitl", "export_dataset.py")] + (["--include-auto"] if b.get("include_auto") else [])
            app.jobs.start("dataset export", argv)
            return {"ok": True}
        raise ValueError("unknown endpoint %s" % path)


def main():
    global APP, TOKEN
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--token", default=os.environ.get("OKRA_WEB_TOKEN"))
    args = ap.parse_args()
    if args.host not in ("127.0.0.1", "localhost") and not args.token:
        sys.exit("refusing to listen on %s without --token (anyone on the network could move the robot)" % args.host)
    TOKEN = args.token
    APP = App()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print("okra operator UI: http://%s:%d/%s" % ("localhost" if args.host == "127.0.0.1" else args.host, args.port,
                                                  "?token=" + TOKEN if TOKEN else ""), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
