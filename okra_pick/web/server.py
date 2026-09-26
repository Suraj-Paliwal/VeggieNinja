#!/usr/bin/env python3
"""
Operator web interface for the G1 okra pick (human-in-the-loop). Runs on the VM, standard library only
(plus the okra_pick/hitl modules). Open http://localhost:8080 in a browser.

    ../../dimos/.venv/bin/python server.py                        # this VM only
    ../../dimos/.venv/bin/python server.py --host 0.0.0.0 --token SECRET   # other devices on the network:
                                                                  # open http://<vm-ip>:8080/?token=SECRET

It talks to robot_agent (live state, STOP, bring-up, steps, waist, gripper, voice/LED) and runs the
pipeline steps (look / plan / reach / pick / record / dataset) as background jobs with a live log.
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
ROBOT_CMDS = {"stop", "status", "ai", "damp", "ready", "start", "step", "head", "gripper", "say", "led", "ping"}
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


# ------------------------------------------------------------------ background jobs

class Jobs:
    def __init__(self):
        self.lock = threading.Lock()
        self.name, self.running, self.rc, self.log, self.t0 = None, False, None, [], 0

    def start(self, name, argv, env=None, on_done=None):
        with self.lock:
            if self.running:
                raise RuntimeError("busy: %s is still running" % self.name)
            self.name, self.running, self.rc, self.log, self.t0 = name, True, None, ["$ " + " ".join(argv)], time.time()
        threading.Thread(target=self._run, args=(argv, env, on_done), daemon=True).start()

    def _run(self, argv, env, on_done):
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

    def state(self):
        return {"name": self.name, "running": self.running, "rc": self.rc, "log": self.log[-120:]}


# ------------------------------------------------------------------ app state

class App:
    def __init__(self):
        self.jobs = Jobs()
        self.poll = StatePoller()
        self.poll.start()
        self.event = store.find()

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
        files = {f: "/data/%s/%s" % (self.rel(ev), f) for f in ("question.jpg", "question_view.jpg", "annotated.jpg", "sim.mp4")
                 if os.path.exists(os.path.join(ev, f))}
        cands = [{k: c[k] for k in ("id", "conf_median", "conf_max", "seen_frac", "accepted_frac", "reject_reasons",
                                    "in_reach", "px", "key_mask", "xyz_pelvis", "length_m")}
                 for c in cj.get("candidates", [])]
        return {"id": os.path.basename(ev), "path": self.rel(ev), "candidates": cands, "pending": pend, "decision": dec,
                "target": store.load(ev, "target.json") is not None,
                "plan": None if not tr else {"duration_s": round(tr["duration_s"], 1), "pitch": tr["approach_pitch_deg"],
                                             "segments": [s["name"] for s in tr["segments"]]},
                "executed": store.load(ev, "trajectory_executed.json"), "outcome": store.load(ev, "outcome.json"),
                "files": files, "detector": cj.get("detector")}

    def state(self):
        return {"robot": self.poll.robot, "robot_error": self.poll.robot_err, "recording": self.poll.recording,
                "job": self.jobs.state(), "event": self.event_view(), "agent": AGENT, "time": time.time()}


APP = None


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
        self.send_response(200)
        self.send_header("Content-Type", ctype or mimetypes.guess_type(path)[0] or "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

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
        sh = os.path.join(OKRA, "okra_pick.sh")
        if path == "/api/robot":
            cmd = b.get("cmd")
            if cmd not in ROBOT_CMDS:
                raise ValueError("command not allowed: %s" % cmd)
            return agent_call(cmd, b.get("args"), timeout=60.0)
        if path == "/api/look":
            mode = b.get("mode") or None

            def done(rc, log):
                d = next((l.split("=", 1)[1] for l in log if l.startswith("EVENT_DIR=")), None)
                if rc == 0 and d:
                    app.event = d
                    dec = confirm.prepare(d, mode)
                    log.append("decision: %s (%s)" % (dec["action"].upper(), "; ".join(dec["reasons"])))
            app.jobs.start("look", [sh, "look"], on_done=done)
            return {"ok": True}
        if path == "/api/ask_again":
            dec = confirm.prepare(ev, "always")
            return {"ok": True, "decision": dec}
        if path == "/api/answer":
            dec = confirm.apply_answer(ev, str(b["answer"]), b.get("operator") or "web")
            return {"ok": True, "decision": dec}
        if path == "/api/plan":
            app.jobs.start("plan", [sh, "plan", os.path.basename(ev)])
            return {"ok": True}
        if path == "/api/execute":
            kind = b.get("kind")
            if kind not in ("dry", "reach", "pick"):
                raise ValueError("kind must be dry, reach or pick")
            if kind != "dry" and b.get("confirm") != kind.upper():
                raise ValueError("type %s to confirm" % kind.upper())
            app.jobs.start(kind, [sh, kind, os.path.basename(ev), "--yes"], env={"OKRA_NONINTERACTIVE": "1"})
            return {"ok": True}
        if path == "/api/outcome":
            if b.get("result") not in ("success", "fail", "partial"):
                raise ValueError("result must be success, fail or partial")
            ex = store.load(ev, "trajectory_executed.json") or {}
            store.save(ev, "outcome.json", {"result": b["result"], "note": b.get("note", ""), "operator": b.get("operator") or "web",
                                            "player_aborted": ex.get("aborted"), "time": time.time()})
            store.reindex()
            return {"ok": True}
        if path == "/api/select_event":
            if not ev:
                raise ValueError("event not found")
            app.event = ev
            return {"ok": True}
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
