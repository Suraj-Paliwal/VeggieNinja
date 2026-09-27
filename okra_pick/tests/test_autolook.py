"""
Offline test of auto-look (no robot): the live feed sees okra -> snapshot + Look -> the operator is ALWAYS asked ->
YES does the work for that okra (plan -> pick) -> outcome -> the next okra. Also: STOP cancels the chain and
disarms, a YES without motion confirmation only plans, a NO starts nothing.

    ../dimos/.venv/bin/python tests/test_autolook.py          (~3 min, VM CPU)

Uses a clip of okra frames (okra_robot/dataset/raw_s02_*), tests/fake_look.py and tests/fake_pick.sh
(nothing real runs), ports 8291/8290/7797 and a temporary OKRA_DATA.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
J = os.path.abspath(os.path.join(HERE, "..", ".."))
PY = sys.executable
TMP = tempfile.mkdtemp(prefix="okra_autolook_test_")
WEB = "http://127.0.0.1:8290"
DS = os.path.join(J, "okra_robot", "dataset")


def make_clip():
    """8 s without okra, then 15 s with pods, 10 fps."""
    os.makedirs(os.path.join(TMP, "clip"))
    neg = sorted(os.listdir(os.path.join(DS, "raw_s02_neg")))[:80]
    pods = sorted(os.listdir(os.path.join(DS, "raw_s02_pods")))[1549:1700]
    for i, (d, f) in enumerate([("raw_s02_neg", f) for f in neg] + [("raw_s02_pods", f) for f in pods]):
        os.symlink(os.path.join(DS, d, f), os.path.join(TMP, "clip", "%05d.jpg" % i))
    out = os.path.join(TMP, "clip.mkv")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", "10", "-i",
                    os.path.join(TMP, "clip", "%05d.jpg"), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-vf", "scale=640:480", out],
                   check=True)
    return out


def api(path, body=None):
    req = urllib.request.Request(WEB + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        return json.load(urllib.request.urlopen(req, timeout=20))
    except urllib.error.HTTPError as e:
        return json.load(e)


def wait(cond, secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        s = api("/api/state")
        if cond(s):
            return s
        time.sleep(0.5)
    return None


def question_open(s):
    return bool(s["event"] and s["event"]["pending"])


def next_okra_question(tries=4):
    """Wait for a question that shows at least one candidate (a Look can also end with 'nothing found';
    those are answered 'not sure' and auto-look tries again). Returns (state, first shown id)."""
    for _ in range(tries):
        s = wait(question_open, 90)
        if s is None:
            return None, None
        if s["event"]["pending"]["shown"]:
            return s, str(s["event"]["pending"]["shown"][0])
        api("/api/answer", {"answer": "s"})
    return None, None


def main():
    clip = make_clip()
    env = dict(os.environ, G1_AGENT="127.0.0.1:7797", OKRA_LIVE="127.0.0.1:8291", OKRA_DATA=os.path.join(TMP, "data"),
               OKRA_LOOK_CMD="%s %s" % (PY, os.path.join(HERE, "fake_look.py")), OKRA_PICK_SH=os.path.join(HERE, "fake_pick.sh"))
    log = open(os.path.join(TMP, "procs.log"), "w")
    procs = [subprocess.Popen([PY, os.path.join(J, "okra_pick", "robot", "live_cam.py"), "--host", "127.0.0.1", "--port", "8291",
                               "--file", clip, "--weights", os.path.join(J, "okra_robot", "models", "okra_seg_s02.pt")],
                              stdout=log, stderr=subprocess.STDOUT),
             subprocess.Popen([PY, os.path.join(J, "robot_agent", "agent.py"), "--sim", "--host", "127.0.0.1", "--port", "7797",
                               "--log", os.path.join(TMP, "agent.log")], stdout=log, stderr=subprocess.STDOUT)]
    time.sleep(3)
    procs.append(subprocess.Popen([PY, os.path.join(J, "okra_pick", "web", "server.py"), "--port", "8290"], env=env,
                                  stdout=log, stderr=subprocess.STDOUT))
    results = []
    try:
        time.sleep(4)
        api("/api/auto", {"armed": True, "min_conf": 0.5, "after_yes": "pick"})
        # 1-2: two okra in a row, each: asked -> YES+PICK -> plan -> pick -> outcome -> next
        prev = None
        for n in (1, 2):
            s, cid = next_okra_question()
            ok = s is not None and s["event"]["id"] != prev and os.path.exists(
                os.path.join(TMP, "data", "events", s["event"]["path"].split("/", 1)[1], "trigger.jpg"))
            eid = s and s["event"]["id"]
            r = api("/api/answer", {"answer": cid, "operator": "test", "confirm": "PICK"})
            s2 = wait(lambda s: (s["auto"]["chain"] or {}).get("step", "").startswith("pick done"), 60)
            logs = "\n".join(s2["job"]["log"]) if s2 else ""
            api("/api/outcome", {"result": "success"})
            s3 = api("/api/state")
            results.append(("okra %d: asked, plan -> pick, outcome" % n, ok and r.get("chain") and "FAKE plan done" in logs
                            and "FAKE pick done" in logs and s3["event"]["outcome"]["result"] == "success", eid))
            prev = eid
        # 3: STOP right after YES: chain cancelled, no pick, auto-look disarmed
        s, cid = next_okra_question()
        api("/api/answer", {"answer": cid, "confirm": "PICK"})
        time.sleep(0.3)
        api("/api/robot", {"cmd": "stop"})
        time.sleep(4)
        s = api("/api/state")
        results.append(("STOP cancels the chain and disarms", (s["auto"]["chain"] or {}).get("step") == "cancelled (STOP)"
                        and not s["auto"]["armed"] and not any("FAKE pick" in l for l in s["job"]["log"]), s["auto"]["chain"]))
        api("/api/outcome", {"result": "skipped"})
        # 4: YES without motion confirmation -> plan only
        api("/api/auto", {"armed": True})
        s, cid = next_okra_question()
        r = api("/api/answer", {"answer": cid})
        s = wait(lambda s: (s["auto"]["chain"] or {}).get("step", "").startswith("planned"), 30)
        results.append(("unconfirmed YES only plans", (r.get("chain") or {}).get("kind") == "plan" and s is not None, r.get("note")))
        api("/api/outcome", {"result": "skipped"})
        # 5: NO -> nothing starts, auto-look goes on
        wait(question_open, 90)
        r = api("/api/answer", {"answer": "n"})
        time.sleep(1)
        s = api("/api/state")
        results.append(("NO starts nothing, keeps watching", r.get("ok") and r.get("chain") is None and not s["job"]["running"] and s["auto"]["armed"],
                        s["auto"]["status"]))
    finally:
        for p in procs:
            p.terminate()
            p.wait()
    ok = 0
    for name, passed, info in results:
        ok += bool(passed)
        print("%s  %s  (%s)" % ("PASS" if passed else "FAIL", name, info))
    print("%d/%d passed (files in %s)" % (ok, len(results), TMP))
    sys.exit(0 if ok == len(results) == 5 else 1)


if __name__ == "__main__":
    main()
