"""
Offline test of the live camera feed (no robot): robot/live_cam.py on a recorded robot video, camera holds,
recorder relay, missing camera, and the web server's relay (web/server.py /api/live.mjpg).

    ../dimos/.venv/bin/python tests/test_live.py

Uses its own ports (8191 live_cam, 8190 web, 7799 sim agent, 5611 relay) and hold/state files in a temp dir,
so it does not disturb a running feed.
"""

import glob
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
TMP = tempfile.mkdtemp(prefix="okra_live_test_")
VIDEO = (sorted(glob.glob(os.path.join(J, "g1_record", "data", "*", "color.mkv"))) or [None])[0]
LIVE, WEB = "127.0.0.1:8191", "127.0.0.1:8190"


def get_json(url):
    return json.load(urllib.request.urlopen(url, timeout=5))


def frames(url, secs):
    """Count multipart frames and return the byte sizes seen in `secs` seconds."""
    sizes, t0 = [], time.time()
    r = urllib.request.urlopen(url, timeout=5)
    while time.time() - t0 < secs:
        line = r.readline()
        if line.startswith(b"Content-Length:"):
            n = int(line.split(b":")[1])
            while r.readline().strip():
                pass
            r.read(n)
            sizes.append(n)
    r.close()
    return sizes


def hold(name, secs):
    os.makedirs(os.path.join(TMP, "hold"), exist_ok=True)
    open(os.path.join(TMP, "hold", name), "w").write(str(time.time() + secs))


def start_live(*extra):
    env = dict(os.environ)
    code = ("import sys; sys.argv[0]=%r; sys.path.insert(0, %r); import live_cam as L; L.HOLD_DIR=%r; "
            "L.STATE_FILE=%r; L.main()" % (os.path.join(J, "okra_pick", "robot", "live_cam.py"),
                                           os.path.join(J, "okra_pick", "robot"), os.path.join(TMP, "hold"),
                                           os.path.join(TMP, "state.json")))
    return subprocess.Popen([PY, "-c", code, "--host", "127.0.0.1", "--port", "8191", "--no-overlay"] + list(extra),
                            stdout=open(os.path.join(TMP, "live.log"), "a"), stderr=subprocess.STDOUT, env=env)


def main():
    if not VIDEO:
        sys.exit("no g1_record/data/*/color.mkv to test with")
    procs, results = [], []
    try:
        # 1: file source -> ~10 fps JPEGs
        p = start_live("--file", VIDEO)
        procs.append(p)
        time.sleep(6)
        st = get_json("http://%s/status" % LIVE)
        n = frames("http://%s/live.mjpg" % LIVE, 3)
        results.append(("file source streams ~10 fps", st["source"] == "file" and 20 <= len(n) <= 40, "%d frames/3 s" % len(n)))

        # 2: hold -> paused (ffmpeg gone, placeholder frames), expiry -> back
        hold("look", 3)
        time.sleep(1)
        st = get_json("http://%s/status" % LIVE)
        paused = st["source"] == "paused" and st["holds"] == ["look"]
        time.sleep(3.5)
        st2 = get_json("http://%s/status" % LIVE)
        results.append(("hold pauses, expiry resumes", paused and st2["source"] == "file", "%s -> %s" % (st["source"], st2["source"])))
        p.terminate()
        p.wait()

        # 3: recorder running -> relay of its local MPEG-TS copy; recorder gone -> camera (missing here: error)
        rec = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-re", "-stream_loop", "-1",
                                "-i", VIDEO, "-an", "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
                                "-g", "30", "-pix_fmt", "yuv420p", "-bsf:v", "dump_extra", "-f", "mpegts",
                                "udp://127.0.0.1:5611?pkt_size=1316"])
        procs.append(rec)
        pidf = os.path.join(TMP, "recorder.pid")
        open(pidf, "w").write(str(rec.pid))
        p = start_live("--device", "/dev/none_for_test", "--recorder-pid", pidf, "--relay", "127.0.0.1:5611")
        procs.append(p)
        time.sleep(7)
        st = get_json("http://%s/status" % LIVE)
        results.append(("recorder running -> relay", st["source"] == "relay" and st["fps"] > 5, "%s %.1f fps" % (st["source"], st["fps"])))
        rec.terminate()
        rec.wait()
        time.sleep(3)
        st = get_json("http://%s/status" % LIVE)
        results.append(("recorder stopped -> camera, missing camera reported", st["source"] == "camera"
                        and "none_for_test" in (st["error"] or ""), st["error"]))
        p.terminate()
        p.wait()

        # 4: web server relay (sim agent), overlay control through the server
        p = start_live("--file", VIDEO)
        procs.append(p)
        ag = subprocess.Popen([PY, os.path.join(J, "robot_agent", "agent.py"), "--sim", "--host", "127.0.0.1", "--port", "7799",
                               "--log", os.path.join(TMP, "agent.log")], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(ag)
        env = dict(os.environ, G1_AGENT="127.0.0.1:7799", OKRA_LIVE=LIVE, OKRA_DATA=os.path.join(TMP, "data"))
        web = subprocess.Popen([PY, os.path.join(J, "okra_pick", "web", "server.py"), "--port", "8190"], env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(web)
        time.sleep(7)
        n = frames("http://%s/api/live.mjpg" % WEB, 3)
        st = get_json("http://%s/api/state" % WEB)["live"]
        results.append(("web relay passes the stream", 20 <= len(n) <= 40 and st["connected"], "%d frames/3 s, %s" % (len(n), st)))
        req = urllib.request.Request("http://%s/api/live" % WEB, data=b'{"action": "overlay_on"}',
                                     headers={"Content-Type": "application/json"})
        r = json.load(urllib.request.urlopen(req, timeout=5))
        results.append(("overlay control via web", r.get("ok") and r["result"]["overlay"] is True, r.get("result")))
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
                p.wait()

    ok = 0
    for name, passed, info in results:
        ok += bool(passed)
        print("%s  %s  (%s)" % ("PASS" if passed else "FAIL", name, info))
    print("%d/%d passed (logs in %s)" % (ok, len(results), TMP))
    sys.exit(0 if ok == len(results) else 1)


if __name__ == "__main__":
    main()
