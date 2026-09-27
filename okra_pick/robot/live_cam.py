#!/usr/bin/env python3
"""
Live head-camera feed for the operator web page. Runs ON THE ROBOT (Orin), conda env g1brainco
(Python 3.8: cv2, numpy, ultralytics for the optional okra overlay). All image work is done here; the VM only
relays the finished JPEGs, so the feed costs the VM almost nothing and uses TCP (no lossy UDP fragments).

    python live_cam.py --host 192.168.123.164 [--port 8091] [--fps 10] [--weights okra_seg_s02.pt]
    python live_cam.py --host 127.0.0.1 --file some.mkv          # tests without the robot (loops the video)

HTTP (robot network only):
  /live.mjpg     multipart JPEG stream (always the newest frame, never a backlog)
  /snap.jpg      newest frame (with outlines)       /raw.jpg   newest camera frame without drawing
  /status        JSON: source, fps, frame age, clients, overlay, hold reasons, last error, and "okra": how many of
                 the last 10 frames had a filter-accepted okra with conf >= min_conf (/status?min_conf=0.5);
                 the web page's auto-look uses it to decide when to take a snapshot and ask the operator
  /control?overlay=0|1
  /control?zone=<json {"poly": [[u, v], ...], "point": [u, v], "label": "..."}> | zone=off
                 placement guide drawn on every frame (computed on the VM from the calibrated camera model)

Only one program can use the head camera. The source is chosen live, every 0.2 s:
  recorder running (~/g1_rec/recorder.pid alive)  -> "relay": decode the recorder's own local copy
                                                     (udp://127.0.0.1:5601), the camera stays with the recorder
  a hold file in /tmp/okra_live_hold/ not expired -> "paused": camera released for okra_perceive.py etc.
  otherwise                                       -> "camera": ffmpeg V4L2 on /dev/video4 (same as the recorder)
Hold files are written by okra_pick.sh / rec.sh on the robot: name = who, content = expiry (robot epoch s).
Every frame carries the robot time it was taken, so the page can show a frozen feed as STALE.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "okra_robot"))           # on the robot (deploy layout)
sys.path.insert(0, os.path.join(HERE, "..", "..", "okra_robot"))     # in the repo (tests)
HOLD_DIR = "/tmp/okra_live_hold"
STATE_FILE = "/tmp/okra_live_state.json"       # current source, read by the hold helpers in the shell scripts
W, H = 640, 480
OVERLAY_CONF = 0.10                              # same floor as okra_perceive.py candidates


def held():
    """Names of unexpired holds (expiry is robot epoch seconds, written by the robot's own `date`)."""
    out, now = [], time.time()
    try:
        names = os.listdir(HOLD_DIR)
    except OSError:
        return out
    for n in names:
        try:
            if float(open(os.path.join(HOLD_DIR, n)).read().strip() or 0) > now:
                out.append(n)
        except (OSError, ValueError):
            pass
    return out


def recorder_running(pid_file):
    try:
        os.kill(int(open(pid_file).read().strip()), 0)
        return True
    except (OSError, ValueError):
        return False


def ffmpeg_argv(source, a):
    base = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin"]
    if source == "camera":
        inp = ["-f", "v4l2", "-input_format", "yuyv422", "-video_size", "%dx%d" % (W, H), "-framerate", "30",
               "-i", a.device]
    elif source == "relay":
        inp = ["-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "500000", "-analyzeduration", "1000000",
               "-i", "udp://%s?overrun_nonfatal=1&fifo_size=500000" % a.relay]
    else:                                          # file (tests)
        inp = ["-re", "-stream_loop", "-1", "-i", a.file]
    vf = "fps=%g,scale=%d:%d" % (a.fps, W, H)
    return base + inp + ["-an", "-vf", vf, "-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1"]


class Feed:
    def __init__(self, a):
        self.a = a
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.t_frame = None, 0, 0.0
        self.source, self.error, self.proc = None, None, None
        self.clients, self.fps, self.overlay = 0, 0.0, not a.no_overlay
        self.det, self.det_err = None, None
        self.raw, self.raw_seq = None, 0
        self.last_raw = None                             # newest frame that was processed (for /raw.jpg)
        self.zone = None                                 # placement guide {"poly", "point", "label"} or None
        self.okra_hist = deque(maxlen=10)                # (robot time, best accepted conf or 0) per processed frame
        self.raw_lock = threading.Lock()
        self.stopping = False

    # ---------------------------------------------------------------- detector (lazy, optional)
    def detector(self):
        if self.det is None and self.det_err is None:
            try:
                from okra_detector import OkraDetector
                self.det = OkraDetector(self.a.weights, stream=True, conf=OVERLAY_CONF)
                print("overlay detector %s" % self.det.weights.name, flush=True)
            except Exception as e:  # noqa: BLE001 - the feed still works without the overlay
                self.det_err = "detector unavailable: %s" % e
                print(self.det_err, flush=True)
        return self.det

    # ---------------------------------------------------------------- source control
    def want(self):
        if self.a.file:
            return "file" if not held() else "paused"
        if recorder_running(self.a.recorder_pid):
            return "relay"
        return "paused" if held() else "camera"

    def start_proc(self, source):
        self.proc = subprocess.Popen(ffmpeg_argv(source, self.a), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     stdin=subprocess.DEVNULL, bufsize=0)
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()

    def stop_proc(self):
        p, self.proc = self.proc, None
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(2)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()

    def _reader(self, p):
        """Read raw frames as fast as ffmpeg makes them; keep only the newest (no backlog = no lag)."""
        n = W * H * 3
        while True:
            buf = b""
            while len(buf) < n:
                chunk = p.stdout.read(n - len(buf))
                if not chunk:
                    err = p.stderr.read().decode(errors="replace").strip().splitlines()
                    if p is self.proc:                  # not a stop we asked for
                        self.error = (err[-1] if err else "source ended (exit %s)" % p.poll())
                    return
                buf += chunk
            with self.raw_lock:
                self.raw, self.raw_seq = np.frombuffer(buf, np.uint8).reshape(H, W, 3), self.raw_seq + 1
                self.t_raw = time.time()

    def control_loop(self):
        backoff = 0.0
        while not self.stopping:
            w = self.want()
            if w != self.source or (self.proc is None and w != "paused" and time.time() > backoff):
                if w != self.source:
                    print("source %s -> %s%s" % (self.source, w, " (%s)" % ",".join(held()) if w == "paused" else ""),
                          flush=True)
                self.stop_proc()
                self.source = w
                if w != "paused":
                    self.error = None
                    self.start_proc(w)
            if self.proc is not None and self.proc.poll() is not None:
                self.proc = None                    # died (camera busy, relay gone, ...): retry soon
                backoff = time.time() + 1.0
            try:
                json.dump({"source": self.source, "t": time.time()}, open(STATE_FILE + ".tmp", "w"))
                os.replace(STATE_FILE + ".tmp", STATE_FILE)
            except OSError:
                pass
            time.sleep(0.2)
        self.stop_proc()

    # ---------------------------------------------------------------- frame production
    def publish(self, img, t):
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.a.quality])
        if not ok:
            return
        with self.cond:
            self.jpeg, self.t_frame = jpg.tobytes(), t
            self.seq += 1
            self.cond.notify_all()

    def placeholder(self, text):
        img = np.full((H, W, 3), 40, np.uint8)
        cv2.putText(img, text, (20, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        cv2.putText(img, time.strftime("robot %H:%M:%S"), (20, H // 2 + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (200, 200, 200), 1)
        return img

    def draw(self, img, t):
        self.last_raw = img.copy()
        if self.overlay and self.source != "relay" and self.detector() is not None:
            try:
                dets = self.det.detect(img, return_rejected=True)
                self.okra_hist.append((t, max([d["conf"] for d in dets if d["accepted"]] or [0.0])))
                acc = [d for d in dets if d["accepted"]]
                self.okra_px = ({"cx": round(float(acc[0]["cx"]), 1), "cy": round(float(acc[0]["cy"]), 1), "conf": round(acc[0]["conf"], 3),
                                 "t": t} if acc else None)             # best accepted pod in the newest frame
                for d in dets:
                    col = (0, 200, 0) if d["accepted"] else (0, 165, 255)
                    cv2.polylines(img, [np.asarray(d["mask"], np.int32)], True, col, 2 if d["accepted"] else 1)
                    x1, y1 = int(d["box"][0]), int(d["box"][1])
                    cv2.putText(img, "%.2f" % d["conf"], (x1, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            except Exception as e:  # noqa: BLE001 - never lose the feed because of the overlay
                self.det_err = "overlay error: %s" % e
        z = self.zone
        if z:
            cv2.polylines(img, [np.asarray(z["poly"], np.int32)], True, (255, 255, 0), 2)
            if z.get("point"):
                u, v = int(z["point"][0]), int(z["point"][1])
                cv2.drawMarker(img, (u, v), (255, 255, 0), cv2.MARKER_CROSS, 24, 2)
            if z.get("label"):
                x0, y0 = np.min(np.asarray(z["poly"]), 0).astype(int)
                cv2.putText(img, z["label"], (max(4, x0), max(16, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
                cv2.putText(img, z["label"], (max(4, x0), max(16, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        label = "%s %s.%d" % (self.source, time.strftime("%H:%M:%S", time.localtime(t)), int(10 * (t % 1)))
        cv2.putText(img, label, (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, label, (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img

    def produce_loop(self):
        last, t_rate, n_rate, t_ph = 0, time.time(), 0, 0.0
        while not self.stopping:
            with self.raw_lock:
                raw, seq, t = self.raw, self.raw_seq, getattr(self, "t_raw", 0.0)
            if seq != last and raw is not None and self.source != "paused":
                last = seq
                self.publish(self.draw(raw.copy(), t), t)
                n_rate += 1
            elif time.time() - t_ph > 1.0 and (self.source == "paused" or self.proc is None):
                t_ph = time.time()
                msg = ("camera in use: %s" % ",".join(held()) if self.source == "paused"
                       else "no picture: %s" % (self.error or "starting"))
                self.publish(self.placeholder(msg[:60]), time.time())
            else:
                time.sleep(0.01)
            if time.time() - t_rate >= 2.0:
                self.fps, n_rate, t_rate = n_rate / (time.time() - t_rate), 0, time.time()

    def okra(self, min_conf):
        # last 10 processed frames, whatever the frame rate; none if the newest is stale (feed paused or stuck)
        h = [c for t, c in self.okra_hist] if self.okra_hist and time.time() - self.okra_hist[-1][0] < 2.0 else []
        px = getattr(self, "okra_px", None)
        return {"frames": len(h), "hits": sum(c >= min_conf for c in h), "best_conf": round(max(h or [0.0]), 3),
                "px": px if px and time.time() - px["t"] < 2.0 else None,
                "min_conf": min_conf, "detecting": self.overlay and self.source in ("camera", "file")}

    def status(self, min_conf=0.5):
        return {"okra": self.okra(min_conf), "source": self.source, "fps": round(self.fps, 1), "clients": self.clients, "error": self.error,
                "frame_age_s": round(time.time() - self.t_frame, 2) if self.t_frame else None,
                "overlay": self.overlay, "overlay_error": self.det_err, "holds": held(), "robot_time": time.time()}


FEED = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(url.query)
        if url.path == "/status":
            return self._send(json.dumps(FEED.status(float(q.get("min_conf", [0.5])[0]))).encode(), "application/json")
        if url.path == "/control":
            if "overlay" in q:
                FEED.overlay = q["overlay"][0] == "1"
            if "zone" in q:
                FEED.zone = None if q["zone"][0] == "off" else json.loads(q["zone"][0])
            return self._send(json.dumps(FEED.status()).encode(), "application/json")
        if url.path == "/snap.jpg":
            if FEED.jpeg is None:
                return self._send(b"no frame yet", "text/plain", 503)
            return self._send(FEED.jpeg, "image/jpeg")
        if url.path == "/raw.jpg":
            if FEED.last_raw is None:
                return self._send(b"no frame yet", "text/plain", 503)
            return self._send(cv2.imencode(".jpg", FEED.last_raw, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes(), "image/jpeg")
        if url.path == "/live.mjpg":
            return self.stream()
        return self._send(b"not found", "text/plain", 404)

    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        FEED.clients += 1
        seen = -1
        try:
            while True:
                with FEED.cond:
                    FEED.cond.wait_for(lambda: FEED.seq != seen, timeout=5.0)
                    jpg, seen, t = FEED.jpeg, FEED.seq, FEED.t_frame
                if jpg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n"
                                 b"X-Robot-Time: %.3f\r\nX-Source: %s\r\n\r\n" % (len(jpg), t, (FEED.source or "").encode()))
                self.wfile.write(jpg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            FEED.clients -= 1


def main():
    global FEED
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.123.164", help="bind address (robot network only)")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--quality", type=int, default=70, help="JPEG quality (640x480 at 70 is ~40 KB)")
    ap.add_argument("--device", default="/dev/video4", help="head RealSense colour (V4L2), as in g1_record")
    ap.add_argument("--relay", default="127.0.0.1:5601", help="recorder's local preview copy")
    ap.add_argument("--recorder-pid", default=os.path.expanduser("~/g1_rec/recorder.pid"))
    ap.add_argument("--weights", default=None, help="overlay detector weights (default: best in okra_robot/models)")
    ap.add_argument("--no-overlay", action="store_true")
    ap.add_argument("--file", default=None, help="test source: loop a video file instead of the camera")
    a = ap.parse_args()
    FEED = Feed(a)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    threading.Thread(target=FEED.control_loop, daemon=True).start()
    threading.Thread(target=FEED.produce_loop, daemon=True).start()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    print("live feed on http://%s:%d/live.mjpg (%g fps, overlay %s)" % (a.host, a.port, a.fps,
          "off" if a.no_overlay else "on"), flush=True)
    try:
        srv.serve_forever()
    finally:
        FEED.stopping = True
        FEED.stop_proc()


if __name__ == "__main__":
    main()
