#!/usr/bin/env python3
"""Live G1 head-camera view with start/stop recording of clips.

Usage:
    .venv/bin/python record.py                     # clips go to ./recordings/
    .venv/bin/python record.py --out ~/clips       # other folder
    .venv/bin/python record.py --duration 10       # record one 10 s clip right away, save, exit

Keys (in the video window):
    r = start recording / stop and save clip
    s = save snapshot (.jpg)
    q / Esc = quit (a clip still recording is saved first)
  with --head (turns the waist; the G1 has no neck):
    j / l = look left / right     i / k = look up / down     c = center
    The first head key engages arm_sdk; quitting centers and releases it.

Clips are saved as <out>/clip_YYYYmmdd_HHMMSS.mp4 at the camera's measured frame rate,
so playback runs at real speed.
"""

import argparse
import os
import threading
import time

import cv2
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

from waist import WaistController

MIN_PERIOD = 1 / 15  # s between frame requests
HEAD_STEP = 0.1  # rad per key press
HEAD_KEYS = {ord("j"): (HEAD_STEP, 0.0), ord("l"): (-HEAD_STEP, 0.0),
             ord("i"): (0.0, -HEAD_STEP), ord("k"): (0.0, HEAD_STEP)}


def save_clip(jpegs: list[bytes], stamps: list[float], out_dir: str, started: float) -> None:
    if len(jpegs) < 2:
        print("clip too short, nothing saved")
        return
    fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    path = os.path.join(out_dir, time.strftime("clip_%Y%m%d_%H%M%S.mp4", time.localtime(started)))
    first = cv2.imdecode(np.frombuffer(jpegs[0], np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for jpg in jpegs:
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            writer.write(img)
    writer.release()
    print(f"saved {path}  ({len(jpegs)} frames, {stamps[-1] - stamps[0]:.1f} s, {fps:.1f} fps)")


class FrameGrabber(threading.Thread):
    """Fetches JPEG frames from the robot's videohub as fast as it answers."""

    def __init__(self, client: VideoClient) -> None:
        super().__init__(daemon=True)
        self.client = client
        self.cond = threading.Condition()
        self.jpg: bytes | None = None
        self.stamp = 0.0
        self.seq = 0

    def run(self) -> None:
        time.sleep(2.0)  # let DDS discovery finish; early requests time out and pile up
        next_t = time.monotonic()
        while True:
            # Pace requests: back-to-back 1080p replies overflow the default 208 KB UDP buffer
            next_t = max(next_t + MIN_PERIOD, time.monotonic())
            time.sleep(max(0.0, next_t - time.monotonic()))
            code, data = self.client.GetImageSample()
            if code != 0 or not data:
                print(f"GetImageSample failed (code={code}), retrying...")
                time.sleep(0.1)
                continue
            with self.cond:
                self.jpg, self.stamp, self.seq = bytes(data), time.time(), self.seq + 1
                self.cond.notify_all()

    def next(self, last_seq: int) -> tuple[int, bytes, float]:
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq)
            return self.seq, self.jpg, self.stamp


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="enp0s8", help="interface connected to the robot")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings"))
    parser.add_argument("--scale", type=float, default=0.5, help="display scale (frames are 1920x1080)")
    parser.add_argument("--duration", type=float, help="record one clip of this many seconds, then exit")
    parser.add_argument("--head", action="store_true", help="enable j/l/i/k/c waist (head) control")
    args = parser.parse_args()
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    ChannelFactoryInitialize(0, args.iface)
    client = VideoClient()
    client.SetTimeout(1.0)  # a lost reply then costs 1 s, not 3 s
    client.Init()
    grabber = FrameGrabber(client)
    grabber.start()
    waist = WaistController() if args.head else None

    # Frames arrive as JPEG; keep the compressed bytes while recording (~0.2 MB/frame)
    # and encode the mp4 on stop, when the real frame rate is known.
    recording = args.duration is not None
    rec_start = time.time()
    jpegs: list[bytes] = []
    stamps: list[float] = []
    frames, t0, fps, seq = 0, time.time(), 0.0, 0
    print(f"saving clips to {out_dir}   keys: r = record/stop, s = snapshot, q = quit")

    while True:
        seq, jpg, now = grabber.next(seq)
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        if recording:
            jpegs.append(jpg)
            stamps.append(now)

        frames += 1
        if now - t0 >= 1.0:
            fps, frames, t0 = frames / (now - t0), 0, now

        view = cv2.resize(img, None, fx=args.scale, fy=args.scale) if args.scale != 1 else img
        cv2.putText(view, f"{fps:.1f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if recording:
            cv2.circle(view, (view.shape[1] - 30, 25), 10, (0, 0, 255), -1)
            cv2.putText(view, f"REC {now - rec_start:5.1f}s", (view.shape[1] - 170, 33),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if waist is not None and waist.engaged.is_set():
            gy, gp = waist.goal()
            cv2.putText(view, f"head yaw {gy:+.2f} pitch {gp:+.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.imshow("G1 head camera", view)
        key = cv2.waitKey(1) & 0xFF

        if args.duration is not None and now - rec_start >= args.duration:
            key = ord("q")
        if waist is not None and (key in HEAD_KEYS or key == ord("c")):
            if not waist.engage():
                print("no rt/lowstate yet, head control unavailable")
            elif key == ord("c"):
                waist.center()
            else:
                waist.nudge(*HEAD_KEYS[key])
        if key == ord("r"):
            if recording:
                save_clip(jpegs, stamps, out_dir, rec_start)
                jpegs, stamps = [], []
            else:
                rec_start = now
                print("recording... press r again to stop and save")
            recording = not recording
        elif key == ord("s"):
            path = os.path.join(out_dir, time.strftime("snapshot_%Y%m%d_%H%M%S.jpg"))
            cv2.imwrite(path, img)
            print("saved", path)
        elif key in (ord("q"), 27):
            if recording:
                save_clip(jpegs, stamps, out_dir, rec_start)
            break

    cv2.destroyAllWindows()
    if waist is not None and waist.engaged.is_set():
        print("centering waist and releasing arm_sdk...")
        waist.release()
    os._exit(0)  # DDS threads otherwise keep the process alive


if __name__ == "__main__":
    main()
