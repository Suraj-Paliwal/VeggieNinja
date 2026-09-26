#!/usr/bin/env python3
"""Live view of the Unitree G1 head camera (RealSense color) via the onboard videohub service.

Usage:
    .venv/bin/python view.py                  # show window
    .venv/bin/python view.py --iface enp0s8   # network interface on 192.168.123.x
    .venv/bin/python view.py --save out.mp4   # also record to file

Keys: q / Esc = quit, s = save snapshot.
"""

import argparse
import time

import cv2
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="enp0s8", help="interface connected to the robot")
    parser.add_argument("--scale", type=float, default=0.5, help="display scale (frames are 1920x1080)")
    parser.add_argument("--save", help="optional output video path, e.g. out.mp4")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    writer = None
    frames, t0, fps = 0, time.time(), 0.0
    while True:
        code, data = client.GetImageSample()
        if code != 0 or not data:
            print(f"GetImageSample failed (code={code}), retrying...")
            time.sleep(0.5)
            continue

        img = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue

        if args.save and writer is None:
            h, w = img.shape[:2]
            writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
        if writer is not None:
            writer.write(img)

        frames += 1
        if time.time() - t0 >= 1.0:
            fps, frames, t0 = frames / (time.time() - t0), 0, time.time()

        view = cv2.resize(img, None, fx=args.scale, fy=args.scale) if args.scale != 1 else img
        cv2.putText(view, f"{fps:.1f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("G1 head camera", view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            name = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
            cv2.imwrite(name, img)
            print("saved", name)

    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
