"""
Live okra detection from a camera (or a video file) using OkraDetector.

    python run_camera.py                  # camera 0, prints okra positions
    python run_camera.py --source 1       # another camera index
    python run_camera.py --source /dev/video4
    python run_camera.py --source clip.mp4 --save out.mp4
    python run_camera.py --show           # open a preview window (needs a display)

Prints one line per frame that has okra:
    frame 120  okra 1: centre (312, 208) px  conf 0.81
"""

import argparse
import time

import cv2

from okra_detector import OkraDetector


def draw(frame, results):
    for d in results:
        x1, y1, x2, y2 = map(int, d["box"])
        if d["accepted"]:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.circle(frame, (int(d["cx"]), int(d["cy"])), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"okra {d['conf']:.2f}", (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (150, 150, 150), 1)
            cv2.putText(frame, d["reason"], (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="camera index, device path or video file")
    ap.add_argument("--weights", default=None, help="model file (default: best one in models/)")
    ap.add_argument("--show", action="store_true", help="show a preview window")
    ap.add_argument("--save", default=None, help="save annotated video to this path")
    ap.add_argument("--debug", action="store_true", help="also draw rejected detections")
    args = ap.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"Could not open source {args.source!r}")

    det = OkraDetector(args.weights)
    print(f"Model: {det.weights.name}")

    writer = None
    idx, t0 = 0, time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        results = det.detect(frame, return_rejected=args.debug)
        okra = [d for d in results if d["accepted"]]
        for n, d in enumerate(okra, 1):
            print(f"frame {idx}  okra {n}: centre ({d['cx']:.0f}, {d['cy']:.0f}) px  conf {d['conf']:.2f}")

        if args.show or args.save:
            vis = draw(frame.copy(), results)
            if args.save:
                if writer is None:
                    h, w = vis.shape[:2]
                    fps = cap.get(cv2.CAP_PROP_FPS) or 30
                    writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
                writer.write(vis)
            if args.show:
                cv2.imshow("okra", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        idx += 1

    elapsed = time.time() - t0
    print(f"\n{idx} frames in {elapsed:.1f}s ({idx / max(elapsed, 1e-6):.1f} FPS)")
    if writer:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
