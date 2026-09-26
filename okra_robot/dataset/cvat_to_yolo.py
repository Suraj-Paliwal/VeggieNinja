#!/usr/bin/env python3
"""
Pull polygon labels from CVAT (org Junction) and write a YOLO-segmentation dataset.

    python cvat_to_yolo.py --task 2623495 --images s02_label/images --out yolo_s02

- Uses SHAPES only (per-frame polygons). Track polygons are taken only on their own keyframes
  (CVAT would otherwise carry them onto every later frame).
- Removes duplicate polygons (same pod outlined twice: centres within 4 px), keeping the larger.
- Frames without polygons are kept as negatives (no okra).
- Split by TIME, not at random: frames are sorted by recording time and cut into 10 blocks.
  default:      blocks 3, 8 -> val (20 %), rest -> train
  --three-way:  blocks 1, 6 -> TEST (never used for training or checkpoint choice), block 8 -> val,
                rest -> train, minus BUFFER frames on each side of a test block (near-copies of test frames).
  Neighbouring frames are near-copies, so a random split would leak.
Reads CVAT_HOST / CVAT_TOKEN from ../.env.
"""

import argparse
import os
import re
import shutil

import numpy as np
import requests

HERE = os.path.dirname(os.path.abspath(__file__))


def env():
    e = {}
    for line in open(os.path.join(HERE, "..", ".env")).read().splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            k, v = line.split("=", 1)
            e[k.strip()] = v.strip()
    return e["CVAT_HOST"].rstrip("/"), e["CVAT_TOKEN"]


def frame_time(name):
    """Seconds into session02 for the frame naming used by select_frames.py (p/n/m prefixes)."""
    m = re.search(r"_([pnm])_(\d+)", name)
    kind, i = m.group(1), int(m.group(2))
    return {"n": i - 1.0, "p": 480 + (i - 1) / 2.0, "m": 1330 + i - 1.0}[kind]


def area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", type=int, default=2623495)
    ap.add_argument("--images", default=os.path.join(HERE, "s02_label", "images"))
    ap.add_argument("--out", default=os.path.join(HERE, "yolo_s02"))
    ap.add_argument("--org", default="Junction")
    ap.add_argument("--three-way", action="store_true", help="train / val / held-out test")
    ap.add_argument("--buffer", type=int, default=2, help="frames dropped next to each test block (three-way)")
    args = ap.parse_args()

    host, tok = env()
    h, P = {"Authorization": "Bearer " + tok}, {"org": args.org}
    ann = requests.get("%s/api/tasks/%d/annotations" % (host, args.task), headers=h, params=P, timeout=60).json()
    frames = [f["name"] for f in requests.get("%s/api/tasks/%d/data/meta" % (host, args.task), headers=h,
                                              params=P, timeout=30).json()["frames"]]
    polys = {i: [] for i in range(len(frames))}
    for s in ann["shapes"]:
        if s["type"] == "polygon":
            polys[s["frame"]].append(np.array(s["points"]).reshape(-1, 2))
    for t in ann["tracks"]:
        for s in t["shapes"]:
            if not s["outside"] and s["type"] == "polygon":
                polys[s["frame"]].append(np.array(s["points"]).reshape(-1, 2))
    dups = 0
    for f, ps in polys.items():
        ps.sort(key=area, reverse=True)
        keep = []
        for p in ps:
            if all(np.abs(p.mean(0) - k.mean(0)).max() >= 4 for k in keep):
                keep.append(p)
            else:
                dups += 1
        polys[f] = keep

    order = sorted(range(len(frames)), key=lambda i: frame_time(frames[i]))
    blocks = np.array_split(order, 10)
    split_of = {}
    if args.three_way:
        for bi, b in enumerate(blocks):
            for i in b:
                split_of[int(i)] = "test" if bi in (1, 6) else "val" if bi == 8 else "train"
        pos = {int(i): k for k, i in enumerate(order)}
        test_pos = sorted(pos[i] for i, sp in split_of.items() if sp == "test")
        for i, sp in list(split_of.items()):
            if sp != "test" and any(abs(pos[i] - t) <= args.buffer for t in test_pos):
                split_of[i] = "dropped"
    else:
        for bi, b in enumerate(blocks):
            for i in b:
                split_of[int(i)] = "val" if bi in (3, 8) else "train"
    if os.path.exists(args.out):
        shutil.rmtree(args.out)
    counts = {sp: [0, 0, 0] for sp in ("train", "val", "test")}   # images, with okra, polygons
    dropped = 0
    for i, name in enumerate(frames):
        split = split_of[i]
        if split == "dropped":
            dropped += 1
            continue
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(args.out, sub, split), exist_ok=True)
        shutil.copy2(os.path.join(args.images, name), os.path.join(args.out, "images", split, name))
        lines = ["0 " + " ".join("%.6f %.6f" % (x / 640.0, y / 480.0) for x, y in p) for p in polys[i]]
        with open(os.path.join(args.out, "labels", split, os.path.splitext(name)[0] + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        c = counts[split]
        c[0] += 1
        c[1] += bool(lines)
        c[2] += len(lines)
    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\n%snames:\n  0: okra\n" % (
            os.path.abspath(args.out), "test: images/test\n" if args.three_way else ""))
    for sp, (n, npos, npoly) in counts.items():
        if n:
            print("%-5s %3d images (%d with okra, %d without), %d polygons" % (sp, n, npos, n - npos, npoly))
    if dropped:
        print("dropped %d buffer frames next to the test blocks" % dropped)
    print("removed %d duplicate polygons; dataset: %s" % (dups, args.out))


if __name__ == "__main__":
    main()
