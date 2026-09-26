"""
Pick a varied subset of frames for labeling (drops near-duplicates from a mostly still robot).

    python select_frames.py raw_session02 to_label --n 150

Greedy: walk the frames in time order and keep one only if its 32x24 grayscale thumbnail
differs from every kept frame by more than --min-diff (mean abs difference, 0-255).
If more than --n remain, take an even spread of them. Also writes contact_sheet.jpg.
"""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np


def thumb(path):
    g = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2GRAY)
    return cv2.resize(g, (32, 24), interpolation=cv2.INTER_AREA).astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--n", type=int, default=150)
    ap.add_argument("--min-diff", type=float, default=6.0)
    ap.add_argument("--prefix", default="")
    args = ap.parse_args()

    frames = sorted(Path(args.src).glob("*.jpg"))
    kept, thumbs = [], []
    for p in frames:
        t = thumb(p)
        if all(np.abs(t - k).mean() > args.min_diff for k in thumbs):
            kept.append(p)
            thumbs.append(t)
    if len(kept) > args.n:
        kept = [kept[i] for i in np.linspace(0, len(kept) - 1, args.n).round().astype(int)]

    dst = Path(args.dst)
    dst.mkdir(parents=True, exist_ok=True)
    for p in kept:
        shutil.copy2(p, dst / (args.prefix + p.name))
    print(f"{len(frames)} frames -> {len(kept)} kept in {dst} (min diff {args.min_diff})")

    tiles = [cv2.resize(cv2.imread(str(p)), (160, 120)) for p in kept[:120]]
    tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 12)
    rows = [np.hstack(tiles[i:i + 12]) for i in range(0, len(tiles), 12)]
    cv2.imwrite(str(dst.parent / f"contact_sheet_{dst.name}.jpg"), np.vstack(rows))


if __name__ == "__main__":
    main()
