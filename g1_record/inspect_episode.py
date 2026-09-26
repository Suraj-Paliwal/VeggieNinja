#!/usr/bin/env python3
"""Summarize a pulled episode: frame counts/rates/gaps and state-topic rates, all on the robot clock.

Usage (VM):  ../g1_video/.venv/bin/python inspect_episode.py data/<episode>
Also importable: load_state(ep, "lowstate") -> dict of concatenated arrays, load_ts(ep, "color") -> epoch s.
"""

import glob
import json
import os
import sys

import numpy as np


def load_ts(ep, stream):
    """Per-frame timestamps (epoch seconds) from <stream>_ts.txt (mkvtimestamp_v2, ms)."""
    path = os.path.join(ep, stream + "_ts.txt")
    if not os.path.exists(path):
        return None
    vals = [float(l) for l in open(path) if l.strip() and not l.startswith("#")]
    return np.asarray(vals) / 1000.0


def load_state(ep, name):
    """Concatenate state/<name>_NNNN.npz chunks. Returns {} if the topic was never received."""
    files = sorted(glob.glob(os.path.join(ep, "state", name + "_*.npz")))
    if not files:
        return {}
    parts = [np.load(f) for f in files]
    return {k: np.concatenate([p[k] for p in parts]) for k in parts[0].files}


def rate_line(t):
    if t is None or len(t) < 2:
        return "n=%d" % (0 if t is None else len(t))
    d = np.diff(t)
    return "n=%d  %.1f Hz  span %.1f s  max gap %.0f ms" % (
        len(t), (len(t) - 1) / (t[-1] - t[0]), t[-1] - t[0], d.max() * 1000)


def main(ep):
    meta = json.load(open(os.path.join(ep, "meta.json")))
    dur = meta.get("stop_time", float("nan")) - meta["start_time"]
    print("episode %s  (%.1f s, args %s)" % (os.path.basename(ep.rstrip("/")), dur, meta["args"]))
    print("ffmpeg exit codes:", meta.get("ffmpeg_exit", "not stopped cleanly"))
    for s in ("color", "depth"):
        t = load_ts(ep, s)
        if t is not None:
            print("  %-18s %s" % (s, rate_line(t)))
    names = sorted({os.path.basename(f).rsplit("_", 1)[0]
                    for f in glob.glob(os.path.join(ep, "state", "*.npz"))})
    for n in names:
        print("  %-18s %s" % (n, rate_line(load_state(ep, n)["t"])))
    missing = set(meta.get("state_rows", {})) - set(names)
    if missing:
        print("  no messages on:", ", ".join(sorted(missing)))


if __name__ == "__main__":
    main(sys.argv[1])
