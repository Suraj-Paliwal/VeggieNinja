#!/usr/bin/env python3
"""
Build a YOLO-segmentation training set from human-confirmed events in okra_data/events.

    python export_dataset.py                    # -> okra_data/datasets/hitl_vNNN/  (next free NNN)
    python export_dataset.py --include-auto     # also use auto-accepted detections (pseudo-labels)

Per event (last frame of the look, the frame the operator saw):
  operator said "these are okra"  -> image + one polygon per confirmed candidate
  operator said "none is okra"    -> image with an empty label file (hard negative)
  operator said "missed one" (m)  -> NOT exported; image copied to okra_data/to_annotate/ for CVAT
  "not sure" / aborted            -> skipped
Split: ~20 % of events go to val (stable by event id). Writes data.yaml, manifest.csv and README.md.
"""

import argparse
import csv
import hashlib
import os
import shutil
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import store  # noqa: E402


def next_version(root):
    os.makedirs(root, exist_ok=True)
    n = [int(d.split("_v")[1]) for d in os.listdir(root) if d.startswith("hitl_v") and d.split("_v")[1].isdigit()]
    return os.path.join(root, "hitl_v%03d" % (max(n) + 1 if n else 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--include-auto", action="store_true")
    ap.add_argument("--out")
    args = ap.parse_args()
    out = args.out or next_version(os.path.join(store.ROOT, "datasets"))
    todo = os.path.join(store.ROOT, "to_annotate")
    rows, stats = [], {"positive": 0, "negative": 0, "polygons": 0, "missed": 0, "skipped": 0}
    for ev in store.all_events():
        dec, cj = store.load(ev, "decision.json"), store.load(ev, "candidates.json")
        eid = os.path.basename(ev)
        frames = sorted(os.listdir(os.path.join(ev, "frames"))) if os.path.isdir(os.path.join(ev, "frames")) else []
        if not dec or not cj or not frames or dec.get("aborted") or dec.get("unsure"):
            stats["skipped"] += 1
            continue
        img_path = os.path.join(ev, "frames", frames[-1])
        if dec.get("missed"):
            os.makedirs(todo, exist_ok=True)
            shutil.copy2(img_path, os.path.join(todo, eid + ".jpg"))
            stats["missed"] += 1
            continue
        okra = [k for k, v in dec["labels"].items() if v == "okra" or (args.include_auto and v == "okra_auto")]
        if not dec["labels"] or (not okra and any(v == "okra_auto" for v in dec["labels"].values())):
            stats["skipped"] += 1                       # auto-accepted but not included
            continue
        split = "val" if int(hashlib.md5(eid.encode()).hexdigest(), 16) % 5 == 0 else "train"
        for sub in ("images", "labels"):
            os.makedirs(os.path.join(out, sub, split), exist_ok=True)
        h, w = cv2.imread(img_path).shape[:2]
        byid = {str(c["id"]): c for c in cj["candidates"]}
        lines = []
        for k in okra:
            pts = byid[k]["key_mask"]
            lines.append("0 " + " ".join("%.6f %.6f" % (x / w, y / h) for x, y in pts))
        shutil.copy2(img_path, os.path.join(out, "images", split, eid + ".jpg"))
        with open(os.path.join(out, "labels", split, eid + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        stats["positive" if lines else "negative"] += 1
        stats["polygons"] += len(lines)
        rows.append({"event_id": eid, "split": split, "n_okra": len(lines),
                     "source": "auto" if any(dec["labels"][k] == "okra_auto" for k in okra) else "human",
                     "operator": dec.get("operator") or "", "detector": cj.get("detector", "")})
    if not rows:
        print("nothing to export yet: %s" % stats)
        return
    with open(os.path.join(out, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: images/train\nval: images/val\nnames:\n  0: okra\n" % os.path.abspath(out))
    with open(os.path.join(out, "manifest.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    with open(os.path.join(out, "README.md"), "w") as f:
        f.write("# %s\n\nBuilt by okra_pick/hitl/export_dataset.py from okra_data/events.\n\n%s\n" % (
            os.path.basename(out), "\n".join("- %s: %s" % kv for kv in stats.items())))
    print("dataset %s: %s" % (out, stats))
    if stats["missed"]:
        print("%d image(s) with missed okra in %s -> label them in CVAT (dataset/cvat_upload.py)" % (stats["missed"], todo))


if __name__ == "__main__":
    main()
