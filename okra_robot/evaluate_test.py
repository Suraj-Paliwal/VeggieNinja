#!/usr/bin/env python3
"""
Base vs fine-tuned okra model on the HELD-OUT TEST split only (never used for training or checkpoint choice).

    python evaluate_test.py --data dataset/yolo_s02_3way --tuned models/okra_seg_s02_3way.pt --out report

Writes report/*.png, report/metrics.json and report/REPORT.md.
Matching: a prediction is correct if its MASK overlaps a labelled pod with IoU >= 0.5 (0.3 for the full
detector, whose polygons are simplified); each labelled pod can be matched once (highest confidence first).
"""

import argparse
import glob
import json
import os

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from ultralytics import YOLO  # noqa: E402

import okra_filter as F  # noqa: E402
from okra_detector import OkraDetector  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
W, H = 640, 480
# reference palette (dataviz skill), categorical slots 2 and 1; text tokens
C_BASE, C_TUNED = "#eb6834", "#2a78d6"
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "axes.edgecolor": GRID,
                     "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2, "text.color": INK,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
                     "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold",
                     "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False})


def mask_of(poly):
    m = np.zeros((H, W), np.uint8)
    cv2.fillPoly(m, [np.asarray(poly, np.float32).round().astype(np.int32)], 1)
    return m.astype(bool)


def iou(a, b):
    u = (a | b).sum()
    return (a & b).sum() / u if u else 0.0


def load_test(d):
    items = []
    for img in sorted(glob.glob(os.path.join(d, "images", "test", "*.jpg"))):
        lab = img.replace("/images/", "/labels/").rsplit(".", 1)[0] + ".txt"
        gts = [np.array(l.split()[1:], float).reshape(-1, 2) * [W, H] for l in open(lab) if l.strip()]
        items.append((img, gts))
    return items


def raw_predictions(weights, items):
    """Every prediction down to conf 0.001: list per image of (conf, mask)."""
    m = YOLO(weights)
    out = []
    for img, _ in items:
        r = m.predict(img, imgsz=640, conf=0.001, verbose=False)[0]
        preds = []
        if r.masks is not None:
            for poly, c in zip(r.masks.xy, r.boxes.conf.tolist()):
                if len(poly) >= 3:
                    preds.append((float(c), mask_of(poly)))
        out.append(preds)
    return out


def match(preds, gts, thr):
    """Greedy by confidence. Returns list of (conf, is_tp) and number of GT."""
    gm = [mask_of(g) for g in gts]
    used, res = set(), []
    for c, pm in sorted(preds, key=lambda x: -x[0]):
        best, bi = 0.0, None
        for i, g in enumerate(gm):
            if i in used:
                continue
            v = iou(pm, g)
            if v > best:
                best, bi = v, i
        if bi is not None and best >= thr:
            used.add(bi)
            res.append((c, True))
        else:
            res.append((c, False))
    return res, len(gts)


def pr_curve(all_res, n_gt):
    all_res = sorted(all_res, key=lambda x: -x[0])
    tp = np.cumsum([r[1] for r in all_res])
    fp = np.cumsum([not r[1] for r in all_res])
    rec = tp / max(1, n_gt)
    prec = tp / np.maximum(1, tp + fp)
    # all-point interpolated AP
    mrec = np.r_[0, rec, 1]
    mpre = np.r_[1, prec, 0]
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    ap = float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))
    return rec, prec, ap, np.array([r[0] for r in all_res])


def at_threshold(all_res, n_gt, conf=0.25):
    kept = [r for r in all_res if r[0] >= conf]
    tp = sum(r[1] for r in kept)
    return tp / max(1, len(kept)), tp / max(1, n_gt), tp, len(kept) - tp, n_gt - tp


def full_detector(weights, items, hue):
    F.GREEN_HUE = hue
    det = OkraDetector(weights=weights, stream=False)
    tp = fp = fn = 0
    per_image, reasons = [], {}
    for img, gts in items:
        res = det.detect(cv2.imread(img), return_rejected=True)
        for d in res:
            if not d["accepted"] and d["conf"] >= 0.25:
                k = d["reason"].split()[0]
                reasons[k] = reasons.get(k, 0) + 1
        res = [d for d in res if d["accepted"]]
        preds = [(d["conf"], mask_of(d["mask"])) for d in res]
        m, n = match(preds, gts, 0.3)
        t = sum(x[1] for x in m)
        tp, fp, fn = tp + t, fp + len(m) - t, fn + n - t
        per_image.append((len(gts), t, len(m) - t))
    return {"recall": tp / max(1, tp + fn), "precision": tp / max(1, tp + fp), "tp": tp, "fp": fp, "fn": fn,
            "filter_rejections_conf_ge_0.25": reasons, "per_image": per_image}


def official(weights, data_yaml):
    r = YOLO(weights).val(data=data_yaml, split="test", imgsz=640, batch=8, device="cpu", verbose=False,
                          plots=False, project=os.path.join(HERE, "runs"), name="eval_test", exist_ok=True)
    return {"mask_mAP50": r.seg.map50, "mask_mAP50_95": r.seg.map, "box_mAP50": r.box.map50, "box_mAP50_95": r.box.map}


def label_bars(ax, bars, fmt="{:.2f}"):
    for b in bars:
        ax.annotate(fmt.format(b.get_height()), (b.get_x() + b.get_width() / 2, b.get_height()),
                    xytext=(0, 3), textcoords="offset points", ha="center", va="bottom", fontsize=10, color=INK)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.join(HERE, "dataset", "yolo_s02_3way"))
    ap.add_argument("--base", default=os.path.join(HERE, "models", "okra_seg.pt"))
    ap.add_argument("--tuned", default=os.path.join(HERE, "models", "okra_seg_s02_3way.pt"))
    ap.add_argument("--out", default=os.path.join(HERE, "report"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    items = load_test(args.data)
    n_gt = sum(len(g) for _, g in items)
    print("test split: %d images, %d labelled pods" % (len(items), n_gt))

    M = {}
    for key, w in (("base", args.base), ("tuned", args.tuned)):
        preds = raw_predictions(w, items)
        allr = []
        for p, (_, g) in zip(preds, items):
            allr += match(p, g, 0.5)[0]
        rec, prec, ap50, confs = pr_curve(allr, n_gt)
        P, R, tp, fp, fn = at_threshold(allr, n_gt, 0.25)
        M[key] = {"official": official(w, os.path.join(args.data, "data.yaml")), "mask_AP50_own": ap50,
                  "precision@0.25": P, "recall@0.25": R, "tp@0.25": tp, "fp@0.25": fp, "fn@0.25": fn,
                  "pr": (rec.tolist(), prec.tolist()),
                  "conf_tp": [c for c, t in allr if t], "conf_fp": [c for c, t in allr if not t and c >= 0.05],
                  "full_old_filter": full_detector(w, items, (30, 90)),
                  "full_new_filter": full_detector(w, items, (30, 105))}
        print("%-6s mask mAP50 %.3f | P %.2f R %.2f @0.25 | full detector (new filter) R %.2f P %.2f" % (
            key, M[key]["official"]["mask_mAP50"], P, R, M[key]["full_new_filter"]["recall"], M[key]["full_new_filter"]["precision"]))
        print("       filter rejections (conf >= 0.25), new filter: %s" % M[key]["full_new_filter"]["filter_rejections_conf_ge_0.25"])

    # ---------- 1 headline bars
    names = ["mask mAP50", "box mAP50", "mask mAP50-95", "precision\n(conf ≥ 0.25)", "recall\n(conf ≥ 0.25)"]
    vals = {k: [M[k]["official"]["mask_mAP50"], M[k]["official"]["box_mAP50"], M[k]["official"]["mask_mAP50_95"],
                M[k]["precision@0.25"], M[k]["recall@0.25"]] for k in M}
    fig, ax = plt.subplots(figsize=(10, 4.6))
    x = np.arange(len(names))
    b1 = ax.bar(x - 0.2, vals["base"], 0.38, color=C_BASE, label="base model (public checkpoint)")
    b2 = ax.bar(x + 0.2, vals["tuned"], 0.38, color=C_TUNED, label="fine-tuned on robot-camera frames")
    label_bars(ax, b1), label_bars(ax, b2)
    ax.set_xticks(x, names), ax.set_ylim(0, 1.1), ax.set_ylabel("score (0-1, higher is better)")
    ax.set_title("Okra detection on %d held-out test images (%d pods) — never seen in training" % (len(items), n_gt),
                 loc="left", pad=30)
    ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2)
    fig.tight_layout(), fig.savefig(os.path.join(args.out, "1_headline_metrics.png"), dpi=150), plt.close(fig)

    # ---------- 2 PR curves
    fig, ax = plt.subplots(figsize=(6.4, 5))
    for k, col, lab in (("base", C_BASE, "base"), ("tuned", C_TUNED, "fine-tuned")):
        r, p = M[k]["pr"]
        ax.plot(r, p, color=col, lw=2, label="%s  (AP50 %.2f)" % (lab, M[k]["mask_AP50_own"]))
    ax.set_xlim(0, 1), ax.set_ylim(0, 1.02), ax.set_xlabel("recall (share of real pods found)"), ax.set_ylabel("precision (share of detections that are pods)")
    ax.set_title("Precision-recall, masks, IoU ≥ 0.5 (test only)", loc="left"), ax.legend(loc="lower left")
    fig.tight_layout(), fig.savefig(os.path.join(args.out, "2_precision_recall.png"), dpi=150), plt.close(fig)

    # ---------- 3 full detector (model + sanity filter), before/after the colour fix
    cfg = [("base", "full_old_filter", "base model\n+ original filter"), ("tuned", "full_old_filter", "fine-tuned\n+ original filter"),
           ("tuned", "full_new_filter", "fine-tuned\n+ colour-fixed filter")]
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.4), sharey=True)
    for ax, metric in zip(axs, ("recall", "precision")):
        bars = ax.bar(range(len(cfg)), [M[k][f][metric] for k, f, _ in cfg], 0.6,
                      color=[C_BASE if k == "base" else C_TUNED for k, _, _ in cfg])
        label_bars(ax, bars)
        ax.set_xticks(range(len(cfg)), [c[2] for c in cfg]), ax.set_ylim(0, 1.12)
        ax.set_title(("Recall: real pods found" if metric == "recall" else "Precision: detections that are pods"), loc="left")
    axs[0].set_ylabel("share (0-1)")
    fig.suptitle("What the robot actually uses: model + sanity filter (test images only)", x=0.01, ha="left", fontweight="bold")
    fig.tight_layout(), fig.savefig(os.path.join(args.out, "3_full_detector.png"), dpi=150), plt.close(fig)

    # ---------- 4 per image: pods found vs labelled
    fig, ax = plt.subplots(figsize=(10, 4.2))
    gt = [g for g, _, _ in M["tuned"]["full_new_filter"]["per_image"]]
    order = np.argsort(gt)[::-1]
    x = np.arange(len(gt))
    ax.bar(x, np.array(gt)[order], 0.8, color="#d9d8d4", label="labelled pods")
    ax.plot(x, np.array([t for _, t, _ in M["base"]["full_old_filter"]["per_image"]])[order], "o", ms=8, color=C_BASE,
            label="found: base + original filter")
    ax.plot(x, np.array([t for _, t, _ in M["tuned"]["full_new_filter"]["per_image"]])[order], "D", ms=8, color=C_TUNED,
            label="found: fine-tuned + colour-fixed filter")
    ax.set_xticks(x, [str(i + 1) for i in range(len(x))]), ax.set_xlabel("test image (sorted by number of pods)")
    ax.set_ylabel("pods"), ax.set_title("Per test image: labelled pods vs pods correctly found", loc="left")
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax.legend(loc="upper right")
    fig.tight_layout(), fig.savefig(os.path.join(args.out, "4_per_image.png"), dpi=150), plt.close(fig)

    # ---------- 5 confidence of correct vs wrong detections
    fig, axs = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    bins = np.linspace(0, 1, 21)
    for ax, k, lab in ((axs[0], "base", "base model"), (axs[1], "tuned", "fine-tuned model")):
        col = C_BASE if k == "base" else C_TUNED
        ax.hist(M[k]["conf_tp"], bins=bins, color=col, label="correct (matches a pod)")
        ax.hist(M[k]["conf_fp"], bins=bins, histtype="step", lw=2, color=INK2, label="wrong (conf ≥ 0.05)")
        ax.axvline(0.60, color=INK, lw=1, ls="--")
        ax.text(0.59, ax.get_ylim()[1] * 0.55, "HITL baseline 0.60\n(below: robot asks)", fontsize=9, color=INK, ha="right")
        ax.set_title(lab, loc="left"), ax.set_xlabel("model confidence"), ax.legend(loc="upper left", fontsize=9)
    axs[0].set_ylabel("detections")
    fig.suptitle("Confidence: correct vs wrong detections (test only)", x=0.01, ha="left", fontweight="bold")
    fig.tight_layout(), fig.savefig(os.path.join(args.out, "5_confidence.png"), dpi=150), plt.close(fig)

    # ---------- 6 examples: labels | base | fine-tuned
    picks = [items[i] for i in np.linspace(0, len(items) - 1, 4).astype(int)]
    rows = []
    dets = {k: OkraDetector(weights=w, stream=False) for k, w in (("base", args.base), ("tuned", args.tuned))}
    for img, gts in picks:
        tiles = []
        for mode in ("gt", "base", "tuned"):
            im = cv2.imread(img)
            if mode == "gt":
                for g in gts:
                    cv2.polylines(im, [g.round().astype(np.int32)], True, (0, 220, 255), 2)
                title = "human labels (%d)" % len(gts)
            else:
                F.GREEN_HUE = (30, 90) if mode == "base" else (30, 105)
                res = [d for d in dets[mode].detect(im.copy()) if d["accepted"]]
                bgr = (52, 104, 235) if mode == "base" else (214, 120, 42)
                for d in res:
                    cv2.polylines(im, [np.asarray(d["mask"], np.int32)], True, bgr, 2)
                title = ("base + original filter" if mode == "base" else "fine-tuned + fixed filter") + " (%d)" % len(res)
            cv2.rectangle(im, (0, 0), (W, 34), (251, 252, 252), -1)
            cv2.putText(im, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (11, 11, 11), 2)
            tiles.append(cv2.resize(im, (480, 360)))
        rows.append(np.hstack(tiles))
    cv2.imwrite(os.path.join(args.out, "6_examples.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])

    slim = {k: {kk: vv for kk, vv in v.items() if kk not in ("pr", "conf_tp", "conf_fp")} for k, v in M.items()}
    for k in slim:
        for f in ("full_old_filter", "full_new_filter"):
            slim[k][f] = {kk: vv for kk, vv in slim[k][f].items() if kk != "per_image"}
    json.dump({"test_images": len(items), "test_pods": n_gt, "models": {"base": args.base, "tuned": args.tuned}, "metrics": slim},
              open(os.path.join(args.out, "metrics.json"), "w"), indent=1)
    print("plots + metrics in", args.out)


if __name__ == "__main__":
    main()
