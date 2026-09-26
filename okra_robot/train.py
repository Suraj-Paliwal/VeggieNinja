#!/usr/bin/env python3
"""
Fine-tune the okra segmentation model on labelled robot-camera frames (from CVAT / HITL exports).

    python train.py --data dataset/yolo_s02/data.yaml --name s02            # full run (CPU ok, slow)
    python train.py --data ... --epochs 1 --name timing                      # quick timing check

Starts from models/okra_seg.pt (never overwritten). Result: models/okra_seg_<name>.pt plus
runs/<name>/ (curves, val metrics) and runs/<name>/compare.txt (base vs fine-tuned on the val split).
To use it on the robot: OkraDetector(weights="models/okra_seg_<name>.pt"), or rename it to okra_seg.pt
and re-export ONNX/TensorRT (export_model.py) on the robot.
"""

import argparse
import os
import shutil
import time

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))


def metrics(model, data, device, name):
    r = model.val(data=data, imgsz=640, batch=8, device=device, project=os.path.join(HERE, "runs"),
                  name=name, exist_ok=True, verbose=False, plots=False)
    return {"box_mAP50": r.box.map50, "box_mAP50_95": r.box.map, "mask_mAP50": r.seg.map50,
            "mask_mAP50_95": r.seg.map, "precision": r.box.mp, "recall": r.box.mr}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--base", default=os.path.join(HERE, "models", "okra_seg.pt"))
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--patience", type=int, default=20)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--freeze", type=int, default=0, help="freeze the first N layers (faster on CPU)")
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    t0 = time.time()
    model = YOLO(args.base)
    model.train(data=args.data, epochs=args.epochs, patience=args.patience, batch=args.batch, imgsz=args.imgsz,
                device=args.device, workers=4, freeze=args.freeze or None, project=os.path.join(HERE, "runs"),
                name=args.name, exist_ok=True, plots=True, verbose=False,
                degrees=5.0, fliplr=0.5, flipud=0.0, mosaic=1.0, close_mosaic=10, seed=0)
    best = os.path.join(HERE, "runs", args.name, "weights", "best.pt")
    out = os.path.join(HERE, "models", "okra_seg_%s.pt" % args.name)
    shutil.copy2(best, out)
    base_m = metrics(YOLO(args.base), args.data, args.device, args.name + "_val_base")
    ft_m = metrics(YOLO(out), args.data, args.device, args.name + "_val_ft")
    lines = ["%-15s %8s %8s" % ("metric (val)", "base", "tuned")]
    lines += ["%-15s %8.3f %8.3f" % (k, base_m[k], ft_m[k]) for k in base_m]
    lines.append("training time %.0f min, model %s" % ((time.time() - t0) / 60, out))
    open(os.path.join(HERE, "runs", args.name, "compare.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
