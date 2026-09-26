"""
Convert models/okra_seg.pt to a faster format. Run this ON THE ROBOT: a TensorRT
engine only works on the exact GPU and TensorRT version it was built on.

    python export_model.py engine   # NVIDIA Jetson (Orin etc.) -> models/okra_seg.engine
    python export_model.py onnx     # any CPU                    -> models/okra_seg.onnx

OkraDetector automatically prefers .engine, then .onnx, then .pt.
"""

import sys
from pathlib import Path

from ultralytics import YOLO

fmt = sys.argv[1] if len(sys.argv) > 1 else "onnx"
pt = Path(__file__).parent / "models" / "okra_seg.pt"

model = YOLO(str(pt))
if fmt == "engine":
    out = model.export(format="engine", imgsz=640, half=True, device=0)
elif fmt == "onnx":
    out = model.export(format="onnx", imgsz=640, simplify=True)
else:
    raise SystemExit("usage: python export_model.py [engine|onnx]")
print(f"Exported: {out}")
