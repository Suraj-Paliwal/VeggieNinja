"""
Okra detector for the robot: segmentation model + sanity filter in one class.

    from okra_detector import OkraDetector
    det = OkraDetector()                      # picks the best model file in models/
    okra = det.detect(frame_bgr)              # list of accepted okra, best first
    okra = det.detect(frame_bgr, depth_m=depth_image, intrinsics=(fx, fy, ppx, ppy))

Each result is a dict:
    cx, cy        mask centre in pixels (where the okra is in the frame)
    box           (x1, y1, x2, y2) in pixels
    conf          model confidence
    mask          Nx2 polygon outline in pixels
    depth_m       median depth inside the mask (only when a depth image is given)
    length_m      estimated pod length in metres (only with depth + intrinsics)
    xyz_m         3D point in the camera frame, metres (only with depth + intrinsics)

Pass `return_rejected=True` to also get the detections the filter threw out
(each with "accepted": False and a "reason"), which helps when tuning.
"""

from pathlib import Path

import numpy as np
from ultralytics import YOLO

from okra_filter import OkraFilter, check_detection

MODELS_DIR = Path(__file__).parent / "models"
# fastest first; .engine only exists after running export_model.py on the robot
MODEL_PREFERENCE = ["okra_seg.engine", "okra_seg.onnx", "okra_seg.pt"]

IMGSZ = 640
CONF = 0.25


def find_model():
    for name in MODEL_PREFERENCE:
        if (MODELS_DIR / name).exists():
            return MODELS_DIR / name
    raise FileNotFoundError(f"No model in {MODELS_DIR} (expected one of {MODEL_PREFERENCE})")


class OkraDetector:
    def __init__(self, weights=None, conf=CONF, imgsz=IMGSZ, stream=True):
        """stream=True keeps the frame-to-frame stability check (use for a live
        camera); stream=False checks every frame on its own (use for single photos)."""
        self.weights = Path(weights) if weights else find_model()
        self.model = YOLO(str(self.weights), task="segment")
        self.conf, self.imgsz = conf, imgsz
        self.filter = OkraFilter() if stream else None

    def detect(self, frame_bgr, depth_m=None, intrinsics=None, return_rejected=False):
        """frame_bgr: HxWx3 uint8 BGR image (OpenCV order).
        depth_m: optional HxW depth image in metres, aligned to frame_bgr.
        intrinsics: optional (fx, fy, ppx, ppy) of the colour camera in pixels."""
        r = self.model.predict(frame_bgr, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        polys = list(r.masks.xy) if r.masks is not None else []
        fx = intrinsics[0] if intrinsics else None

        depths = [self._mask_depth(p, depth_m) for p in polys] if depth_m is not None else None
        if self.filter is not None:
            checks = self.filter(polys, frame_bgr, depths, fx)
        else:
            checks = [check_detection(p, frame_bgr, d, fx)
                      for p, d in zip(polys, depths or [None] * len(polys))]

        results = []
        for i, (box, poly, c) in enumerate(zip(r.boxes, polys, checks)):
            if not c.ok and not return_rejected:
                continue
            d = {
                "accepted": c.ok,
                "reason": c.reason,
                "cx": c.cx,
                "cy": c.cy,
                "box": tuple(float(v) for v in box.xyxy[0].tolist()),
                "conf": float(box.conf),
                "mask": poly,
            }
            if depths and depths[i]:
                d["depth_m"] = depths[i]
                if intrinsics:
                    fx_, fy_, ppx, ppy = intrinsics
                    d["length_m"] = c.length_m
                    d["xyz_m"] = ((c.cx - ppx) * depths[i] / fx_,
                                  (c.cy - ppy) * depths[i] / fy_,
                                  depths[i])
            results.append(d)

        results.sort(key=lambda d: (not d["accepted"], -d["conf"]))
        return results

    @staticmethod
    def _mask_depth(poly, depth_m):
        """Median of valid depth pixels inside the mask (robust to holes and edges)."""
        import cv2

        mask = np.zeros(depth_m.shape[:2], np.uint8)
        cv2.fillPoly(mask, [np.asarray(poly, np.int32)], 255)
        vals = depth_m[mask > 0]
        vals = vals[np.isfinite(vals) & (vals > 0.05) & (vals < 5.0)]
        return float(np.median(vals)) if len(vals) else None
