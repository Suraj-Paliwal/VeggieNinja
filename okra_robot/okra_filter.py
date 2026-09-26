"""
Sanity checks applied on top of the okra segmentation model.

The model alone happily labels anything green or elongated as okra (rugs, legs,
stools). A real okra pod is small, long-and-thin, and green, and it doesn't
flicker in and out between frames, so each detection must pass all of:

  size     mask area within [MIN_AREA_FRAC, MAX_AREA_FRAC] of the frame, or, when a
           depth value is supplied, physical length within [MIN_LEN_M, MAX_LEN_M]
  shape    long side / short side of the rotated bounding rect within
           [MIN_ASPECT, MAX_ASPECT] (rejects blobs like rugs and stakes/poles)
  colour   at least MIN_GREEN_FRAC of mask pixels are okra-green in HSV
  stable   seen near the same spot in >= MIN_HITS of the last HISTORY frames
           (video/stream only; OkraFilter keeps that state)

Thresholds are starting points for a 640x480 camera about 0.3-1 m from the plant;
tune them on your own footage.
"""

from __future__ import annotations  # lets this run on the robot (Python 3.8)

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

# size (pixel fallback when there's no depth)
MIN_AREA_FRAC = 0.0005   # ~150 px at 640x480
MAX_AREA_FRAC = 0.08     # ~25k px at 640x480 (a close-up pod); rug detections are far larger
# size (physical, when depth is available): okra pods are roughly 5-25 cm long
MIN_LEN_M = 0.05
MAX_LEN_M = 0.25
# shape: pods are long and thin, but not pole-thin (a 15x2 cm pod is ~7)
MIN_ASPECT = 2.0
MAX_ASPECT = 7.0
# colour: OpenCV hue is 0-180; okra green sits around 30-90
GREEN_HUE = (30, 90)
MIN_SAT = 50
MIN_VAL = 40
MIN_GREEN_FRAC = 0.5
# temporal stability
HISTORY = 5
MIN_HITS = 3
MATCH_DIST_FRAC = 0.08   # centroid must be within 8% of frame diagonal


@dataclass
class Check:
    ok: bool
    reason: str          # "ok" or the first check that failed
    cx: float
    cy: float
    area_px: float
    aspect: float
    green_frac: float
    length_m: float | None = None


def check_detection(poly, frame_bgr, depth_m=None, fx=None):
    """Run the per-frame checks (size, shape, colour) on one mask polygon.

    poly: Nx2 mask outline in pixels (ultralytics `r.masks.xy[i]`).
    depth_m / fx: optional distance to the pod and camera focal length in pixels;
    when both are given, size is checked in metres instead of pixels.
    """
    h, w = frame_bgr.shape[:2]
    pts = np.asarray(poly, dtype=np.float32)
    if len(pts) < 3:
        return Check(False, "no mask", 0, 0, 0, 0, 0)

    m = cv2.moments(pts)
    area = abs(m["m00"])
    cx, cy = (m["m10"] / m["m00"], m["m01"] / m["m00"]) if m["m00"] else pts.mean(axis=0)
    (_, _), (rw, rh), _ = cv2.minAreaRect(pts)
    long_side, short_side = max(rw, rh), max(min(rw, rh), 1.0)
    aspect = long_side / short_side

    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [pts.astype(np.int32)], 255)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)[mask > 0]
    green = ((hsv[:, 0] >= GREEN_HUE[0]) & (hsv[:, 0] <= GREEN_HUE[1])
             & (hsv[:, 1] >= MIN_SAT) & (hsv[:, 2] >= MIN_VAL))
    green_frac = float(green.mean()) if len(hsv) else 0.0

    c = Check(True, "ok", float(cx), float(cy), area, aspect, green_frac)

    if depth_m and fx:
        c.length_m = long_side * depth_m / fx
        if not MIN_LEN_M <= c.length_m <= MAX_LEN_M:
            c.ok, c.reason = False, f"size {c.length_m * 100:.0f}cm"
            return c
    elif not MIN_AREA_FRAC * w * h <= area <= MAX_AREA_FRAC * w * h:
        c.ok, c.reason = False, f"size {100 * area / (w * h):.1f}%"
        return c
    if not MIN_ASPECT <= aspect <= MAX_ASPECT:
        c.ok, c.reason = False, f"shape {aspect:.1f}"
        return c
    if green_frac < MIN_GREEN_FRAC:
        c.ok, c.reason = False, f"colour {green_frac:.0%}"
        return c
    return c


class OkraFilter:
    """Stateful filter for a video/camera stream: per-frame checks plus stability."""

    def __init__(self):
        self.history = deque(maxlen=HISTORY)  # centroids that passed, per frame

    def __call__(self, polys, frame_bgr, depths_m=None, fx=None):
        h, w = frame_bgr.shape[:2]
        max_dist = MATCH_DIST_FRAC * np.hypot(w, h)
        depths_m = depths_m or [None] * len(polys)

        checks = [check_detection(p, frame_bgr, d, fx) for p, d in zip(polys, depths_m)]
        passed_now = [(c.cx, c.cy) for c in checks if c.ok]

        for c in checks:
            if not c.ok:
                continue
            hits = 1 + sum(
                any(np.hypot(c.cx - px, c.cy - py) <= max_dist for px, py in past)
                for past in self.history
            )
            if hits < MIN_HITS:
                c.ok, c.reason = False, f"unstable {hits}/{MIN_HITS}"

        self.history.append(passed_now)
        return checks
