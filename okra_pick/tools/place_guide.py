#!/usr/bin/env python3
"""
Terminal guide for hanging the pod where the arm reaches comfortably (pick_config.PLACE_ZONE). Nothing moves.

    ../dimos/.venv/bin/python tools/place_guide.py            # one reading
    ../dimos/.venv/bin/python tools/place_guide.py --watch    # every second until the pod is inside (Ctrl-C stops)

Where the pod is: the live feed's detector gives its pixel; the camera ray through it (URDF + CAM_CORR + the robot's
intrinsics from the latest Look) is cut with the zone's middle height, so it ASSUMES the pod centre hangs at that
height (~90 cm above the floor; set that with a tape). The Look step afterwards measures it properly with depth.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from place_zone import C, LIVE, Chain, T_pelvis_optical, agent_status, store  # noqa: E402


def live_status():
    import socket
    host, port = LIVE.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=3)
    s.sendall(b"GET /status?min_conf=0.5 HTTP/1.0\r\n\r\n")
    data = b"".join(iter(lambda: s.recv(65536), b""))
    s.close()
    return json.loads(data.split(b"\r\n\r\n", 1)[1])


def camera():
    """Camera data from the newest Look, and the pod height it measured with depth (if that Look is recent and saw
    a pod): moving the stand does not change the pod's height, so the guide uses it instead of assuming one."""
    for row in reversed(store.reindex()):
        d = store.find(row["event_id"])
        rs = store.load(d, "robot_state.json") if d else None
        if rs and "intrinsics" in rs and "color_to_depth" in rs:
            cj = store.load(d, "candidates.json") or {}
            zs = [c["xyz_pelvis"][2] for c in cj.get("candidates", []) if c.get("xyz_pelvis") and c["seen_frac"] > 0.5]
            fresh = time.time() - cj.get("time", 0) < 30 * 60
            return rs, (zs[0] if zs and fresh else None), row["event_id"]
    sys.exit("no Look event with camera intrinsics yet: run one Look first")


def reading(chain, rs, z_pod=None):
    st, lv = agent_status(), live_status()
    px = lv["okra"].get("px")
    if px is None:
        return "no okra seen by the live camera right now (source %s)" % lv["source"], False
    K = rs["intrinsics"]
    T = T_pelvis_optical(chain, st["waist"], np.radians(C.CAM_CORR_ROTVEC_DEG)) @ np.asarray(rs["color_to_depth"])
    ray = T[:3, :3] @ np.array([(px["cx"] - K["ppx"]) / K["fx"], (px["cy"] - K["ppy"]) / K["fy"], 1.0])
    (x0, x1), (y0, y1), (z0, z1) = C.PLACE_ZONE
    zm = (z0 + z1) / 2 if z_pod is None else z_pod
    if ray[2] >= -1e-6:
        return "pod seen above the camera's horizon: lower it", False
    p = T[:3, 3] + ray * (zm - T[:3, 3][2]) / ray[2]
    dx = 0.0 if x0 <= p[0] <= x1 else (x0 - p[0] if p[0] < x0 else x1 - p[0])
    dy = 0.0 if y0 <= p[1] <= y1 else (y0 - p[1] if p[1] < y0 else y1 - p[1])
    tips = []
    if dx:
        tips.append("%d cm %s the robot" % (round(100 * abs(dx)) + 2, "AWAY from" if dx > 0 else "CLOSER to"))
    if dy:
        tips.append("%d cm to the robot's %s" % (round(100 * abs(dy)) + 2, "LEFT" if dy > 0 else "RIGHT"))
    where = "pod at %.0f cm in front, %.0f cm to the robot's %s (conf %.2f)" % (
        100 * p[0], 100 * abs(p[1]), "left" if p[1] > 0 else "right", px["conf"])
    if not tips:
        return where + "  ->  INSIDE the zone. Now run Look.", True
    return where + "  ->  move it " + " and ".join(tips), False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true")
    a = ap.parse_args()
    chain = Chain(os.path.join(HERE, "..", "robot", "g1.urdf"))
    rs, z_pod, eid = camera()
    (x0, x1), (y0, y1), (z0, z1) = C.PLACE_ZONE
    print("target: %d-%d cm in front of the hips, %d-%d cm to the robot's right, centre %d-%d cm above the floor"
          % (100 * x0, 100 * x1, -100 * y1, -100 * y0, 100 * (z0 + 0.75), 100 * (z1 + 0.75)))
    print("pod height: %s" % ("%d cm above the floor, measured by Look %s (depth)" % (round(100 * (z_pod + 0.75)), eid)
                              if z_pod is not None else "ASSUMED %d cm above the floor (no recent Look saw a pod)"
                              % round(100 * ((z0 + z1) / 2 + 0.75))))
    while True:
        msg, inside = reading(chain, rs, z_pod)
        print(time.strftime("%H:%M:%S"), msg, flush=True)
        if not a.watch or inside:
            return
        time.sleep(1.0)


if __name__ == "__main__":
    main()
