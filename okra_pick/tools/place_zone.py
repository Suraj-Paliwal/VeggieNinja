#!/usr/bin/env python3
"""
Draw the pod placement zone (pick_config.PLACE_ZONE) on the live camera feed, so the operator can hang the pod where
the arm reaches comfortably. Runs on the VM; nothing moves.

    ../dimos/.venv/bin/python tools/place_zone.py          # draw (uses the live waist angle and the calibrated camera)
    ../dimos/.venv/bin/python tools/place_zone.py --off    # remove

Projection: pelvis-frame zone -> head camera (URDF + CAM_CORR_ROTVEC_DEG + the robot's own colour->depth offset and
intrinsics from the latest Look event) -> pixels. Drawn at the zone's middle height (pod centre).
"""

import argparse
import json
import os
import socket
import sys
import urllib.parse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "robot"), os.path.join(HERE, "..", "hitl")]
import pick_config as C  # noqa: E402
import store  # noqa: E402
from g1_chain import Chain, T_pelvis_optical  # noqa: E402

AGENT = os.environ.get("G1_AGENT", "192.168.123.164:7777")
LIVE = os.environ.get("OKRA_LIVE", AGENT.rsplit(":", 1)[0] + ":8091")


def agent_status():
    host, port = AGENT.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=3)
    s.sendall(b'{"id": 1, "cmd": "status"}\n')
    line = s.makefile("r").readline()
    s.close()
    return json.loads(line)["result"]


def live_control(query):
    host, port = LIVE.rsplit(":", 1)
    s = socket.create_connection((host, int(port)), timeout=3)
    s.sendall(("GET /control?%s HTTP/1.0\r\n\r\n" % query).encode())
    data = b"".join(iter(lambda: s.recv(65536), b""))
    s.close()
    return json.loads(data.split(b"\r\n\r\n", 1)[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--off", action="store_true")
    a = ap.parse_args()
    if a.off:
        live_control("zone=off")
        print("placement zone removed")
        return
    st = agent_status()                                     # measured now, not assumed
    waist = st.get("waist")
    if waist is None:
        sys.exit("no fresh joint state from the robot agent")
    rs = None
    for row in reversed(store.reindex()):                   # newest Look with the robot's own camera data
        d = store.find(row["event_id"])
        rs = store.load(d, "robot_state.json") if d else None
        if rs and "intrinsics" in rs and "color_to_depth" in rs:
            break
    else:
        sys.exit("no Look event with camera intrinsics yet: run one Look first")
    K = rs["intrinsics"]
    T = T_pelvis_optical(Chain(os.path.join(HERE, "..", "robot", "g1.urdf")), waist,
                         np.radians(C.CAM_CORR_ROTVEC_DEG)) @ np.asarray(rs["color_to_depth"])
    Ti = np.linalg.inv(T)

    def px(p):
        c = Ti[:3, :3] @ np.asarray(p, float) + Ti[:3, 3]
        return [round(K["fx"] * c[0] / c[2] + K["ppx"], 1), round(K["fy"] * c[1] / c[2] + K["ppy"], 1)]

    (x0, x1), (y0, y1), (z0, z1) = C.PLACE_ZONE
    zm = (z0 + z1) / 2
    poly = [px((x0, y0, zm)), px((x1, y0, zm)), px((x1, y1, zm)), px((x0, y1, zm))]
    point = px(((x0 + x1) / 2, (y0 + y1) / 2, zm))
    label = "okra here (centre ~%d cm above floor)" % round(100 * (zm + 0.75))   # pelvis ~0.75 m up (floor check)
    live_control("zone=" + urllib.parse.quote(json.dumps({"poly": poly, "point": point, "label": label})))
    print("zone drawn on the live feed: corners %s, centre %s (waist %s, camera data from %s)"
          % (poly, point, waist, row["event_id"]))


if __name__ == "__main__":
    main()
