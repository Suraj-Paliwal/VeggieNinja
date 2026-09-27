"""
Stand-in for `okra_pick.sh look` in offline tests and demos (no robot, no depth camera). The web server uses it
when OKRA_LOOK_CMD is set:

    OKRA_LOOK_CMD="../dimos/.venv/bin/python tests/fake_look.py" OKRA_DATA=/tmp/demo ./okra_pick.sh web

It takes the live feed's clean frame (live_cam.py /raw.jpg), runs the real detector on the VM CPU and writes a normal
event bundle. There is no depth, so every candidate gets a SYNTHETIC in-reach 3D point (marked in candidates.json).
Never use it with the real robot: the plan would be for a made-up position.
"""

import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
J = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path[:0] = [os.path.join(J, "okra_pick", "robot"), os.path.join(J, "okra_pick", "hitl"), os.path.join(J, "okra_pick", "plan"),
                os.path.join(J, "okra_robot")]
import store  # noqa: E402
from candidates import Candidates  # noqa: E402
from okra_detector import OkraDetector  # noqa: E402

LIVE = os.environ.get("OKRA_LIVE", "127.0.0.1:8091")
SYNTH_XYZ = (0.42, -0.12, 0.15)          # pelvis frame, inside the reach box (pick_config)
FRAMES = 3


def annotate(color, cands):
    """Same drawing as okra_perceive.annotate (that module needs pyrealsense2, which only the robot has)."""
    img = color.copy()
    for c in cands:
        col = (0, 200, 0) if c["in_reach"] else (0, 200, 255)
        cv2.polylines(img, [np.asarray(c["key_mask"], np.int32)], True, col, 2)
        x, y = int(c["px"][0]), int(c["px"][1])
        cv2.putText(img, "%d" % c["id"], (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(img, "%d" % c["id"], (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    return img


def main():
    d = store.new_event_dir()
    os.makedirs(os.path.join(d, "frames"))
    det = OkraDetector(os.environ.get("OKRA_WEIGHTS_PATH", os.path.join(J, "okra_robot", "models", "okra_seg_s02.pt")),
                       stream=False, conf=0.10)
    cands, img = Candidates(FRAMES), None
    for f in range(FRAMES):
        data = urllib.request.urlopen("http://%s/raw.jpg" % LIVE, timeout=5).read()
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imwrite(os.path.join(d, "frames", "%03d.jpg" % f), img)
        obs = []
        for k, x in enumerate(det.detect(img, return_rejected=True)):
            o = {kk: x[kk] for kk in ("cx", "cy", "conf", "accepted", "reason", "box")}
            o["mask"] = np.asarray(x["mask"]).tolist()
            o["p"] = (np.array(SYNTH_XYZ) + [0, -0.08 * k, 0]).tolist()
            o["axis"], o["length_m"] = [0, 0, 1], 0.1
            obs.append(o)
        cands.add_frame(f, obs)
        print("frame %d/%d: %d detections" % (f + 1, FRAMES, len(obs)), flush=True)
        time.sleep(0.1)
    reach = lambda p: 0.35 <= p[0] <= 0.5 and -0.4 <= p[1] <= 0.05 and 0.1 <= p[2] <= 0.4
    summary = cands.summary(reach)
    json.dump({"candidates": summary, "frames": FRAMES, "conf_floor": 0.10, "detector": det.weights.name + " (fake_look: SYNTHETIC xyz)",
               "time": time.time()}, open(os.path.join(d, "candidates.json"), "w"))
    from okra_plan import synthetic_target
    json.dump({"q": synthetic_target(0, 0, 0)["q"], "imu_rpy": [0, 0, 0], "synthetic": True},
              open(os.path.join(d, "robot_state.json"), "w"))
    cv2.imwrite(os.path.join(d, "annotated.jpg"), annotate(img, summary))
    print("EVENT %s: %d candidates (SYNTHETIC positions)" % (d, len(summary)))
    print("EVENT_DIR=%s" % d)


if __name__ == "__main__":
    main()
