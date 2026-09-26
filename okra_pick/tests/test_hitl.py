"""
Offline end-to-end test of the human-in-the-loop flow (no robot): candidates -> decide -> confirm
(non-interactive answers, robot voice/LED through a simulated robot_agent) -> export_dataset.

    ../dimos/.venv/bin/python tests/test_hitl.py

Uses real session02 frames and the team's CVAT polygons as fake detections, in a temporary OKRA_DATA.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
J = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
from candidates import Candidates  # noqa: E402

PY = sys.executable
IMG = os.path.join(J, "okra_robot", "dataset", "s02_label", "images")
ANN = os.environ.get("CVAT_ANN")          # optional: cached CVAT export with real polygons
DATA = tempfile.mkdtemp(prefix="okra_data_test_")
ENV = dict(os.environ, OKRA_DATA=DATA, G1_AGENT="127.0.0.1:7798", OKRA_OPERATOR="tester")


def polys_for(frame_name):
    """Real CVAT polygons for a frame if available, else a synthetic vertical pod."""
    if ANN and os.path.exists(ANN):
        D = json.load(open(ANN))
        f = D["frames"].index(frame_name)
        out = [s["points"] for s in D["ann"]["shapes"] if s["frame"] == f]
        out += [s["points"] for t in D["ann"]["tracks"] for s in t["shapes"] if s["frame"] == f and not s["outside"]]
        if out:
            return [np.array(p).reshape(-1, 2) for p in out]
    return [np.array([[500, 200], [512, 200], [515, 300], [503, 300]], float)]


def make_event(name, frame_name, specs):
    """specs: list of (conf, accepted, xyz or None) per polygon (extra polygons ignored)."""
    d = os.path.join(DATA, "events", "2026-09-27", name)
    os.makedirs(os.path.join(d, "frames"))
    polys = polys_for(frame_name)
    cands = Candidates(15)
    rng = np.random.default_rng(0)
    for f in range(15):
        shutil.copy2(os.path.join(IMG, frame_name), os.path.join(d, "frames", "%03d.jpg" % f))
        obs = []
        for poly, (conf, acc, xyz) in zip(polys, specs):
            c = poly.mean(0) + rng.normal(0, 1.0, 2)
            o = {"cx": c[0], "cy": c[1], "conf": conf + rng.normal(0, 0.02), "accepted": acc,
                 "reason": "ok" if acc else "colour 40%", "mask": poly.tolist(),
                 "box": [*poly.min(0), *poly.max(0)]}
            if xyz is not None:
                o["p"] = (np.array(xyz) + rng.normal(0, 0.003, 3)).tolist()
                o["axis"], o["length_m"] = [0, 0, 1], 0.1
            obs.append(o)
        cands.add_frame(f, obs)
    reach = lambda p: 0.35 <= p[0] <= 0.5 and -0.4 <= p[1] <= 0.05 and 0.1 <= p[2] <= 0.4
    json.dump({"candidates": cands.summary(reach), "frames": 15, "detector": "test"}, open(os.path.join(d, "candidates.json"), "w"))
    sys.path.insert(0, os.path.join(HERE, "..", "plan"))
    from okra_plan import synthetic_target                       # realistic standing pose (arms relaxed)
    json.dump({"q": synthetic_target(0, 0, 0)["q"], "imu_rpy": [0, 0, 0]}, open(os.path.join(d, "robot_state.json"), "w"))
    return d


def confirm(d, answer=None):
    cmd = [PY, os.path.join(J, "okra_pick", "hitl", "confirm.py"), d, "--no-window"] + (["--answer", answer] if answer else [])
    r = subprocess.run(cmd, env=ENV, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr, json.load(open(os.path.join(d, "decision.json")))


def main():
    agent = subprocess.Popen([sys.executable, os.path.join(J, "robot_agent", "agent.py"), "--sim", "--host", "127.0.0.1",
                              "--port", "7798", "--log", os.path.join(DATA, "agent.log")],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.5)
    results = []
    try:
        # 1: two pods, low confidence + close -> ask; operator says 1 is okra
        d = make_event("e1_uncertain", "s02_p_01630.jpg", [(0.45, True, (0.42, -0.15, 0.15)), (0.40, False, (0.60, -0.1, 0.2))])
        rc, out, dec = confirm(d, "1")
        results.append(("uncertain pair -> ask, answer 1", dec["asked"] and dec["labels"] == {"1": "okra", "2": "not_okra"}
                        and dec["target"] == 1 and rc == 0 and os.path.exists(os.path.join(d, "target.json")), dec["reasons"]))
        # 2: one confident, stable, in reach -> auto, no question
        d = make_event("e2_confident", "s02_p_01602.jpg", [(0.85, True, (0.42, -0.15, 0.15))])
        rc, out, dec = confirm(d)
        results.append(("confident single -> auto", dec["action"] == "auto" and not dec["asked"] and rc == 0, dec["reasons"]))
        # 3: nothing detected -> ask 'did I miss one' -> m
        d = make_event("e3_nothing", "s02_p_01602.jpg", [])
        rc, out, dec = confirm(d, "m")
        results.append(("nothing -> ask, answer m (missed)", dec["asked"] and dec["missed"] and rc == 3, dec["reasons"]))
        # 4: candidate but operator says none is okra -> negative
        d = make_event("e4_leaf", "s02_p_01602.jpg", [(0.5, True, (0.42, -0.15, 0.15))])
        rc, out, dec = confirm(d, "n")
        results.append(("leaf -> ask, answer n", dec["labels"] == {"1": "not_okra"} and dec["target"] is None and rc == 3, dec["reasons"]))
        # 5: unsure -> nothing learned
        d = make_event("e5_unsure", "s02_p_01602.jpg", [(0.3, True, (0.42, -0.15, 0.15))])
        rc, out, dec = confirm(d, "s")
        results.append(("unsure -> skipped", dec["unsure"] and rc == 3, dec["reasons"]))

        exp = subprocess.run([PY, os.path.join(J, "okra_pick", "hitl", "export_dataset.py")], env=ENV,
                             capture_output=True, text=True)
        ds = os.path.join(DATA, "datasets", "hitl_v001")
        labels = {f: open(os.path.join(ds, "labels", sp, f)).read().strip().splitlines()
                  for sp in ("train", "val") if os.path.isdir(os.path.join(ds, "labels", sp))
                  for f in os.listdir(os.path.join(ds, "labels", sp))}
        ok_export = (len(labels.get("e1_uncertain.txt", [])) == 1 and labels.get("e4_leaf.txt") == []
                     and "e5_unsure.txt" not in labels and "e2_confident.txt" not in labels
                     and os.path.exists(os.path.join(DATA, "to_annotate", "e3_nothing.jpg")))
        results.append(("export: 1 positive, 1 negative, 1 to_annotate, unsure+auto skipped", ok_export, exp.stdout.strip()))
        log = open(os.path.join(DATA, "agent.log")).read()
        results.append(("robot spoke + LED via agent", log.count(" say ") >= 4 and log.count(" led ") >= 8,
                        "%d say, %d led" % (log.count(" say "), log.count(" led "))))
        idx = open(os.path.join(DATA, "index.csv")).read().strip().splitlines()
        results.append(("index.csv has 5 events", len(idx) == 6, idx[0][:60]))
    finally:
        agent.terminate()
    ok = True
    for name, passed, info in results:
        ok &= bool(passed)
        print("%s  %-62s %s" % ("PASS" if passed else "FAIL", name, info))
    print("test data in", DATA)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
