#!/usr/bin/env python3
"""
Human-in-the-loop confirmation for one perception event. Runs on the VM.

    python confirm.py EVENT_DIR                       # decide; if unsure, ask the operator
    python confirm.py EVENT_DIR --mode always         # always ask
    python confirm.py EVENT_DIR --answer 1            # non-interactive (tests / scripted)

When asking: a window shows the frame with the candidates numbered, the robot says the question and turns
its LED amber (via robot_agent, if running), and the terminal asks:

    1,2,..   these numbers ARE okra (others shown are not)     n   none of these is okra
    m        there is an okra you did not mark (missed)         s   not sure (nothing is learned)
    q        abort

Writes EVENT_DIR/decision.json (always) and EVENT_DIR/target.json (when an okra in reach is confirmed or
auto-accepted). Every answer becomes training data via export_dataset.py.
"""

import argparse
import getpass
import json
import os
import socket
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hitl_config as H  # noqa: E402
import store  # noqa: E402
from decide import decide  # noqa: E402

AGENT = os.environ.get("G1_AGENT", "192.168.123.164:7777")


def robot(cmd, args):
    """Best-effort call to robot_agent (say / led). Never blocks the operator for long."""
    if not H.SPEAK:
        return
    try:
        host, port = AGENT.rsplit(":", 1)
        s = socket.create_connection((host, int(port)), timeout=0.5)
        s.settimeout(3.0)
        s.sendall((json.dumps({"id": 1, "cmd": cmd, "args": args}) + "\n").encode())
        s.makefile("r").readline()
        s.close()
    except OSError:
        pass


def key_image(ev, cands, shown):
    frames = sorted(os.listdir(os.path.join(ev, "frames")))
    img = cv2.imread(os.path.join(ev, "frames", frames[-1]))
    for c in cands:
        if c["id"] not in shown:
            continue
        col = (0, 200, 0) if c["in_reach"] else (0, 180, 255)
        cv2.polylines(img, [np.asarray(c["key_mask"], np.int32)], True, col, 2)
        x, y = int(c["px"][0]), int(c["px"][1])
        for th, cc in ((5, (0, 0, 0)), (2, col)):
            cv2.putText(img, str(c["id"]), (x + 10, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 1.0, cc, th)
    bar = np.full((70, img.shape[1], 3), 255, np.uint8)
    cv2.putText(bar, "Is it okra?  1,2..=okra  n=none  m=missed one  s=not sure", (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
    cv2.putText(bar, "green = within reach, orange = out of reach", (8, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 80), 1)
    return np.vstack([img, bar])


def show(img, path, window=True):
    cv2.imwrite(path, img)
    if not window or not os.environ.get("DISPLAY"):     # Qt aborts the process without a display
        return "file only"
    try:
        cv2.imshow("G1 asks: is this okra?", img)
        for _ in range(10):
            cv2.waitKey(30)
        return "window"
    except cv2.error:
        try:
            subprocess.Popen(["xdg-open", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return "viewer"
        except OSError:
            return "file"


def parse(ans, shown):
    ans = ans.strip().lower().replace(" ", "")
    if ans in ("n", "m", "s", "q"):
        return ans, []
    try:
        ids = [int(x) for x in ans.split(",") if x]
    except ValueError:
        return None, []
    if not ids or any(i not in shown for i in ids):
        return None, []
    return "ids", ids


def make_target(ev, c):
    st = store.load(ev, "robot_state.json")
    return {"xyz_pelvis": c["xyz_pelvis"], "axis_pelvis": c["axis_pelvis"], "length_m": c["length_m"],
            "spread_m": c["spread_m"], "conf": c["conf_median"], "candidate_id": c["id"], "q": st["q"],
            "source": "hitl", "time": time.time(),
            "perceived_at_robot": st.get("time")}          # ROBOT clock (robot_state.json is written on the Orin)


def prepare(ev, mode=None):
    """Decide; if asking, write question.jpg, a pending decision (decision_pending.json) and tell the robot
    to ask (LED + voice). Auto decisions are finalised immediately. Returns the decision dict."""
    cands = store.load(ev, "candidates.json")["candidates"]
    d = decide(cands, mode)
    dec = {"policy": mode or H.MODE, "baseline_conf": H.BASELINE_CONF, "action": d["action"],
           "reasons": d["reasons"], "shown": d["shown"], "suggested": d["suggested"], "asked": d["action"] == "ask",
           "labels": {}, "missed": False, "unsure": False, "target": None, "operator": None, "t_asked": time.time()}
    if d["action"] == "ask":
        q = key_image(ev, cands, d["shown"])
        cv2.imwrite(os.path.join(ev, "question.jpg"), q)                  # with the keyboard legend (terminal)
        cv2.imwrite(os.path.join(ev, "question_view.jpg"), q[:q.shape[0] - 70])   # without it (web UI)
        store.save(ev, "decision_pending.json", dec)
        robot("led", {"rgb": list(H.LED_ASK)})
        robot("say", {"text": H.SAY_ASK if d["shown"] else H.SAY_NOTHING})
        return dec
    if d["action"] == "auto":
        dec["labels"] = {str(d["suggested"]): "okra_auto"}
        dec["target"] = d["suggested"]
    finalise(ev, dec)
    return dec


def apply_answer(ev, ans, operator):
    """Apply the operator's answer to the pending decision. Raises ValueError on a bad answer."""
    dec = store.load(ev, "decision_pending.json") or store.load(ev, "decision.json")
    if dec is None:
        raise ValueError("no pending question for this event")
    cands = store.load(ev, "candidates.json")["candidates"]
    byid = {c["id"]: c for c in cands}
    kind, ids = parse(ans, dec["shown"])
    if kind is None:
        raise ValueError("answer with shown numbers, n, m, s or q")
    dec.update(answer_raw=ans, operator=operator, response_s=round(time.time() - dec.get("t_asked", time.time()), 1),
               labels={}, missed=False, unsure=False, target=None, aborted=False)
    if kind == "q":
        dec["aborted"] = True
    elif kind == "s":
        dec["unsure"] = True
    else:
        dec["missed"] = kind == "m"
        for i in dec["shown"]:
            dec["labels"][str(i)] = "okra" if i in ids else ("unsure" if kind == "m" else "not_okra")
        reach = [i for i in ids if byid[i]["in_reach"] and byid[i]["xyz_pelvis"] is not None]
        dec["target"] = reach[0] if reach else None
        dec["note"] = "confirmed okra is out of reach: move closer and look again" if ids and not reach else ""
    yes = dec["target"] is not None
    robot("led", {"rgb": list(H.LED_YES if yes else H.LED_NO)})
    robot("say", {"text": H.SAY_YES if yes else H.SAY_NO})
    finalise(ev, dec)
    return dec


def finalise(ev, dec):
    byid = {c["id"]: c for c in store.load(ev, "candidates.json")["candidates"]}
    store.save(ev, "decision.json", dec)
    pend = os.path.join(ev, "decision_pending.json")
    if os.path.exists(pend):
        os.remove(pend)
    tgt = os.path.join(ev, "target.json")
    if dec["target"] is not None:
        store.save(ev, "target.json", make_target(ev, byid[dec["target"]]))
    elif os.path.exists(tgt):
        os.remove(tgt)
    store.reindex()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("event")
    ap.add_argument("--mode", choices=["always", "uncertain", "never"])
    ap.add_argument("--answer", help="non-interactive answer")
    ap.add_argument("--operator", default=os.environ.get("OKRA_OPERATOR", getpass.getuser()))
    ap.add_argument("--no-window", action="store_true", help="only write question.jpg (no image window)")
    args = ap.parse_args()
    ev = args.event
    dec = prepare(ev, args.mode)
    cands = store.load(ev, "candidates.json")["candidates"]
    print("candidates: " + ("; ".join("%d conf %.2f seen %.0f%% %s" % (c["id"], c["conf_median"], 100 * c["seen_frac"],
                                                                       "reach" if c["in_reach"] else "no-reach")
                                        for c in cands if c["id"] in dec["shown"]) or "none"))
    print("decision: %s  (%s)" % (dec["action"].upper(), "; ".join(dec["reasons"])))
    if dec["action"] == "ask":
        how = show(cv2.imread(os.path.join(ev, "question.jpg")), os.path.join(ev, "question.jpg"), not args.no_window)
        print("image: %s (%s)" % (os.path.join(ev, "question.jpg"), how))
        while True:
            ans = args.answer if args.answer is not None else input(
                "Which are okra? [numbers, e.g. 1 or 1,3] / n none / m missed one / s not sure / q abort: ")
            try:
                dec = apply_answer(ev, ans, args.operator)
                break
            except ValueError as e:
                if args.answer is not None:
                    sys.exit("bad --answer %r: %s" % (ans, e))
                print("  " + str(e))
        if dec.get("note"):
            print(dec["note"])
        if how == "window":
            cv2.destroyAllWindows()
    if dec["target"] is not None:
        print("TARGET candidate %d -> %s" % (dec["target"], os.path.join(ev, "target.json")))
    sys.exit(0 if dec["target"] is not None else 3)


if __name__ == "__main__":
    main()
