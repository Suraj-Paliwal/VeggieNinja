#!/usr/bin/env python3
"""
Record what happened after a pick attempt (the operator's judgement + the player's result).

    python outcome.py EVENT_DIR                         # asks: did it pick the okra? y / n / p(artial)
    python outcome.py EVENT_DIR --result success --note "pod came off cleanly"

Writes EVENT_DIR/outcome.json and refreshes okra_data/index.csv.
"""

import argparse
import getpass
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import store  # noqa: E402

MAP = {"y": "success", "n": "fail", "p": "partial", "success": "success", "fail": "fail", "partial": "partial"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("event")
    ap.add_argument("--result", choices=["success", "fail", "partial"])
    ap.add_argument("--note", default="")
    ap.add_argument("--episode", default="", help="g1_record episode name of the attempt")
    ap.add_argument("--operator", default=os.environ.get("OKRA_OPERATOR", getpass.getuser()))
    args = ap.parse_args()
    res = args.result
    while res is None:
        res = MAP.get(input("Did the robot pick the okra? y = yes / n = no / p = partly: ").strip().lower())
    note = args.note or (input("note (optional): ").strip() if args.result is None else "")
    ex = store.load(args.event, "trajectory_executed.json") or {}
    store.save(args.event, "outcome.json", {"result": res, "note": note, "operator": args.operator,
                                            "player_aborted": ex.get("aborted"), "episode": args.episode,
                                            "time": time.time()})
    store.reindex()
    print("outcome %s recorded in %s" % (res, os.path.join(args.event, "outcome.json")))


if __name__ == "__main__":
    main()
