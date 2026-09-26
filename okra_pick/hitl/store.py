"""okra_data event store: one folder per robot "look", plus index.csv. See okra_data/README.md.

    python store.py reindex          # rebuild okra_data/index.csv from the event folders
    python store.py list [N]         # last N events
"""

import csv
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hitl_config as H  # noqa: E402

ROOT = os.path.expanduser(os.environ.get("OKRA_DATA", H.DATA_ROOT))   # OKRA_DATA overrides (tests)
EVENTS = os.path.join(ROOT, "events")
FIELDS = ["event_id", "date", "n_candidates", "top_conf", "policy", "asked", "reasons", "answer", "n_okra",
          "n_not_okra", "missed", "unsure", "operator", "response_s", "target_in_reach", "planned",
          "executed", "outcome", "detector"]


def new_event_dir():
    eid = time.strftime("%Y%m%d_%H%M%S")
    d = os.path.join(EVENTS, eid[:4] + "-" + eid[4:6] + "-" + eid[6:8], eid)
    os.makedirs(d, exist_ok=True)
    return d


def all_events():
    out = []
    if os.path.isdir(EVENTS):
        for day in sorted(os.listdir(EVENTS)):
            for e in sorted(os.listdir(os.path.join(EVENTS, day))):
                out.append(os.path.join(EVENTS, day, e))
    return out


def find(eid=None):
    evs = all_events()
    if not evs:
        return None
    if eid is None:
        return evs[-1]
    for e in evs:
        if os.path.basename(e) == eid or e == eid:
            return e
    return None


def load(d, name):
    p = os.path.join(d, name)
    return json.load(open(p)) if os.path.exists(p) else None


def save(d, name, obj):
    json.dump(obj, open(os.path.join(d, name), "w"), indent=1)


def row(d):
    c = load(d, "candidates.json") or {}
    dec = load(d, "decision.json") or {}
    out = load(d, "outcome.json") or {}
    cands = c.get("candidates", [])
    labels = list((dec.get("labels") or {}).values())
    return {
        "event_id": os.path.basename(d), "date": os.path.basename(os.path.dirname(d)),
        "n_candidates": len(cands), "top_conf": round(cands[0]["conf_median"], 3) if cands else "",
        "policy": dec.get("policy", ""), "asked": dec.get("asked", ""), "reasons": " | ".join(dec.get("reasons", [])),
        "answer": dec.get("answer_raw", ""), "n_okra": sum(l.startswith("okra") for l in labels),
        "n_not_okra": labels.count("not_okra"), "missed": dec.get("missed", ""), "unsure": dec.get("unsure", ""),
        "operator": dec.get("operator", ""), "response_s": dec.get("response_s", ""),
        "target_in_reach": dec.get("target") is not None, "planned": os.path.exists(os.path.join(d, "trajectory.json")),
        "executed": os.path.exists(os.path.join(d, "trajectory_executed.json")), "outcome": out.get("result", ""),
        "detector": c.get("detector", ""),
    }


def reindex():
    os.makedirs(ROOT, exist_ok=True)
    rows = [row(d) for d in all_events()]
    with open(os.path.join(ROOT, "index.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        w.writerows(rows)
    return rows


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "list"
    rows = reindex()
    if cmd == "list":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        for r in rows[-n:]:
            print("%(event_id)s  cands %(n_candidates)s top %(top_conf)s  asked %(asked)s  answer %(answer)s  "
                  "okra %(n_okra)s  outcome %(outcome)s" % r)
    print("%d events, index: %s" % (len(rows), os.path.join(ROOT, "index.csv")))
