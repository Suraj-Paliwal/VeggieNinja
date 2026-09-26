"""Human-in-the-loop policy: act automatically, or ask the operator (and why)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hitl_config as H  # noqa: E402


def decide(cands, mode=None):
    """Returns {"action": "auto"|"ask"|"none", "suggested": id|None, "reasons": [...], "shown": [ids]}."""
    mode = mode or H.MODE
    shown = [c for c in cands if c["conf_median"] >= H.SHOW_FLOOR or c["conf_max"] >= H.BASELINE_CONF]
    reasons = []
    if not shown:
        if mode != "never" and H.ASK_WHEN_NOTHING:
            return {"action": "ask", "suggested": None, "shown": [], "reasons": ["no okra detected: did I miss one?"]}
        return {"action": "none", "suggested": None, "shown": [], "reasons": ["no okra detected"]}
    best = shown[0]
    if best["conf_median"] < H.BASELINE_CONF:
        reasons.append("confidence %.2f below baseline %.2f" % (best["conf_median"], H.BASELINE_CONF))
    others = [c for c in shown[1:] if best["conf_median"] - c["conf_median"] < H.CONFUSION_GAP]
    if others:
        reasons.append("confusion: candidate(s) %s almost as likely as %d" % (",".join(str(c["id"]) for c in others), best["id"]))
    if best["seen_frac"] < H.MIN_SEEN_FRAC:
        reasons.append("seen in only %.0f%% of frames" % (100 * best["seen_frac"]))
    if best["accepted_frac"] < H.MIN_FILTER_OK:
        reasons.append("sanity filter rejected it in %.0f%% of sightings (%s)" % (
            100 * (1 - best["accepted_frac"]), ", ".join(sorted(best["reject_reasons"])) or "-"))
    if best["spread_m"] is not None and best["spread_m"] > H.MAX_SPREAD_M:
        reasons.append("position unstable (%.1f cm)" % (100 * best["spread_m"]))
    if best["xyz_pelvis"] is None:
        reasons.append("no depth for it")
    elif not best["in_reach"]:
        reasons.append("outside the arm's reach")
    ids = [c["id"] for c in shown]
    if mode == "always":
        return {"action": "ask", "suggested": best["id"], "shown": ids, "reasons": ["policy: always ask"] + reasons}
    if reasons and mode != "never":
        return {"action": "ask", "suggested": best["id"], "shown": ids, "reasons": reasons}
    if reasons:
        return {"action": "none", "suggested": None, "shown": ids, "reasons": reasons}
    return {"action": "auto", "suggested": best["id"], "shown": ids,
            "reasons": ["confident: %.2f >= %.2f, stable, in reach" % (best["conf_median"], H.BASELINE_CONF)]}
