"""Group per-frame okra detections into candidates (numpy only, Python 3.8: runs on the robot and the VM).

A "look" = N frames. Every detection above the candidate floor (accepted by the filter or not) is an
observation. Observations of the same physical thing are merged: by pelvis position (within MERGE_M) when
depth gave a 3D point, else by pixel centre (within MERGE_PX). Each candidate keeps summary statistics
that the human-in-the-loop policy uses (okra_pick/hitl/decide.py).
"""

import numpy as np

MERGE_M = 0.04
MERGE_PX = 40.0


class Candidates:
    def __init__(self, n_frames_planned):
        self.n_planned = n_frames_planned
        self.frames_seen = 0
        self.items = []                         # list of dicts with "obs": [...]

    def add_frame(self, frame_idx, observations):
        """observations: dicts with cx, cy, conf, accepted, reason, mask (Nx2), and optionally
        p (pelvis xyz), axis (pelvis unit vector), length_m."""
        self.frames_seen += 1
        for o in observations:
            o = dict(o, frame=frame_idx)
            best, best_d = None, None
            for c in self.items:
                if o.get("p") is not None and c["p_ref"] is not None:
                    d, lim = np.linalg.norm(np.asarray(o["p"]) - c["p_ref"]), MERGE_M
                else:
                    d, lim = np.hypot(o["cx"] - c["px_ref"][0], o["cy"] - c["px_ref"][1]), MERGE_PX
                if d <= lim and (best_d is None or d < best_d):
                    best, best_d = c, d
            if best is None:
                best = {"obs": [], "p_ref": None, "px_ref": (o["cx"], o["cy"])}
                self.items.append(best)
            best["obs"].append(o)
            if o.get("p") is not None:
                ps = [np.asarray(x["p"]) for x in best["obs"] if x.get("p") is not None]
                best["p_ref"] = np.median(ps, axis=0)
            best["px_ref"] = (float(np.median([x["cx"] for x in best["obs"]])),
                              float(np.median([x["cy"] for x in best["obs"]])))

    def summary(self, in_reach_fn):
        """List of candidate dicts, best first (score = median conf * fraction of frames seen)."""
        out = []
        for c in self.items:
            obs = c["obs"]
            conf = np.array([o["conf"] for o in obs])
            ps = [np.asarray(o["p"]) for o in obs if o.get("p") is not None]
            p = np.median(ps, axis=0) if ps else None
            spread = float(np.max(np.linalg.norm(np.array(ps) - p, axis=1))) if len(ps) > 1 else None
            axes = [np.asarray(o["axis"]) for o in obs if o.get("axis") is not None]
            axis = None
            if axes:
                a = np.median(axes, axis=0)
                axis = (a / np.linalg.norm(a)).tolist() if np.linalg.norm(a) > 1e-6 else None
            reasons = {}
            for o in obs:
                if not o["accepted"]:
                    key = o["reason"].split(" ")[0]
                    reasons[key] = reasons.get(key, 0) + 1
            key_obs = max(obs, key=lambda o: (o["frame"], o["conf"]))          # latest sighting
            best_obs = max(obs, key=lambda o: o["conf"])
            lengths = [o["length_m"] for o in obs if o.get("length_m")]
            out.append({
                "hits": len(obs),
                "frames": self.frames_seen,
                "seen_frac": len(set(o["frame"] for o in obs)) / float(max(1, self.frames_seen)),
                "conf_median": float(np.median(conf)), "conf_max": float(conf.max()),
                "accepted_frac": float(np.mean([o["accepted"] for o in obs])),
                "reject_reasons": reasons,
                "xyz_pelvis": None if p is None else p.tolist(),
                "spread_m": spread,
                "axis_pelvis": axis,
                "length_m": float(np.median(lengths)) if lengths else None,
                "in_reach": bool(p is not None and in_reach_fn(p)),
                "px": [round(c["px_ref"][0], 1), round(c["px_ref"][1], 1)],
                "key_frame": key_obs["frame"],
                "key_mask": np.asarray(key_obs["mask"]).round(1).tolist(),
                "key_box": [round(float(v), 1) for v in key_obs["box"]],
                "best_frame": best_obs["frame"],
                "best_mask": np.asarray(best_obs["mask"]).round(1).tolist(),
            })
        out.sort(key=lambda c: -c["conf_median"] * c["seen_frac"])
        for i, c in enumerate(out, 1):
            c["id"] = i
        return out
