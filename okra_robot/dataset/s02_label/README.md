# Okra labeling set — session02 (2026-09-26)

150 frames from the G1 head camera (RealSense D435i colour, 640x480), taken from
`g1_record/data/session02_20260926_200103/color.mkv`:

| Files | Count | Source | Expect |
|---|---|---|---|
| `images/s02_p_*.jpg` | 110 | t = 8–22 min, 2 fps, near-duplicates removed | okra stand in view; most frames have pods |
| `images/s02_n_*.jpg`, `s02_m_*.jpg` | 40 | t < 8 min and t > 22 min, 1 fps | mostly no okra (rug, box, legs, stands) |

Frame number → time: `p_NNNNN` = 480 s + (NNNNN − 1) / 2; `n_NNNNN` = NNNNN − 1 s; `m_NNNNN` = 1330 s + NNNNN − 1.

## How to label

- Tool: CVAT or Roboflow (or any tool that exports **YOLO segmentation** / Ultralytics format).
- One class: `okra` (id 0). Draw a **polygon around each pod**, including partly hidden pods.
- Do **not** label leaves, green stakes, the rug or anything else green. Frames without pods get no labels
  (keep them — they teach the model what is *not* okra).
- Export as YOLO segmentation: `labels/<same name>.txt`, one line per pod: `0 x1 y1 x2 y2 ...` (normalized).
- Put the exported `labels/` folder next to `images/` here, then tell Claude to train.

## Then

Fine-tune `models/okra_seg.pt` on these (hold out ~20 % for validation), check on held-out frames,
and re-test live on the robot (`okra_test` on the Orin, env `g1brainco`, GPU).
