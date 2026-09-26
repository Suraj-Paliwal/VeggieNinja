# 08 — Episode data format and loading

## Layout

On the robot: `~/g1_rec/episodes/<name>_<YYYYmmdd_HHMMSS>/`
On the VM after `rec.sh pull`: `~/Junction/g1_record/data/<same name>/` (git-ignored)

```
<episode>/
├── meta.json          args, start/stop time, ffmpeg commands + exit codes, row counts
├── recorder.log       recorder stdout/stderr (progress every --flush s)
├── color.mkv          H.264 yuv420p, timeline starts at 0
├── color_ts.txt       one line per frame: epoch ms (Orin clock)
├── depth.mkv          FFV1 gray16le, mm       (only with --depth)
├── depth_ts.txt       one line per frame      (only with --depth)
└── state/
    ├── lowstate_0000.npz, lowstate_0001.npz, ...
    ├── dex1_left_state_0000.npz, ...
    └── arm_sdk_*.npz, dex1_*_cmd_*.npz          (only if those topics had messages)
```

## `meta.json` keys

`start_time`, `stop_time` (epoch s), `host`, `args` (all recorder options), `color_dev`,
`depth_dev`, `depth_units`, `clock`, `ffmpeg_color`, `ffmpeg_depth` (exact commands),
`ffmpeg_exit` (255 = stopped by SIGINT = normal), `rezero_exit` (0 = ok), `state_rows`.
If `stop_time` is missing, the recorder did not stop cleanly; data up to the last
flushed chunk is still valid.

## Inspect

```bash
cd ~/Junction/g1_record
../g1_video/.venv/bin/python inspect_episode.py data/<episode>
```
Prints duration, ffmpeg exit codes, and per stream: count, rate, span, max gap.

## Load in Python

```python
import sys; sys.path.insert(0, "/home/ros2/Junction/g1_record")
from inspect_episode import load_state, load_ts

ep = "/home/ros2/Junction/g1_record/data/test_20260926_154834"
ls  = load_state(ep, "lowstate")     # dict: t, tick, q (N,35), dq, tau, imu_*
gl  = load_state(ep, "dex1_left_state")
tc  = load_ts(ep, "color")           # (F,) epoch seconds, one per color frame
```

Read video frames with OpenCV (color) or ffmpeg (depth, keeps 16 bits):
```python
import cv2
cap = cv2.VideoCapture(ep + "/color.mkv")          # frame i  <->  tc[i]

import subprocess, numpy as np
raw = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", ep + "/depth.mkv",
                      "-f", "rawvideo", "-pix_fmt", "gray16le", "-"], capture_output=True).stdout
depth = np.frombuffer(raw, np.uint16).reshape(-1, 480, 640)   # mm, 0 = invalid
```

## Align streams (nearest state sample per frame)

```python
import numpy as np
idx = np.searchsorted(ls["t"], tc).clip(1, len(ls["t"]) - 1)
idx -= (tc - ls["t"][idx - 1]) < (ls["t"][idx] - tc)      # pick the closer neighbour
q_at_frame = ls["q"][idx]                                  # (F, 35)
```
All clocks are the Orin wall clock, so no offset is needed. Frames before the first
state sample (the first ~0.1 s) map to sample 0 — trim them if exactness matters.

## Size guide (640x480 @ 30 fps)

| Stream | Per hour |
|---|---|
| color (crf 20) | ~0.5–1.4 GB (depends on scene motion) |
| depth (FFV1) | ~8 GB |
| state (all topics, float64) | ~3.6 GB |

## Housekeeping

- `rec.sh list` shows episodes and sizes on the robot.
- Episodes are not deleted automatically on the robot after `pull`. Delete with
  `ssh unitree@192.168.123.164 'rm -rf ~/g1_rec/episodes/<episode>'` once verified on the VM.
