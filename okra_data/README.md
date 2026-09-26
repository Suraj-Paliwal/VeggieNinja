# okra_data — everything the robot saw, asked and did (human-in-the-loop)

Written by `okra_pick/okra_pick.sh` (perceive → confirm → plan → pick → outcome). One folder per robot
"look" (event). Heavy files are git-ignored; this README and the schema below are tracked.

```
okra_data/
├── README.md                this file (schema)
├── index.csv                one row per event (rebuilt by okra_pick/hitl/store.py reindex)
├── events/YYYY-MM-DD/<EVENT_ID>/        EVENT_ID = YYYYmmdd_HHMMSS
│   ├── frames/NNN.jpg       every camera frame of the look (640x480, JPEG 95)
│   ├── depth_key.npz        depth_mm (uint16) of the last frame, aligned to colour
│   ├── candidates.json      every candidate: conf median/max, seen %, filter-ok %, reject reasons,
│   │                        pelvis xyz, spread, pod axis, length, in_reach, key/best mask polygons
│   ├── robot_state.json     29 joint angles, IMU rpy, waist, camera intrinsics, pelvis<-camera 4x4
│   ├── annotated.jpg        last frame, all candidates numbered (robot's view)
│   ├── question.jpg         what the operator was shown (only if asked)
│   ├── decision.json        policy, auto/ask, reasons for asking, operator answer, per-candidate labels
│   │                        (okra / not_okra / unsure / okra_auto), missed, unsure, response time, target
│   ├── target.json          chosen pod for the planner (only if confirmed and in reach)
│   ├── trajectory.json      planned arm motion (okra_plan.py);  sim.mp4 = its MuJoCo video
│   ├── trajectory_executed.json   what the arm player did (log, abort reason)
│   └── outcome.json         operator's verdict: success / fail / partial, note, recording episode
├── datasets/hitl_vNNN/      YOLO-seg exports (images/, labels/, data.yaml, manifest.csv, README.md)
└── to_annotate/             frames where the operator said "you missed an okra" -> label in CVAT
```

## Labels and what they become in training

| Operator answer | decision.json labels | export_dataset.py |
|---|---|---|
| `1` / `1,3` (these are okra) | listed → `okra`, other shown → `not_okra` | image + polygons of the okra |
| `n` (none is okra) | all shown → `not_okra` | image, empty label (hard negative) |
| `m` (missed one) | shown → `unsure`, `missed: true` | copied to `to_annotate/` for CVAT |
| `s` (not sure) | `unsure: true` | skipped |
| not asked (confident) | best → `okra_auto` | only with `--include-auto` (pseudo-label) |

## Using the data later

- Detector: `okra_pick.sh dataset` → `datasets/hitl_vNNN/data.yaml` (ultralytics format); merge with the CVAT set.
- Policy tuning: `index.csv` (top_conf vs operator answer) → choose `BASELINE_CONF` in `okra_pick/hitl/hitl_config.py`.
- Grasp analysis: target.json + trajectory_executed.json + outcome.json (+ the g1_record episode for video/state).
- Re-projection: depth_key.npz + robot_state.json give 3D for any pixel of the last frame.
