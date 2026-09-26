# Code Repeatability — G1 data-collection stack

How to reproduce the whole setup from scratch and where each piece is documented.
Every report in this folder is at most 150 lines. Verified on 2026-09-26.

## Reports (one per functionality)

| # | Report | Covers |
|---|---|---|
| 01 | [01_network_connection.md](01_network_connection.md) | VM ↔ robot link, IP-fragment tuning, `g1_connect/check.sh` |
| 02 | [02_robot_services.md](02_robot_services.md) | Unitree services on the Orin, `g1_connect/services.sh` |
| 03 | [03_robot_python_env.md](03_robot_python_env.md) | Offline Python libs on the robot (`setup_robot.sh`) |
| 04 | [04_episode_recording.md](04_episode_recording.md) | `rec.sh` workflow and `recorder.py` design |
| 05 | [05_camera_capture.md](05_camera_capture.md) | RealSense color/depth capture with ffmpeg, timestamps |
| 06 | [06_state_topics.md](06_state_topics.md) | DDS topics logged, fields, measured rates |
| 07 | [07_live_preview.md](07_live_preview.md) | UDP preview stream to the VM |
| 08 | [08_data_format.md](08_data_format.md) | Episode folder layout, loading, aligning streams |
| 09 | [09_troubleshooting.md](09_troubleshooting.md) | Every failure hit so far and its fix |
| 10 | [10_bringup_and_motion.md](10_bringup_and_motion.md) | FSM bring-up, balance conditions, waist moves |
| 11 | [11_okra_pick_pipeline.md](11_okra_pick_pipeline.md) | Okra detection → IK plan → on-robot grasp, sim, tests |

## System at a glance

```
 VM (Ubuntu, VirtualBox)                    G1 robot
 192.168.123.100 (enp0s8)                   ├─ motion controller 192.168.123.161 (no login)
 ├─ g1_connect/  check / services            └─ Orin onboard PC 192.168.123.164 (ssh unitree@, aarch64)
 ├─ g1_record/rec.sh  ── ssh ──────────────▶     ~/g1_rec/recorder.py  (ffmpeg + DDS logging)
 │                    ◀── UDP 5600 preview ──     RealSense D435i on /dev/video0..5
 └─ g1_record/data/   ◀── rsync episodes ───     ~/g1_rec/episodes/<name>_<time>/
```

Design rule: **record on the robot, control from the VM.** The VM link loses IP fragments
(see 01), so data recorded on the VM has gaps; data recorded on the Orin does not.

## Reproduce from zero (checklist)

1. **Physical**: robot powered, Ethernet from laptop to robot, VirtualBox adapter for
   `enp0s8` bridged to that Ethernet NIC. VM IP `192.168.123.100/24`.
2. **VM network tuning** (once, persists):
   `sudo ~/Junction/g1_connect/setup_host.sh --persist` → writes `/etc/sysctl.d/90-g1.conf`.
3. **VM Python env**: `~/Junction/g1_video/.venv` (Python 3.10, `unitree_sdk2py`,
   `cyclonedds==0.10.2`, numpy, opencv). Used by `check.sh` and `inspect_episode.py`.
4. **SSH key** to `unitree@192.168.123.164` (BatchMode must work: `ssh -o BatchMode=yes ... true`).
5. **Health check**: `~/Junction/g1_connect/check.sh` → expect `RESULT: ALL OK`.
6. **Robot libs**: `~/Junction/g1_record/rec.sh setup` → expect `pylib ok` and `ffmpeg ok`.
7. **Test episode**: `rec.sh start test --depth`, wait ~15 s, `rec.sh stop`, `rec.sh pull`,
   `../g1_video/.venv/bin/python inspect_episode.py data/test_*`.
8. Compare against the reference numbers below. Then `rec.sh release` if you need `view.py`.

## Reference result (test episode, 2026-09-26, 640x480 + depth)

```
color              n=503  30.0 Hz  span 16.7 s  max gap 34 ms
depth              n=510  30.0 Hz  span 17.0 s  max gap 34 ms
dex1_left_state    n=8478  493.8 Hz
dex1_right_state   n=8470  493.6 Hz
lowstate           n=17016  990.2 Hz  max gap 123 ms
no messages on: arm_sdk, dex1_left_cmd, dex1_right_cmd   (nothing was commanding the robot)
```
Sizes for ~17 s: color 2.3 MB, depth 38 MB, state 17 MB (≈ 0.14 / 8 / 3.6 GB per hour).
Preview: 498 of 503 frames arrived on the VM.

## Versions pinned

| Component | Where | Version |
|---|---|---|
| Python (robot) | system `/usr/bin/python3` | 3.8.10, numpy 1.17.4 |
| cyclonedds (robot) | cached wheel in `~/.cache/pip/wheels` | 0.10.2 cp38 aarch64 |
| unitree_sdk2py (robot) | copied from `~/unitree_sdk2_python` | local source copy |
| ffmpeg (robot) | apt | 4.2.7-0ubuntu0.1 |
| Python (VM) | `g1_video/.venv` | 3.10, cyclonedds 0.10.2 |
| Robot clock | Orin | ~11.5 min behind the VM (irrelevant: one clock per episode) |

## Files added in this work

```
g1_connect/services.sh          start/stop Unitree services on the Orin
g1_record/rec.sh                VM controller
g1_record/robot/recorder.py     on-robot recorder
g1_record/robot/setup_robot.sh  offline libs on robot
g1_record/inspect_episode.py    episode summary + loaders
g1_record/README.md, .gitignore (data/ is not committed)
Guide/*.md                      these reports
```

## Safety

Recording never commands the robot; it only subscribes. Anything that moves the robot
(`arm_move.py`, teleop) requires the robot on the gantry and a person at the e-stop.

## Not yet verified

- `arm_sdk` / `dex1_*_cmd` logging with real commands flowing (topics were silent in the test).
- 1280x720 / 1920x1080 color at a steady 30 fps.
- Episodes longer than ~20 s (disk and CPU headroom look ample: 1.8 TB free, 8 cores).
