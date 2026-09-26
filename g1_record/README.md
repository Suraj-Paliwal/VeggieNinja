# g1_record — on-robot episode recorder for the Unitree G1

Records the head RealSense (color, optional depth) and the robot's DDS state **on the robot's
Orin**, so nothing crosses the lossy VM link. The VM only drives it and shows a live preview.

```bash
./rec.sh setup                  # once per robot: copy code, build ~/g1_rec/pylib offline
./rec.sh start pick01 --depth   # start an episode (stops Unitree videohub for the head cam)
./rec.sh preview                # optional live view on the VM
./rec.sh stop                   # finalize files on the robot
./rec.sh pull                   # rsync episodes to ./data/
../g1_video/.venv/bin/python inspect_episode.py data/pick01_*   # rates / gaps check
./rec.sh release                # give the head camera back to videohub (view.py)
```

| File | Runs on | Purpose |
|---|---|---|
| `rec.sh` | VM | start/stop/status/preview/list/pull/release |
| `robot/recorder.py` | robot | ffmpeg capture + DDS topic logging for one episode |
| `robot/setup_robot.sh` | robot | offline Python libs in `~/g1_rec/pylib` |
| `inspect_episode.py` | VM | summary + `load_state()` / `load_ts()` helpers |

Full documentation: [`../Guide/code_repeatability.md`](../Guide/code_repeatability.md).
