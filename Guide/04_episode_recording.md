# 04 — Episode recording (`g1_record/rec.sh` + `robot/recorder.py`)

## Workflow

```bash
cd ~/Junction/g1_record
./rec.sh start pick01 --depth    # 1. start (name gets a timestamp suffix)
./rec.sh preview                 # 2. optional: watch live in a window on the VM
#    ... move the robot / teleop / do the task ...
./rec.sh stop                    # 3. stop; prints log tail and file sizes
./rec.sh pull                    # 4. rsync all episodes to ./data/  (or: pull <episode>)
../g1_video/.venv/bin/python inspect_episode.py data/pick01_*   # 5. check
./rec.sh release                 # 6. when done: give the head camera back to videohub
```
Other commands: `./rec.sh status`, `./rec.sh list` (episodes + sizes on the robot).

## `rec.sh start NAME [options]`

1. Refuses if a recorder is already running (`~/g1_rec/recorder.pid` alive).
2. Copies the current `robot/recorder.py` to the robot (edits take effect immediately).
3. Stops `video_hub_pc4` via `mscli` if it is running (frees `/dev/video4`).
4. Creates `~/g1_rec/episodes/NAME_YYYYmmdd_HHMMSS/` (VM clock in the name) and
   writes its path to `~/g1_rec/current`.
5. Launches `recorder.py` with `nohup` in the background, log to `recorder.log`.
6. Waits 4 s, shows the log, confirms the process is alive → `RECORDING episodes/...`.

Extra options are passed to `recorder.py`:

| Option | Default | Meaning |
|---|---|---|
| `--depth` | off | also record 16-bit depth |
| `--size` | 640x480 | color size (320x180 … 1920x1080, YUYV modes of `/dev/video4`) |
| `--depth-size` | 640x480 | depth size (Z16 modes of `/dev/video0`) |
| `--fps` | 30 | frame rate for both streams |
| `--crf` | 20 | H.264 quality (lower = better, larger) |
| `--preset` | veryfast | x264 speed preset |
| `--preview` | 192.168.123.100:5600 | UDP target; `--preview ''` disables |
| `--flush` | 10 | seconds between state chunk writes |
| `--iface` | eth0 | Orin interface for DDS |

## `rec.sh stop`

Sends SIGINT to the recorder and waits up to 5 min (long depth files take a while to re-zero). The recorder then:
1. closes the DDS subscribers,
2. sends SIGINT to each ffmpeg (ffmpeg finalizes the files; exit code **255 is normal**),
3. re-zeros each `.mkv` timeline (stream copy, see 05),
4. writes the last state chunk and final `meta.json` (`stop_time`, row counts, exit codes).

## `recorder.py` structure

```
main()
 ├─ ffmpeg color  : v4l2 /dev/video4 → x264 → tee: color.mkv | color_ts.txt | UDP preview
 ├─ ffmpeg depth  : v4l2 /dev/video0 (gray16le) → FFV1 → tee: depth.mkv | depth_ts.txt
 ├─ DDS subscribers (unitree_sdk2py ChannelSubscriber, queue 50) → TopicLog.cb
 │     each callback: row = fields of the message + t = time.time()
 └─ loop every 1 s: if an ffmpeg died → stop; every --flush s → TopicLog.flush() → .npz chunk
```
If ffmpeg exits early (camera busy, bad size), the whole recording stops and the log says
`ffmpeg color exited early (code N)`.

## One clock

ffmpeg stamps frames with `-use_wallclock_as_timestamps 1` and DDS callbacks use
`time.time()` — the **same Orin wall clock**. Streams align without any offset
estimation. The Orin clock was ~11.5 min behind the VM on 2026-09-26; this does not
matter inside an episode, but don't compare episode times with VM-side logs.

## Measured load (640x480 color + depth + all topics)

Recording kept 30.0 fps on both cameras and ~990 Hz lowstate. 1.8 TB free on the Orin.

## Limitations

- Callback-time stamps (`t`) include Python/DDS latency; `lowstate.tick` (ms, robot)
  is the more precise clock for joint data.
- Max lowstate gap was 123 ms, most likely at chunk writes. Lower `--flush` if that matters.
- The recorder does not move the robot. Commands you send from the VM (e.g. `arm_move.py`
  on `rt/arm_sdk`) are captured by the robot-side subscription — not yet tested with live commands.
