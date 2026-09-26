# 09 — Troubleshooting log

Every failure met while building this stack (2026-09-26), with the cause and the fix.

## Connection

| Symptom | Cause | Fix |
|---|---|---|
| ping 161/164 fails, `ip neigh` shows `FAILED` | robot off, cable, or VirtualBox bridge on the wrong NIC | power / cable / VM network settings (01) |
| `check.sh` WARN sysctl tuning, lowstate ~22 Hz | VM rebooted, tuning was not persisted | `sudo g1_connect/setup_host.sh --persist` |
| `sudo: a password is required` from scripts | sudo is not passwordless on VM or robot | run it yourself: `! sudo <cmd>` in Claude Code, or a terminal |
| Loco RPC returns 3102 / 3104 | lossy link (`RPC_ERR_CLIENT_SEND`) | retry (`rpc_retry` in `g1_conn.py`); use `rt/arm_sdk` for arms |

## Robot environment

| Symptom | Cause | Fix |
|---|---|---|
| `ensurepip is not available` creating a venv on the Orin | `python3.8-venv` not installed, no internet | private `pylib` folder with `pip --target` (03) |
| `No module named 'typing_extensions'` | cyclonedds depends on it | copy the file from conda env `g1brainco` (03) |
| DDS topic listing via builtin topics hung | not investigated (not needed) | topic names taken from the gripper service log instead |

## Recording

| Symptom | Cause | Fix |
|---|---|---|
| `rec.sh start`: `cat: current: No such file`, `tail: cannot open '/recorder.log'` | `cd dir && ... &` backgrounds the whole list, so `echo $! > recorder.pid` ran in `~` | fixed in `rec.sh`: `cd` first, then start only python in the background |
| `error writing header: Invalid data found when processing input` for `color.mkv`, recorder stops after ~1 s | Matroska tee slave needs the codec's global header | `-flags +global_header` (and `bsfs/v=dump_extra` for the TS preview) |
| `color_ts.txt` has only the header, mkv unreadable | used `-t N` together with `-copyts` | never use `-t` with epoch timestamps; stop with SIGINT |
| mkv duration shows ~1.79e9 s | `-copyts` keeps epoch times in the container | recorder re-zeros after stop (05) |
| `ffmpeg_exit: 255` in meta.json | ffmpeg's exit code after SIGINT | normal |
| `[Reader] take sample error` at stop | subscriber closed while DDS was delivering | harmless |
| `ffmpeg color exited early` right after start | `/dev/video4` still held by videohub (or another program), or unsupported `--size` | `g1_connect/services.sh stop head_cam`, `fuser /dev/video4`; valid sizes in 05 |
| `view.py` shows nothing after recording | videohub was stopped by `rec.sh start` | `./rec.sh release` |
| `check.sh` reports `videohub_pc4` not running | same | same |

## Quick diagnostics

```bash
./rec.sh status                                          # recording? last log lines
ssh unitree@192.168.123.164 'pgrep -af "recorder.py|ffmpeg"'
ssh unitree@192.168.123.164 'cat ~/g1_rec/$(cat ~/g1_rec/current)/recorder.log'
ssh unitree@192.168.123.164 '/unitree/sbin/mscli listservice'
ssh unitree@192.168.123.164 'fuser /dev/video4 /dev/video0'    # who holds the camera
```

## Stale recorder

If the VM lost connection during an episode, the recorder keeps running on the robot
(started with `nohup`). `./rec.sh status` / `./rec.sh stop` work again once reconnected.
If `recorder.pid` points to a dead process, `rec.sh` treats it as not recording.

Stop a stray recorder by hand:
```bash
ssh unitree@192.168.123.164 'pkill -INT -f recorder.py'
```
SIGINT (not SIGKILL) so ffmpeg can finalize the files.

## Known oddities (not errors)

- ~4 % of lowstate rows repeat the previous `tick` (06).
- The Orin clock is ~11.5 min behind the VM clock (04).
- Gripper service log reports motor id 1 (left) not replying, yet left state messages arrive (02).
