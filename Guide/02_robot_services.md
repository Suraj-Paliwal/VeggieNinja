# 02 — Robot services on the Orin

All of these already run **on the robot's Orin** (192.168.123.164) at boot.

| Service | Process | Started by | Runs as | Control |
|---|---|---|---|---|
| head camera | `videohub_pc4 /dev/video4` | Unitree `master_service` | root | `mscli` (no sudo) |
| chest camera | `videohub_pc4_chest /dev/video10` | Unitree `master_service` | root | `mscli` (no sudo) |
| Dex1 gripper | `dex1_1_gripper_server` | systemd `dex1_gripper.service` (`Restart=always`) | unitree | `sudo systemctl` |
| supervisor | `master_service` | systemd `master_service.service` | root | not touched |
| OTA | `ota_pipe_service` | master_service | root | not touched |

`/unitree/sbin/mscli` is Unitree's CLI for services under `master_service`:
`listservice`, `getservice NAME`, `startservice NAME`, `stopservice NAME`, `restartservice NAME`, …
It works as user `unitree` without sudo. `master_service.json` is encrypted; use `mscli`.

The gripper unit restarts itself after 5 s if killed, so it must be stopped via systemd,
which needs the robot password (sudo on the Orin is not passwordless).

## Controller: `g1_connect/services.sh` (runs on the VM)

```bash
./services.sh                     # status of all
./services.sh stop cams           # both cameras
./services.sh start head_cam
./services.sh restart gripper     # asks for the robot sudo password
./services.sh stop all
```
Names: `head_cam`, `chest_cam`, `gripper`, `cams`, `all`.
After `start|stop|restart` it prints status again.

Status output example:
```
#1 name:video_hub_pc4, starttime:1790401308, status:0, enable:1
#2 name:video_hub_pc4_chest, starttime:1790401309, status:0, enable:1
gripper (dex1_gripper.service): active
2162 /unitree/module/video_hub_pc4/videohub_pc4 /dev/video4
```
`status:0` = running, `status:1` = stopped. `mscli stopservice` also sets `enable:0`.
`startservice` brings it back (verify after a reboot whether `enable:0` persists).

## Interaction with recording

The head RealSense can be streamed by one program at a time. `g1_record/rec.sh start`
stops `video_hub_pc4` automatically; `rec.sh release` starts it again. While it is
stopped, `g1_video/view.py` and `record.py` get no frames.

The gripper server must stay **running** during recording: its `rt/dex1/*/state` topics
are what the recorder logs (~494 Hz per side).

## Gripper details (from the service log)

- Topics: `rt/dex1/left|right/cmd` (`unitree_go MotorCmds_`) and
  `rt/dex1/left|right/state` (`unitree_go MotorStates_`).
- Serial: `/dev/ttyUSB0`. In the 2026-09-01 log, motor id 1 did not reply and only
  the right motor (id 0) was detected, yet state messages arrive for both sides.
  Check which side is physically attached before trusting `left` data.
- Source: `~/dex1_1_service/main.cpp`; test subscriber `~/dex1_1_service/test/test_gripper_sub.py`.

## Other things on the Orin (not used by this stack)

`~/workSpace/dimos_min`, `~/workSpace/g1_joint_logger`, `~/workSpace/teleimager`,
`~/run_ik_camera.sh` (kills videohub), conda envs `g1brainco` and `teleimager`.
