# 06 — Robot state topics (DDS)

`recorder.py` subscribes on the Orin (`ChannelFactoryInitialize(0, "eth0")`, domain 0) to:

| Topic | IDL type | File prefix | Measured rate |
|---|---|---|---|
| `rt/lowstate` | `unitree_hg LowState_` | `lowstate` | ~990 Hz |
| `rt/arm_sdk` | `unitree_hg LowCmd_` | `arm_sdk` | only when something commands the arms |
| `rt/dex1/left/state` | `unitree_go MotorStates_` | `dex1_left_state` | ~494 Hz |
| `rt/dex1/right/state` | `unitree_go MotorStates_` | `dex1_right_state` | ~494 Hz |
| `rt/dex1/left/cmd` | `unitree_go MotorCmds_` | `dex1_left_cmd` | only when commanded |
| `rt/dex1/right/cmd` | `unitree_go MotorCmds_` | `dex1_right_cmd` | only when commanded |

Topics with no messages produce no files; `inspect_episode.py` lists them as
`no messages on: ...`.

## Fields saved (one row per received message)

Every row also has `t` = `time.time()` on the Orin at callback time (epoch s, float64).

**lowstate**

| Key | Shape | Meaning |
|---|---|---|
| `tick` | (N,) | robot's ms counter from the message |
| `mode_machine` | (N,) | 5 = 29-DoF G1 |
| `q`, `dq`, `tau` | (N, 35) | joint position (rad), velocity, estimated torque, per motor slot |
| `imu_quat` | (N, 4) | w, x, y, z |
| `imu_gyro`, `imu_acc`, `imu_rpy` | (N, 3) | angular velocity, acceleration, roll/pitch/yaw |

35 slots: the G1 29-DoF body uses 0–28; 29–34 are unused. Joint order follows Unitree's
G1 29-DoF index (legs 0–11, waist 12–14, left arm 15–21, right arm 22–28).

**arm_sdk** (commands seen on the bus): `mode_machine`, `q`, `dq`, `tau`, `kp`, `kd`, each (N, 35).
In the arm_sdk convention, slot 29's `q` is the blend weight (0 → 1) used by `arm_move.py`.

**dex1_*_state**: `q`, `dq`, `tau` (N,) from `states[0]` (`tau` = `tau_est`).
**dex1_*_cmd**: `q`, `dq`, `tau`, `kp`, `kd` (N,) from `cmds[0]`. Empty lists → NaN.

## Chunking

Rows are buffered in memory and written every `--flush` seconds (default 10) as
`state/<prefix>_0000.npz`, `_0001.npz`, … . A crash loses at most the last chunk.
`inspect_episode.load_state(ep, prefix)` concatenates them.

## Observations from the test episode (17 s)

- lowstate: 17016 rows, **682 rows repeat the previous `tick`** (~4 %). The same state
  arrived twice; drop duplicates with `np.diff(tick) != 0` if needed.
- lowstate max gap 123 ms, dex1 state max gaps 124/218 ms — likely Python pauses during
  chunk writes. Median spacing is ~1 ms (lowstate) / ~2 ms (gripper).
- All values are float64: ~3.6 GB per hour of state. Switch to float32 if storage matters.

## Why not record lowstate on the VM?

Over the VM link it arrives at 172 Hz at best, with gaps up to seconds (see 01).
On the Orin it arrives at ~1 kHz on the local network interface.

## Adding a topic

Add one line to `TOPICS` in `robot/recorder.py`: `(topic, IdlType, row_fn, file_prefix)`,
where `row_fn(msg)` returns a dict of scalars/lists with the same keys every call.
Then `rec.sh start` (it copies the new recorder automatically).
