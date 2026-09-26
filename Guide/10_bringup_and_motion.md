# 10 — Robot bring-up and waist motion (with balance conditions)

## Tools

| File | Runs on | Purpose |
|---|---|---|
| `g1_connect/robot_mode.sh` → `robot_mode.py` | VM → robot | LocoClient FSM: `status`, `damp`, `ready`, `start`, `zero` |
| `g1_record/head_teleop.py` | VM | aim the head camera by turning the waist (`rt/arm_sdk`) |
| `g1_video/waist.py` | VM (library) | waist controller used by `head_teleop.py` and `record.py --head` |

`robot_mode.py` runs **on the Orin** because Loco RPC from the VM fails (3102) over the lossy
link; from the Orin it answers at once. It uses `~/g1_rec/pylib` (03).

## FSM ids (this firmware, confirmed by the SDK, dimos and `g1_fsm.py`)

| id | State | Notes |
|---|---|---|
| 0 | zero torque | limp; the robot was in this state on 2026-09-26 → arm_sdk had no effect |
| 1 | damp | soft, resists motion; nothing moves by itself |
| 4 | stand up / locked stand | legs go to a fixed standing pose and **freeze**; does not balance |
| 200 | AI balance | what `robot_mode.sh start` sends; worked earlier on 2026-09-26 (arms moved) |
| 500 | main control | SDK `Start()`; on this firmware **acknowledged but ignored** from locked stand |
| 501 | main control | what the remote (R1 + X) gave; waist moves and remote walking worked |

## Bring-up sequence

```bash
cd ~/Junction/g1_connect
./robot_mode.sh status        # read-only, ends with "start ok?: True/False <reason>"
./robot_mode.sh ai            # after a reboot, if the switcher mode name is '' (Loco RPC then fails 3102)
./robot_mode.sh damp          # robot supported
./robot_mode.sh ready         # robot HANGING, feet clear of the floor: legs move to stand pose
#   operator: lower the gantry until both feet are flat, torso upright, rope just slack
./robot_mode.sh start         # refused unless the balance condition holds
```

## Balance conditions

**`robot_mode.sh start`** (checked on the robot, before sending FSM 500):
- |roll| and |pitch| < 3° (`MAX_TILT = 0.05` rad) — never skippable
- |L knee τ| + |R knee τ| > 15 Nm (`MIN_KNEE_LOAD`) — legs carry the weight;
  `start --ignore-load` skips only this part
- after sending: 5 s of tilt monitoring, max tilt printed

**`head_teleop.py`** (checked on the VM from `rt/lowstate`):
- refuses to engage if |roll| or |pitch| > 6° (`START_TILT`) — nothing is sent
- while engaged, aborts (center + release) if the tilt drifts > 5° from the start (`ABORT_DRIFT`)
- also aborts if no `rt/lowstate` arrives for 0.5 s (`STALE_S`): the balance can't be seen.
  On the lossy VM link this can trigger on a normal gap; that aborts on the safe side.

Offline tests (fake robot, 2026-09-26): upright → full move and release; tilted 11.5° → refused,
0 commands; tipping during hold → abort at 5.9°; state stops → abort after 0.5 s.

## Waist moves are relative to the start pose

`waist.py` stores the waist pose at `engage()` as *home*. Goals, limits (yaw ±0.6, pitch ±0.3 rad),
`center()` and release are all relative to home. Before this fix it used absolute 0: with the
waist at −0.613 rad (limp robot), "center" meant a ~35° swing.

**Run it on the robot** (`./rec.sh head ...`): copies `head_teleop.py` + `waist.py` to `~/g1_rec`
and runs them on the Orin (`--iface eth0`). From the VM the 0.5 s stale-state abort fired
mid-move and only ~17 of 50 commands/s arrived; on the Orin it ran cleanly.

```bash
cd ~/Junction/g1_record
./rec.sh head --yaw -0.15 --hold 3                           # on robot: ~9° right, hold, back
./rec.sh head                                                # on robot: keys (ssh -t, real terminal)
../g1_video/.venv/bin/python head_teleop.py --yaw -0.15      # VM-side (not recommended)
../g1_video/.venv/bin/python head_teleop.py --sweep          # left/right/up/down/center
../g1_video/.venv/bin/python head_teleop.py                  # keys a/d/w/s, c, q (real terminal)
```
Combine with recording: `./rec.sh start NAME` → move → `./rec.sh stop` (04).

## What happened on 2026-09-26

1. FSM 0: waist command recorded (288 arm_sdk msgs arrived, ~36 Hz of 50) but ignored.
2. `damp` → `ready`: knees went from −0.02/+0.58 to +0.59/+0.59 rad, waist yaw −0.613 → 0.
3. `start` sent with feet down but rope carrying the weight: code 0, **FSM stayed 4**.
4. Rope slackened: pitch grew 3.5° → 10° → 15.5° (locked stand does not balance); stopped,
   rope re-tensioned. Balance gate added. Knee load stayed 5–10 Nm throughout.

5. Operator put it in main control (FSM **501**; balance check passed: pitch 2.0°, knees 26 Nm).
6. `move_right02` (VM-side): stale-state abort after blend-in; waist did not move; tilt ≤ 2°.
7. `move_right03` (`rec.sh head`, on robot): waist yaw 0 → **−0.167 rad (9.6°)** → 0, max tilt
   2.9°, video shows the view turning right.

## Open questions

- The recorder logged only 66 of ~400 `rt/arm_sdk` messages (gap 2.9 s) even with the sender
  on the same Orin; the real motion is in `lowstate.q[:,12]`. Cause not investigated yet.

- Whether FSM 500 is refused because of low foot load or needs the remote (R1 + X) on this firmware.
- `tau_est` values in locked stand looked low for a standing robot; the 15 Nm threshold is a guess.
