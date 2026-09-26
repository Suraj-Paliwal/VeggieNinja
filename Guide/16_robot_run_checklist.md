# 16 — Robot run day: what runs where, VM limits, safety gate, checklist

Written 2026-09-27 before the first real okra reach/pick. Read with 11 (pipeline), 13 (web UI), 15 (VM).

## 1. What runs where (Python SDK = unitree_sdk2py over DDS)

| Step | Runs on | SDK/DDS? | Link from the VM |
|---|---|---|---|
| `g1_connect/robot_mode.sh` ai / damp / ready / start | Orin (ssh) | yes | ssh only |
| `okra_pick.sh look` / `perceive` (camera + detector + lowstate) | Orin, conda `g1brainco` | yes | ssh; bundle back by rsync |
| human check, `plan`, sim video, web UI | VM | no | — (VM CPU) |
| `okra_pick.sh dry / reach / pick / gripper` → `arm_player.py` | Orin, `python3` + `~/g1_rec/pylib` | yes | ssh must stay up → see §3 |
| recording `g1_record/rec.sh` | Orin | yes | ssh |
| `robot_agent` (status, STOP, walk, head) | Orin | yes | TCP :7777 |

**Rule: all SDK code runs on the Orin.** The VM link loses UDP fragments (01), which breaks DDS
from the VM (2 s gaps, RPC 3102/3104) — but ssh/TCP/rsync retransmit, so the pipeline is not affected.
Do **not** run `g1_video/*` (`view.py`, `arm_move.py`, `record.py --head`, `waist.py`, `g1_fsm.py`)
from the VM on a run day: they speak DDS across the lossy link.

## 2. STOP — the layers, fastest first

| Layer | How | Latency |
|---|---|---|
| physical e-stop | in hand, always | instant, independent of everything |
| Ctrl-C in the `./okra_pick.sh reach/pick` terminal | SIGINT through the pty | < 0.1 s |
| web **STOP** / Esc | `server.stop_all()`: ssh `pkill -INT` of arm_player **and** agent `stop` in parallel | ~1 s (ssh) |
| agent `stop` (`g1ctl stop`) | zero velocity ×3, cancel agent task, SIGINT to arm_player | ~0.1 s once deployed |

SIGINT = the player's clean abort: before/during the pull → open gripper if closed, go back along the
executed path, blend out; after the pull → stop there, keep the pod, blend out.
Until 2026-09-27 web STOP only reached the agent (zero velocity) and **did not stop the arm**.

## 3. Operator link lost (ssh drops, VM freezes, terminal/web server closed)

`arm_player.py` treats it as STOP and finishes the safe exit **on the robot**:

| What happens | Before the fix | Now |
|---|---|---|
| `ssh -t` dropped (terminal) → SIGHUP + EIO | process killed mid-motion | abort + return + blend out |
| `ssh -T` dropped (web) → next print EPIPE | crash mid-motion | abort + return + blend out |
| SIGTERM / any unexpected exception | crash mid-motion | same safe exit |

Everything the player prints also goes to `~/okra_pick/runs/<id>/player.log` (copied back into the event
folder after the run). Test: `tests/test_link_loss.py <trajectory.json>` → 4/4 PASS (VM, fake robot).

## 4. Safety gate before every step (nothing about the robot is assumed)

`okra_pick.sh` runs `safety/safety_check.py STEP` before gripper, floor, look/perceive, plan, dry, reach and
pick — reach/pick/gripper **again right after you type the confirmation word**. It runs
`robot/safety_probe.py` on the Orin (read-only, ~2 s) and measures everything live: robot clock, network
interface (route to the motion controller → passed as `--iface`, no fixed `eth0`), lowstate rate and gaps,
FSM + switcher mode, tilt, knee load, per-motor temperature/error bits, Dex1 state, **who is already publishing
`rt/arm_sdk` / `rt/dex1/right/cmd`**, motion programs running, battery (BMS if it answers, else "unknown"),
RealSense on USB, disk and Orin temperature. FAIL = the step does not run (exit 3).

| Check | look/floor | plan | gripper | dry/reach/pick |
|---|---|---|---|---|
| ssh link, SDK imports, interface route, lowstate ≥ 100 Hz and gap ≤ 0.4 s | FAIL | FAIL | FAIL | FAIL |
| camera on USB | FAIL | – | – | – |
| perception age ≤ 300 s **on the robot clock**, waist unchanged since perception | – | FAIL | – | FAIL |
| nobody else on arm_sdk / gripper cmd, no motion program, Dex1 state alive | – | – | FAIL | FAIL |
| FSM in 200/500/501, knee load ≥ 15 Nm, tilt ≤ 3°, right-arm error bits, battery ≥ 30 % | warn | warn | warn | FAIL |
| arm within 0.15 rad of the plan's start pose | – | – | – | FAIL |
| mode_machine = first value measured on this robot (`okra_data/safety/robot_baseline.json`) | warn | warn | warn | FAIL |
| battery unknown, motor ≥ 70 °C, Orin ≥ 85 °C, VM busy, recorder already running | warn | warn | warn | warn |

- Limits (policy) live in `safety/safety_limits.py` / `robot/pick_config.py`; unverified ones only warn.
- Every result is saved: `<event>/safety/<step>_<time>.json` (or `okra_data/safety/<day>/`).
- By hand: `./okra_pick.sh safety [STEP] [EVENT]`; web: **Safety check** / **Safety: reach** buttons.
- One-run override for a named FAIL (logged): `OKRA_SAFETY_ACCEPT=battery ./okra_pick.sh reach`.
- Player preflight (on the robot, at start) repeats FSM, tilt, lowstate, start pose, waist and the
  perception age — also on the robot clock (`perceived_at_robot` travels robot_state → target → trajectory).
  A plan without it (synthetic) is refused. During the move the player monitors every tick (11, 14).
- Tests: `tests/test_safety.py` → 26/26 (21 gate cases, 1 operator-accept case, 4 player-age cases).
- Fixed by this: the old plan-age check compared the VM clock with the Orin clock (~11.5 min behind),
  which silently disabled it.

## 5. Known issues (still open)

- **Agent not deployed yet** (12): web STOP still works via ssh; voice/LED prompts do nothing.
- **Waist/head during a pick**: the gate refuses to start if anything publishes `rt/arm_sdk`, but does
  not stop a waist command sent after the start — don't use look/head while reach/pick runs.
- **Recording can fail silently** in reach/pick: watch for the line `recording started`.
- **Uncalibrated:** `TCP_XYZ`, `CLOSE_AXIS = "y"`, `GRIP_EMPTY_Q` (`robot/pick_config.py`). Wrong
  `CLOSE_AXIS` = jaws 90° off, and the 45° pull twist is about the same axis.
- **Gantry with feet off the floor** fails `knee_load` for reach/pick (the robot is not balancing then).
- `tests/test_hitl.py` case 1 fails (older issue, labelling only): the out-of-reach second candidate no
  longer appears in the question.

## 6. Detector choice

| Model | Held-out numbers | Note |
|---|---|---|
| `okra_seg_s02.pt` (pipeline default) | val mask mAP50 0.96 (30 val images) | no separate test split |
| `okra_seg_s02_3way.pt` | **test** mask mAP50 0.83, P 1.00 / R 0.71; full detector R 0.70 P 1.00 | `okra_robot/report/REPORT.md` |

Use the 3-way model: `OKRA_WEIGHTS=okra_seg_s02_3way.pt ./okra_pick.sh look`. Misses are mostly small,
far or edge-cut pods; the base checkpoint finds 0 of 90 test pods.

## 7. Checklist (in order)

VM
1. `df -h /` ≥ 15 GB free (`uv cache prune`, 15). No green turtle, laptop on charger.
2. `~/Junction/g1_connect/check.sh` → `RESULT: ALL OK` (robot powered).
3. After bring-up (step 4): `okra_pick/okra_pick.sh safety status` → no FAIL (§4).

Robot bring-up (gantry, e-stop in hand)
4. `g1_connect/robot_mode.sh ai` (after a reboot) → `damp` → `ready` → `start` (FSM 200).
5. `okra_pick/okra_pick.sh deploy` → `perceive env ok, cuda True` and `player env ok`.
6. Optional: `robot_agent/agent.sh deploy && robot_agent/agent.sh start` (12).

Calibration and dry runs
7. `./okra_pick.sh floor` (camera mount; 1° ≈ 9 mm at the pod).
8. `./okra_pick.sh gripper open` / `close` → check `CLOSE_AXIS`, `TCP_XYZ` by eye.
9. STOP test (do once, on the first `reach` in step 12): while the arm moves in `to_pregrasp`, press web
   STOP → the terminal shows `ABORT … stopped` and the arm goes back; the web log shows
   `python3 killed (pid …)` ("killed" is pkill's wording; it sends SIGINT). Then run `reach` again.

Okra (pod 0.85–0.95 m high, 0.35–0.50 m in front, 0–0.2 m to the right; 11)
10. `look` (or `perceive`) → answer the human check → `plan` → watch `sim.mp4`.
11. `./okra_pick.sh dry` → `preflight ok`.
12. `./okra_pick.sh reach` **from a terminal** (Ctrl-C ready) → type `reach`.
13. Only if reach looked right: `./okra_pick.sh pick` → `outcome` is asked afterwards.
Keep the VM idle during 12–13 (no training, no heavy jobs, no sleep).

## 8. Files changed for this (2026-09-27)

```
okra_pick/robot/arm_player.py   SafeOut + SIGHUP/SIGTERM/SIGINT abort, any error → safe exit, player.log
okra_pick/web/server.py         stop_all(): ssh pkill -INT arm_player + agent stop
robot_agent/agent.py            stop also SIGINTs arm_player (interrupt_arm_player)
okra_pick/okra_pick.sh          copies player.log back after reach/pick
okra_pick/tests/test_link_loss.py   link-loss tests (4 cases)
okra_robot/evaluate_test.py, okra_robot/report/   base vs fine-tuned on the test split
okra_pick/robot/safety_probe.py     live read-only robot probe (Orin)
okra_pick/safety/safety_check.py, safety_limits.py   the gate + policy limits (VM)
okra_pick/okra_pick.sh              gate before every step, measured --iface, `safety` command
okra_pick/hitl/confirm.py, plan/okra_plan.py   carry perceived_at_robot (robot clock) into the plan
okra_pick/web/server.py, index.html  /api/safety + Safety buttons
okra_pick/tests/test_safety.py      gate + player-age tests
```
