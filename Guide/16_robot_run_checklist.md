# 16 — Robot run day: what runs where, VM limits, safety, checklist

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

## 4. Known issues (not fixed)

- **Clock skew:** preflight refuses a plan older than 300 s, but compares `planned_at` (VM clock) with
  the Orin clock, which is ~11.5 min behind (code_repeatability.md). The check is then effectively off
  (plans up to ~16 min pass); if the Orin were ahead it would refuse every plan.
  Check: `date +%s; ssh unitree@192.168.123.164 date +%s`. Workaround: plan right before reach/pick.
- **Agent not deployed yet** (12): web STOP still works via ssh; voice/LED prompts do nothing.
- **Waist/head vs pick:** agent `head` and `arm_player` both write `rt/arm_sdk` — never use
  look/head buttons while reach/pick runs.
- **SDK import is lazy:** `deploy` passes even if `unitree_sdk2py` is missing. The first real check is
  `dry` (player env) and the first `look` (perceive env).
- **Recording can fail silently** in reach/pick: watch for the line `recording started`.
- **Uncalibrated:** `TCP_XYZ`, `CLOSE_AXIS = "y"`, `GRIP_EMPTY_Q` in `robot/pick_config.py`. Wrong
  `CLOSE_AXIS` = jaws 90° off, and the 45° pull twist is about the same axis.
- **VM disk** 87 % full (15): recordings / rsync / sim.mp4 fail when it fills. Keep ≥ 15 GB free.

## 5. Detector choice

| Model | Held-out numbers | Note |
|---|---|---|
| `okra_seg_s02.pt` (pipeline default) | val mask mAP50 0.96 (30 val images) | no separate test split |
| `okra_seg_s02_3way.pt` | **test** mask mAP50 0.83, P 1.00 / R 0.71; full detector R 0.70 P 1.00 | `okra_robot/report/REPORT.md` |

Use the 3-way model: `OKRA_WEIGHTS=okra_seg_s02_3way.pt ./okra_pick.sh look`. Misses are mostly small,
far or edge-cut pods; the base checkpoint finds 0 of 90 test pods.

## 6. Checklist (in order)

VM
1. `df -h /` ≥ 15 GB free (`uv cache prune`, 15). No green turtle, laptop on charger.
2. `~/Junction/g1_connect/check.sh` → `RESULT: ALL OK` (robot powered).
3. `date +%s; ssh unitree@192.168.123.164 date +%s` → note the skew (§4).

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

## 7. Files changed for this (2026-09-27)

```
okra_pick/robot/arm_player.py   SafeOut + SIGHUP/SIGTERM/SIGINT abort, any error → safe exit, player.log
okra_pick/web/server.py         stop_all(): ssh pkill -INT arm_player + agent stop
robot_agent/agent.py            stop also SIGINTs arm_player (interrupt_arm_player)
okra_pick/okra_pick.sh          copies player.log back after reach/pick
okra_pick/tests/test_link_loss.py   link-loss tests (4 cases)
okra_robot/evaluate_test.py, okra_robot/report/   base vs fine-tuned on the test split
```
