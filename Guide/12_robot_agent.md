# 12 — Robot agent (`robot_agent/`): fast, reliable VM → robot commands

## Why

Measured on 2026-09-26/27:

| Problem | Cause | Evidence |
|---|---|---|
| every command took ~8–10 s | each call = new ssh + Python start + DDS discovery | collection steps every 8–9 s for 2 s of walking |
| lost / late data | VirtualBox bridge loses **IP fragments** (DDS messages > 1.5 kB) | VM kernel: 39,896 fragmented packets, 24 % reassembled, 20,756 failed; unfragmented errors 0 |
| Loco RPC fails from the VM (3102/3104) | discovery messages are fragmented too | same calls answer instantly on the Orin |
| "stop" was slow | stop was also a new ssh + DDS session | ~8 s before zero velocity was sent |

## Design

```
VM                                   robot Orin (192.168.123.164)
 g1ctl  ── TCP :7777, JSON lines ──►  agent.py (persistent, PYTHONPATH=~/g1_rec/pylib)
        ◄── progress + reply ───────   one DDS participant on eth0: rt/lowstate (~1 kHz, local),
        ◄── state stream (watch) ───   Dex1 state/cmd, LocoClient, MotionSwitcher, waist (arm_sdk)
```
- TCP segments are sized to the link, so **nothing is IP-fragmented**; lost segments are retransmitted.
- All loops (tilt watch, waist control, gripper) run on the robot next to the 1 kHz state.
- Binds to the robot network address only. No authentication: keep that network private.

## Commands (`robot_agent/g1ctl`)

| Command | Does | Gate |
|---|---|---|
| `status`, `watch [HZ]`, `ping` | state: FSM, mode, tilt, knee load, gripper, lowstate Hz, busy, balance check | — |
| `stop` | zero velocity ×3 + cancel the running task | always accepted, also while busy |
| `ai` | MotionSwitcher SelectMode("ai") (after a reboot) | — |
| `damp` / `ready` / `start` | FSM 1 / 4 / 200 | `start`: upright < 3° and knees > 15 Nm; tilt watched 5 s |
| `zero --confirm` | FSM 0 (limp) | needs `--confirm` |
| `step VY [VX] [DUR] [WZ]`, `left/right/forward/back CM` | walk | balance FSM, |v| ≤ 0.2, dur ≤ 2 s, balance gate, abort > 8° tilt, always ends with zero velocity |
| `head YAW [PITCH] [HOLD] [SPEED]` | waist move from the current pose, then back | upright < 6°, abort on 5° tilt drift or stale state |
| `gripper open/close/Q` | Dex1 right | — |

One motion at a time: a second one gets `busy with <task> (send stop first)`. Every command is logged
with the client address in `~/robot_agent/agent_commands.log`.

## Running it

```bash
cd ~/Junction/robot_agent
./agent.sh deploy && ./agent.sh start      # after each robot boot
./agent.sh status | log | restart | stop
```
Not started at boot automatically (would need a systemd service with sudo on the robot).

## Test results (simulated robot, 2026-09-27; not yet on hardware)

| Test | Result |
|---|---|
| round trip `ping` | 3–5 ms (21 ms first connection) |
| `status` | 4 ms |
| `stop` during a 1.5 s step, from a second client | step cancelled in 0.4 s, reply lists `cancelled_task: step` |
| second motion while busy | refused `busy with step` |
| too-fast step, `zero` without confirm | refused |
| `left 20`, `gripper close/open`, `head`, `start` | complete; start reached FSM 200 |

## Still to do

1. Deploy on the robot, measure real round trip and `stop` latency.
2. Point the old scripts at the agent (`g1_connect/robot_mode.sh`, `g1_record/rec.sh head`,
   `okra_pick.sh reach/pick`) so everything uses the fast path.
3. Phase 0/1 from the plan (measure fragment loss; Windows NIC settings) if the VM still needs raw DDS.
