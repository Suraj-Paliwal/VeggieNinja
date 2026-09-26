# STARTER — connect to the G1 and make it move from the PC

Standard procedure, in order. Everything here was done on this robot on 2026-09-26/27 unless marked
**(untested on hardware)**. Details and troubleshooting: `Guide/` (report numbers in brackets).

## 0. Safety rules (always)

- Robot on the **gantry** until it is balancing; rope **slack but attached** while it stands.
- One person holds the **e-stop** whenever anything can move. Nobody within arm's reach of the robot.
- Stop from the PC: web UI **STOP** / **Esc** (also interrupts a running okra reach/pick over ssh), Ctrl-C in
  the `okra_pick.sh reach/pick` terminal, or `robot_agent/g1ctl stop`. If the ssh link drops during a
  reach/pick, the robot aborts on its own (back along the path, blend out) [16].
- Every okra step runs a **live safety gate** first (robot measured at that moment); a FAIL means
  the step did not run — read the FAIL lines, fix the cause, run it again [16].
- Never send `zero` (limp) or `damp` to a robot that is standing on its own: it will fold.

## 1. Hardware and network

1. Robot hanging on the gantry, feet a few cm above the floor. Battery in, **power on**.
2. Ethernet cable: laptop ↔ robot.
3. VirtualBox → VM → Settings → Network: the adapter for `enp0s8` is **bridged to the laptop NIC that
   goes to the robot** (check this again after the laptop changed networks).
4. Wait ~1–2 min: the onboard computer (Orin) boots, and joint states appear ~1–2 min after power-on.

| Address | What |
|---|---|
| 192.168.123.100 | VM (`enp0s8`) |
| 192.168.123.161 | motion controller (ping only) |
| 192.168.123.164 | Orin onboard PC, `ssh unitree@192.168.123.164` (key, no password) |

## 2. Check the connection

```bash
~/Junction/g1_connect/check.sh          # expect: RESULT: ALL OK
```
- ping fails / `FAILED` in `ip neigh` → robot off, cable, or VirtualBox bridge on the wrong NIC [01].
- `sysctl tuning` WARN → `sudo ~/Junction/g1_connect/setup_host.sh --persist` (once per VM install) [01].
- A "ping onboard pc" failure that passes on a second try is a glitch, not a problem.

## 3. Start the robot agent (fast commands from the PC) **(untested on hardware)**

```bash
cd ~/Junction/robot_agent
./agent.sh deploy && ./agent.sh start   # after every robot boot
./g1ctl status                          # FSM, mode, tilt, knee load, balance check
```
Web alternative (recommended for operators): `cd ~/Junction/okra_pick && ./okra_pick.sh web` → open
http://localhost:8080 (big STOP button, all steps below as buttons) [13].
If the agent is not available, every step below also works with the proven scripts in `g1_connect/`
(`robot_mode.sh`, ~8 s per command) [10].

## 4. Bring-up: from power-on to balancing

| Step | Robot must be | PC command (agent) | Proven script | What you should see |
|---|---|---|---|---|
| a. motion service on | hanging | `g1ctl ai` | `robot_mode.sh ai` | status shows mode `ai` (after every reboot the mode is empty and nothing responds) |
| b. damp | hanging / supported | `g1ctl damp` | `robot_mode.sh damp` | FSM 1; joints resist slowly, nothing moves |
| c. locked stand | **hanging freely, feet off the floor**, nobody near the legs | `g1ctl ready` | `robot_mode.sh ready` | FSM 4; **legs move** to the standing pose, knees ≈ 0.59 rad both sides, waist straightens |
| d. put it down | — | (operator) lower the gantry until both feet are flat, **hold the torso upright**, rope just slack | — | knee load rises above 15 Nm; locked stand does NOT balance — it tips forward if left alone |
| e. balance | feet flat, upright (< 3°), knees > 15 Nm | `g1ctl start` | `robot_mode.sh start` | FSM **200** (AI balance); refused if not upright/loaded; tilt watched 5 s |

Notes:
- FSM **500** (SDK "Start") is acknowledged but ignored on this firmware; `start` sends 200.
- With the Unitree remote, R1 + X from locked stand gave FSM 501 (also works). In 501 the robot ignored
  walking commands from the PC (probably because the remote had control) — use 200 from the PC.
- Status numbers: 0 zero torque (limp), 1 damp, 4 locked stand, 200/500/501 balancing.

## 5. Make it move from the PC (robot balancing, FSM 200)

| Action | Agent / web | Proven script |
|---|---|---|
| step sideways / forward | `g1ctl left 20`, `right 10`, `forward 20`, `back 20` (≤ 40 cm per command) | `robot_mode.sh step 0.1 0 2.0` (vy vx dur) |
| look (turn waist, then back) | `g1ctl head 0.15` (+ left) / `head -0.15` / `head 0 0.1` (down) | `g1_record/rec.sh head --yaw 0.15 --hold 3` |
| gripper (Dex1, right) | `g1ctl gripper open` / `close` | `okra_pick/okra_pick.sh gripper open` |
| stop | `g1ctl stop` / web STOP / Esc (both also interrupt the okra arm player) | Ctrl-C in the okra terminal; web STOP works without the agent |

Body awareness: web pill **body Δ mm · feet ok** / `g1ctl status` → `body` (pelvis shift vs the feet since
the last anchor; red **FOOT MOVED** if the feet disagree). It re-anchors after each step; `g1ctl anchor`
or **Re-anchor body** after moving the robot by hand. The okra reach corrects for body motion up to 3 cm
and aborts beyond that [14].
Every walk and waist command checks balance first and aborts on tilt. Waist moves are relative to where
the waist is. Limits: speed ≤ 0.2 m/s, 2 s per step, waist ±0.6 rad yaw / ±0.3 rad pitch.

## 6. Record what the robot sees

```bash
cd ~/Junction/g1_record
./rec.sh start NAME        # camera 30 fps + all joint/gripper data, recorded ON the robot
./rec.sh preview           # optional live view on the VM
./rec.sh stop && ./rec.sh pull      # -> g1_record/data/NAME_<time>/
./rec.sh release           # give the camera back to Unitree's viewer
```
MP4s go to `~/Junction/recordings/` [04, 08].

## 7. Okra pick (human-in-the-loop)

Web UI → **1 Look → 2 Is it okra? → 3 Plan → 4 Reach test / Pick → 5 Outcome** [11, 13].
Pods must hang ~0.85–0.95 m above the floor, 35–50 cm in front, slightly to the robot's right.

```bash
cd ~/Junction/okra_pick
./okra_pick.sh deploy                 # after code changes; ends with a live safety check
./okra_pick.sh safety                 # any time: PASS / WARN / FAIL per check (web: "Safety check")
OKRA_WEIGHTS=okra_seg_s02_3way.pt ./okra_pick.sh go     # look + human check + plan + sim video
./okra_pick.sh dry                    # robot-side checks, nothing moves
./okra_pick.sh reach                  # first real motion: no grasp, from a TERMINAL (Ctrl-C ready)
./okra_pick.sh pick                   # only after a good reach
```
The gate checks, live: link, robot interface, joint-state rate, FSM 200/500/501, tilt, knee load (robot
standing on its feet, rope slack), battery, motor temperature/errors, nothing else commanding the arm,
camera, disk, and that the look is < 5 min old **on the robot clock** with the waist unchanged. Full
run-day checklist: Guide 16 §7. Before the first grasp: calibrate `TCP_XYZ` / `CLOSE_AXIS` [11].

## 8. Shut down

1. Stop recording (`rec.sh stop`), finish any motion (`g1ctl stop`).
2. Take up the gantry rope so it **carries the robot**.
3. `g1ctl damp` (or `robot_mode.sh damp`) — only now, while the gantry holds it.
4. Power off the robot. `./agent.sh stop` is not needed (it ends with the robot).

## 9. LEDs

| Situation | LED | Source |
|---|---|---|
| robot asks "is this okra?" | amber | set by our software (robot_agent `led`) **(untested on hardware)** |
| operator confirmed / rejected | green / red | set by our software **(untested on hardware)** |
| boot, damp, locked stand, balance, errors | **not verified — fill in from the Unitree G1 manual or by observing the robot** | — |

## 10. Quick troubleshooting

| Symptom | Fix |
|---|---|
| nothing moves, commands "accepted" | check FSM: 0 = limp (run `ai`, `damp`, `ready`, `start`); in 501 use the remote or switch to 200 |
| Loco errors 3102/3104 from the VM | normal over the VM link — use the agent or scripts that run on the robot |
| tips forward in locked stand | take up the rope, straighten it, then `start` only when < 3° |
| camera viewer black | `g1_record/rec.sh release` (a recording or the okra pipeline holds the camera) |
| robot did not step | speeds below ~0.1 m/s may be ignored; check FSM 200 and that the remote is not in control |
| `SAFETY GATE: '<step>' not run` | a check FAILed: the line above names it (e.g. `arm_sdk_free`, `knee_load`, `event_age`); fix and rerun [16] |
| `perception is N s old on the robot clock` | the look is too old or the waist moved: `look` again, then `plan` |
| `battery unknown` WARN | no BMS message was received: read the battery on the robot/remote yourself |
