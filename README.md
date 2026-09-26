# Junction — Unitree G1 okra picking from a PC

Software to run a **Unitree G1 humanoid (29-DoF)** from an Ubuntu 22.04 PC (currently a VirtualBox VM):
connection checks, bring-up, on-robot recording, a persistent robot agent, and a **camera-guided okra
pick** — detect the pod with the head RealSense, ask a human when unsure, plan the right-arm motion, grasp
with the right **Dex1** gripper, pull with a wrist twist, and keep every decision as training data.

> **Safety first.** Robot on a **gantry** until it balances, one person with the **e-stop** in hand, nobody
> within arm's reach. Every motion ramps in and out slowly. The okra pipeline runs a **live safety gate
> before every step**, and it aborts safely on STOP or when the PC's ssh link drops (§9). Never send
> `damp` / `zero` to a robot that is standing on its own: it folds.

**Status (2026-09-27).** On the robot: connection, bring-up to FSM 200, arm and waist motion, recording,
and the detector running on the Orin GPU. Offline only (VM tests, simulation): the okra plan, the arm
player, the safety gate, STOP and link-loss handling, the web UI, and the robot agent. **The first real
reach and pick have not been done yet** — follow the run-day checklist (§5, Guide 16).

## Contents

1. [How it works](#1-how-it-works) · 2. [Repository layout](#2-repository-layout) ·
3. [Hardware and network](#3-hardware-and-network) · 4. [Software environments](#4-software-environments) ·
5. [Quick start (run day)](#5-quick-start-run-day) · 6. [Bring-up and motion](#6-bring-up-and-motion) ·
7. [Recording](#7-recording) · 8. [Okra pick pipeline](#8-okra-pick-pipeline) ·
9. [Safety features](#9-safety-features) · 10. [Okra detector](#10-okra-detector) · 11. [Tests](#11-tests) ·
12. [Robot facts](#12-robot-facts) · 13. [VM limits and tuning](#13-vm-limits-and-tuning) ·
14. [Troubleshooting](#14-troubleshooting) · 15. [Open issues](#15-open-issues) · 16. [Documentation](#16-documentation)

---

## 1. How it works

```
 PC: Ubuntu 22.04 VM (enp0s8 192.168.123.100)            Unitree G1
 ─────────────────────────────────────────────           ────────────────────────────────────────────
 okra_pick.sh / web UI (planner, human check,  ── ssh ──▶ Orin 192.168.123.164 (onboard PC, aarch64)
   sim, safety gate)                                        okra_perceive.py  camera + detector (CUDA)
 g1_connect/robot_mode.sh  (bring-up)          ── ssh ──▶   arm_player.py     plays the plan (50 Hz)
 g1_record/rec.sh          (recording)         ── ssh ──▶   recorder.py       camera + DDS logging
 robot_agent/g1ctl         (status, STOP, walk) ── TCP ──▶  agent.py          owns DDS, :7777
                                                            safety_probe.py   live read-only snapshot
                                                                 │ DDS (CycloneDDS 0.10)
                                                            motion controller 192.168.123.161
                                                              balance, walking, rt/lowstate, rt/arm_sdk
```

- The robot speaks **DDS** (publish/subscribe over UDP). Unitree's Python SDK `unitree_sdk2py` wraps it
  with **CycloneDDS 0.10.x**. There is no ROS in the loop.
- **Design rule: SDK code that moves or records the robot runs ON THE ORIN.** The VM link loses UDP
  fragments (DDS gaps up to ~2 s, RPC errors 3102/3104). ssh, TCP and rsync retransmit, so the VM only
  plans, asks the human and sends commands.
- **The built-in Unitree controller always balances the robot.** We never command the legs. Upper-body
  targets go on `rt/arm_sdk` and are blended in by a weight (`motor_cmd[29].q`, 0 = ignore, 1 = follow).
- **Nothing about the robot is assumed.** Clock, network interface, FSM, battery, temperatures and who
  else is commanding the arm are measured live before each step (§9).

## 2. Repository layout

| Folder | Runs on | What |
|---|---|---|
| `g1_connect/` | VM → robot | `check.sh` health check, `robot_mode.sh` bring-up (runs on the Orin), `services.sh`, `setup_host.sh` |
| `g1_record/` | VM → robot | `rec.sh`: record camera + state **on the robot**, preview, pull, waist `head` moves |
| `robot_agent/` | robot + `g1ctl` on the VM | persistent DDS owner: status, STOP, bring-up, steps, waist, gripper, voice/LED |
| `okra_pick/` | VM + robot | okra pipeline (`okra_pick.sh`), safety gate, planner, arm player, web UI, tests |
| `okra_robot/` | VM + robot | okra detector (YOLO11n-seg + sanity filter), dataset tools, training, evaluation report |
| `okra_data/` | VM | every look → question → answer → plan → outcome → safety result (schema in its README) |
| `g1_video/` | VM | first VM-side DDS tools (camera viewer, FSM, arm demo, waist) — lab use only (§12) |
| `dimos/` | VM | dimos framework; its `.venv` is also the Python for `okra_pick` and `okra_robot` |
| `Guide/` | — | reports 01–16 (≤ 150 lines each), index in `Guide/code_repeatability.md` |
| `STARTER.md` | — | power-on → balancing → moving → okra pick, step by step |
| `recordings/` | VM | exported MP4s (git-ignored, index in its README) |

**Not in git** (see `.gitignore`): model weights `okra_robot/models/*.pt|onnx` (~17 MB, share separately),
datasets and labelling images, `okra_data/events|safety|datasets`, recordings, training runs, `.venv/`,
and `.env` (CVAT credentials).

## 3. Hardware and network

| Address | What |
|---|---|
| 192.168.123.100 | VM, interface `enp0s8` (static, /24, no gateway) |
| 192.168.123.161 | motion controller (ping only, no login) |
| 192.168.123.164 | Orin onboard PC: `ssh unitree@192.168.123.164` (key login, no internet on the robot) |

1. Ethernet cable laptop ↔ robot.
2. VirtualBox → Settings → Network → **Adapter 2**: Bridged to the laptop's **wired** NIC, type
   **Paravirtualized Network (virtio-net)**, Promiscuous Mode **Allow All**. Adapter 1 stays NAT (internet).
3. VM network tuning, once (UDP buffers 64 MB + IP-fragment settings, persisted):
   `sudo ~/Junction/g1_connect/setup_host.sh --persist` [Guide 01].
4. Check: `~/Junction/g1_connect/check.sh` → `RESULT: ALL OK`.

## 4. Software environments

| Where | Python | Used by | Contents |
|---|---|---|---|
| VM `g1_video/.venv` | 3.10 | `g1_connect/check.sh`, `g1_video/*` | `unitree_sdk2py` (editable, `g1_video/unitree_sdk2_python`), **cyclonedds 0.10.2**, numpy, opencv |
| VM `dimos/.venv` | 3.12 | `okra_pick`, `okra_robot`, web UI, tests | ultralytics 8.4, torch (CPU), mujoco 3.x, matplotlib, opencv, unitree-sdk2py-dimos |
| Orin system `python3` + `~/g1_rec/pylib` | 3.8 | `arm_player.py`, `safety_probe.py`, `recorder.py`, `agent.py`, `robot_mode.py` | unitree_sdk2py, cyclonedds 0.10.2, numpy 1.17 (offline libs, `rec.sh setup`) [Guide 03] |
| Orin conda `g1brainco` | 3.8 | `okra_perceive.py` | torch + CUDA, ultralytics, pyrealsense2, unitree_sdk2py |

> **Use CycloneDDS 0.10.x.** The robot firmware speaks 0.10; with cyclonedds 11.x the PC received nothing.
> Code that runs on the Orin must stay **Python 3.8** compatible.

Rebuild the VM SDK venv on a new machine:

```bash
git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds ~/src/cyclonedds
cd ~/src/cyclonedds && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=OFF && cmake --build . --target install
cd ~/Junction/g1_video && uv venv --python 3.10 .venv
export CYCLONEDDS_HOME=~/src/cyclonedds/install
uv pip install --python .venv/bin/python cyclonedds==0.10.2
uv pip install --python .venv/bin/python -e unitree_sdk2_python opencv-python numpy
```

## 5. Quick start (run day)

Full procedure: [`STARTER.md`](STARTER.md); checklist and safety details: [Guide 16](Guide/16_robot_run_checklist.md).

```bash
# VM
df -h /                                          # >= 15 GB free (Guide 15)
~/Junction/g1_connect/check.sh                   # RESULT: ALL OK

# bring-up (robot on the gantry, e-stop in hand)
cd ~/Junction/g1_connect
./robot_mode.sh ai                               # after every reboot
./robot_mode.sh damp && ./robot_mode.sh ready    # hanging freely: legs move to the stand pose
# lower the gantry: feet flat, torso upright, rope slack
./robot_mode.sh start                            # FSM 200 (AI balance); refused unless upright and loaded

# okra
cd ~/Junction/okra_pick
./okra_pick.sh deploy                            # copy code + models; ends with a live safety check
./okra_pick.sh web                               # operator UI: http://localhost:8080  (or the CLI below)
OKRA_WEIGHTS=okra_seg_s02_3way.pt ./okra_pick.sh go   # look + human check + plan + sim video
./okra_pick.sh dry                               # robot-side checks, nothing moves
./okra_pick.sh reach                             # first real motion, no grasp — from a TERMINAL
./okra_pick.sh pick                              # only after a good reach; outcome is asked afterwards
```

Shut down: stop recording and motion, take up the gantry rope so it carries the robot, **then** `damp`,
then power off.

## 6. Bring-up and motion

| Step | Robot must be | Agent (`robot_agent/g1ctl`) | Proven script (`g1_connect/robot_mode.sh`) |
|---|---|---|---|
| motion service on | hanging | `ai` | `ai` (mode is empty after every reboot) |
| damp (FSM 1) | hanging / supported | `damp` | `damp` |
| locked stand (FSM 4) | hanging, feet off the floor | `ready` | `ready` |
| balance (FSM 200) | feet flat, upright < 3°, knee load > 15 Nm | `start` | `start` |
| step | FSM 200 | `left 20`, `forward 20` (≤ 40 cm, ≤ 0.2 m/s) | `step VY VX DUR` |
| look (waist) | FSM 200 | `head 0.15`, `head 0 0.1` | `g1_record/rec.sh head --yaw 0.15` |
| gripper (Dex1 right) | any | `gripper open/close` | `okra_pick.sh gripper open/close` |
| stop | any | `stop` (also interrupts the okra arm player) | web STOP / Ctrl-C |

- FSM ids: 0 zero torque (limp), 1 damp, 4 locked stand, 200/500/501 balancing. **500 is ignored on this
  firmware** — `start` sends 200. In 501 (remote R1+X) the robot ignores PC walking commands.
- The agent (`./agent.sh deploy && ./agent.sh start`) makes commands instant instead of ~8 s per ssh call.
  It is tested in sim mode, **not yet deployed on the robot** [Guide 12].
- Body awareness: pelvis shift relative to the feet (leg FK + IMU), re-anchored after each step [Guide 14].

## 7. Recording

```bash
cd ~/Junction/g1_record
./rec.sh setup                 # once: offline libs on the robot
./rec.sh start NAME [--depth]  # camera 30 fps + all joint/gripper topics, recorded ON the robot
./rec.sh preview               # optional live view on the VM (UDP 5600)
./rec.sh stop && ./rec.sh pull # -> g1_record/data/NAME_<time>/
./rec.sh release               # give the camera back to Unitree's videohub
```
Reach and pick start a recording automatically (`okra_<step>_<event>`). Layout and loaders: [Guide 04, 08].

## 8. Okra pick pipeline

```
look (robot: 10 frames, detector + depth) → candidates in the pelvis frame
  → human check (asks when conf < 0.60, confusion, unstable, out of reach, nothing found) → target.json
  → plan (VM: IK, approach, grasp, pull with 45° wrist twist, retreat, home) → trajectory.json + sim.mp4
  → dry → reach (no grasp) → pick (recorded) → outcome
```

| Command (`okra_pick/okra_pick.sh`) | Does |
|---|---|
| `deploy` | copy robot code + detector + models to `~/okra_pick` on the Orin, check both Python envs, safety check |
| `safety [STEP] [EVENT]` | run the live safety gate by hand |
| `look` / `perceive` | one look → event in `okra_data/events/<day>/<id>/`; `perceive` also asks the human check |
| `confirm [EVENT]`, `outcome [EVENT]` | re-run the human check; record success / fail / partial |
| `plan [EVENT]`, `sim [EVENT]` | plan the grasp + MuJoCo video; watch it in a window |
| `floor` | camera extrinsic check (floor plane vs leg kinematics) |
| `gripper open\|close` | Dex1 right only (calibration) |
| `dry` / `reach` / `pick [EVENT]` | preflight only / move to the pod and back / full grasp + pull + retreat + home |
| `go` | look + human check + plan + sim, then tells you to reach or pick |
| `dataset` | export human-confirmed events as a YOLO-seg training set |
| `web [--host 0.0.0.0 --token T]` | operator UI: STOP (Esc), live state, bring-up, walk, look, gripper, record, steps 1–5, Safety check |

- Workspace: pod ~0.85–0.95 m above the floor, 0.35–0.50 m in front of the pelvis, 0–0.2 m to the right.
- Closed loop: the grasp target is corrected at 25 Hz for body sway; > 3 cm body shift or a foot moving
  aborts ("look again") [Guide 14].
- **CALIBRATE before the first grasp** (`okra_pick/robot/pick_config.py`): `TCP_XYZ`, `CLOSE_AXIS`,
  `GRIP_EMPTY_Q`, and the camera mount (`floor`) [Guide 11, 14].
- Human-in-the-loop answers become labels (okra / not okra / missed → CVAT) [Guide 13, `okra_data/README.md`].

## 9. Safety features

| Layer | When | What |
|---|---|---|
| **Safety gate** `okra_pick/safety/safety_check.py` | before **every** step; reach/pick/gripper again after the confirm prompt | runs `robot/safety_probe.py` on the Orin (read-only, ~2 s) and checks live values; a FAIL means the step does not run (exit 3) |
| Player preflight | on the robot, right before moving | FSM, upright, fresh state, arm at the plan's start pose, waist unchanged, perception age |
| Per-tick monitors | every 20 ms while moving | tilt drift, stale state, tracking error, pull torque, closed-loop body/foot check → abort |
| STOP | any time | e-stop; Ctrl-C in the terminal; web STOP / `g1ctl stop` → SIGINT to the player over ssh **and** via the agent |
| Link loss | ssh drops, VM freezes, terminal closed | the player treats it as STOP and finishes the safe exit on the robot; output in `player.log` |

Safe exit = before/during the pull: open the gripper if closed, go back along the executed path, blend
out; after the pull: stop there, keep the pod, blend out.

The gate measures, at that moment: robot clock, network interface (route to the motion controller →
passed as `--iface`), `rt/lowstate` rate and gaps, FSM and switcher mode, tilt, knee load, per-motor
temperature and error bits, Dex1 state, **anybody else publishing `rt/arm_sdk` or `rt/dex1/right/cmd`**,
other motion programs, battery (BMS if it answers, otherwise "unknown" → WARN), RealSense on USB, disk
(robot and VM) and Orin temperature. **Perception age is measured on the robot clock only**
(`perceived_at_robot` travels robot_state → target → trajectory); plans without it are refused.

- Limits are policy in `okra_pick/safety/safety_limits.py` and `okra_pick/robot/pick_config.py`; limits
  not checked against the Unitree spec only warn. `mode_machine` is compared with the first value measured
  on this robot (`okra_data/safety/robot_baseline.json`).
- Results: `okra_data/events/.../safety/<step>_<time>.json` (with the full probe).
- One-run override of a named FAIL, logged: `OKRA_SAFETY_ACCEPT=battery ./okra_pick.sh reach`.

## 10. Okra detector

YOLO11n-seg (2.8 M params) + a sanity filter (size, shape, colour hue 30–105, stability), in `okra_robot/`.
Runs on the Orin in conda `g1brainco` on CUDA (~33 FPS with the base model).

| Model | Evaluated on | mask mAP50 | P / R @0.25 | full detector R / P |
|---|---|---|---|---|
| `okra_seg.pt` (public base) | 30 held-out test images, 90 pods | 0.013 | 0.00 / 0.00 | 0 / – |
| `okra_seg_s02.pt` (pipeline default) | 30 val images (no separate test split) | 0.962 | 0.92 / 0.92 | 0.89 / 0.95 |
| **`okra_seg_s02_3way.pt`** | 30 held-out test images, 90 pods | **0.832** | **1.00 / 0.71** | **0.70 / 1.00** |

Choose with `OKRA_WEIGHTS=<file>` (models live in `okra_robot/models/`). Full comparison with plots:
[`okra_robot/report/REPORT.md`](okra_robot/report/REPORT.md). All frames come from one room and session.

```bash
cd ~/Junction/okra_robot; PY=../dimos/.venv/bin/python
$PY dataset/cvat_to_yolo.py --three-way --out dataset/yolo_s02_3way     # CVAT task -> train/val/test (needs ../.env)
$PY train.py --data dataset/yolo_s02_3way/data.yaml --name s02_3way     # -> models/okra_seg_s02_3way.pt (~45 min CPU)
$PY evaluate_test.py --data dataset/yolo_s02_3way --tuned models/okra_seg_s02_3way.pt --out report
$PY run_camera.py --source video.mp4 --show                             # try the detector on a video
```
Labelling: CVAT org **Junction**, project `okra_g1`; `.env` holds `CVAT_HOST` and `CVAT_TOKEN`.

## 11. Tests

All offline, on the VM, with `dimos/.venv/bin/python` from `okra_pick/`
(`T=../okra_data/events/0000-examples/example_synthetic/trajectory.json`):

| Test | Covers | Result (2026-09-27) |
|---|---|---|
| `tests/test_safety.py` | gate rules (clock skew, stale/future perception, arm_sdk conflict, FSM, battery, ...), override, player age check | 26/26 |
| `tests/test_link_loss.py $T` | ssh -T pipe closed, SIGHUP, ssh -t pty closed, SIGINT mid-approach → safe exit | 4/4 |
| `tests/test_player_sim.py $T` | nominal, empty grasp, stuck pod, tilt in approach/retreat, stale state | 6/6 |
| `tests/test_closed_loop.py $T` | body sway corrected, large sway / foot step abort | 5/5 |
| `tests/test_perceive_math.py` | camera ↔ pelvis transforms, pod axis | pass |
| `tests/test_hitl.py` | decisions, answers, robot voice/LED via the sim agent, dataset export | 7/8 (see §15) |

## 12. Robot facts

| Topic / service | Rate | Used for |
|---|---|---|
| `rt/lowstate` (unitree_hg LowState_) | ~990 Hz on the Orin (10–175 Hz across the VM link) | joints, IMU, `mode_machine`, motor temperature/error bits |
| `rt/arm_sdk` (LowCmd_) | when commanded | waist 12–14 + arms 15–28, weight in `motor_cmd[29].q` |
| `rt/dex1/right/cmd`, `rt/dex1/right/state` | ~494 Hz state | Dex1 right: q ≈ 0 closed, ≈ 5.4 open (**left Dex1, motor id 1, does not reply**) |
| Loco RPC / MotionSwitcher RPC | request/reply | FSM get/set; controller mode (`ai`) |

- Head camera: RealSense D435i, fixed to the torso (**the G1 has no neck** — "look" turns the waist),
  served by `videohub_pc4`; our pipeline stops it to use the camera (`rec.sh release` restores it).
  A chest camera is served by `videohub_pc4_chest /dev/video10`.
- `~/run_ik_camera.sh` on the Orin streams LCM for dimos and **kills videohub**.
- Robot clock: differs from the VM (≈ 11.5 min behind when last measured) — the pipeline never compares
  the two clocks.
- `g1_video/` tools (`view.py`, `record.py [--head]`, `g1_fsm.py`, `arm_move.py`, `waist.py`,
  `wait_normal.py`) speak DDS from the VM; they worked on 2026-09-26 but waist moves from the VM failed
  where the same move on the Orin worked. Use them for lab checks, not on a run day.

## 13. VM limits and tuning

- **No GPU in VirtualBox**: detection runs on the Orin; training runs on the VM CPU (~45 min) — or train
  on the Orin / dual-boot native Ubuntu 22.04.
- **Disk**: 64 GB, 87 % full on 2026-09-27; the safety gate refuses steps below 10 GB free.
- **Green turtle** in the VM status bar = VirtualBox on Hyper-V (2–3× slower).
- Step-by-step GUI fixes (disk resize, storage, display, network, Hyper-V off): [Guide 15](Guide/15_vm_tuning.md).
- Keep the VM idle during reach/pick (no training, no sleep).

## 14. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `check.sh` ping fails / `FAILED` in `ip neigh` | robot off, cable, or VirtualBox adapter bridged to the wrong NIC |
| no DDS data at all on the VM | cyclonedds 11.x instead of 0.10.2; UDP buffers not tuned (`setup_host.sh --persist`) |
| Loco **3102** / **3104** from the VM | RPC unreachable / timeout over the lossy link — use `robot_mode.sh` or the agent (they run on the Orin) |
| nothing responds after a reboot | motion service off: `robot_mode.sh ai`, then damp → ready → start |
| tips forward in locked stand | locked stand does not balance: take up the rope, upright, then `start` |
| `SAFETY GATE: '<step>' not run` | a live check FAILed; the line above names it (`arm_sdk_free`, `knee_load`, `event_age`, ...) |
| `perception is N s old on the robot clock` | the look is too old or the waist moved: look again, then plan |
| `battery unknown` WARN | no BMS message: read the battery on the robot / remote yourself |
| camera viewer black | a recording or the okra pipeline holds the camera: `g1_record/rec.sh release` |
| left gripper does nothing | known hardware issue; only `rt/dex1/right/*` is used |

More: [Guide 09](Guide/09_troubleshooting.md).

## 15. Open issues

- First real `reach` / `pick`, the safety gate and web STOP have not run on the robot yet.
- `robot_agent` is not deployed on the robot (web STOP still works through ssh; voice/LED prompts do not).
- Uncalibrated: `TCP_XYZ`, `CLOSE_AXIS`, `GRIP_EMPTY_Q`, camera mount; the 45° pull twist is about `CLOSE_AXIS`.
- A waist/head command sent **after** a reach started is not blocked (the gate only checks before start).
- Recording start can fail silently inside reach/pick — watch for `recording started`.
- `tests/test_hitl.py` case 1 fails (labelling only): the out-of-reach second candidate is not shown.
- Detector data comes from a single room and session.

## 16. Documentation

| # | Report | # | Report |
|---|---|---|---|
| 01 | [Network connection](Guide/01_network_connection.md) | 09 | [Troubleshooting](Guide/09_troubleshooting.md) |
| 02 | [Robot services](Guide/02_robot_services.md) | 10 | [Bring-up and motion](Guide/10_bringup_and_motion.md) |
| 03 | [Robot Python env](Guide/03_robot_python_env.md) | 11 | [Okra pick pipeline](Guide/11_okra_pick_pipeline.md) |
| 04 | [Episode recording](Guide/04_episode_recording.md) | 12 | [Robot agent](Guide/12_robot_agent.md) |
| 05 | [Camera capture](Guide/05_camera_capture.md) | 13 | [Human in the loop + web UI](Guide/13_human_in_the_loop.md) |
| 06 | [State topics](Guide/06_state_topics.md) | 14 | [Spatial awareness, closed loop](Guide/14_spatial_awareness_closed_loop.md) |
| 07 | [Live preview](Guide/07_live_preview.md) | 15 | [VM tuning](Guide/15_vm_tuning.md) |
| 08 | [Data format](Guide/08_data_format.md) | 16 | [Run day: safety gate, STOP, checklist](Guide/16_robot_run_checklist.md) |

Folder READMEs: [`okra_pick`](okra_pick/README.md) · [`okra_robot`](okra_robot/README.md) ·
[`okra_data`](okra_data/README.md) · [`robot_agent`](robot_agent/README.md) · [`g1_connect`](g1_connect/README.md) ·
[`g1_record`](g1_record/README.md) · [`recordings`](recordings/README.md)

## License

Apache License 2.0 — see [`LICENSE`](LICENSE).
