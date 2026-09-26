# 11 — Okra pick pipeline (`okra_pick/`)

Camera-guided grasp of a hanging okra pod with the right arm + Dex1 gripper, robot standing in a
balance state. Built and tested offline 2026-09-26; **not yet run on the robot**.

## Architecture

```
ROBOT (Orin)                                        VM
 robot/okra_perceive.py  (conda g1brainco, py3.8)    plan/okra_plan.py  (dimos venv, pinocchio)
  RealSense colour + depth aligned (pyrealsense2)      target -> grasp poses -> right-arm IK
  OkraDetector on GPU (~33 FPS)                        straight-line approach / pull / retreat
  pod xyz + axis -> pelvis frame (URDF + waist q)      checks: reach, IK error, limits, torso, speed
  median of 15 sightings        -> target.json ---->   -> trajectory.json
                                                     plan/sim_view.py  meshed G1 (MuJoCo Menagerie)
 robot/arm_player.py  (~/g1_rec/pylib, py3.8)  <----   live window or sim.mp4
  rt/arm_sdk 50 Hz + rt/dex1/right/cmd
  preflight + monitors + phase-aware abort           okra_pick.sh  drives everything (+ g1_record)
```
Shared settings: `robot/pick_config.py`. Kinematics: `robot/g1_chain.py` (numpy, = pinocchio to 5e-16).

## Commands (VM, `~/Junction/okra_pick`)

| Step | Command | Moves robot? |
|---|---|---|
| copy code/model to robot | `./okra_pick.sh deploy` | no |
| camera extrinsic check | `./okra_pick.sh floor` | no |
| gripper test | `./okra_pick.sh gripper open` / `close` | gripper only |
| find pod | `./okra_pick.sh perceive` → `runs/<t>/target.json`, `target.jpg` | no |
| plan + video | `./okra_pick.sh plan` → `trajectory.json`, `sim.mp4` | no |
| watch plan live | `./okra_pick.sh sim` (MuJoCo window on the VM desktop) | no |
| robot-side checks | `./okra_pick.sh dry` | no |
| reach test | `./okra_pick.sh reach` (to the pod and back, gripper open) | **yes** |
| full pick | `./okra_pick.sh pick` (recorded with g1_record) | **yes** |
| shortcut | `./okra_pick.sh go` = perceive + plan, then asks | no |

`reach` / `pick` ask to type the word; they start `rec.sh` before and stop it after.

## Where the pod must hang (reach ∩ camera view)

Reachability map (pregrasp 8 cm, grid 5 cm, IK with limits and torso check) intersected with the
camera view (camera 0.47 m above pelvis, pitched 47.6° down):
- **0.35–0.50 m in front of the pelvis, 0.10–0.20 m above it = ~0.85–0.95 m above the floor**
- sideways: about **0 to 0.2 m to the robot's right** (further right leaves the camera view;
  y = −0.30 m is out of view at waist 0)
- planner refuses outside `REACH_X/Y/Z` in `pick_config.py`

## Grasp sequence (trajectory segments)

1. `to_pregrasp` joint space (peak 0.5 rad/s) to 8 cm behind the pod, jaws open
2. `approach` straight line at 4 cm/s to 1 cm past the pod centre (straight to 0.1 mm)
3. close gripper; holding = jaws stop at q > 0.4 rad (else "closed on nothing" → abort)
4. `pull` straight back 6 cm and down 2 cm at 2 cm/s (harvest)
5. `retreat` another 9 cm back, `home` joint space to the start pose; gripper stays closed

Jaws close across the pod: closing axis = approach × pod axis. Approach horizontal first, then
tilted ±15°/±30° if no IK; both jaw orientations tried, best joint-limit margin wins.

## Safety (arm_player.py)

Preflight: FSM 200/500/501, |roll|,|pitch| < 3°, fresh lowstate, plan start pose within 0.15 rad of
the arm now, plan < 5 min old. Blend in/out 2 s holding waist + left arm where they are.
Every tick: tilt drift > 5°, lowstate stale > 0.4 s, tracking error > 0.30 rad, pull torque > 12 Nm.
Abort: before the pull → back along the executed path (gripper opened if it had closed);
during the pull → open gripper, then back; after the pull → stop there, keep the pod, blend out.
Ctrl-C = abort.

## Offline tests (all pass, 2026-09-26)

| Test | Result |
|---|---|
| `g1_chain` vs pinocchio, 120 random poses | 5.3e-16 |
| planner, 3 in-zone pods | plans 13–16 s, margin ≥ 0.42 rad; 2 out-of-zone refused |
| plan checked with independent FK | TCP at pod to 1 mm, approach straight to 0.1 mm, ends at start |
| `tests/test_player_sim.py` (fake robot) | nominal, empty grasp, stuck pod, tilt in approach, tilt in retreat, stale state: 6/6 |
| `tests/test_perceive_math.py` | pelvis→pixel→pelvis exact; vertical pod axis within 0.7° |

## Calibration before the first real grasp (marked CALIBRATE in pick_config.py)

1. **Detector**: fine-tuned model must detect these pods (base model does not; see CVAT task).
2. **Camera extrinsic**: `./okra_pick.sh floor` — floor from camera vs leg kinematics within ~2 cm.
   Then hang a pod, `perceive`, and tape-measure it from the pelvis.
3. **Gripper mount (`CLOSE_AXIS`)**: with the arm in the ready pose, `gripper open/close` and look:
   do the jaws move left-right ("y") or up-down ("z") relative to the forearm?
4. **TCP_XYZ**: distance from the wrist to the middle of the jaws (default 0.13 m along the forearm).
5. **GRIP_EMPTY_Q**: close on nothing and on a pod; set between the two readings.
6. First motion: `reach` (no grasp) with the robot on the gantry, then `pick`.

## Simulation view (`./okra_pick.sh sim [RUN]`, video `runs/<RUN>/sim.mp4`)

The real end effector is the **Dex1: a parallel two-jaw gripper** on the right wrist (the model's rubber
hand is hidden). What is on screen:

| Object | Looks like | Meaning |
|---|---|---|
| OKRA (label) | green capsule, dark cap on top | the pod as perceived (position, long axis, length) |
| stem | thin green line pod-top → pole | pod still attached; disappears once the pull starts |
| PLANT (label) | tall dark-green pole 7 cm behind the pod | stand-in for the plant/stake (not perceived) |
| gripper | dark box on the wrist + two light jaws | Dex1 stand-in; jaws slide open/closed along `CLOSE_AXIS` |
| blue / orange / grey line | thin lines | gripper path: approach, pull, retreat |
| top-left text | `segment \| gripper open/CLOSED \| t` | where in the plan it is |

Sequence: arm swings to 8 cm in front of the pod (jaws open) → straight in → jaws close across the pod →
pod moves with the gripper (stem gone = harvested) → pull back/down → retreat → arm returns home.
Video: left = front view, right = side view (shows that the approach is a straight line).

Limits: kinematic replay only (no physics, no contact forces); the pole position is invented for the
picture; nothing checks collisions with the real plant; the jaw geometry is approximate.

## Known gaps

- MuJoCo view is kinematic only (no contact/physics); Dex1 jaws are drawn as simple boxes.
- No collision check against the plant/stand; approach assumes free space in front of the pod.
- Pull may not detach a real pod; torque limit then releases and retreats (tune `MAX_PULL_TAU`).
