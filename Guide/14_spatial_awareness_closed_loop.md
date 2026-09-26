# 14 — Spatial awareness and closed-loop hand control

Built and tested offline 2026-09-27 (not yet on hardware). Code: `okra_pick/robot/body_pose.py`,
`okra_pick/robot/closed_loop.py`, used by `arm_player.py` and `robot_agent/agent.py`.

## Why

The pod is measured once (camera + depth) in the pelvis frame; the arm then moves for ~16 s. If the body
moves in between, an open-loop arm misses by that much — and a pod is ~2 cm wide.
Measured on session02 (24 min, 115 quiet 10-s standing windows): pelvis vs feet moves **0.1 mm** (median),
worst 13 mm — and that worst window was a small foot shuffle (feet disagreed by 12 mm). Sway *during an
arm move* is not measured yet (weight shift) — first thing to log on hardware.

## Where is the body? (`body_pose.py`)

While standing, the feet do not move. Stance frame S = left foot at the anchor time.
- pelvis in S from the 12 leg encoders: `T_S_pelvis = inv(FK_pelvis→left_foot(q))` (exact, ~kHz)
- **right foot = second opinion**: both feet must give the same pelvis; > 1 cm apart = a foot moved
- **IMU cross-check**: tilt change seen by the legs vs by the pelvis IMU; > 4° apart = foot not flat
- any point measured earlier can be moved into the current body frame: `p_now = FK_L(q_now) inv(FK_L(q_then)) p_then`

Validated on the recording: all 29/29 windows with leg motion flagged (feet disagree 167 mm median).

Visible live: web UI pill **body Δ … mm · feet ok** (red "FOOT MOVED" otherwise), `g1ctl status` →
`body`. The agent re-anchors after every step it makes; **Re-anchor body** / `g1ctl anchor` does it by hand.

## Closed-loop reach (`closed_loop.py` in `arm_player.py`)

At 25 Hz during the reach:
1. pod in the current pelvis frame (above) → `delta` = how far it moved relative to the body
2. 2 Gauss-Newton steps on the grasp point's geometric Jacobian (7 joints, waist fixed) → `dq`
3. command = plan + weight·dq (weight ramps in during `to_pregrasp`, 1 in approach/pull/retreat,
   ramps out in `home`), lightly filtered, ≤ 0.3 rad per joint
4. **abort** (then the usual safe exit) if a foot moved, or the body moved > `MAX_CORRECTION` (3 cm):
   "look again". Preflight also refuses if the waist moved > 0.05 rad since perception.
`arm_player.py --open-loop` disables it. Corrections are logged in `trajectory_executed.json`.

Cost: 2.4 ms per update on the VM (~7 ms estimated on the Orin; runs every 2nd 20 ms arm tick).

## Test (`okra_pick/tests/test_closed_loop.py`) — 5/5 pass

Fake robot tips forward on both ankles during the reach; error = real grasp point vs the fixed pod
(stance frame) at the moment the jaws close:

| body moved | open loop | closed loop |
|---|---|---|
| 0 | 1.5 mm | 1.5 mm |
| 14 mm | 20.4 mm | **2.6 mm** |
| 21 mm | 30.0 mm | **3.4 mm** |
| 42 mm | 58.5 mm | **abort: body moved 5.7 cm** |
| right foot steps | 1.5 mm (unaware) | **abort: a foot moved (41 mm)** |

Also re-run: `test_player_sim.py` 6/6 (player unchanged otherwise).

## Camera ↔ gripper from kinematics (no marker) and its error budget

Chain: camera optical frame ← d435_link (URDF mount) ← torso ← waist (3 encoders) ← pelvis → shoulder →
elbow → wrist (7 encoders, URDF links) → Dex1 grasp point (`TCP_XYZ`). Example plan (pod 0.42/−0.15/0.15):
gripper 46 cm from the camera at pregrasp, **52 cm at the grasp, in view at pixel (531,166)**; pod at
(532,174) — the camera sees the hand from pregrasp to retreat (usable to check the grasp visually).

Error at the grasp per uncertain input (math is exact; inputs are not):

| input | error | handled by |
|---|---|---|
| camera mount angle 1° pitch / yaw / roll | 8.5 / 9.0 / 3.1 mm | `okra_pick.sh floor` (camera vs floor from legs); marker (E) |
| camera mount position 5 mm | 5 mm | floor check (height); marker (E) |
| depth 1 % at 0.5 m | 5 mm | median over the mask and 15 frames |
| RealSense colour-vs-depth sensor offset | ~15 mm | **factory extrinsics read in `okra_perceive.py`** (2026-09-27) |
| grasp point 1 cm off (Dex1 not in URDF) | 10 mm | measure TCP: camera sees the fingers at the grasp / marker |
| Dex1 mounted 5° rolled | 7.8 mm | same |
| one arm encoder 0.5° | 0.7–4 mm | (encoders are good) |

Remaining unknown: whether the URDF's d435_link is exactly the depth module origin (Unitree's model).

## Next (needs the robot)

| | What | Gain |
|---|---|---|
| D | keep detecting the pod during the approach (GPU, ~30 FPS) until the hand hides it | swinging pods, perception error |
| E | AprilTag on the gripper, seen by the head camera | measures camera-mount + grasp-point errors directly (replaces 2 CALIBRATE steps) |
| F | Livox MID-360 lidar (in the URDF; check if running) + map | know where the robot is in the room, walk to plants |
| — | log body sway during a real arm move (agent `watch`) | tells whether 3 cm limit is right |
