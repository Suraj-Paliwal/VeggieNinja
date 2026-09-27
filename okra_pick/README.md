# okra_pick — camera-guided okra grasp for the G1 (right arm + Dex1)

Full documentation: [`../Guide/11_okra_pick_pipeline.md`](../Guide/11_okra_pick_pipeline.md),
human-in-the-loop: [`../Guide/13_human_in_the_loop.md`](../Guide/13_human_in_the_loop.md),
run day (safety gate, STOP, checklist): [`../Guide/16_robot_run_checklist.md`](../Guide/16_robot_run_checklist.md).

```bash
./okra_pick.sh web           # operator web interface: http://localhost:8080  (recommended)
./okra_pick.sh deploy        # once per code change: copy to the robot (ends with a live safety check)
./okra_pick.sh safety        # live safety check by hand (also: safety reach [EVENT])
./okra_pick.sh go            # perceive (+ asks you if unsure) + plan + sim video (nothing moves)
./okra_pick.sh sim           # watch the plan in a MuJoCo window
./okra_pick.sh dry           # all robot-side checks, nothing moves
./okra_pick.sh reach         # first real test: to the pod and back, no grasp  (run from a terminal)
./okra_pick.sh pick          # full grasp, recorded
OKRA_WEIGHTS=okra_seg_s02_3way.pt ./okra_pick.sh look    # detector with held-out test numbers
```

**Safety (Guide 16):** every robot step first runs a live safety gate — the robot is measured at that
moment (clock, network interface, FSM, tilt, knee load, battery, motor temperatures/errors, anybody else
commanding `rt/arm_sdk`, camera, disk); a FAIL stops the step, reach/pick/gripper are checked again after
you confirm. Nothing about the robot is hard-coded: the interface is taken from the route to the motion
controller, and the plan age is measured on the robot clock only. Stop: e-stop, Ctrl-C in the terminal,
or web STOP (interrupts the arm player over ssh and via the agent). If the ssh link drops, the player
aborts on its own (gripper open if needed, back along the path, blend out) and logs to `player.log`.

| Path | Runs on | What |
|---|---|---|
| `okra_pick.sh` | VM | orchestration |
| `robot/okra_perceive.py` | robot (conda g1brainco) | detector + depth → pod in pelvis frame |
| `robot/arm_player.py` | robot (~/g1_rec/pylib) | executes the plan: preflight, per-tick monitors, closed loop, safe abort on STOP / link loss |
| `robot/safety_probe.py` | robot (~/g1_rec/pylib) | read-only live robot snapshot for the safety gate |
| `robot/live_cam.py` | robot (conda g1brainco) | live head-camera MJPEG for the web page, okra outlines, camera hand-over (Guide 17) |
| `safety/safety_check.py`, `safety/safety_limits.py` | VM | safety gate before every step + the policy limits |
| `robot/body_pose.py`, `robot/closed_loop.py` | robot | body pose from legs + IMU, grasp correction (Guide 14) |
| `robot/pick_config.py`, `robot/g1_chain.py`, `robot/g1.urdf` | both | settings, kinematics |
| `plan/okra_plan.py` | VM (dimos venv) | grasp poses, IK, trajectory, checks |
| `plan/sim_view.py` | VM | meshed G1 playback (window / mp4) |
| `robot/candidates.py` | both | merges per-frame detections into candidates |
| `hitl/` | VM | human-in-the-loop: `decide.py`, `confirm.py`, `outcome.py`, `export_dataset.py`, `store.py`, `hitl_config.py` |
| `web/` | VM | operator web interface (`server.py` + `index.html`), live camera relay (Guide 17) |
| `tests/` | VM | offline tests: `test_safety.py`, `test_link_loss.py`, `test_player_sim.py`, `test_closed_loop.py`, `test_perceive_math.py`, `test_hitl.py` |
| `../okra_data/` | VM | every event (look → question → answer → plan → outcome), see its README |

Example without the robot: `okra_data/events/0000-examples/example_synthetic/` → `./okra_pick.sh sim example_synthetic`.
