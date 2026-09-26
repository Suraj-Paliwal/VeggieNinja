# okra_pick — camera-guided okra grasp for the G1 (right arm + Dex1)

Full documentation: [`../Guide/11_okra_pick_pipeline.md`](../Guide/11_okra_pick_pipeline.md),
human-in-the-loop: [`../Guide/13_human_in_the_loop.md`](../Guide/13_human_in_the_loop.md).

```bash
./okra_pick.sh web           # operator web interface: http://localhost:8080  (recommended)
./okra_pick.sh deploy        # once per code change: copy to the robot
./okra_pick.sh go            # perceive (+ asks you if unsure) + plan + sim video (nothing moves)
./okra_pick.sh sim           # watch the plan in a MuJoCo window
./okra_pick.sh reach         # first real test: to the pod and back, no grasp
./okra_pick.sh pick          # full grasp, recorded
```

| Path | Runs on | What |
|---|---|---|
| `okra_pick.sh` | VM | orchestration |
| `robot/okra_perceive.py` | robot (conda g1brainco) | detector + depth → pod in pelvis frame |
| `robot/arm_player.py` | robot (~/g1_rec/pylib) | executes the plan with safety monitors |
| `robot/pick_config.py`, `robot/g1_chain.py`, `robot/g1.urdf` | both | settings, kinematics |
| `plan/okra_plan.py` | VM (dimos venv) | grasp poses, IK, trajectory, checks |
| `plan/sim_view.py` | VM | meshed G1 playback (window / mp4) |
| `robot/candidates.py` | both | merges per-frame detections into candidates |
| `hitl/` | VM | human-in-the-loop: `decide.py`, `confirm.py`, `outcome.py`, `export_dataset.py`, `store.py`, `hitl_config.py` |
| `web/` | VM | operator web interface (`server.py` + `index.html`) |
| `tests/` | VM | offline tests (`test_hitl.py`, `test_player_sim.py`, `test_perceive_math.py`) |
| `../okra_data/` | VM | every event (look → question → answer → plan → outcome), see its README |

Example without the robot: `okra_data/events/0000-examples/example_synthetic/` → `./okra_pick.sh sim example_synthetic`.
