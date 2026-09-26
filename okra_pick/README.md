# okra_pick — camera-guided okra grasp for the G1 (right arm + Dex1)

Full documentation: [`../Guide/11_okra_pick_pipeline.md`](../Guide/11_okra_pick_pipeline.md).

```bash
./okra_pick.sh deploy        # once per code change: copy to the robot
./okra_pick.sh go            # perceive + plan + sim video (nothing moves)
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
| `tests/` | VM | offline tests |
| `runs/` | VM | per-attempt target, trajectory, sim video (git-ignored) |

Example without the robot: `runs/example_synthetic/` (synthetic pod) → `./okra_pick.sh sim example_synthetic`.
