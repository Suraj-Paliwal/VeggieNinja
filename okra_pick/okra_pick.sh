#!/usr/bin/env bash
# Okra pick pipeline, driven from the VM. Robot must be standing in a balance state (g1_connect/robot_mode.sh).
#
#   ./okra_pick.sh deploy               copy code + detector + model to the robot (~/okra_pick)
#   ./okra_pick.sh gripper open|close   Dex1 right gripper only (calibration / test)
#   ./okra_pick.sh floor                camera extrinsic check (floor plane vs leg kinematics)
#   ./okra_pick.sh perceive             find a pod -> runs/<time>/target.json + target.jpg
#   ./okra_pick.sh plan [RUN]           plan the grasp for the latest (or given) run -> trajectory.json + sim.mp4
#   ./okra_pick.sh sim [RUN]            watch the plan live in a MuJoCo window on the VM desktop
#   ./okra_pick.sh dry [RUN]            all robot-side preflight checks, nothing moves
#   ./okra_pick.sh reach [RUN]          REAL: move to the pod and back, gripper stays open (no grasp)
#   ./okra_pick.sh pick [RUN]           REAL: full grasp + pull + retreat + home, recorded with g1_record
#   ./okra_pick.sh go                   perceive + plan + sim video, then asks before 'pick'
#
# REAL steps ask for confirmation (type the word shown); --yes skips it (only after a human confirmed).
set -e
here="$(cd "$(dirname "$0")" && pwd)"
J="$here/.."
host=unitree@192.168.123.164
rdir=okra_pick
PY_VM="$J/dimos/.venv/bin/python"
PY_DET="~/miniconda3/envs/g1brainco/bin/python"          # robot: torch+CUDA, pyrealsense2, sdk
PY_ARM="PYTHONPATH=~/g1_rec/pylib python3"               # robot: player (same env as the recorder)
run() { ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" "$@"; }
yes_flag=0; for a in "$@"; do [ "$a" = "--yes" ] && yes_flag=1; done
confirm() {  # word
  [ $yes_flag = 1 ] && return 0
  read -r -p "Robot will MOVE ($1). Area clear, e-stop in hand? Type '$1' to continue: " ans
  [ "$ans" = "$1" ] || { echo "cancelled"; exit 1; }
}
latest() { ls -1d "$here"/runs/*/ 2>/dev/null | tail -1 | xargs -r basename; }
rundir() { local r="${1:-$(latest)}"; [ -n "$r" ] && [ -d "$here/runs/$r" ] || { echo "no run ${1:-found}" >&2; exit 1; }; echo "$here/runs/$r"; }
free_camera() { run "/unitree/sbin/mscli getservice video_hub_pc4 | grep -q 'status:0' && /unitree/sbin/mscli stopservice video_hub_pc4 >/dev/null; true"; }

cmd="${1:-help}"; shift || true
case "$cmd" in
  deploy)
    run "mkdir -p $rdir/robot $rdir/okra_robot/models $rdir/runs"
    scp -q "$here"/robot/{okra_perceive.py,arm_player.py,pick_config.py,g1_chain.py,g1.urdf} "$host:$rdir/robot/"
    scp -q "$J"/okra_robot/{okra_detector.py,okra_filter.py} "$host:$rdir/okra_robot/"
    rsync -a "$J"/okra_robot/models/ "$host:$rdir/okra_robot/models/"
    run "cd $rdir/robot && $PY_DET -c 'import pyrealsense2, torch, ultralytics, okra_perceive; print(\"perceive env ok, cuda\", torch.cuda.is_available())' 2>&1 | tail -1;
         $PY_ARM -c 'import arm_player; print(\"player env ok\")'"
    ;;
  gripper)
    [ "$1" = open ] || [ "$1" = close ] || { echo "usage: gripper open|close"; exit 2; }
    confirm gripper
    run "cd $rdir/robot && $PY_ARM arm_player.py --gripper $1"
    ;;
  floor)
    free_camera
    run "cd $rdir/robot && $PY_DET okra_perceive.py --floor-check 2>&1 | grep -v -i warning"
    ;;
  perceive)
    r=$(date +%Y%m%d_%H%M%S); mkdir -p "$here/runs/$r"
    free_camera
    run "mkdir -p $rdir/runs/$r && cd $rdir/robot && $PY_DET okra_perceive.py --out ~/$rdir/runs/$r/target.json $* 2>&1 | grep -v -iE 'warning|settings'" || true
    scp -q "$host:$rdir/runs/$r/target.*" "$here/runs/$r/" 2>/dev/null || { echo "no target (see above)"; exit 1; }
    echo "run $r: $(python3 -c "import json;t=json.load(open('$here/runs/$r/target.json'));print('pod', [round(v,3) for v in t['xyz_pelvis']], 'spread %.1f cm' % (100*t['spread_m']))")"
    ;;
  plan)
    d=$(rundir "$1")
    "$PY_VM" "$here/plan/okra_plan.py" "$d/target.json" -o "$d/trajectory.json"
    MUJOCO_GL=glfw "$PY_VM" "$here/plan/sim_view.py" "$d/trajectory.json" --video "$d/sim.mp4" || echo "(sim video failed; plan is still valid)"
    ;;
  sim)
    d=$(rundir "$1"); MUJOCO_GL=glfw exec "$PY_VM" "$here/plan/sim_view.py" "$d/trajectory.json"
    ;;
  dry|reach|pick)
    d=$(rundir "$1"); r=$(basename "$d")
    [ -f "$d/trajectory.json" ] || { echo "plan first: ./okra_pick.sh plan $r"; exit 1; }
    scp -q "$d/trajectory.json" "$host:$rdir/runs/$r/"
    if [ "$cmd" = dry ]; then
      run "cd $rdir/robot && $PY_ARM arm_player.py ~/$rdir/runs/$r/trajectory.json --dry"; exit
    fi
    confirm "$cmd"
    extra=""; [ "$cmd" = reach ] && extra="--until approach"
    "$J/g1_record/rec.sh" start "okra_${cmd}_$r" >/dev/null && echo "recording started"
    set +e
    ssh -t -o ConnectTimeout=5 "$host" "cd $rdir/robot && $PY_ARM arm_player.py ~/$rdir/runs/$r/trajectory.json $extra"
    rc=$?
    set -e
    sleep 1; "$J/g1_record/rec.sh" stop | tail -1
    scp -q "$host:$rdir/runs/$r/trajectory_executed.json" "$d/" 2>/dev/null || true
    echo "player exit code $rc; video: g1_record/rec.sh pull, episode okra_${cmd}_$r"
    exit $rc
    ;;
  go)
    "$0" perceive
    "$0" plan
    echo "check $(rundir)/target.jpg and sim.mp4, then: ./okra_pick.sh reach   (first time)   or   ./okra_pick.sh pick"
    ;;
  *) sed -n '2,17p' "$0"; exit 2 ;;
esac
