#!/usr/bin/env bash
# Okra pick pipeline, driven from the VM. Robot must be standing in a balance state (g1_connect/robot_mode.sh).
#
#   ./okra_pick.sh deploy               copy code + detector + model to the robot (~/okra_pick)
#   ./okra_pick.sh gripper open|close   Dex1 right gripper only (calibration / test)
#   ./okra_pick.sh floor                camera extrinsic check (floor plane vs leg kinematics)
#   ./okra_pick.sh perceive [--mode always]   one "look": evidence bundle -> okra_data/events/<day>/<EVENT>/,
#                                        then the human-in-the-loop check (asks you when unsure) -> target.json
#   ./okra_pick.sh confirm [EVENT]      re-run the human check on an event
#   ./okra_pick.sh plan [EVENT]         plan the grasp for the latest (or given) event -> trajectory.json + sim.mp4
#   ./okra_pick.sh sim [RUN]            watch the plan live in a MuJoCo window on the VM desktop
#   ./okra_pick.sh dry [RUN]            all robot-side preflight checks, nothing moves
#   ./okra_pick.sh reach [RUN]          REAL: move to the pod and back, gripper stays open (no grasp)
#   ./okra_pick.sh pick [RUN]           REAL: full grasp + pull + retreat + home, recorded with g1_record
#   ./okra_pick.sh go                   perceive (+ human check) + plan + sim video, then asks before 'pick'
#   ./okra_pick.sh outcome [EVENT]      record whether the pick worked (asked automatically after 'pick')
#   ./okra_pick.sh dataset              export human-confirmed events as a YOLO training set
#   ./okra_pick.sh web [--host 0.0.0.0 --token T]   operator web interface on http://localhost:8080
#   ./okra_pick.sh point [X Y Z]         target a fixed spot (pelvis frame, m; default: the comfortable zone's middle)
#                                        from the live joint state, then plan / dry / reach it; OKRA_HOLD=S makes
#                                        reach hold there S s (place the pod at the gripper)
#   ./okra_pick.sh live start|stop|status  live head-camera feed for the web page (robot/live_cam.py, port 8091);
#                                        look/floor and the recorder take the camera from it automatically
#   ./okra_pick.sh safety [STEP] [EVENT]  run the live safety gate by hand (robot probe + VM + event checks)
#
# Every robot step first runs the safety gate (safety/safety_check.py, Guide 16): the robot is measured live
# (clock, interface, FSM, tilt, battery, temperatures, who commands the arm...) and a FAIL stops the step.
# Motion steps run it again right after you confirm. OKRA_SAFETY_ACCEPT=check1,.. downgrades named FAILs.
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
WEIGHTS="${OKRA_WEIGHTS:-okra_seg_s02.pt}"                # detector in okra_robot/models (fine-tuned 2026-09-27)
run() { ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" "$@"; }
yes_flag=0; for a in "$@"; do [ "$a" = "--yes" ] && yes_flag=1; done
confirm() {  # word
  [ $yes_flag = 1 ] && return 0
  read -r -p "Robot will MOVE ($1). Area clear, e-stop in hand? Type '$1' to continue: " ans
  [ "$ans" = "$1" ] || { echo "cancelled"; exit 1; }
}
HITL="$here/hitl"
rundir() {  # event folder in okra_data (latest, or by id)
  local d; d=$("$PY_VM" -c "import sys; sys.path.insert(0,'$HITL'); import store; print(store.find(${1:+'$1'}) or '')")
  [ -n "$d" ] && [ -d "$d" ] || { echo "no event ${1:-found}" >&2; exit 1; }; echo "$d"; }
IFACE=""
gate() {  # STEP [EVENT_DIR] [TRAJ]: live safety gate; exits on FAIL; sets IFACE (robot interface, measured)
  local out rc; set +e
  out=$("$PY_VM" "$here/safety/safety_check.py" "$1" ${2:+--event "$2"} ${3:+--traj "$3"}); rc=$?; set -e
  echo "$out" | grep -v '^IFACE='
  IFACE=$(echo "$out" | sed -n 's/^IFACE=//p')
  [ $rc = 0 ] && [ -n "$IFACE" ] || { echo "SAFETY GATE: '$1' not run (see above)"; exit 3; }
}
# The live feed (robot/live_cam.py) lets go of the camera while a hold file exists (content = expiry, robot clock).
hold_camera() {  # WHO SECONDS: ask the live feed to release the camera, wait until it has (max 3 s)
  run "mkdir -p /tmp/okra_live_hold && echo \$(( \$(date +%s) + $2 )) > /tmp/okra_live_hold/$1
       for i in \$(seq 30); do pgrep -f '^[^ ]*python[^ ]* [^ ]*live_cam\.py' >/dev/null || break
         grep -q '\"source\": \"camera\"' /tmp/okra_live_state.json 2>/dev/null || break; sleep 0.1; done"; }
release_camera() { run "rm -f /tmp/okra_live_hold/$1" || true; }
free_camera() {  # the camera must be free for pyrealsense2: stop videohub, and pause a running recording
  if "$J/g1_record/rec.sh" status 2>/dev/null | head -1 | grep -q '^RECORDING'; then
    echo "recording paused for this step (the camera is needed; 'record everything' starts a new one afterwards)"
    "$J/g1_record/rec.sh" stop >/dev/null || true
  fi
  stop_videohub; }
stop_videohub() { run "/unitree/sbin/mscli getservice video_hub_pc4 | grep -q 'status:0' && /unitree/sbin/mscli stopservice video_hub_pc4 >/dev/null; true"; }

cmd="${1:-help}"; shift || true
case "$cmd" in
  deploy)
    run "mkdir -p $rdir/robot $rdir/okra_robot/models $rdir/runs $rdir/events"
    scp -q "$here"/robot/{okra_perceive.py,live_cam.py,candidates.py,arm_player.py,pick_config.py,g1_chain.py,g1.urdf,body_pose.py,closed_loop.py,safety_probe.py} "$host:$rdir/robot/"
    scp -q "$J"/okra_robot/{okra_detector.py,okra_filter.py} "$host:$rdir/okra_robot/"
    rsync -a "$J"/okra_robot/models/ "$host:$rdir/okra_robot/models/"
    run "cd $rdir/robot && $PY_DET -c 'import pyrealsense2, torch, ultralytics, okra_perceive; print(\"perceive env ok, cuda\", torch.cuda.is_available())' 2>&1 | tail -1;
         $PY_ARM -c 'import arm_player; print(\"player env ok\")'"
    gate status
    ;;
  safety)
    st="${1:-status}"; ev=""; tr=""
    if [ -n "$2" ] || [[ " plan dry reach pick " == *" $st "* ]]; then ev=$(rundir "$2"); [ -f "$ev/trajectory.json" ] && tr="$ev/trajectory.json"; fi
    gate "$st" "$ev" "$tr"
    ;;
  gripper)
    [ "$1" = open ] || [ "$1" = close ] || { echo "usage: gripper open|close"; exit 2; }
    gate gripper
    confirm gripper
    gate gripper
    run "cd $rdir/robot && $PY_ARM arm_player.py --gripper $1 --iface $IFACE"
    ;;
  floor)
    gate floor
    hold_camera floor 120; trap 'release_camera floor' EXIT
    free_camera
    run "cd $rdir/robot && $PY_DET okra_perceive.py --floor-check --iface $IFACE 2>&1 | grep -v -i warning"
    ;;
  perceive)
    gate look
    d=$("$PY_VM" -c "import sys; sys.path.insert(0,'$HITL'); import store; print(store.new_event_dir())"); r=$(basename "$d")
    hold_camera look 180; trap 'release_camera look' EXIT
    free_camera
    run "cd $rdir/robot && $PY_DET okra_perceive.py --iface $IFACE --weights ~/$rdir/okra_robot/models/$WEIGHTS --event-dir ~/$rdir/events/$r 2>&1 | grep -v -iE 'warning|settings'"
    rsync -a "$host:$rdir/events/$r/" "$d/" || { echo "no event bundle (see above)"; exit 1; }
    echo "event $r -> $d"
    "$PY_VM" "$HITL/confirm.py" "$d" "$@"
    ;;
  point)  # a target at a fixed spot, no perception: the operator hangs the pod where the arm goes
    gate status
    d=$("$PY_VM" -c "import sys; sys.path.insert(0,'$HITL'); import store; print(store.new_event_dir())")
    run "cd $rdir/robot && PYTHONPATH=~/g1_rec/pylib timeout 30 python3 safety_probe.py 2>/dev/null | tail -1" > "$d/probe.json"
    HERE_OKRA="$here" "$PY_VM" - "$d" "$@" <<'PY'
import json, os, sys
sys.path.insert(0, os.path.join(os.environ.get("HERE_OKRA", "."), "robot"))
d, a = sys.argv[1], [float(x) for x in sys.argv[2:5]]
pr = json.load(open(os.path.join(d, "probe.json")))
ls = pr.get("lowstate") or {}
if not ls.get("q"):
    sys.exit("no live joint state from the robot probe")
import pick_config as C
xyz = a if len(a) == 3 else [sum(r) / 2 for r in C.PLACE_ZONE]
tgt = {"xyz_pelvis": xyz, "axis_pelvis": [0.0, 0.0, 1.0], "length_m": 0.10, "q": ls["q"], "source": "fixed_point",
       "perceived_at_robot": pr["robot_time"], "note": "no perception: the operator places the pod where the arm goes"}
json.dump(tgt, open(os.path.join(d, "target.json"), "w"), indent=1)
json.dump({"q": ls["q"], "imu_rpy": ls.get("rpy_rad"), "source": "fixed_point", "time": pr["robot_time"]},
          open(os.path.join(d, "robot_state.json"), "w"))
json.dump({"action": "fixed_point", "target": 1, "asked": False, "reasons": ["operator: fixed point"], "labels": {}},
          open(os.path.join(d, "decision.json"), "w"))
print("fixed point %s (pelvis frame) from the live joint state" % [round(v, 3) for v in xyz])
PY
    echo "EVENT_DIR=$d"
    echo "next: ./okra_pick.sh plan $(basename "$d")  ->  dry  ->  OKRA_HOLD=20 ./okra_pick.sh reach"
    ;;
  look)   # perceive only (the web interface asks the question itself); prints EVENT_DIR=<path>
    gate look
    d=$("$PY_VM" -c "import sys; sys.path.insert(0,'$HITL'); import store; print(store.new_event_dir())"); r=$(basename "$d")
    hold_camera look 180; trap 'release_camera look' EXIT
    free_camera
    run "cd $rdir/robot && $PY_DET okra_perceive.py --iface $IFACE --weights ~/$rdir/okra_robot/models/$WEIGHTS --event-dir ~/$rdir/events/$r 2>&1 | grep -v -iE 'warning|settings'"
    rsync -a "$host:$rdir/events/$r/" "$d/" || { echo "no event bundle (see above)"; exit 1; }
    echo "EVENT_DIR=$d"
    ;;
  confirm)
    d=$(rundir "$1"); shift || true
    "$PY_VM" "$HITL/confirm.py" "$d" "$@"
    ;;
  outcome)
    d=$(rundir "$1"); shift || true
    "$PY_VM" "$HITL/outcome.py" "$d" "$@"
    ;;
  dataset)
    "$PY_VM" "$HITL/export_dataset.py" "$@"
    ;;
  live)
    case "${1:-status}" in
      start)
        stop_videohub      # (a running recording is fine: the feed then shows its copy)
        run "cd $rdir/robot && if pgrep -f '^[^ ]*python[^ ]* [^ ]*live_cam\.py' >/dev/null; then echo 'live feed already running'; else
             PYTHONUNBUFFERED=1 nohup $PY_DET live_cam.py --host ${host#*@} --weights ~/$rdir/okra_robot/models/$WEIGHTS > ~/$rdir/live.log 2>&1 < /dev/null &
             sleep 5; tail -4 ~/$rdir/live.log; fi"
        ;;
      stop) run "pkill -f '^[^ ]*python[^ ]* [^ ]*live_cam\.py' && echo 'live feed stopped' || echo 'live feed not running'" ;;
      status)
        run "pgrep -af '^[^ ]*python[^ ]* [^ ]*live_cam\.py' || echo 'live feed not running'
             curl -s --max-time 2 http://${host#*@}:8091/status; echo; tail -3 ~/$rdir/live.log 2>/dev/null" || true
        ;;
      *) echo "usage: live start|stop|status"; exit 2 ;;
    esac
    ;;
  web)
    exec "$PY_VM" "$here/web/server.py" "$@"
    ;;
  plan)
    d=$(rundir "$1")
    [ -f "$d/target.json" ] || { echo "no confirmed target in $(basename "$d") (perceive / confirm first)"; exit 1; }
    gate plan "$d"
    "$PY_VM" "$here/plan/okra_plan.py" "$d/target.json" -o "$d/trajectory.json"
    MUJOCO_GL=glfw "$PY_VM" "$here/plan/sim_view.py" "$d/trajectory.json" --video "$d/sim.mp4" || echo "(sim video failed; plan is still valid)"
    ;;
  sim)
    d=$(rundir "$1"); MUJOCO_GL=glfw exec "$PY_VM" "$here/plan/sim_view.py" "$d/trajectory.json"
    ;;
  dry|reach|pick)
    d=$(rundir "$1"); r=$(basename "$d")
    [ -f "$d/trajectory.json" ] || { echo "plan first: ./okra_pick.sh plan $r"; exit 1; }
    gate "$cmd" "$d" "$d/trajectory.json"
    run "mkdir -p $rdir/runs/$r"; scp -q "$d/trajectory.json" "$host:$rdir/runs/$r/"
    if [ "$cmd" = dry ]; then
      run "cd $rdir/robot && $PY_ARM arm_player.py ~/$rdir/runs/$r/trajectory.json --dry --iface $IFACE"; exit
    fi
    confirm "$cmd"
    [ $yes_flag = 1 ] || gate "$cmd" "$d" "$d/trajectory.json"     # again: time passed at the prompt
    extra=""; [ "$cmd" = reach ] && extra="--until approach${OKRA_HOLD:+ --hold $OKRA_HOLD}${OKRA_JAW_CYCLE:+ --jaw-cycle}"
    own_rec=0
    if "$J/g1_record/rec.sh" status 2>/dev/null | head -1 | grep -q '^RECORDING'; then
      echo "a recording is already running (record everything): the motion is in it"
    else
      "$J/g1_record/rec.sh" start "okra_${cmd}_$r" >/dev/null && own_rec=1 && echo "recording started"
    fi
    set +e
    tflag=-t; [ -n "$OKRA_NONINTERACTIVE" ] && tflag=-T
    ssh $tflag -o ConnectTimeout=5 "$host" "cd $rdir/robot && $PY_ARM arm_player.py ~/$rdir/runs/$r/trajectory.json --iface $IFACE $extra"
    rc=$?
    set -e
    sleep 1; [ $own_rec = 1 ] && "$J/g1_record/rec.sh" stop | tail -1
    scp -q "$host:$rdir/runs/$r/"{trajectory_executed.json,player.log} "$d/" 2>/dev/null || true
    echo "player exit code $rc; video: g1_record/rec.sh pull, episode okra_${cmd}_$r"
    [ "$cmd" = pick ] && [ -z "$OKRA_NONINTERACTIVE" ] && "$PY_VM" "$HITL/outcome.py" "$d" --episode "okra_${cmd}_$r" || true
    exit $rc
    ;;
  go)
    "$0" perceive "$@" || { echo "no confirmed okra target: nothing to plan"; exit 1; }
    "$0" plan
    echo "check $(rundir)/question.jpg / annotated.jpg and sim.mp4, then: ./okra_pick.sh reach (first time) or ./okra_pick.sh pick"
    ;;
  *) sed -n '2,27p' "$0"; exit 2 ;;
esac
