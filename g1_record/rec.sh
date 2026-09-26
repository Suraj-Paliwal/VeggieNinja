#!/usr/bin/env bash
# Control the on-robot episode recorder from the VM.
#   ./rec.sh setup                 copy robot/ to the robot and build its Python libs (once)
#   ./rec.sh start NAME [opts]     start an episode; opts go to recorder.py (--depth, --size 1280x720, ...)
#   ./rec.sh stop                  stop the running episode (files are finalized on the robot)
#   ./rec.sh status                is it recording? last log lines
#   ./rec.sh preview               live view on the VM (UDP 5600), close the window to quit
#   ./rec.sh list                  episodes on the robot
#   ./rec.sh pull [EPISODE]        rsync all (or one) episodes to ./data/
#   ./rec.sh release               give the camera back to Unitree's videohub (for view.py)
set -e
here="$(cd "$(dirname "$0")" && pwd)"
host=unitree@192.168.123.164
rdir=g1_rec                     # on the robot, relative to ~unitree
pidf=$rdir/recorder.pid
run() { ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" "$@"; }

running() { run "[ -f $pidf ] && kill -0 \$(cat $pidf) 2>/dev/null"; }

case "${1:-status}" in
  setup)
    run "mkdir -p $rdir"
    scp -q "$here"/robot/recorder.py "$here"/robot/setup_robot.sh "$host:$rdir/"
    run "bash $rdir/setup_robot.sh"
    ;;
  start)
    name=${2:?usage: rec.sh start NAME [recorder options]}; shift 2
    if running; then echo "already recording (./rec.sh stop first)"; exit 1; fi
    scp -q "$here"/robot/recorder.py "$host:$rdir/"          # always run the current version
    # Unitree's videohub holds the head camera; stop it (restart with ./rec.sh release).
    run "/unitree/sbin/mscli getservice video_hub_pc4 | grep -q 'status:0' && /unitree/sbin/mscli stopservice video_hub_pc4 >/dev/null; sleep 1; true"
    ep="episodes/${name}_$(date +%Y%m%d_%H%M%S)"
    run "cd $rdir && mkdir -p $ep && PYTHONPATH=pylib nohup python3 -u recorder.py --out $ep $* \
         > $ep/recorder.log 2>&1 < /dev/null & echo \$! > recorder.pid; echo $ep > current"
    sleep 4
    run "cd $rdir && tail -5 \$(cat current)/recorder.log"
    running && echo "RECORDING $ep" || { echo "recorder did not stay up, see log above"; exit 1; }
    ;;
  stop)
    running || { echo "not recording"; exit 0; }
    run "kill -INT \$(cat $pidf); for i in \$(seq 30); do kill -0 \$(cat $pidf) 2>/dev/null || break; sleep 0.5; done; \
         cd $rdir && tail -3 \$(cat current)/recorder.log && ls -lh \$(cat current) && rm -f recorder.pid"
    ;;
  status)
    if running; then echo "RECORDING $(run "cat $rdir/current")"; else echo "not recording"; fi
    run "cd $rdir && [ -f current ] && tail -3 \$(cat current)/recorder.log || true"
    ;;
  preview)
    exec ffplay -hide_banner -loglevel warning -fflags nobuffer -flags low_delay -framedrop \
      -window_title "G1 head (recording preview)" "udp://@:5600?fifo_size=1000000&overrun_nonfatal=1"
    ;;
  list)
    run "cd $rdir/episodes 2>/dev/null && du -sh * 2>/dev/null || echo 'no episodes'"
    ;;
  pull)
    mkdir -p "$here/data"
    rsync -a --info=progress2 "$host:$rdir/episodes/${2:-}" "$here/data/"
    ;;
  release)
    running && { echo "recording in progress, stop first"; exit 1; }
    run "/unitree/sbin/mscli startservice video_hub_pc4"
    ;;
  *) sed -n '2,10p' "$0"; exit 2 ;;
esac
