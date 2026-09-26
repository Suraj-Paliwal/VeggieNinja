#!/usr/bin/env bash
# Manage the robot agent on the G1 Orin from the VM.
#   ./agent.sh deploy     copy agent.py (+ g1_video/waist.py) to ~/robot_agent on the robot
#   ./agent.sh start      start it in the background (nohup), wait until it answers
#   ./agent.sh stop       stop it (SIGTERM)
#   ./agent.sh restart    stop + deploy + start
#   ./agent.sh status     running? pid, uptime, and a ping round trip
#   ./agent.sh log [N]    last N lines of the command log and stdout (default 30)
set -e
here="$(cd "$(dirname "$0")" && pwd)"
host=unitree@192.168.123.164
rdir=robot_agent
run() { ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" "$@"; }
alive() { run "[ -f $rdir/agent.pid ] && kill -0 \$(cat $rdir/agent.pid) 2>/dev/null"; }

case "${1:-status}" in
  deploy)
    run "mkdir -p $rdir"
    scp -q "$here/agent.py" "$here/../g1_video/waist.py" "$here"/../okra_pick/robot/{body_pose.py,g1_chain.py,g1.urdf} "$host:$rdir/"
    echo "deployed to $host:~/$rdir"
    ;;
  start)
    if alive; then echo "already running (pid $(run "cat $rdir/agent.pid"))"; exit 0; fi
    run "cd $rdir || exit 1; PYTHONPATH=~/g1_rec/pylib nohup python3 -u agent.py >> agent_stdout.log 2>&1 < /dev/null & echo \$! > agent.pid"
    for i in $(seq 20); do
      "$here/g1ctl" ping >/dev/null 2>&1 && { echo "agent up (pid $(run "cat $rdir/agent.pid"))"; exit 0; }
      sleep 0.5
    done
    echo "agent did not answer; last log lines:"; run "tail -15 $rdir/agent_stdout.log"; exit 1
    ;;
  stop)
    alive && run "kill \$(cat $rdir/agent.pid) && rm -f $rdir/agent.pid" && echo stopped || echo "not running"
    ;;
  restart)
    "$0" stop; "$0" deploy; "$0" start
    ;;
  status)
    if alive; then
      echo "running: pid $(run "cat $rdir/agent.pid"), up $(run "ps -o etime= -p \$(cat $rdir/agent.pid)" | tr -d ' ')"
      "$here/g1ctl" ping | tail -1
    else
      echo "not running"
    fi
    ;;
  log)
    run "tail -${2:-30} $rdir/agent_commands.log; echo ---; tail -${2:-30} $rdir/agent_stdout.log"
    ;;
  *) sed -n '2,8p' "$0"; exit 2 ;;
esac
