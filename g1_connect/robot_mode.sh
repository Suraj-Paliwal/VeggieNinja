#!/usr/bin/env bash
# Run robot_mode.py on the G1 Orin (see its docstring):  ./robot_mode.sh status|ai|damp|ready|start [--ignore-load]|zero|step VY [VX] [DUR]
here="$(cd "$(dirname "$0")" && pwd)"
host=unitree@192.168.123.164
scp -q "$here/robot_mode.py" "$host:g1_rec/robot_mode.py"
exec ssh -o BatchMode=yes "$host" "cd ~/g1_rec && PYTHONPATH=pylib timeout 60 python3 -u robot_mode.py $*"
