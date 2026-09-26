#!/usr/bin/env bash
# Control the Unitree services on the G1 onboard computer (Orin, 192.168.123.164) from the VM.
#   ./services.sh [status]                 show all
#   ./services.sh start|stop|restart NAME  NAME: head_cam | chest_cam | gripper | cams | all
# Cameras go through Unitree's mscli (no sudo). The gripper is a systemd unit with
# Restart=always, so it needs sudo on the robot (you will be asked for the password).
set -e
host=unitree@192.168.123.164
mscli=/unitree/sbin/mscli

declare -A ms=([head_cam]=video_hub_pc4 [chest_cam]=video_hub_pc4_chest)

run()  { ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" "$@"; }
runt() { ssh -t -o ConnectTimeout=5 "$host" "$@"; }

status() {
  run "$mscli listservice | grep -E 'video_hub'; \
       echo -n 'gripper (dex1_gripper.service): '; systemctl is-active dex1_gripper.service; \
       pgrep -a videohub_pc4 || echo 'no videohub processes'"
}

act() {  # act ACTION NAME
  case "$2" in
    head_cam|chest_cam) run "$mscli ${1}service ${ms[$2]}" ;;
    gripper)            runt "sudo systemctl $1 dex1_gripper.service" ;;
    cams)               act "$1" head_cam; act "$1" chest_cam ;;
    all)                act "$1" cams; act "$1" gripper ;;
    *) echo "unknown service '$2' (head_cam|chest_cam|gripper|cams|all)" >&2; exit 2 ;;
  esac
}

cmd=${1:-status}
case "$cmd" in
  status) status ;;
  start|stop|restart)
    [ -n "$2" ] || { echo "usage: $0 $cmd NAME" >&2; exit 2; }
    act "$cmd" "$2"; sleep 1; status ;;
  *) sed -n '2,7p' "$0"; exit 2 ;;
esac
