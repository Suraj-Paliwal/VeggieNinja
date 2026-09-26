#!/usr/bin/env bash
# Move the G1 to several viewpoints around the okra stand while g1_record is recording.
# Every walking step goes through robot_mode.sh (balance gate + tilt watch); stops at the first refusal.
# Never walks closer than the starting point: back / sideways first, then returns.
#   ./collect_views.sh            (robot in a balance state, area clear, rec.sh recording)
set -u
J=~/Junction
step() {  # vy vx dur label
  echo "== $(date +%T) step $4"
  $J/g1_connect/robot_mode.sh step "$1" "$2" "$3" 2>&1 | grep -E "balance check|REFUSED|max tilt"
  [ "${PIPESTATUS[0]}" -eq 0 ] || { echo "STOPPED: step refused"; exit 1; }
}
look() {  # waist left and right at the current spot
  echo "== $(date +%T) look left/right"
  $J/g1_record/rec.sh head --yaw 0.25 --hold 3 --speed 0.2 2>&1 | grep -E "balance|ABORT|released"
  $J/g1_record/rec.sh head --yaw -0.25 --hold 3 --speed 0.2 2>&1 | grep -E "balance|ABORT|released"
}
hold() { echo "== $(date +%T) view $1 (hold ${2:-5} s)"; sleep "${2:-5}"; }

hold 1; look
step 0 -0.1 2.0 "back 20 cm";   hold 2; look
step -0.1 0 2.0 "right 20 cm";  hold 3
step -0.1 0 2.0 "right 20 cm";  hold 4; look
step 0 -0.1 2.0 "back 20 cm";   hold 5
step 0.1 0 2.0 "left 20 cm";    hold 6
step 0.1 0 2.0 "left 20 cm";    hold 7
step 0.1 0 2.0 "left 20 cm";    hold 8; look
step 0 0.1 2.0 "forward 20 cm"; hold 9
echo "== $(date +%T) done (net: 40 cm back, 20 cm left of start)"
