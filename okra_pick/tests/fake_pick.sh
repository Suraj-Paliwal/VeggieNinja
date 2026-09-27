#!/usr/bin/env bash
# Stand-in for okra_pick.sh in offline tests and demos (OKRA_PICK_SH=tests/fake_pick.sh). NOTHING real happens:
# plan copies the example trajectory, reach/pick just write a trajectory_executed.json. Never use with the robot.
set -e
here="$(cd "$(dirname "$0")" && pwd)"; J="$here/../.."
ev=$("$J/dimos/.venv/bin/python" -c "import sys; sys.path.insert(0,'$here/../hitl'); import store; print(store.find('$2') or '')")
case "$1" in
  plan) echo "FAKE plan (offline demo) for $2"; sleep 2
        cp "$J/okra_data/events/0000-examples/example_synthetic/trajectory.json" "$ev/"
        cp "$J/okra_data/events/0000-examples/example_synthetic/sim.mp4" "$ev/" 2>/dev/null || true
        echo "FAKE plan done" ;;
  reach|pick) echo "FAKE safety gate: PASS (offline demo)"; echo "FAKE $1 of $2: arm moving (simulated)"; sleep 3
        echo '{"aborted": null, "fake": true}' > "$ev/trajectory_executed.json"; echo "FAKE $1 done" ;;
  *) echo "fake_pick.sh: $1 not simulated"; exit 2 ;;
esac
