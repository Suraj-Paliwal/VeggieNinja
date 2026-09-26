#!/usr/bin/env python3
"""
Operator link lost in the middle of a pick: the player must still take its safe exit (abort, arm back
along the executed path, blend out) and write player.log, instead of dying mid-motion.

    ../dimos/.venv/bin/python tests/test_link_loss.py <trajectory.json>

Cases (fake robot from test_player_sim, player started like okra_pick.sh starts it):
  pipe_closed  ssh -T dropped (web interface): stdout pipe closes -> next print fails (EPIPE)
  sighup       SIGHUP delivered (ssh session ends)
  pty_closed   ssh -t dropped (terminal): controlling pty hangs up -> SIGHUP + EIO
  sigint       Ctrl-C / web STOP (pkill -INT)
"""

import json
import os
import pty
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WAIT_FOR = b"segment: approach"


def child(traj_path, out_dir):
    sys.path.insert(0, HERE)
    sys.argv = [sys.argv[0], traj_path]                 # test_player_sim reads nothing else at import
    import numpy as np
    import test_player_sim as T                        # sets C.DT (fast) and imports arm_player as AP
    AP = T.AP
    AP.install_safety(os.path.join(out_dir, "player.log"))
    traj = json.load(open(traj_path))
    T.C.DT = 0.02                                      # real time: gives the parent time to cut the link
    r = T.FakeRobot(traj, "nominal")
    p = AP.Player(r, traj, None)
    aborted = p.run()
    home = bool(np.abs(r.qarm - np.array(traj["start_q"])).max() < 0.05)
    json.dump({"aborted": aborted, "back_at_start": home, "grip": r.grip_target},
              open(os.path.join(out_dir, "result.json"), "w"))
    print("result: %s" % aborted, flush=True)


def read_until(fd, token, timeout=30.0):
    buf, t0 = b"", time.time()
    while token not in buf and time.time() - t0 < timeout:
        try:
            chunk = os.read(fd, 4096)
        except OSError:
            break
        if not chunk:
            break
        buf += chunk
    return token in buf


def run_case(case, traj):
    d = tempfile.mkdtemp(prefix="link_%s_" % case)
    argv = [sys.executable, os.path.abspath(__file__), "--child", traj, d]
    if case == "pty_closed":
        pid, master = pty.fork()
        if pid == 0:
            os.execv(argv[0], argv)
        ok = read_until(master, WAIT_FOR)
        os.close(master)                               # the terminal disappears
        _, status = os.waitpid(pid, 0)
    else:
        p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        ok = read_until(p.stdout.fileno(), WAIT_FOR)
        if case == "pipe_closed":
            p.stdout.close()
        elif case == "sighup":
            p.send_signal(signal.SIGHUP)
        elif case == "sigint":
            p.send_signal(signal.SIGINT)
        if case != "pipe_closed":
            p.stdout.close()
        p.wait(timeout=120)
    res_path = os.path.join(d, "result.json")
    res = json.load(open(res_path)) if os.path.exists(res_path) else None
    log = open(os.path.join(d, "player.log")).read() if os.path.exists(os.path.join(d, "player.log")) else ""
    good = bool(ok and res and res["aborted"] and res["back_at_start"] and "blend out" in log)
    why = (res or {}).get("aborted") or "player died without the safe exit"
    print("%-12s %s  back_at_start=%-5s  %s   (log %s)" % (case, "PASS" if good else "FAIL",
          (res or {}).get("back_at_start"), why, d))
    return good


if __name__ == "__main__":
    if sys.argv[1] == "--child":
        child(sys.argv[2], sys.argv[3])
        sys.exit(0)
    results = [run_case(c, os.path.abspath(sys.argv[1])) for c in ("pipe_closed", "sighup", "pty_closed", "sigint")]
    print("ALL PASS" if all(results) else "SOME FAILED")
    sys.exit(0 if all(results) else 1)
