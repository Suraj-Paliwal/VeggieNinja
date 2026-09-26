#!/usr/bin/env python3
"""G1 connection check. Read-only; sends no motion commands.

    ./check.sh              # full check
    ./check.sh --watch 30   # also print lowstate rate every second for 30 s

Exit code 0 = everything OK, 1 = something failed (see FAIL lines).
"""
import argparse
import math
import os
import subprocess
import sys
import time

import g1_conn as g

WANT_SYSCTL = {
    "net.ipv4.ipfrag_high_thresh": 67108864,
    "net.ipv4.ipfrag_low_thresh": 50331648,
    "net.ipv4.ipfrag_time": 3,
    "net.ipv4.ipfrag_max_dist": 0,
}
ROBOT_SERVICES = ["master_service", "videohub_pc4", "dex1_1_gripper_server"]

fails = []


def report(ok, name, detail="", warn=False):
    tag = "OK  " if ok else ("WARN" if warn else "FAIL")
    print(f"[{tag}] {name}" + (f": {detail}" if detail else ""), flush=True)
    if not ok and not warn:
        fails.append(name)
    return ok


def sh(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return 124, ""


def reasm():
    lines = [l.split() for l in open("/proc/net/snmp") if l.startswith("Ip:")]
    d = dict(zip(lines[0][1:], map(int, lines[1][1:])))
    return d["ReasmOKs"], d["ReasmFails"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=int, default=0, help="seconds to print lowstate rate")
    args = ap.parse_args()
    print(f"== G1 connection check (iface {g.IFACE}) ==")

    # 1. host interface
    rc, out = sh(f"ip -br addr show {g.IFACE}")
    if not report(rc == 0 and "UP" in out and g.HOST_IP in out, "interface",
                  out or f"{g.IFACE} missing"):
        print(f"  fix: sudo ip addr add {g.HOST_IP}/24 dev {g.IFACE}; sudo ip link set {g.IFACE} up")
        return 1

    # 2. host sysctls (fragment reassembly tuning)
    bad = []
    for k, v in WANT_SYSCTL.items():
        rc, out = sh(f"sysctl -n {k}")
        if out != str(v):
            bad.append(f"{k}={out} (want {v})")
    report(not bad, "sysctl tuning", "; ".join(bad) or "applied", warn=True)
    if bad:
        print("  fix: ./setup_host.sh")

    # 3. ping
    for name, ip in (("motion controller", g.MCU_IP), ("onboard pc", g.PC_IP)):
        rc, out = sh(f"ping -c3 -i0.2 -W1 {ip}")
        loss = next((l for l in out.splitlines() if "packet loss" in l), "no reply")
        report(rc == 0, f"ping {name} {ip}", loss.split(",")[2].strip() if "," in loss else loss)

    # 4. ssh + robot services
    rc, out = sh(f"ssh -o BatchMode=yes -o ConnectTimeout=5 unitree@{g.PC_IP} "
                 "'uptime -p; ps -eo args'")
    if report(rc == 0, "ssh unitree@" + g.PC_IP, out.splitlines()[0] if rc == 0 else "failed"):
        for s in ROBOT_SERVICES:
            report(s in out, f"robot service {s}", "running" if s in out else "not running",
                   warn=(s != "master_service"))

    # 5. DDS lowstate
    ok0, f0 = reasm()
    t0 = time.monotonic()
    mon = g.LowStateMonitor()
    got = mon.wait(timeout=25)
    report(got, "DDS rt/lowstate",
           f"first msg after {time.monotonic() - t0:.1f}s" if got else "nothing in 25s")
    if got:
        time.sleep(3)
        r = mon.rate()
        m = mon.latest()
        report(r >= 50, "lowstate rate", f"{r:.0f} Hz (age {mon.age() * 1000:.0f} ms), "
               f"mode_machine={m.mode_machine}", warn=(r > 0))

    # 6. MotionSwitcher RPC
    mode = g.check_mode(verbose=True)
    report(mode is not None, "MotionSwitcher CheckMode", str(mode) if mode else "no reply after retries")

    # 7. optional watch
    if args.watch and got:
        print(f"-- lowstate rate for {args.watch}s --")
        for i in range(args.watch):
            time.sleep(1)
            a = mon.age()
            print(f"  t={i + 1:3d}s  rate={mon.rate():5.0f} Hz  total={mon.count():6d}  "
                  f"age={'-' if math.isinf(a) else f'{a * 1000:.0f} ms'}", flush=True)

    ok1, f1 = reasm()
    print(f"   (IP reassembly during check: {ok1 - ok0} ok, {f1 - f0} failed)")
    print("== RESULT:", "ALL OK" if not fails else "FAILED: " + ", ".join(fails), "==")
    return 1 if fails else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)  # cyclonedds threads can hang interpreter shutdown
