"""Slow, reversible G1 arm motion via rt/arm_sdk (built-in controller keeps balance).

Usage: .venv/bin/python arm_move.py [iface]   (default enp0s8)
Ctrl-C at any time ramps arm_sdk weight back to 0 before exiting.
"""
import os
import signal
import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# Left arm 15-21, right arm 22-28 (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
ARM = list(range(15, 29))
WEIGHT_IDX = 29  # motor_cmd[29].q = arm_sdk blend weight
DT, KP, KD = 0.02, 40.0, 1.5

# Offsets from the start pose (rad), applied in sequence: (label, {joint: target}, seconds)
L_SP, L_SR, L_EL, R_SP, R_SR, R_EL = 15, 16, 18, 22, 23, 25
POSES = [
    ("raise both arms forward", {L_SP: -0.9, R_SP: -0.9, L_EL: -0.5, R_EL: -0.5}, 4.0),
    ("open arms out", {L_SP: -0.9, R_SP: -0.9, L_SR: 0.5, R_SR: -0.5, L_EL: -0.5, R_EL: -0.5}, 3.0),
    ("back to start pose", {}, 4.0),
]

state = None
stop = False


def on_state(msg):
    global state
    state = msg


def on_sigint(*_):
    global stop
    stop = True


def main():
    ChannelFactoryInitialize(0, sys.argv[1] if len(sys.argv) > 1 else "enp0s8")
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    ChannelSubscriber("rt/lowstate", LowState_).Init(on_state, 10)
    signal.signal(signal.SIGINT, on_sigint)

    t0 = time.time()
    while state is None:
        if time.time() - t0 > 5:
            sys.exit("no rt/lowstate received")
        time.sleep(0.05)

    start = np.array([state.motor_state[j].q for j in ARM])
    cmd, crc = unitree_hg_msg_dds__LowCmd_(), CRC()

    next_t = [time.monotonic()]

    def tick():  # fixed-rate loop timing, independent of send() cost
        next_t[0] += DT
        time.sleep(max(0.0, next_t[0] - time.monotonic()))

    def send(q, weight):
        cmd.motor_cmd[WEIGHT_IDX].q = float(weight)
        for j, qj in zip(ARM, q):
            m = cmd.motor_cmd[j]
            m.q, m.dq, m.tau, m.kp, m.kd = float(qj), 0.0, 0.0, KP, KD
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    def ramp_weight(q, w_from, w_to, secs):
        for k in range(int(secs / DT) + 1):
            send(q, w_from + (w_to - w_from) * k / (secs / DT))
            tick()

    print("engaging arm_sdk (holding current pose)")
    ramp_weight(start, 0.0, 1.0, 2.0)

    cur = start.copy()
    for label, offsets, secs in POSES:
        if stop:
            break
        target = start.copy()
        for j, off in offsets.items():
            target[ARM.index(j)] += off
        print(label)
        n = int(secs / DT)
        prev = cur.copy()
        for k in range(1, n + 1):
            if stop:
                break
            s = 0.5 - 0.5 * np.cos(np.pi * k / n)  # smooth ease-in/out
            cur = prev + s * (target - prev)
            send(cur, 1.0)
            tick()

    if stop:
        print("interrupted: returning to start pose")
        prev = cur.copy()
        for k in range(1, 101):
            send(prev + (start - prev) * k / 100, 1.0)
            tick()
    print("releasing arm_sdk")
    ramp_weight(start, 1.0, 0.0, 2.0)
    print("done")
    os._exit(0)  # DDS threads otherwise keep the process alive


if __name__ == "__main__":
    main()
