"""Read or set the G1 sport-controller FSM state, one step at a time.

Usage: .venv/bin/python g1_fsm.py            # print current FSM id
       .venv/bin/python g1_fsm.py <fsm_id>   # set it, then print the result
FSM ids: 0 zero torque, 1 damp, 4 lock stand, 500 balance/walk (Start), 706 squat<->stand
"""
import functools
import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

print = functools.partial(print, flush=True)  # os._exit skips buffer flush

ChannelFactoryInitialize(0, os.environ.get("G1_IFACE", "enp0s8"))
loco = LocoClient()
loco.SetTimeout(5.0)
loco.Init()
time.sleep(1.0)  # DDS discovery

print("fsm before:", loco.GetFsmId())
if len(sys.argv) > 1:
    code = loco.SetFsmId(int(sys.argv[1]))
    print(f"SetFsmId({sys.argv[1]}) ->", code)
    time.sleep(2.0)
    print("fsm after:", loco.GetFsmId())
os._exit(0)  # DDS threads otherwise keep the process alive
