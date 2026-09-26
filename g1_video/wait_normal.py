import time, subprocess
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
ChannelFactoryInitialize(0, "enp0s8")
m = MotionSwitcherClient(); m.SetTimeout(2.0); m.Init()
l = LocoClient(); l.SetTimeout(2.0); l.Init()
went_down = False
end = time.time() + 900
while time.time() < end:
    up = subprocess.run(["ping", "-c1", "-W1", "192.168.123.161"], capture_output=True).returncode == 0
    if not up and not went_down:
        went_down = True; print("robot went offline (power cycle seen)", flush=True)
    if up and went_down:
        code, fsm = l.GetFsmId()
        if code == 0:
            print("NORMAL MODE: loco service up, fsm_id =", fsm, "| motion switcher:", m.CheckMode(), flush=True)
            break
    time.sleep(2)
else:
    print("timed out after 15 min", flush=True)
