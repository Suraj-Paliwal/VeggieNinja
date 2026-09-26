# robot_agent — fast, reliable commands from the VM to the G1

One persistent process on the robot (`agent.py`, Orin, `~/g1_rec/pylib`) owns the DDS connection;
the VM sends commands over one TCP connection with `g1ctl`. Details: [`../Guide/12_robot_agent.md`](../Guide/12_robot_agent.md).

```bash
./agent.sh deploy && ./agent.sh start     # once per robot boot (or after code changes: ./agent.sh restart)
./g1ctl status                            # instant state + balance check
./g1ctl watch                             # live state line
./g1ctl stop                              # zero velocity + cancel whatever is running
./g1ctl ai | damp | ready | start         # bring-up
./g1ctl left 20 | right 10 | forward 20 | back 20
./g1ctl head 0.15 | head -0.2 0.1 3 0.2   # waist yaw [pitch hold speed]
./g1ctl gripper open | close
```
Test without the robot: `python3 agent.py --sim --host 127.0.0.1 --port 7799` and `G1_AGENT=127.0.0.1:7799 ./g1ctl status`.
