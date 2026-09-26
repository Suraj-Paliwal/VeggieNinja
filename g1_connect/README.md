# g1_connect

Connection setup and health check for the Unitree G1 on the wired link.
Read-only: nothing here sends motion commands.

## Every session
```
./check.sh              # full check, exit 0 = OK
./check.sh --watch 30   # also print lowstate rate every second
```
If the `sysctl tuning` line says WARN, run `./setup_host.sh` (needs sudo).
Run `./setup_host.sh --persist` once to make the tuning survive reboots.

## What it checks
1. `enp0s8` is up with 192.168.123.100 (override with `G1_IFACE`).
2. Kernel IP-fragment settings (see below).
3. Ping to the motion controller (.161) and the onboard PC (.164).
4. SSH `unitree@192.168.123.164` and whether robot services are running.
5. DDS `rt/lowstate`: time to first message, rate, and message age.
6. MotionSwitcher `CheckMode` RPC, with retries.

## Using it in your own scripts
```python
import sys; sys.path.insert(0, "/home/ros2/Junction/g1_connect")
from g1_conn import connect, LowStateMonitor, check_mode, rpc_retry
connect()
mon = LowStateMonitor()
assert mon.wait(timeout=25), "no lowstate"
if not mon.fresh(0.5):
    ...  # stale state: don't act on it
code, data = rpc_retry(some_client.SomeCall)
```
Run with `../g1_video/.venv/bin/python`.

## Why the link is flaky
`rt/lowstate` (~2 KB) is larger than the 1500-byte MTU, so every message
is sent as IP fragments. The VirtualBox adapter drops some fragments, and
one missing fragment loses the whole message. Typical result: 10-175 Hz
with gaps of up to about 2 s instead of 500 Hz. The sysctl tuning stops the
kernel from making it worse, but it can't recover dropped packets.
Discovery can also take 5-20 s, and RPCs sometimes return 3102/3104
before succeeding on retry.

The real fix is on the host side: a USB-Ethernet adapter passed straight
to the VM, or running Linux natively.
