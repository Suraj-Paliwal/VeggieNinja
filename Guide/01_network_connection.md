# 01 — Network connection (VM ↔ G1)

## Addresses

| Host | IP | Access |
|---|---|---|
| VM interface `enp0s8` | 192.168.123.100/24 | local |
| G1 motion controller | 192.168.123.161 | ping only |
| G1 Orin onboard PC | 192.168.123.164 | `ssh unitree@192.168.123.164` (key, no password) |

The robot has no internet access.

## The problem: lossy DDS over VirtualBox

`rt/lowstate` messages are ~2 KB, larger than one Ethernet frame, so they are split into
IP fragments. VirtualBox drops some fragments; the kernel then holds incomplete packets
until they time out and the reassembly buffer fills. Effect with default kernel settings:

| Setting | Default | Tuned |
|---|---|---|
| `net.ipv4.ipfrag_high_thresh` | 4194304 | 67108864 |
| `net.ipv4.ipfrag_low_thresh` | 3145728 | 50331648 |
| `net.ipv4.ipfrag_time` | 30 | 3 |
| `net.ipv4.ipfrag_max_dist` | 64 | 0 |

Measured on 2026-09-26 with `check.sh`:

| | Default | Tuned |
|---|---|---|
| first lowstate message | 6.8 s | 1.0 s |
| lowstate rate on VM | 22 Hz, 1.5 s stale | 172 Hz, 3 ms fresh |
| reassembly failed / ok | 1378 / 329 | 716 / 3144 |

The same topic read **on the robot** runs at ~990 Hz. That is why recording happens there.

## Apply the tuning

```bash
sudo ~/Junction/g1_connect/setup_host.sh            # now only
sudo ~/Junction/g1_connect/setup_host.sh --persist  # also /etc/sysctl.d/90-g1.conf (done 2026-09-26)
```

## Health check

```bash
~/Junction/g1_connect/check.sh
```
Read-only; never commands the robot. Checks, in order:
1. `enp0s8` up with an address
2. sysctl tuning applied
3. ping 161 and 164
4. SSH to the Orin, uptime
5. robot services `master_service`, `videohub_pc4`, `dex1_1_gripper_server` running
6. DDS: first `rt/lowstate` message and its rate / age, `mode_machine`
7. MotionSwitcher `CheckMode` (reports mode name, e.g. `ai`)
8. kernel IP-reassembly counters during the check

Expected last line: `== RESULT: ALL OK ==`. `WARN` lines include a `fix:` hint.

Note: after `rec.sh start`, `videohub_pc4` is intentionally stopped, so `check.sh`
will report it; run `g1_record/rec.sh release` to restore it.

## Python helper `g1_connect/g1_conn.py`

Functions used by `check.py` and reusable in scripts: connect (DDS init on `enp0s8`),
`LowStateMonitor` (rate/age of `rt/lowstate`), `rpc_retry` (retries RPCs that fail with
3102/3104 on the lossy link), `check_mode` (MotionSwitcher mode).

## When nothing answers

Symptom: ping fails, `ip neigh show dev enp0s8` shows `FAILED` for 161/164.
The robot is not answering ARP, so the problem is below IP. Check, in order:
1. Robot powered and booted (Orin takes ~1 min).
2. Ethernet cable seated.
3. VirtualBox → VM Settings → Network → the adapter for `enp0s8` is bridged to the NIC
   that goes to the robot (changes easily after the host switches networks).

`/sys/class/net/enp0s8/carrier` is always 1 inside VirtualBox; it proves nothing.

## Why not WSL2

WSL2 defaults to NAT, which blocks DDS multicast discovery. Mirrored networking might
work but is untested here and would need a fresh SDK install. Not pursued.
