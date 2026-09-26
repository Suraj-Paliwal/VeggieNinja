# Junction: controlling the Unitree G1 from this PC

This folder holds the scripts and setup used to talk to a **Unitree G1 humanoid (29-DoF)**
from an Ubuntu 22.04 PC (currently a VirtualBox VM). It covers the camera stream, stand-up
and FSM control, arm and waist motion, and the dimos framework for the later pick pipeline.

> **Safety first.** Before sending any motion command, keep the robot on a gantry (or
> have it standing on the floor with someone watching), keep the remote/e-stop in hand, and
> have a clear area around it. Every motion script here ramps its control in and out
> slowly, and Ctrl-C releases control.

---

## 1. How it works

```
 ┌──────────────── PC (Ubuntu 22.04 VM) ────────────────┐        Ethernet        ┌──────────── Unitree G1 ─────────────┐
 │                                                       │   192.168.123.0/24    │                                      │
 │  g1_video/*.py  ──► unitree_sdk2py ──► CycloneDDS ────┼───────────────────────┼──► 192.168.123.161 motion controller │
 │  (camera, FSM,        (Python SDK)     0.10.2         │   DDS / UDP multicast │     (balance, walking, rt/lowstate,  │
 │   arms, waist)                                        │                       │      rt/arm_sdk, Loco RPC)           │
 │                                                       │                       │                                      │
 │  enp0s8 = 192.168.123.100                             │                       │  192.168.123.164 Orin (onboard PC)   │
 │                                                       │                       │     videohub → head D435i camera     │
 │  dimos/ (framework, own .venv)                        │                       │     dex1_1_service → Dex1 grippers   │
 └───────────────────────────────────────────────────────┘                       └──────────────────────────────────────┘
```

- The robot speaks **DDS**, a publish/subscribe protocol over UDP multicast. Unitree's
  Python SDK (`unitree_sdk2py`) wraps it with **CycloneDDS**.
- There's no ROS in the loop. The scripts subscribe and publish to the robot's DDS topics
  directly:

  | Topic / service | Direction | Used for |
  |---|---|---|
  | `rt/lowstate` | robot → PC (~130 Hz) | joint angles, IMU, `mode_machine` (5 = 29-DoF) |
  | `rt/arm_sdk` | PC → robot | upper-body joint targets (waist 12–14, arms 15–28), blended over the built-in controller by a weight in `motor_cmd[29].q` |
  | Loco RPC (`LocoClient`) | request/reply | read/set the FSM (damp, stand, balance) |
  | MotionSwitcher RPC | request/reply | check which controller mode is active (reports `ai`) |
  | Video RPC (`VideoClient`) | request/reply | JPEG frames from the head D435i via `videohub` |
  | `rt/dex1/right/cmd`, `rt/dex1/right/state` | both | Dex1 right gripper (the **left gripper, motor id 1, doesn't reply**) |

- **The built-in Unitree controller always keeps the robot balanced.** We never send
  full low-level leg commands. For upper-body moves we publish on `rt/arm_sdk`, and the
  controller blends our targets in with the weight we set (0 = ignore us, 1 = follow us).
  That makes it safe to move the arms and waist while the robot stands on its own.

---

## 2. Network setup (one time)

1. Connect an Ethernet cable from the PC to the robot.
2. In **VirtualBox → Settings → Network → Adapter 2**:
   - Attached to: **Bridged Adapter** → your laptop's Ethernet port
   - Advanced → Adapter Type: **Paravirtualized Network (virtio-net)** (recommended; the default emulated Intel 82540EM is slow)
   - Promiscuous Mode: **Allow All**
3. Inside Ubuntu, give that interface (`enp0s8`) a static address:
   - Address `192.168.123.100`, netmask `255.255.255.0`, no gateway
4. **Increase the UDP buffers** (already done on this VM). Without this, camera frames are
   dropped and the video lags or shows nothing:
   ```bash
   printf 'net.core.rmem_max=67108864\nnet.core.rmem_default=67108864\nnet.core.wmem_max=67108864\n' \
     | sudo tee /etc/sysctl.d/60-g1-dds.conf && sudo sysctl --system
   ```
5. Check that the robot is reachable:
   ```bash
   ping -c 3 192.168.123.161   # motion controller
   ping -c 3 192.168.123.164   # onboard Orin
   ssh unitree@192.168.123.164 # key-based login is set up
   ```

To check for dropped packets at any time, run `netstat -su | grep "buffer errors"`. The
count should stay at 0.

---

## 3. Python environment (`g1_video/.venv`)

The scripts use their own virtual environment (Python 3.10.12, created with `uv`):

| Package | Version | Notes |
|---|---|---|
| `unitree_sdk2py` | 1.0.1 | installed **editable** from `g1_video/unitree_sdk2_python/` |
| `cyclonedds` | **0.10.2** | built against `~/src/cyclonedds` (C library installed in `~/src/cyclonedds/install`) |
| `numpy` | 2.2.6 | |
| `opencv-python` | 5.0.0 | video window and recording |

> **Use CycloneDDS 0.10.x.** The robot's firmware speaks 0.10. With cyclonedds 11.x the
> PC received nothing from the robot.

To rebuild from scratch (for example on a new machine):

```bash
# 1. CycloneDDS C library, 0.10.x branch
git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds ~/src/cyclonedds
cd ~/src/cyclonedds && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=OFF
cmake --build . --target install

# 2. Python venv
cd ~/Junction/g1_video
uv venv --python 3.10 .venv
export CYCLONEDDS_HOME=~/src/cyclonedds/install
uv pip install --python .venv/bin/python cyclonedds==0.10.2
uv pip install --python .venv/bin/python -e unitree_sdk2_python opencv-python numpy
```

Run every script with the venv's Python: `.venv/bin/python <script>.py`, from inside `g1_video/`.

---

## 4. The scripts (`g1_video/`)

All of them default to network interface `enp0s8`.

| Script | What it does | Moves the robot? |
|---|---|---|
| `view.py` | Live head-camera view (1920×1080, shown at 0.5×). `s` = snapshot, `q`/Esc = quit, `--save out.mp4` records | No |
| `record.py` | Camera viewer with clip recording: `r` starts/stops a clip (saved to `recordings/clip_*.mp4` at real speed), `s` = snapshot, `--duration 10` records one clip and exits | No (unless `--head`) |
| `record.py --head` | Same, plus turning the waist to aim the camera: `j`/`l` left/right, `i`/`k` up/down, `c` center | **Yes (waist)**, untested on hardware |
| `waist.py` | `WaistController` used by `record.py --head`. Waist yaw ±0.6 rad, pitch ±0.3 rad, max 0.4 rad/s; arms held where they were | **Yes** |
| `g1_fsm.py` | Print the current FSM id, or set one: `g1_fsm.py 4` | **Yes** |
| `arm_move.py` | Demo: raise both arms, open them, return to the start pose over `rt/arm_sdk`. Ctrl-C ramps the weight back to 0 | **Yes (arms)** |
| `wait_normal.py` | After a power cycle, waits (up to 15 min) until the Loco service answers, then prints the FSM and mode | No |

### The G1 has no neck

The D435i is fixed to the torso, so "looking around" means turning the **waist**
(joints 12–14). That's what `waist.py` does.

---

## 5. Typical session

```bash
cd ~/Junction/g1_video

# 0. Robot powered on, 1–2 min after boot (rt/lowstate appears after that)
ping -c 2 192.168.123.161

# 1. Look through the camera
.venv/bin/python view.py

# 2. Stand up (robot on gantry, e-stop in hand). Run these one at a time and watch the robot:
.venv/bin/python g1_fsm.py        # print current FSM (0 = zero torque)
.venv/bin/python g1_fsm.py 1      # damp
.venv/bin/python g1_fsm.py 4      # lock stand
.venv/bin/python g1_fsm.py 200    # AI balance (on this firmware 500 is refused, use 200)

# 3. Arm demo while it balances
.venv/bin/python arm_move.py

# 4. Camera + waist aiming + recording
.venv/bin/python record.py --head
```

FSM ids used here: `0` zero torque, `1` damp, `4` lock stand, `200` AI balance,
`706` squat ↔ stand. (`500` balance/walk is refused on this robot's firmware.)

---

## 6. Robot-side notes (Orin, 192.168.123.164)

- `videohub_pc4 /dev/video4` serves the **head** D435i (what `view.py`/`record.py` show).
  `videohub_pc4_chest /dev/video10` serves a chest camera.
- `~/run_ik_camera.sh` on the Orin streams RGB + point cloud over **LCM** multicast
  (`239.255.76.67:7667`, ~3 Hz) for dimos. **It kills videohub**, so `view.py` and
  `record.py` stop working while it runs.
- `dex1_1_service` runs the Dex1 grippers. Only the right one answers.
- A stripped-down `~/workSpace/dimos_min` exists on the Orin. The full dimos isn't
  installed there.
- The robot has no internet access.

---

## 7. dimos (`dimos/`)

`dimos/` is the dimos robotics framework with its own `.venv`, where `unitree-sdk2py-dimos`
and cyclonedds 0.10.2 (built from `~/src/cyclonedds`) are installed. So far it has been
run in simulation (`g1sim.log`). The next phases of the project build a G1 pick pipeline
in it:

1. **Stand and walk** with the built-in controller (keyboard teleop, slow limits).
2. **Click a target** in the camera view → 3D point from the point cloud → robot (pelvis) frame.
3. **Reach and grasp** with the right arm (IK → `rt/arm_sdk` trajectory) and close the Dex1 right gripper.
4. Later: detect objects by name instead of clicking (needs a GPU; see below).

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Video lags, freezes or `GetImageSample failed` | UDP buffers too small. Apply the sysctl in §2 and check `netstat -su` buffer errors |
| No reply from anything, `no rt/lowstate received` | Robot not booted yet (wait 1–2 min), cable unplugged, wrong interface, or the VM adapter isn't bridged |
| PC receives no DDS data at all | cyclonedds version mismatch: use 0.10.2, not 11.x |
| `GetFsmId` returns code **3102** | `RPC_ERR_CLIENT_SEND`: the Loco RPC is unreachable (the service isn't up yet, or a network problem). It does **not** mean debug mode |
| Loco returns **3104** | RPC timeout: usually dropped UDP packets, so fix the buffers |
| `view.py` stopped working | `run_ik_camera.sh` is running on the Orin and killed videohub. Restart videohub or reboot the robot |
| Robot stuck in debug mode | Power-cycle it with the battery and leave the remote untouched. Then run `wait_normal.py` |
| Left gripper does nothing | Known hardware issue (motor id 1 doesn't reply). Use `rt/dex1/right/*` |

---

## 9. VM limitations

- **No GPU in VirtualBox.** VirtualBox can't pass the laptop's GPU through for CUDA, so
  object detection models run slowly on the CPU.
- The emulated network card adds latency. Switch to virtio-net (§2).
- **Recommended long-term setup:** dual-boot native Ubuntu 22.04 on the laptop. That gives
  you the real Ethernet port, all CPU and RAM, and the NVIDIA GPU. Then copy `~/Junction`
  across, rebuild the venvs (§3) and apply the sysctl (§2).
