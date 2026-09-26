# 15 — VM tuning: disk, VirtualBox settings, green turtle (Hyper-V)

VM `tamROS2-2503` (Ubuntu, VirtualBox on a Windows laptop, i7-9750H 6 cores / 12 threads).
State on 2026-09-27: 8 vCPU, 11629 MB RAM, **64 GB disk 87 % full**, 3D off, robot NIC = virtio bridged
to the Realtek wired port (good). Do the steps in order. **Steps 3–9 need the VM fully powered off.**

## 1. Free space now (VM running, safe)

```
uv cache prune
rm -rf ~/.cache/go-build
df -h /                      # want >= 15 GB free before a robot run
```

## 2. Power the VM off

```
sudo poweroff
```
Use a real shutdown, not "Save the machine state". In VirtualBox Manager the VM must show **Powered Off**.
Optional backup first: right-click the VM → **Clone...** (Full clone), or copy the `.vdi` file.

## 3. Grow the disk 64 → 128 GB (GUI)

1. VirtualBox Manager → **File → Tools → Virtual Media Manager**
   (VirtualBox 7: left panel **Tools → Media**).
2. **Hard disks** tab → select `tamROS2-2503.vdi`.
3. **Properties** tab below → drag **Size** to **128 GB** → **Apply**.
4. If the VM has snapshots, the resize must be done on the newest snapshot disk (or delete snapshots first).

The partition inside Ubuntu is grown in step 10.

## 4. Storage (Settings → Storage)

1. Select **Controller: SATA** → check **Use Host I/O Cache**.
2. Select `tamROS2-2503.vdi` under it → check **Solid-state Drive** (if the laptop has an SSD).
3. Select **Controller: IDE → VBoxGuestAdditions.iso** → click the disc icon on the right →
   **Remove Disk from Virtual Drive** (Guest Additions are already installed).

## 5. Display (Settings → Display → Screen)

1. **Monitor Count: 1** (was 2).
2. **Video Memory: 128 MB** (or 256 MB if the slider allows it).
3. Graphics Controller: **VMSVGA** (keep).
4. Check **Enable 3D Acceleration**.
   If the desktop glitches or the MuJoCo sim window crashes afterwards, uncheck it again
   (the plan still works; only `sim.mp4` / the sim window need it).

## 6. Audio (Settings → Audio)

Uncheck **Enable Audio** (unless you need sound in the VM).

## 7. Network (Settings → Network)

- **Adapter 1:** NAT, Intel PRO/1000 MT Desktop (internet) — keep.
- **Adapter 2:** Bridged Adapter → **Realtek PCIe GBE Family Controller**, type **Paravirtualized
  Network (virtio-net)** — keep. Open **Advanced** → **Promiscuous Mode: Allow All**,
  **Cable Connected** checked.

## 8. System (Settings → System)

- **Processor:** keep **8** (do not give all 12 threads; host and VM would fight).
- **Base Memory:** keep 11629 MB unless Windows has lots of RAM
  (Task Manager → Performance → Memory): 16 GB total → keep; 32 GB total → 16384 MB.
- **Acceleration:** Paravirtualization **KVM**, **Nested Paging** checked — keep.

## 9. Green turtle = VirtualBox running on top of Hyper-V (Windows side)

**Check:** start the VM and look at the status bar, bottom right of the VM window.
A **green turtle** icon = slow mode (2–3× slower CPU, worse network timing).
A blue **"V"** chip icon = normal. You can also check the log: VM → **Show Log...** →
search `NEM` (present = Hyper-V mode).

**Fix (Windows, needs admin, then a full restart):**

1. **Windows features:** press Win+R → `optionalfeatures` → **uncheck**:
   - Hyper-V
   - Virtual Machine Platform
   - Windows Hypervisor Platform
   - Windows Sandbox (if listed)
   → OK.
2. **Memory integrity off:** Windows Security → **Device security** → **Core isolation details** →
   **Memory integrity: Off**.
3. **Hypervisor off at boot:** Start → type `cmd` → right-click **Run as administrator** →
   ```
   bcdedit /set hypervisorlaunchtype off
   ```
4. **Restart** (use Restart, not Shut down, so Fast Startup does not keep the hypervisor).
5. **Verify:** Win+R → `msinfo32` → bottom of System Summary:
   "Virtualization-based security: **Not enabled**", and "A hypervisor has been detected" must be
   **absent**. Start the VM: no turtle.

**Side effects:** WSL2, Docker Desktop (WSL2 backend), Windows Sandbox and the Hyper-V Android
emulator stop working. **Undo** = re-check the features and run `bcdedit /set hypervisorlaunchtype auto`,
then restart.

If the turtle is still there: a Windows update may have re-enabled Memory integrity, or
Credential Guard is on (company-managed laptops; ask IT).

## 10. Grow the partition inside Ubuntu (VM running)

```
lsblk                                  # sda should now show 128G, sda3 still ~63.5G
sudo apt install -y cloud-guest-utils
sudo growpart /dev/sda 3
sudo resize2fs /dev/sda3
df -h /                                # / should show ~125G
```

## 11. Laptop power (Windows)

- Keep the charger in (the i7-9750H slows down a lot on battery).
- Settings → System → Power → **Power mode: Best performance**.
- Use the vendor's "performance" fan profile if the laptop has one; keep the vents free.

## 12. Check after boot

```
nproc                                  # 8
free -h                                # ~11 GB
df -h /                                # lots of free space
ip -br addr show enp0s8                # 192.168.123.100/24
ethtool -i enp0s8 | grep driver        # virtio_net
sysctl net.core.rmem_max               # 67108864 (from /etc/sysctl.d/, see 01/09)
~/Junction/g1_connect/check.sh         # robot link health (robot powered on)
```

## What a VM setting cannot fix

- **No GPU in the VM**: VirtualBox cannot pass through the laptop NVIDIA GPU. For fast training,
  train on the robot Orin (CUDA, conda env `g1brainco`) or dual-boot native Ubuntu 22.04.
- **DDS over the VM link stays lossy**: always run robot SDK code **on the Orin** (robot_mode.sh,
  rec.sh, okra_pick.sh, robot_agent). Do not run `g1_video/*` motion tools from the VM (10, 12).
