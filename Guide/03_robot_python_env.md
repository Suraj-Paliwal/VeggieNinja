# 03 — Python environment on the robot (offline)

The recorder runs on the Orin, which has **no internet**, **no `python3-venv`**
(`ensurepip` missing) and **no unitree_sdk2py** on the system Python. The VM's packages
cannot be copied: the VM is x86_64, the Orin is aarch64.

## Solution: a private library folder

`~/g1_rec/pylib` on the robot, used only via `PYTHONPATH=pylib`. Nothing is installed
system-wide and no existing environment is modified.

| Package | Source on the robot | Notes |
|---|---|---|
| `cyclonedds` 0.10.2 | `~/.cache/pip/wheels/.../cyclonedds-0.10.2-cp38-cp38-linux_aarch64.whl` | installed with `pip --target --no-index --no-deps` |
| `unitree_sdk2py` | `~/unitree_sdk2_python/unitree_sdk2py` | directory copy |
| `typing_extensions` | conda env `g1brainco` site-packages (4.13.2) | single file copy; cyclonedds needs it |
| numpy 1.17.4 | system Python 3.8 | not copied |

Total size ~3.8 MB.

## Build / rebuild

From the VM:
```bash
~/Junction/g1_record/rec.sh setup
```
This copies `robot/recorder.py` and `robot/setup_robot.sh` to `~/g1_rec/` and runs the
setup script. Each step is skipped if already done, so it is safe to re-run.
Expected output:
```
pylib ok: numpy 1.17.4
ffmpeg ok
```

Manual equivalent on the robot:
```bash
cd ~/g1_rec
python3 -m pip install --no-index --no-deps --target pylib ~/.cache/pip/wheels/*/*/*/*/cyclonedds-0.10.2-cp38-cp38-linux_aarch64.whl
cp -r ~/unitree_sdk2_python/unitree_sdk2py pylib/
cp ~/miniconda3/envs/g1brainco/lib/python3.8/site-packages/typing_extensions.py pylib/
cd / && PYTHONPATH=~/g1_rec/pylib python3 -c "import cyclonedds, unitree_sdk2py"
```

Test the import from `/` (or any folder that does not contain a `unitree_sdk2py` directory) so
a stray local copy can't shadow the one in pylib.

## Why not the existing conda env `g1brainco`?

It already imports unitree_sdk2py (Python 3.8.20), but it belongs to another project
(BrainCo hand). Depending on it means someone else's upgrade can break recording.
The private folder is small, pinned and owned by this project.

## If the robot is re-flashed

The three sources above may disappear. Before re-flashing, back up `~/g1_rec/pylib`
from the VM:
```bash
rsync -a unitree@192.168.123.164:g1_rec/pylib ~/Junction/g1_record/robot_pylib_backup/
```
Restoring it is a plain copy back (it is architecture-specific: aarch64 / cp38 only).

## Verified

2026-09-26: imports `cyclonedds`, `unitree_sdk2py`, `unitree_hg LowState_/LowCmd_`,
`unitree_go MotorStates_/MotorCmds_`; the probe received `rt/lowstate` at ~1 kHz on `eth0`.
