#!/usr/bin/env bash
# Runs ON THE ROBOT. Builds ~/g1_rec/pylib offline (the robot has no internet and no python3-venv):
#   cyclonedds 0.10.2 (cached aarch64 wheel), unitree_sdk2py (source copy), typing_extensions.
# numpy comes from the system Python 3.8. Safe to re-run.
set -e
dst=~/g1_rec/pylib
mkdir -p "$dst"
if ! PYTHONPATH=$dst python3 -c "import cyclonedds" 2>/dev/null; then
  whl=$(ls ~/.cache/pip/wheels/*/*/*/*/cyclonedds-0.10.2-cp38-cp38-linux_aarch64.whl | head -1)
  python3 -m pip install -q --no-index --no-deps --target "$dst" "$whl"
fi
[ -d "$dst/unitree_sdk2py" ] || cp -r ~/unitree_sdk2_python/unitree_sdk2py "$dst/"
if [ ! -f "$dst/typing_extensions.py" ]; then
  src=$(~/miniconda3/envs/g1brainco/bin/python -c "import typing_extensions as t; print(t.__file__)")
  cp "$src" "$dst/"
fi
cd /
PYTHONPATH=$dst python3 -c "
import cyclonedds, numpy, unitree_sdk2py
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
print('pylib ok: numpy', numpy.__version__)"
command -v ffmpeg >/dev/null && echo "ffmpeg ok" || echo "MISSING ffmpeg"
