#!/usr/bin/env bash
# Run the G1 connection check with the g1_video venv.
here="$(cd "$(dirname "$0")" && pwd)"
exec "$here/../g1_video/.venv/bin/python" "$here/check.py" "$@"
