"""Safety gate policy: the LIMITS we choose, in one place. Robot facts (clock, interface, mode, temperatures,
who is commanding the arm, ...) are never written here: they are measured live by robot/safety_probe.py.
A limit that has not been checked against the Unitree spec is marked POLICY-UNVERIFIED and only WARNs."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot"))
import pick_config as C  # noqa: E402

# which checks gate which okra_pick.sh step
MOTION_STEPS = ("dry", "reach", "pick")            # arm moves (dry: checks only, same gate so dry == rehearsal)
CAMERA_STEPS = ("look", "perceive", "floor")
EVENT_STEPS = ("plan", "dry", "reach", "pick")     # need a perception event
GRIPPER_STEPS = ("gripper",) + MOTION_STEPS

MAIN_FSM = (200, 500, 501)                         # balance states the player accepts (arm_player.MAIN_FSM)

# link / state stream (measured on the Orin, so VM link loss does not count here)
MIN_LOWSTATE_HZ = 100.0
MAX_LOWSTATE_GAP_S = C.STALE_S                     # the player aborts above this, so refuse to start above it
MIN_GRIP_STATE_HZ = 10.0

# posture (shared with the player)
MAX_TILT_RAD = C.MAX_TILT
MIN_KNEE_LOAD_NM = 15.0                            # same as robot_mode.py / robot_agent balance gate
MAX_WAIST_DIFF_RAD = C.MAX_WAIST_DIFF
MAX_START_DIFF_RAD = C.MAX_START_DIFF

# time since perception, measured on the ROBOT clock only (perception time and "now" both from the Orin)
MAX_EVENT_AGE_S = C.MAX_EVENT_AGE_S

# resources
MIN_ROBOT_DISK_GB = 5.0
MIN_VM_DISK_GB = 10.0
MIN_BATTERY_SOC = 30                               # %; battery unknown -> WARN (never assumed full)
MOTOR_TEMP_WARN_C = 70                             # POLICY-UNVERIFIED: warn only
ORIN_TEMP_WARN_C = 85.0                            # POLICY-UNVERIFIED: warn only

RIGHT_ARM_IDX = C.RIGHT_ARM_IDX
