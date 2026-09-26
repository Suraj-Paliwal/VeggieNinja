"""Shared settings for the okra pick (planner on the VM, player on the robot). Python 3.8 compatible.

Everything marked CALIBRATE must be checked on the real robot before the first grasp
(see Guide/11_okra_pick_pipeline.md, "Calibration").
"""

# ---- right arm (G1 29-DoF): URDF joint names and rt/lowstate / rt/arm_sdk motor indices ----
RIGHT_ARM = ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
             "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
             "right_wrist_yaw_joint"]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
WAIST_IDX = [12, 13, 14]
UPPER_IDX = list(range(12, 29))          # everything arm_sdk drives (waist + both arms)
WEIGHT_IDX = 29                          # motor_cmd[29].q = arm_sdk blend weight
KP_ARM, KD_ARM = 40.0, 1.5               # proven in g1_video/arm_move.py
KP_WAIST, KD_WAIST = 60.0, 2.0           # proven in g1_video/waist.py
DT = 0.02                                # 50 Hz

# ---- Dex1 gripper on the right wrist (the left Dex1 motor does not reply) ----
GRIP_CMD_TOPIC, GRIP_STATE_TOPIC = "rt/dex1/right/cmd", "rt/dex1/right/state"
GRIP_OPEN_Q = 5.2                        # rad, ~fully open (5.35 measured at rest)
GRIP_CLOSE_Q = 0.0                       # rad, fully closed (calibrated zero)
GRIP_KP, GRIP_KD = 5.0, 0.05             # Unitree test_gripper.cpp values
GRIP_EMPTY_Q = 0.4                       # rad: closed below this = nothing between the jaws (CALIBRATE)
GRIP_SETTLE_S = 3.0                      # max wait for the jaws to stop

# ---- tool (grasp point between the jaws), in right_wrist_yaw_link ----
# Dex1 base is mounted where the rubber hand was (right_hand_palm_joint: x=0.0415, y=-0.003);
# the pad centre sits ~0.09 m further along the gripper axis (dex1_1.urdf finger joints at 0.059+0.038).
TCP_XYZ = (0.0415 + 0.09, -0.003, 0.0)   # CALIBRATE
CLOSE_AXIS = "y"                         # wrist axis the jaws close along: "y" or "z"  (CALIBRATE)

# ---- grasp geometry (pelvis frame, metres) ----
PREGRASP_BACK = 0.08                     # start of the straight approach, behind the pod (0.12 shrinks the reach, see Guide)
GRASP_DEPTH = 0.01                       # go this far past the pod centre so it sits deep in the jaws
PULL_BACK, PULL_DOWN = 0.06, 0.02        # harvesting pull after closing
PULL_TWIST_DEG = 45.0                    # wrist twist about the gripper axis during the pull (0 = straight pull)
TWIST_FRAC = 0.5                         # the twist happens during the first half of the pull, then pull straight
TWIST_SPEED_DEG = 25.0                   # deg/s cap for the twist (~0.44 rad/s on the wrist, like the 0.5 rad/s joint cap)
RETREAT_BACK = 0.15
APPROACH_PITCHES_DEG = (0, -15, 15, -30, 30)   # tried in order if the horizontal approach has no IK

# ---- speeds and limits ----
JOINT_SPEED = 0.5                        # rad/s cap for joint-space moves
APPROACH_SPEED = 0.04                    # m/s straight approach
PULL_SPEED = 0.02                        # m/s harvesting pull
IK_POS_TOL, IK_ROT_TOL_DEG = 0.004, 3.0
LIMIT_MARGIN = 0.05                      # rad kept away from joint limits
# workspace the planner accepts for the pod (pelvis frame); outside = refuse
REACH_X = (0.35, 0.50)
REACH_Y = (-0.40, 0.05)
REACH_Z = (0.10, 0.40)

# ---- player safety monitors ----
MAX_START_DIFF = 0.15                    # rad: plan's start pose vs the robot's current arm pose
MAX_TILT = 0.05                          # rad (~3 deg) to start
ABORT_TILT_DRIFT = 0.08                  # rad (~5 deg) tilt change aborts
MAX_TRACK_ERR = 0.30                     # rad commanded-vs-measured on any right-arm joint aborts
MAX_PULL_TAU = 12.0                      # Nm on any right-arm joint during the pull aborts (release + retreat)
MAX_CORRECTION = 0.03                    # m: body moved more than this since perception -> abort, look again
CORRECT_HZ = 25.0                        # how often the closed-loop correction is recomputed
MAX_WAIST_DIFF = 0.05                    # rad: waist now vs at perception (the plan assumes the same waist)
STALE_S = 0.4                            # s without rt/lowstate aborts (on the robot: ~1 kHz, gaps up to ~0.18 s seen)
