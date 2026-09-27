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

# ---- gravity feed-forward for arm_sdk (waist + right arm). Without it the PD springs sag under the arm's and
# torso's weight: measured 2026-09-27 on the robot: shoulder pitch 7.9 deg, waist pitch 6.4 deg, gripper 9.5 cm off.
# tau_ff = GRAVITY_FF_GAIN * scale[joint] * model gravity torque (pinocchio, g1.urdf); the per-joint scales were fitted
# to the motor torques measured during that reach (the model has the rubber hand, the robot a heavier Dex1) and
# checked on held-out samples (0.2-0.4 N m). 0 = off.
GRAVITY_FF_GAIN = 1.0
GRAVITY_SCALE = {"waist_yaw_joint": 1.0, "waist_roll_joint": 0.0, "waist_pitch_joint": 0.98, "right_shoulder_pitch_joint": 1.77, "right_shoulder_roll_joint": 1.27, "right_shoulder_yaw_joint": 2.34, "right_elbow_joint": 1.24, "right_wrist_roll_joint": 1.0, "right_wrist_pitch_joint": 2.93, "right_wrist_yaw_joint": 1.0}
GRAVITY_TAU_MAX = 12.0                   # N m, clip per joint

# ---- head camera mount: how the real D435 differs from the URDF (rotation vector in d435_link, degrees) ----
# Measured with `okra_pick.sh floor` (robot standing, feet flat): it prints the value to put here; re-run until the
# floor tilt is < 1 deg. A turn of the camera about the vertical is not observable from a floor (not included).
CAM_CORR_ROTVEC_DEG = (-0.75, 3.90, -0.83)   # floor check 2026-09-27 on the robot: 2x10 frames, sd < 0.06 deg
CLOSE_AXIS = "y"                         # wrist axis the jaws close along. VERIFIED 2026-09-27 (jaw test at the grasp
                                         # pose, head camera: fingers upright, closing left-right as planned)

# ---- grasp geometry (pelvis frame, metres) ----
PREGRASP_BACK = 0.08                     # start of the straight approach, behind the pod (0.12 shrinks the reach, see Guide)
GRASP_DEPTH = 0.01                       # go this far past the pod centre so it sits deep in the jaws
# Harvest pull after closing. 2026-09-27 (operator): pull straight UP 5 cm, no twist: simpler and it keeps the most
# wrist room (vertical pod: 38 deg on the approach). Detaches a pod whose stem is BELOW it (tip up, as okra grows).
# Previous default (pod hanging from its stem): PULL_BACK, PULL_DOWN = 0.06, 0.02 and PULL_TWIST_DEG = 45.0.
PULL_BACK, PULL_DOWN = 0.0, -0.05        # m; PULL_DOWN < 0 = upwards
PULL_TWIST_DEG = 0.0                     # wrist twist about the gripper axis during the pull (0 = straight pull)
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
# Comfortable reach box: green in the question image, preferred target. NOT a refusal any more (2026-09-27, the
# operator wants it to try): the planner's IK + joint-limit margin + torso check decide. PLAN_SANITY only rejects
# absurd positions (bad depth).
PLAN_SANITY = ((0.20, 0.70), (-0.60, 0.40), (-0.20, 0.55))
REACH_X = (0.35, 0.50)
REACH_Y = (-0.40, 0.05)
REACH_Z = (0.10, 0.40)
# Comfortable sub-zone for placing a pod (pelvis frame, pod centre): every corner planned with >= 0.11 rad
# joint-limit margin on 2026-09-27 (the reach box edges can be as low as 0.07 or fail). tools/place_zone.py
# draws it on the live feed.
PLACE_ZONE = ((0.39, 0.45), (-0.16, -0.03), (0.11, 0.19))

# ---- player safety monitors ----
MAX_START_DIFF = 0.15                    # rad: plan's start pose vs the robot's current arm pose
MAX_TILT = 0.05                          # rad (~3 deg) to start
ABORT_TILT_DRIFT = 0.08                  # rad (~5 deg) tilt change aborts
MAX_TRACK_ERR = 0.30                     # rad commanded-vs-measured on any right-arm joint aborts
MAX_PULL_TAU = 12.0                      # Nm on any right-arm joint during the pull aborts (release + retreat)
MAX_PULL_EXTRA_TAU = 6.0                 # Nm beyond the planned gravity torque (plans with tau_ff): same margin as
                                         # before gravity feed-forward (then ~6 Nm of the 12 was the arm's weight)
MAX_CORRECTION = 0.03                    # m: body moved more than this since perception -> abort, look again
CORRECT_HZ = 25.0                        # how often the closed-loop correction is recomputed
MAX_WAIST_DIFF = 0.05                    # rad: waist now vs at perception (the plan assumes the same waist)
MAX_EVENT_AGE_S = 300.0                  # s since perception, robot clock only (safety gate + player preflight)
STALE_S = 0.4                            # s without rt/lowstate aborts (on the robot: ~1 kHz, gaps up to ~0.18 s seen)
