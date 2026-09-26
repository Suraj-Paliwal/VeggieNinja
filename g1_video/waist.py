"""Point the G1 head camera by turning the waist through rt/arm_sdk.

The G1 has no neck: the head D435i is fixed to the torso, so "look left/down" means
waist yaw/pitch. arm_sdk blends our targets over the built-in controller, which keeps
balancing. Arms are held at the pose they had when control was first engaged.

Nothing is sent until engage() is called; release() ramps the blend weight back to 0.
All goals are RELATIVE to the waist pose at engage() ("home"): center() returns there and
release() only lets go once the waist command is back at home. (Absolute 0 is not safe: this
robot's waist yaw reads about -0.61 rad at rest.)
"""
from __future__ import annotations  # Python 3.8 on the robot

import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14
JOINTS = list(range(12, 29))  # waist 12-14, left arm 15-21, right arm 22-28
WEIGHT_IDX = 29  # motor_cmd[29].q = arm_sdk blend weight
DT = 0.02
KP_WAIST, KD_WAIST = 60.0, 2.0
KP_ARM, KD_ARM = 40.0, 1.5

# Conservative limits (rad, relative to home) and max joint speed (rad/s)
YAW_LIMIT = 0.6     # ~34 deg left/right of home; positive = turn left
PITCH_LIMIT = 0.3   # ~17 deg of home; positive = lean forward / look down
SPEED = 0.4


class WaistController(threading.Thread):
    def __init__(self, speed: float = SPEED) -> None:
        super().__init__(daemon=True)
        self.speed = min(max(float(speed), 0.05), SPEED)  # rad/s, 0.05..SPEED
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.state = None
        ChannelSubscriber("rt/lowstate", LowState_).Init(self._on_state, 10)
        self.lock = threading.Lock()
        self.goal_yaw = self.goal_pitch = 0.0      # offsets from home
        self.home = None                           # (yaw, roll, pitch) at engage()
        self.weight_goal = 0.0
        self.engaged = threading.Event()
        self.released = threading.Event()

    def _on_state(self, msg) -> None:
        self.state = msg

    def ready(self) -> bool:
        return self.state is not None

    def engage(self) -> bool:
        """Start blending in (holds the current pose). Returns False without lowstate."""
        if self.engaged.is_set():
            return True
        if self.state is None:
            return False
        with self.lock:
            ms = self.state.motor_state
            self.home = (ms[WAIST_YAW].q, ms[WAIST_ROLL].q, ms[WAIST_PITCH].q)
            self.goal_yaw = self.goal_pitch = 0.0
            self.weight_goal = 1.0
        self.engaged.set()
        self.start()
        return True

    def nudge(self, d_yaw: float = 0.0, d_pitch: float = 0.0) -> None:
        with self.lock:
            self.goal_yaw = float(np.clip(self.goal_yaw + d_yaw, -YAW_LIMIT, YAW_LIMIT))
            self.goal_pitch = float(np.clip(self.goal_pitch + d_pitch, -PITCH_LIMIT, PITCH_LIMIT))

    def center(self) -> None:
        with self.lock:
            self.goal_yaw = self.goal_pitch = 0.0

    def goal(self) -> tuple[float, float]:
        """Current (yaw, pitch) goal as offsets from home."""
        with self.lock:
            return self.goal_yaw, self.goal_pitch

    def release(self, timeout: float = 8.0) -> None:
        """Center the waist, ramp the weight to 0, and stop sending."""
        if not self.engaged.is_set():
            return
        self.center()
        with self.lock:
            self.weight_goal = 0.0
        self.released.wait(timeout)

    def run(self) -> None:
        s = self.state
        hold = np.array([s.motor_state[j].q for j in JOINTS])  # arms stay here
        home_yaw, home_roll, home_pitch = self.home
        yaw, roll, pitch = hold[0], hold[1], hold[2]
        weight = 0.0
        cmd, crc = unitree_hg_msg_dds__LowCmd_(), CRC()
        step = self.speed * DT
        next_t = time.monotonic()
        while True:
            with self.lock:
                gy, gp, gw = self.goal_yaw, self.goal_pitch, self.weight_goal
            # Only center-then-release: keep weight up until the waist command is back home
            if gw == 0.0 and max(abs(yaw - home_yaw), abs(roll - home_roll), abs(pitch - home_pitch)) > 0.02:
                gw = 1.0
            yaw += float(np.clip(home_yaw + gy - yaw, -step, step))
            pitch += float(np.clip(home_pitch + gp - pitch, -step, step))
            roll += float(np.clip(home_roll - roll, -step, step))
            weight += float(np.clip(gw - weight, -DT / 2.0, DT / 2.0))  # 2 s full ramp

            q = hold.copy()
            q[0], q[1], q[2] = yaw, roll, pitch
            cmd.motor_cmd[WEIGHT_IDX].q = weight
            for i, j in enumerate(JOINTS):
                m = cmd.motor_cmd[j]
                waist = j <= WAIST_PITCH
                m.q, m.dq, m.tau = float(q[i]), 0.0, 0.0
                m.kp, m.kd = (KP_WAIST, KD_WAIST) if waist else (KP_ARM, KD_ARM)
            cmd.crc = crc.Crc(cmd)
            self.pub.Write(cmd)

            if gw == 0.0 and weight <= 0.0:
                self.released.set()
                return
            next_t += DT
            time.sleep(max(0.0, next_t - time.monotonic()))
