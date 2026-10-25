"""
movement.py - Locomotion, 4WD motor actuation, and proportional lane-keeping.
"""

import math
from typing import Dict, Optional
from config import (
    WHEEL_RADIUS,
    WHEEL_BASE_WIDTH,
    TARGET_LANE_Y,
    REQUIRED_MOTOR_NAMES,
)


class MovementController:
    def __init__(self, motors: Dict[str, any], base_speed: float = 0.4):
        """
        Initializes the movement subsystem with verified Webots RotationalMotors.
        """
        self.motors = motors
        self.base_speed = base_speed  # linear speed in m/s
        self.kp_y = 1.2               # Proportional gain for lateral lane error
        self.kp_yaw = 0.8             # Proportional gain for yaw heading error
        self.max_angular_vel = 1.5    # rad/s clamp for turning

        # Verify all 4 required motors are present
        for name in REQUIRED_MOTOR_NAMES:
            if name not in self.motors or self.motors[name] is None:
                raise RuntimeError(
                    f"FATAL: Required drive motor '{name}' is missing or uninitialized. "
                    f"Autonomous locomotion cannot proceed."
                )

        # Configure motors for continuous velocity control
        for motor in self.motors.values():
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def set_speed(self, speed: float):
        """Updates the nominal cruising speed."""
        self.base_speed = max(0.05, min(1.2, speed))

    def emergency_brake(self):
        """Immediately zeroes velocity on all 4 motors."""
        for motor in self.motors.values():
            motor.setVelocity(0.0)

    def drive_step(
        self,
        current_y: Optional[float] = None,
        current_yaw: Optional[float] = None,
        target_y: float = TARGET_LANE_Y,
    ) -> float:
        """
        Executes one movement step with proportional lane-keeping.
        If GPS/IMU readings are unavailable, drives straight.
        Returns commanded linear speed.
        """
        steering = 0.0

        if current_y is not None:
            # Lateral error: positive if robot is to the right of lane center (needs to turn left)
            error_y = target_y - current_y
            steering += self.kp_y * error_y

        if current_yaw is not None:
            # Heading error: desired yaw is 0 (facing +X)
            # Normalize yaw to [-pi, pi]
            norm_yaw = (current_yaw + math.pi) % (2 * math.pi) - math.pi
            error_yaw = 0.0 - norm_yaw
            steering += self.kp_yaw * error_yaw

        # Clamp steering angular rate
        steering = max(-self.max_angular_vel, min(self.max_angular_vel, steering))

        # Differential drive kinematic velocity equations:
        # v_left = v - steering * (track_width / 2)
        # v_right = v + steering * (track_width / 2)
        v_left = self.base_speed - (steering * WHEEL_BASE_WIDTH / 2.0)
        v_right = self.base_speed + (steering * WHEEL_BASE_WIDTH / 2.0)

        # Convert linear wheel velocities to rotational motor speeds (omega = v / radius)
        omega_left = v_left / WHEEL_RADIUS
        omega_right = v_right / WHEEL_RADIUS

        # Command motors (FL, RL left side; FR, RR right side)
        self.motors["wheel_fl"].setVelocity(omega_left)
        self.motors["wheel_rl"].setVelocity(omega_left)
        self.motors["wheel_fr"].setVelocity(omega_right)
        self.motors["wheel_rr"].setVelocity(omega_right)

        return self.base_speed
