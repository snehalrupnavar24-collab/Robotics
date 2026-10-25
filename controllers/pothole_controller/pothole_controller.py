#!/usr/bin/env python3
"""
pothole_controller.py - Main entry point and state machine for Webots Robot.

Architecture:
- State Machine: RUNNING -> STOPPED_POTHOLE -> (resume via 'R' or --auto-resume) -> RUNNING -> FINISHED (at x=20.0m)
  Fail-safe: DETECTION_OFF / DETECTION_UNAVAILABLE stops vehicle unless overridden by --blind.
- Devices:
  - Drive motors: wheel_fl, wheel_fr, wheel_rl, wheel_rr
  - Navigation: gps, imu
  - Perception: camera (OpenCV pinhole projection)
  - Warnings: led_red, led_green, speaker (alarm.wav), lcd (Webots Display API)
  - User Input: Webots Keyboard
"""

import math
import sys
import time
from typing import Optional

# Configuration and Modules
from config import (
    RobotState,
    DetectionMode,
    DEFAULT_TIMESTEP,
    ROAD_FINISH_X,
    TARGET_LANE_Y,
    DEVICE_WHEEL_FL,
    DEVICE_WHEEL_FR,
    DEVICE_WHEEL_RL,
    DEVICE_WHEEL_RR,
    DEVICE_GPS,
    DEVICE_IMU,
    DEVICE_CAMERA,
    DEVICE_LED_RED,
    DEVICE_LED_GREEN,
    DEVICE_SPEAKER,
    DEVICE_DISPLAY,
    parse_controller_arguments,
)
from movement import MovementController
from warning_system import WarningSystem
from detection import SimulationDetector, CameraDetector, DetectionResult
from logger import ExperimentLogger


def run_robot_controller(mock_robot=None, cli_args=None):
    """
    Main controller loop. Accepts mock_robot and cli_args for offline unittest verification.
    """
    args = parse_controller_arguments(cli_args)

    # Initialize Webots Robot
    if mock_robot is not None:
        robot = mock_robot
    else:
        try:
            from controller import Robot
            robot = Robot()
        except ImportError:
            raise RuntimeError(
                "Webots Python controller API ('controller.Robot') is not installed in this environment. "
                "Run through Webots IDE or use tests/mock_webots.py for offline verification."
            )

    # Simulation timestep
    try:
        timestep = int(robot.getBasicTimeStep())
    except Exception:
        timestep = DEFAULT_TIMESTEP

    # 1. Initialize Motors (Fatal if missing)
    motor_names = [DEVICE_WHEEL_FL, DEVICE_WHEEL_FR, DEVICE_WHEEL_RL, DEVICE_WHEEL_RR]
    motors = {}
    for m_name in motor_names:
        dev = robot.getDevice(m_name)
        if dev is None:
            raise RuntimeError(f"FATAL: Motor '{m_name}' not found on robot. Cannot initialize locomotion.")
        motors[m_name] = dev

    movement = MovementController(motors=motors, base_speed=args.speed)

    # 2. Initialize Navigation Sensors (Graceful fallback)
    gps = robot.getDevice(DEVICE_GPS)
    if gps is not None:
        gps.enable(timestep)
    else:
        print("[WARN] GPS sensor not detected. Lane keeping will assume straight trajectory.")

    imu = robot.getDevice(DEVICE_IMU)
    if imu is not None:
        imu.enable(timestep)
    else:
        print("[WARN] IMU sensor not detected. Yaw stabilization will assume 0 heading.")

    # 3. Initialize Camera (Perception)
    camera = robot.getDevice(DEVICE_CAMERA)
    if camera is not None:
        camera.enable(timestep)
    else:
        print("[WARN] Camera device not detected on robot.")

    # 4. Initialize Warning Devices
    led_red = robot.getDevice(DEVICE_LED_RED)
    led_green = robot.getDevice(DEVICE_LED_GREEN)
    speaker = robot.getDevice(DEVICE_SPEAKER)
    display = robot.getDevice(DEVICE_DISPLAY)

    warnings = WarningSystem(
        led_red=led_red,
        led_green=led_green,
        speaker=speaker,
        display=display,
    )

    # 5. Initialize Keyboard
    keyboard = None
    try:
        from controller import Keyboard
        keyboard = robot.getKeyboard()
        keyboard.enable(timestep)
    except Exception:
        try:
            keyboard = robot.getKeyboard()
            keyboard.enable(timestep)
        except Exception:
            pass

    # 6. Initialize Detection Subsystem
    sim_detector = SimulationDetector(
        detect_distance=args.detect_distance,
        inject_false=args.inject_false,
    )
    cam_detector = CameraDetector(
        detect_distance=args.detect_distance,
        dark_threshold=args.dark_threshold,
    )

    curr_mode = DetectionMode(args.mode)
    logger = ExperimentLogger()

    # Controller State Variables
    state = RobotState.RUNNING
    stopped_timestamp_sim = 0.0
    active_detection: Optional[DetectionResult] = None
    last_clear_log_time = -10.0

    print("=" * 60)
    print(" Smart Pothole Detection and Warning Robot - Active")
    print(f" Mode: {curr_mode.value.upper()} | Speed: {args.speed:.2f} m/s | Detect Dist: {args.detect_distance:.2f} m")
    print(" Keys: [R] Resume | [M] Cycle Mode | [F] Toggle Ghost | [+/-] Speed | [H] Help")
    print("=" * 60)

    logger.log_event(
        sim_time_s=robot.getTime(),
        event="RUN_START",
        mode=curr_mode.value,
        robot_state=state.value,
        note=f"Controller started (speed={args.speed}, mode={curr_mode.value})",
    )

    # Main Simulation Step Loop
    while robot.step(timestep) != -1:
        sim_time = robot.getTime()

        # Read GPS Pose
        robot_x, robot_y, robot_z = 0.0, TARGET_LANE_Y, 0.0
        if gps is not None:
            pos = gps.getValues()
            if pos is not None and len(pos) >= 3 and not math.isnan(pos[0]):
                robot_x, robot_y, robot_z = pos[0], pos[1], pos[2]

        # Read IMU Heading (Roll, Pitch, Yaw)
        robot_yaw = 0.0
        if imu is not None:
            rpy = imu.getRollPitchYaw()
            if rpy is not None and len(rpy) >= 3 and not math.isnan(rpy[2]):
                robot_yaw = rpy[2]

        # Process Keyboard Input
        if keyboard is not None:
            key = keyboard.getKey()
            while key > 0:
                key_char = chr(key & 0xFF).upper() if key < 256 else ""

                if key_char == "R":
                    if state == RobotState.STOPPED_POTHOLE:
                        print(f"[{sim_time:.2f}s] Manual RESUME commanded via keyboard 'R'.")
                        logger.log_event(
                            sim_time_s=sim_time,
                            event="RESUME",
                            mode=curr_mode.value,
                            robot_x=robot_x,
                            robot_y=robot_y,
                            robot_yaw=robot_yaw,
                            robot_state="RESUMING",
                            note="User manual resume",
                        )
                        state = RobotState.RUNNING
                        active_detection = None

                elif key_char == "M":
                    # Cycle Mode: sim -> camera -> off -> sim
                    if curr_mode == DetectionMode.SIM:
                        curr_mode = DetectionMode.CAMERA
                    elif curr_mode == DetectionMode.CAMERA:
                        curr_mode = DetectionMode.OFF
                    else:
                        curr_mode = DetectionMode.SIM
                    print(f"[{sim_time:.2f}s] Detection mode switched to: {curr_mode.value.upper()}")

                elif key_char == "F":
                    sim_detector.inject_false = not sim_detector.inject_false
                    status_str = "ENABLED" if sim_detector.inject_false else "DISABLED"
                    print(f"[{sim_time:.2f}s] Ghost pothole injection {status_str}.")

                elif key in [ord("+"), ord("=")]:
                    movement.set_speed(movement.base_speed + 0.1)
                    print(f"[{sim_time:.2f}s] Speed increased to: {movement.base_speed:.2f} m/s")
                    logger.log_event(
                        sim_time_s=sim_time,
                        event="SPEED_CHANGED",
                        mode=curr_mode.value,
                        robot_x=robot_x,
                        robot_y=robot_y,
                        robot_yaw=robot_yaw,
                        robot_state=state.value,
                        note=f"Speed set to {movement.base_speed:.2f}",
                    )

                elif key in [ord("-"), ord("_")]:
                    movement.set_speed(movement.base_speed - 0.1)
                    print(f"[{sim_time:.2f}s] Speed decreased to: {movement.base_speed:.2f} m/s")
                    logger.log_event(
                        sim_time_s=sim_time,
                        event="SPEED_CHANGED",
                        mode=curr_mode.value,
                        robot_x=robot_x,
                        robot_y=robot_y,
                        robot_yaw=robot_yaw,
                        robot_state=state.value,
                        note=f"Speed set to {movement.base_speed:.2f}",
                    )

                elif key_char == "H":
                    print("\n--- CONTROL HELP ---")
                    print("  R   : Resume movement after stopping for pothole")
                    print("  M   : Cycle detection mode (SIM -> CAMERA -> OFF)")
                    print("  F   : Toggle ghost pothole injection")
                    print("  +/- : Increase / decrease cruise speed")
                    print("  H   : Show this help menu\n")

                key = keyboard.getKey()

        # Goal Completion Check
        if robot_x >= ROAD_FINISH_X:
            if state != RobotState.FINISHED:
                movement.emergency_brake()
                warnings.show_finished()
                logger.log_event(
                    sim_time_s=sim_time,
                    event="RUN_FINISHED",
                    mode=curr_mode.value,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    robot_state=RobotState.FINISHED.value,
                    note=f"Destination reached at x={robot_x:.2f}m",
                )
                print(f"[{sim_time:.2f}s] RUN FINISHED: Destination reached at x={robot_x:.2f}m.")
                state = RobotState.FINISHED
            continue

        # Automated Resume Check (used during automated regression tests)
        if state == RobotState.STOPPED_POTHOLE and args.auto_resume is not None:
            if (sim_time - stopped_timestamp_sim) >= args.auto_resume:
                print(f"[{sim_time:.2f}s] Automated RESUME triggered after {args.auto_resume:.1f}s.")
                logger.log_event(
                    sim_time_s=sim_time,
                    event="RESUME",
                    mode=curr_mode.value,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    robot_state="RESUMING",
                    note=f"Auto resume after {args.auto_resume}s",
                )
                state = RobotState.RUNNING
                active_detection = None

        # Mode Availability & Fail-safe Handling
        detection_unavailable = False
        if curr_mode == DetectionMode.OFF:
            detection_unavailable = True
        elif curr_mode == DetectionMode.CAMERA and (not cam_detector.is_available or camera is None):
            detection_unavailable = True

        if detection_unavailable:
            if args.blind:
                # Blind override: robot continues driving, but shows NOT MONITORING
                warnings.show_detection_off(blind_mode=True)
                movement.drive_step(current_y=robot_y, current_yaw=robot_yaw)
                if sim_time - last_clear_log_time >= 2.0:
                    logger.log_event(
                        sim_time_s=sim_time,
                        event="NOT_MONITORING",
                        mode=curr_mode.value,
                        robot_x=robot_x,
                        robot_y=robot_y,
                        robot_yaw=robot_yaw,
                        robot_state="BLIND_DRIVING",
                        note="Detection disabled with --blind override",
                    )
                    last_clear_log_time = sim_time
            else:
                # Fail-safe: stop vehicle immediately, display DETECTION OFF
                movement.emergency_brake()
                warnings.show_detection_off(blind_mode=False)
                if state != RobotState.DETECTION_OFF:
                    logger.log_event(
                        sim_time_s=sim_time,
                        event="DETECTION_UNAVAILABLE",
                        mode=curr_mode.value,
                        robot_x=robot_x,
                        robot_y=robot_y,
                        robot_yaw=robot_yaw,
                        robot_state=RobotState.DETECTION_OFF.value,
                        note="Fail-safe stop: detection disabled or unavailable",
                    )
                    state = RobotState.DETECTION_OFF
            continue
        else:
            if state == RobotState.DETECTION_OFF:
                state = RobotState.RUNNING

        # State: STOPPED_POTHOLE
        if state == RobotState.STOPPED_POTHOLE:
            # Command 0 velocity every single timestep while stopped
            movement.emergency_brake()
            if active_detection is not None:
                warnings.show_pothole_alert(
                    pothole_id=active_detection.pothole_id,
                    distance_m=active_detection.distance_m,
                )
            continue

        # State: RUNNING -> Execute Detection Pipeline
        detection: Optional[DetectionResult] = None

        if curr_mode == DetectionMode.SIM:
            detection = sim_detector.update(
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw,
            )
        elif curr_mode == DetectionMode.CAMERA:
            img = camera.getImage()
            if img:
                detection = cam_detector.process_frame(
                    image_bytes=img,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                )

        if detection is not None:
            # STOP ALL MOTORS FIRST
            movement.emergency_brake()

            # Transition state
            state = RobotState.STOPPED_POTHOLE
            stopped_timestamp_sim = sim_time
            active_detection = detection

            # Activate warning system (LEDs, Speaker alarm, LCD message)
            warnings.show_pothole_alert(
                pothole_id=detection.pothole_id,
                distance_m=detection.distance_m,
            )

            # Log detection event
            logger.log_event(
                sim_time_s=sim_time,
                event="POTHOLE_DETECTED",
                mode=curr_mode.value,
                pothole_id=detection.pothole_id,
                est_x=detection.est_x,
                est_y=detection.est_y,
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw,
                robot_state=state.value,
                distance_m=detection.distance_m,
                classification=detection.classification,
                note=detection.note,
            )
            print(
                f"[{sim_time:.2f}s] ALERT: Pothole '{detection.pothole_id}' detected at "
                f"x={detection.est_x:.2f}m, dist={detection.distance_m:.2f}m! Robot stopped."
            )
        else:
            # Road is clear: Green LED ON, Red LED OFF, LCD "ROAD CLEAR"
            warnings.show_road_clear()
            movement.drive_step(current_y=robot_y, current_yaw=robot_yaw)

            # Periodic clear road status logging
            if sim_time - last_clear_log_time >= 1.5:
                logger.log_event(
                    sim_time_s=sim_time,
                    event="CLEAR_ROAD",
                    mode=curr_mode.value,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    robot_yaw=robot_yaw,
                    robot_state=state.value,
                    note="Road clear, cruising within lane corridor",
                )
                last_clear_log_time = sim_time


if __name__ == "__main__":
    run_robot_controller()
