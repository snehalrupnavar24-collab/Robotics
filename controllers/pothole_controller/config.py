"""
Configuration and parameter definitions for the Smart Pothole Detection Robot.
Centralizes device names, physical constraints, thresholds, and CLI parsing.
"""

import argparse
import sys
from enum import Enum


class RobotState(Enum):
    RUNNING = "RUNNING"
    STOPPED_POTHOLE = "STOPPED_POTHOLE"
    DETECTION_OFF = "DETECTION_OFF"
    FINISHED = "FINISHED"


class DetectionMode(Enum):
    SIM = "sim"
    CAMERA = "camera"
    OFF = "off"


# Device names (must match names defined in worlds/pothole_road.wbt)
DEVICE_WHEEL_FL = "wheel_fl"
DEVICE_WHEEL_FR = "wheel_fr"
DEVICE_WHEEL_RL = "wheel_rl"
DEVICE_WHEEL_RR = "wheel_rr"

DEVICE_WHEEL_FL_SENSOR = "wheel_fl_sensor"
DEVICE_WHEEL_FR_SENSOR = "wheel_fr_sensor"
DEVICE_WHEEL_RL_SENSOR = "wheel_rl_sensor"
DEVICE_WHEEL_RR_SENSOR = "wheel_rr_sensor"

DEVICE_GPS = "gps"
DEVICE_IMU = "imu"
DEVICE_CAMERA = "camera"
DEVICE_LED_RED = "led_red"
DEVICE_LED_GREEN = "led_green"
DEVICE_SPEAKER = "speaker"
DEVICE_DISPLAY = "lcd"

REQUIRED_MOTOR_NAMES = [
    DEVICE_WHEEL_FL,
    DEVICE_WHEEL_FR,
    DEVICE_WHEEL_RL,
    DEVICE_WHEEL_RR,
]

OPTIONAL_SENSOR_NAMES = [
    DEVICE_GPS,
    DEVICE_IMU,
    DEVICE_CAMERA,
    DEVICE_LED_RED,
    DEVICE_LED_GREEN,
    DEVICE_SPEAKER,
    DEVICE_DISPLAY,
]

ALL_DEVICE_NAMES = REQUIRED_MOTOR_NAMES + OPTIONAL_SENSOR_NAMES

# Physical robot & road constants
WHEEL_RADIUS = 0.06           # meters
WHEEL_BASE_WIDTH = 0.22       # track width between left and right wheels
DEFAULT_TIMESTEP = 32         # ms

# Lane & corridor geometry
TARGET_LANE_Y = -0.50         # robot nominal lane center in ENU
LANE_CORRIDOR_MIN_Y = -1.05   # right boundary of robot lane
LANE_CORRIDOR_MAX_Y = -0.05   # left boundary (near center dividing line)
ROAD_FINISH_X = 20.0          # x coordinate to conclude autonomous run

# Detection defaults
DEFAULT_SPEED = 0.4           # m/s
DEFAULT_DETECT_DIST = 0.6     # meters ahead of robot bumper
DEFAULT_DARK_THRESHOLD = 45   # 0-255 grayscale threshold for OpenCV camera
PERSISTENCE_FRAMES = 3        # consecutive frames required in camera mode

# LCD configuration
LCD_WIDTH = 256
LCD_HEIGHT = 128


def parse_controller_arguments(arg_list=None):
    """
    Parses controller arguments passed via controllerArgs in Webots or CLI.
    """
    parser = argparse.ArgumentParser(
        description="Smart Pothole Detection and Warning Robot Controller"
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="sim",
        choices=["sim", "camera", "off"],
        help="Detection mode: sim (ground truth), camera (OpenCV vision), or off (fail-safe)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED,
        help="Nominal cruising speed in m/s (default: 0.4)",
    )
    parser.add_argument(
        "--detect-distance",
        type=float,
        default=DEFAULT_DETECT_DIST,
        dest="detect_distance",
        help="Detection threshold distance in meters (default: 0.6)",
    )
    parser.add_argument(
        "--inject-false",
        action="store_true",
        dest="inject_false",
        help="Inject phantom ghost pothole at x=16.0m for false-positive validation",
    )
    parser.add_argument(
        "--blind",
        action="store_true",
        help="Override fail-safe stop when detection is off or unavailable; drives without monitoring",
    )
    parser.add_argument(
        "--auto-resume",
        type=float,
        default=None,
        dest="auto_resume",
        help="Automatically resume after SECONDS in stopped state (used for automated test passes)",
    )
    parser.add_argument(
        "--dark-threshold",
        type=int,
        default=DEFAULT_DARK_THRESHOLD,
        dest="dark_threshold",
        help="Grayscale threshold for OpenCV dark pothole detection (default: 45)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress repetitive per-timestep debug output",
    )

    if arg_list is None:
        arg_list = sys.argv[1:]

    return parser.parse_args(arg_list)
