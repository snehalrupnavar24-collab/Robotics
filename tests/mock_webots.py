"""
tests/mock_webots.py - Comprehensive Offline Webots API Mock and Kinematics Simulator.

Mocks:
- controller.Robot, Motor, PositionSensor, GPS, InertialUnit, Camera, LED, Speaker, Display, Keyboard
- 2D differential drive kinematics adhering to ENU coordinates
- Synthetic pinhole camera image synthesizer rendering road texture, real recessed potholes, and oil stain
"""

import json
import math
import os
import sys
from typing import Dict, List, Optional


class MockMotor:
    def __init__(self, name: str):
        self.name = name
        self.position = 0.0
        self.velocity = 0.0
        self.target_position = float("inf")

    def setPosition(self, pos: float):
        self.target_position = pos

    def setVelocity(self, vel: float):
        self.velocity = vel

    def getVelocity(self) -> float:
        return self.velocity


class MockPositionSensor:
    def __init__(self, name: str):
        self.name = name
        self.value = 0.0
        self.enabled = False

    def enable(self, timestep: int):
        self.enabled = True

    def getValue(self) -> float:
        return self.value


class MockGPS:
    def __init__(self, name: str = "gps"):
        self.name = name
        self.enabled = False
        self.values = [0.0, -0.5, 0.12]

    def enable(self, timestep: int):
        self.enabled = True

    def getValues(self) -> List[float]:
        return list(self.values)


class MockInertialUnit:
    def __init__(self, name: str = "imu"):
        self.name = name
        self.enabled = False
        self.rpy = [0.0, 0.0, 0.0]

    def enable(self, timestep: int):
        self.enabled = True

    def getRollPitchYaw(self) -> List[float]:
        return list(self.rpy)


class MockLED:
    def __init__(self, name: str):
        self.name = name
        self.value = 0

    def set(self, val: int):
        self.value = val

    def get(self) -> int:
        return self.value


class MockSpeaker:
    def __init__(self, name: str = "speaker"):
        self.name = name
        self.last_played = None

    def playSound(self, *args, **kwargs):
        # Accepts (speaker, file, ...) or (file, volume)
        for a in args:
            if isinstance(a, str):
                self.last_played = a
                break
        return 0


class MockDisplay:
    def __init__(self, name: str = "lcd", width: int = 256, height: int = 128):
        self.name = name
        self.width = width
        self.height = height
        self.current_color = 0
        self.draw_history = []

    def setFont(self, font: str, size: int, anti_aliased: bool):
        pass

    def setColor(self, color: int):
        self.current_color = color

    def fillRectangle(self, x: int, y: int, width: int, height: int):
        self.draw_history.append(("rect", x, y, width, height, self.current_color))

    def drawText(self, text: str, x: int, y: int):
        self.draw_history.append(("text", x, y, text, self.current_color))


class MockKeyboard:
    def __init__(self):
        self.enabled = False
        self.key_queue = []

    def enable(self, timestep: int):
        self.enabled = True

    def queueKey(self, key_code: int):
        self.key_queue.append(key_code)

    def getKey(self) -> int:
        if self.key_queue:
            return self.key_queue.pop(0)
        return -1


class MockCamera:
    """
    Synthetic camera generator using pinhole camera projection model.
    Produces raw BGRA byte buffer (160x120x4).
    """
    def __init__(self, name: str = "camera", width: int = 160, height: int = 120, fov: float = 1.0):
        self.name = name
        self.width = width
        self.height = height
        self.fov = fov
        self.enabled = False
        self.cam_height = 0.12  # mounting height above road
        self.cam_pitch = 0.45   # rad pitch down
        self.cam_x_offset = 0.15 # front bumper offset
        self.focal_px = (self.width / 2.0) / math.tan(self.fov / 2.0)
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Load road obstacles & decoys from pothole_map.json
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        map_path = os.path.join(base_dir, "assets", "pothole_map.json")
        self.features = []
        if os.path.exists(map_path):
            with open(map_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                self.features = data.get("potholes", []) + data.get("decoys", [])

        self.last_image_bytes = None

    def enable(self, timestep: int):
        self.enabled = True

    def render_frame(self, robot_x: float, robot_y: float, robot_yaw: float = 0.0) -> bytes:
        """
        Synthesizes a 160x120 BGRA image of the road ahead.
        Road asphalt = Gray (80, 80, 80, 255)
        Potholes / Oil Stains = Dark (15, 15, 15, 255)
        """
        try:
            import numpy as np
        except ImportError:
            # Fallback if numpy is absent: generate empty uniform buffer
            return bytes([80, 80, 80, 255] * (self.width * self.height))

        # Initialize image with asphalt color (BGRA)
        img = np.full((self.height, self.width, 4), [80, 80, 80, 255], dtype=np.uint8)

        # For efficient rasterization, sample grid or project features to image coordinates
        # Reverse projection: for each dark feature in ground truth ahead, project its bounding box to pixels
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        for feat in self.features:
            fx = feat["x"]
            fy = feat["y"]
            fsx = feat.get("length_x", 0.6)
            fsy = feat.get("width_y", 0.5)

            # Check if feature is in front of robot
            dx = fx - robot_x
            dy = fy - robot_y
            # Transform to robot frame
            x_rob = dx * cos_yaw + dy * sin_yaw
            y_rob = -dx * sin_yaw + dy * cos_yaw

            # Feature forward distance from camera
            x_cam = x_rob - self.cam_x_offset
            if x_cam <= 0.1 or x_cam > 6.0:
                continue

            # Feature bounds in robot frame
            x_min = x_cam - fsx / 2.0
            x_max = x_cam + fsx / 2.0
            y_min = y_rob - fsy / 2.0
            y_max = y_rob + fsy / 2.0

            if x_min <= 0.1:
                x_min = 0.1

            # Project 4 corner points to image coordinates:
            # ray_pitch = cam_pitch + atan(tan_theta_v)
            # x_cam = cam_height / tan(ray_pitch)
            # => tan(ray_pitch) = cam_height / x_cam
            # => ray_pitch = atan(cam_height / x_cam)
            # => atan(tan_theta_v) = ray_pitch - cam_pitch
            # => tan_theta_v = tan(ray_pitch - cam_pitch)
            # => v = cy + focal_px * tan_theta_v
            # y_rob = -x_cam * tan_theta_h => tan_theta_h = -y_rob / x_cam
            # => u = cx + focal_px * (-y_rob / x_cam)

            def proj_pt(gx, gy):
                pitch = math.atan2(self.cam_height, gx)
                d_pitch = pitch - self.cam_pitch
                tan_v = math.tan(d_pitch)
                v = self.cy + self.focal_px * tan_v
                tan_h = -gy / gx
                u = self.cx + self.focal_px * tan_h
                return int(round(u)), int(round(v))

            u1, v1 = proj_pt(x_max, y_min)
            u2, v2 = proj_pt(x_min, y_max)

            umin, umax = max(0, min(u1, u2)), min(self.width, max(u1, u2))
            vmin, vmax = max(0, min(v1, v2)), min(self.height, max(v1, v2))

            if umax > umin and vmax > vmin:
                # Fill dark depression
                dark_color = [15, 15, 15, 255] if feat.get("type") != "oil_stain" else [20, 20, 20, 255]
                img[vmin:vmax, umin:umax] = dark_color

        self.last_image_bytes = img.tobytes()
        return self.last_image_bytes

    def getImage(self) -> Optional[bytes]:
        return self.last_image_bytes


class MockRobot:
    """
    Simulates a Webots Robot with physical differential kinematics in ENU coordinate system.
    """
    def __init__(self, initial_x: float = 0.0, initial_y: float = -0.5, initial_yaw: float = 0.0):
        self.sim_time = 0.0
        self.basic_time_step = 32
        self.step_count = 0
        self.max_steps = 1500  # Safety execution limit

        # Kinematic state
        self.x = initial_x
        self.y = initial_y
        self.z = 0.12
        self.yaw = initial_yaw
        self.wheel_radius = 0.06
        self.track_width = 0.22

        # Device registry
        self.motors: Dict[str, MockMotor] = {
            "wheel_fl": MockMotor("wheel_fl"),
            "wheel_fr": MockMotor("wheel_fr"),
            "wheel_rl": MockMotor("wheel_rl"),
            "wheel_rr": MockMotor("wheel_rr"),
        }
        self.gps = MockGPS("gps")
        self.imu = MockInertialUnit("imu")
        self.camera = MockCamera("camera")
        self.led_red = MockLED("led_red")
        self.led_green = MockLED("led_green")
        self.speaker = MockSpeaker("speaker")
        self.display = MockDisplay("lcd")
        self.keyboard = MockKeyboard()

        self._update_sensors()

    def getBasicTimeStep(self) -> int:
        return self.basic_time_step

    def getTime(self) -> float:
        return self.sim_time

    def getDevice(self, name: str):
        if name in self.motors:
            return self.motors[name]
        devs = {
            "gps": self.gps,
            "imu": self.imu,
            "camera": self.camera,
            "led_red": self.led_red,
            "led_green": self.led_green,
            "speaker": self.speaker,
            "lcd": self.display,
        }
        return devs.get(name)

    def getKeyboard(self) -> MockKeyboard:
        return self.keyboard

    def _update_sensors(self):
        if self.gps is not None:
            self.gps.values = [self.x, self.y, self.z]
        if self.imu is not None:
            self.imu.rpy = [0.0, 0.0, self.yaw]
        if self.camera is not None and self.camera.enabled:
            self.camera.render_frame(self.x, self.y, self.yaw)

    def step(self, timestep: int) -> int:
        if self.step_count >= self.max_steps:
            return -1

        dt = timestep / 1000.0
        self.sim_time += dt
        self.step_count += 1

        # Calculate differential kinematics from motor rotational velocities
        omega_l = (self.motors["wheel_fl"].velocity + self.motors["wheel_rl"].velocity) / 2.0
        omega_r = (self.motors["wheel_fr"].velocity + self.motors["wheel_rr"].velocity) / 2.0

        v_l = omega_l * self.wheel_radius
        v_r = omega_r * self.wheel_radius

        linear_v = (v_l + v_r) / 2.0
        angular_w = (v_r - v_l) / self.track_width

        # Integrate pose in ENU (+x forward, +y left, +z up)
        self.x += linear_v * math.cos(self.yaw) * dt
        self.y += linear_v * math.sin(self.yaw) * dt
        self.yaw += angular_w * dt

        self._update_sensors()
        return 0
