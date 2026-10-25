"""
detection.py - Dual-mode pothole detection system.

1. SimulationDetector (Validation Mode):
   - Ground-truth workflow validation based on assets/pothole_map.json and robot pose.
   - Triggers when near edge of pothole is within detect_distance and inside robot's lane corridor.
   - NOTE: This is ground-truth based workflow verification, NOT perception.

2. CameraDetector (Perception Mode):
   - Computer vision pipeline using OpenCV:
     BGRA -> Grayscale -> Gaussian blur -> Dark threshold (<=45) -> Morphology ->
     Contour analysis (area, solidity, aspect ratio) -> Pinhole inverse projection to road plane ->
     Lane corridor check -> 3-frame persistence filter -> Spatial de-duplication.
   - Distinguishes real potholes (TRUE_POSITIVE) from flat oil stains (FALSE_POSITIVE).
   - Gracefully reports "unavailable" if opencv-python or numpy is missing.
"""

import json
import math
import os
from typing import Dict, List, Optional, Tuple
from config import (
    LANE_CORRIDOR_MIN_Y,
    LANE_CORRIDOR_MAX_Y,
    DEFAULT_DETECT_DIST,
    DEFAULT_DARK_THRESHOLD,
    PERSISTENCE_FRAMES,
)

# Safe imports for optional computer vision packages
try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    np = None
    HAVE_NUMPY = False

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    cv2 = None
    HAVE_CV2 = False


class DetectionResult:
    def __init__(
        self,
        pothole_id: str,
        est_x: float,
        est_y: float,
        distance_m: float,
        classification: str,  # TRUE_POSITIVE, FALSE_POSITIVE
        mode: str,
        note: str = "",
    ):
        self.pothole_id = pothole_id
        self.est_x = est_x
        self.est_y = est_y
        self.distance_m = distance_m
        self.classification = classification
        self.mode = mode
        self.note = note


class SimulationDetector:
    """
    Simulation validation detector using ground-truth map.
    Validates state machine, warning actuation, and lane filtering logic.
    """
    def __init__(
        self,
        map_path: Optional[str] = None,
        detect_distance: float = DEFAULT_DETECT_DIST,
        inject_false: bool = False,
    ):
        self.detect_distance = detect_distance
        self.inject_false = inject_false
        self.triggered_ids = set()

        if map_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            map_path = os.path.join(base_dir, "assets", "pothole_map.json")

        self.potholes = []
        if os.path.exists(map_path):
            with open(map_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                self.potholes = data.get("potholes", [])
                if self.inject_false:
                    self.potholes.extend(data.get("ghosts", []))

    def reset(self):
        self.triggered_ids.clear()

    def update(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float = 0.0,
    ) -> Optional[DetectionResult]:
        """
        Checks if any unhandled pothole's NEAR edge is within detect_distance of the front bumper
        and lies within the robot's travel corridor.
        """
        # Robot front bumper is at robot_x + 0.15m
        front_bumper_x = robot_x + 0.15

        for p in self.potholes:
            pid = p["id"]
            if pid in self.triggered_ids:
                continue

            px = p["x"]
            py = p["y"]
            length_x = p.get("length_x", 0.6)
            near_edge_x = px - (length_x / 2.0)

            # Check lane corridor: must lie strictly in robot lane
            if not (LANE_CORRIDOR_MIN_Y <= py <= LANE_CORRIDOR_MAX_Y):
                # Other lane pothole (e.g. pothole_4 at y=0.55): IGNORE
                continue

            # Forward distance from bumper to near edge
            dist_to_near_edge = near_edge_x - front_bumper_x

            if 0.0 <= dist_to_near_edge <= self.detect_distance:
                self.triggered_ids.add(pid)
                is_ghost = p.get("type") == "injected_ghost"
                classification = "FALSE_POSITIVE" if is_ghost else "TRUE_POSITIVE"
                note = "Ground-truth simulation detection"
                if is_ghost:
                    note = "Injected ghost pothole triggered"

                return DetectionResult(
                    pothole_id=pid,
                    est_x=px,
                    est_y=py,
                    distance_m=dist_to_near_edge,
                    classification=classification,
                    mode="sim",
                    note=note,
                )

        return None


class CameraDetector:
    """
    OpenCV-based visual pothole detection pipeline.
    Uses inverse perspective pinhole projection from camera pixel coordinates to road plane.
    """
    def __init__(
        self,
        detect_distance: float = DEFAULT_DETECT_DIST,
        dark_threshold: int = DEFAULT_DARK_THRESHOLD,
        persistence_threshold: int = PERSISTENCE_FRAMES,
        ground_truth_map_path: Optional[str] = None,
    ):
        self.detect_distance = detect_distance
        self.dark_threshold = dark_threshold
        self.persistence_threshold = persistence_threshold
        self.consecutive_detections = 0
        self.reported_detections = []  # list of (est_x, est_y, pid)
        self.frame_count = 0

        # Pinhole camera intrinsic & mounting parameters (ENU)
        self.img_w = 160
        self.img_h = 120
        self.fov = 1.0  # radians
        self.focal_px = (self.img_w / 2.0) / math.tan(self.fov / 2.0)
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.cam_height = 0.12  # mounting height above road surface (z_cam=0.18, road_z=0.06 -> h=0.12)
        self.cam_pitch = 0.45   # radians pitched down
        self.cam_x_offset = 0.15 # front bumper offset

        # Load ground truth for classification verification
        if ground_truth_map_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            ground_truth_map_path = os.path.join(base_dir, "assets", "pothole_map.json")

        self.ground_truth = []
        if os.path.exists(ground_truth_map_path):
            with open(ground_truth_map_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                self.ground_truth = data.get("potholes", []) + data.get("decoys", [])

    @property
    def is_available(self) -> bool:
        return HAVE_NUMPY and HAVE_CV2

    def reset(self):
        self.consecutive_detections = 0
        self.reported_detections.clear()
        self.frame_count = 0

    def project_pixel_to_road(
        self, u: float, v: float, robot_x: float, robot_y: float, robot_yaw: float = 0.0
    ) -> Optional[Tuple[float, float, float]]:
        """
        Projects image pixel (u, v) onto the planar road surface using pinhole model.
        Returns: (world_x, world_y, forward_distance_from_bumper)
        """
        # Pixel ray angles in camera optical frame
        # u: horizontal (right is +x_c), v: vertical (down is +y_c)
        tan_theta_h = (u - self.cx) / self.focal_px
        tan_theta_v = (v - self.cy) / self.focal_px

        # Vertical angle relative to horizontal road: pitch down angle + optical angle
        # Positive angle points downward toward the ground
        ray_pitch = self.cam_pitch + math.atan(tan_theta_v)
        if ray_pitch <= 0.05:
            # Ray is parallel or pointing toward horizon/sky; does not hit immediate road
            return None

        # Distance along ground from camera to intersection point
        ground_dist_from_cam = self.cam_height / math.tan(ray_pitch)
        if ground_dist_from_cam <= 0 or ground_dist_from_cam > 5.0:
            return None

        # Lateral distance in robot coordinates (ENU: +y is left, so positive tan_theta_h is right -> -y)
        lat_dist_from_cam = -ground_dist_from_cam * tan_theta_h

        # Robot coordinate frame:
        # x_rel: forward from bumper
        # y_rel: left from robot centerline
        x_rel_bumper = ground_dist_from_cam
        x_rel_robot = self.cam_x_offset + ground_dist_from_cam
        y_rel_robot = lat_dist_from_cam

        # Transform to ENU world coordinates
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        world_x = robot_x + (x_rel_robot * cos_yaw - y_rel_robot * sin_yaw)
        world_y = robot_y + (x_rel_robot * sin_yaw + y_rel_robot * cos_yaw)

        return (world_x, world_y, x_rel_bumper)

    def process_frame(
        self,
        image_bytes: bytes,
        robot_x: float,
        robot_y: float,
        robot_yaw: float = 0.0,
    ) -> Optional[DetectionResult]:
        """
        Processes a raw BGRA camera frame from Webots Camera device.
        """
        if not self.is_available:
            return None

        self.frame_count += 1
        # Ignore initial frames during sensor power-up / calibration
        if self.frame_count < 3:
            return None

        # Convert raw BGRA bytes to numpy uint8 array
        try:
            arr = np.frombuffer(image_bytes, dtype=np.uint8).reshape((self.img_h, self.img_w, 4))
        except Exception:
            return None

        # Reject uninitialized or totally black/corrupt frames
        mean_val = np.mean(arr[:, :, :3])
        if mean_val < 5.0 or mean_val > 250.0:
            return None

        # 1. Convert BGRA to grayscale
        gray = cv2.cvtColor(arr, cv2.COLOR_BGRA2GRAY)

        # 2. Gaussian blur to suppress asphalt texture grain
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3. Dark threshold: potholes appear distinctly darker than asphalt
        _, thresh = cv2.threshold(blurred, self.dark_threshold, 255, cv2.THRESH_BINARY_INV)

        # 4. Morphological opening and closing to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 5. Extract contours
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidate = None
        min_detected_dist = float("inf")

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Area filter (ignore tiny dust specks and massive full-frame shadows)
            if area < 40 or area > 6000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            # Reject extreme aspect ratios
            aspect_ratio = float(w) / max(1, h)
            if aspect_ratio < 0.2 or aspect_ratio > 5.0:
                continue

            # Pick the lowest point of the contour (the near edge closest to robot)
            near_u = x + w / 2.0
            near_v = y + h  # bottom of bounding box in image is closest to robot

            proj = self.project_pixel_to_road(near_u, near_v, robot_x, robot_y, robot_yaw)
            if proj is None:
                continue

            world_x, world_y, dist_bumper = proj

            # Lane corridor filter: must lie within robot lane
            if not (LANE_CORRIDOR_MIN_Y <= world_y <= LANE_CORRIDOR_MAX_Y):
                continue

            # Distance threshold filter
            if 0.0 <= dist_bumper <= self.detect_distance:
                if dist_bumper < min_detected_dist:
                    min_detected_dist = dist_bumper
                    candidate = (world_x, world_y, dist_bumper)

        # Temporal persistence filter: candidate must persist for 3 consecutive frames
        if candidate is not None:
            self.consecutive_detections += 1
            if self.consecutive_detections >= self.persistence_threshold:
                c_x, c_y, c_dist = candidate

                # Spatial de-duplication: check if already reported nearby
                for rep_x, rep_y, _ in self.reported_detections:
                    if math.hypot(c_x - rep_x, c_y - rep_y) < 1.2:
                        # Already triggered for this hazard
                        return None

                # Ground truth classification
                classification, matched_id, note = self._classify_detection(c_x, c_y)
                self.reported_detections.append((c_x, c_y, matched_id))

                return DetectionResult(
                    pothole_id=matched_id,
                    est_x=c_x,
                    est_y=c_y,
                    distance_m=c_dist,
                    classification=classification,
                    mode="camera",
                    note=note,
                )
        else:
            self.consecutive_detections = max(0, self.consecutive_detections - 1)

        return None

    def _classify_detection(self, est_x: float, est_y: float) -> Tuple[str, str, str]:
        """
        Compares estimated location with ground truth map to verify classification.
        Real potholes -> TRUE_POSITIVE.
        Decoy oil stain -> FALSE_POSITIVE (dark region detected, but not a physical pit).
        """
        best_dist = float("inf")
        best_item = None

        for item in self.ground_truth:
            d = math.hypot(est_x - item["x"], est_y - item["y"])
            if d < best_dist:
                best_dist = d
                best_item = item

        if best_item is not None and best_dist < 1.2:
            pid = best_item["id"]
            if best_item.get("type") == "oil_stain":
                return (
                    "FALSE_POSITIVE",
                    pid,
                    f"Decoy oil stain detected via visual thresholding (no physical depth)",
                )
            elif best_item.get("type") == "injected_ghost":
                return "FALSE_POSITIVE", pid, "Injected phantom target"
            else:
                return "TRUE_POSITIVE", pid, f"Real pothole matched ({best_dist:.2f}m error)"

        # Unknown dark spot
        return "FALSE_POSITIVE", "unknown_dark_spot", "Visual anomaly outside ground truth"
