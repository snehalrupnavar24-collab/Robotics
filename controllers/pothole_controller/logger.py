"""
logger.py - Structured CSV event logging and ground-truth validation.
Records all robotic transitions, hazard detections, and warning activations into results/detection_log.csv.
"""

import csv
import datetime
import os
import uuid
from typing import Optional


LOG_COLUMNS = [
    "run_id",
    "wall_time",
    "sim_time_s",
    "event",
    "mode",
    "pothole_id",
    "est_x",
    "est_y",
    "robot_x",
    "robot_y",
    "robot_yaw",
    "robot_state",
    "distance_m",
    "classification",
    "note",
]


class ExperimentLogger:
    def __init__(self, log_path: Optional[str] = None, run_id: Optional[str] = None):
        if log_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            log_path = os.path.join(base_dir, "results", "detection_log.csv")

        self.log_path = log_path
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.run_id = run_id or str(uuid.uuid4())[:8]

        # Write CSV header if file is empty or does not exist
        if not os.path.exists(self.log_path) or os.path.getsize(self.log_path) == 0:
            with open(self.log_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(LOG_COLUMNS)

    def log_event(
        self,
        sim_time_s: float,
        event: str,
        mode: str,
        pothole_id: str = "NA",
        est_x: Optional[float] = None,
        est_y: Optional[float] = None,
        robot_x: Optional[float] = None,
        robot_y: Optional[float] = None,
        robot_yaw: Optional[float] = None,
        robot_state: str = "RUNNING",
        distance_m: Optional[float] = None,
        classification: str = "NA",
        note: str = "",
    ):
        """Appends a structured event row to detection_log.csv."""
        wall_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        def fmt(val, prec=3):
            return f"{val:.{prec}f}" if val is not None else "NA"

        row = [
            self.run_id,
            wall_time,
            f"{sim_time_s:.3f}",
            event,
            mode,
            pothole_id,
            fmt(est_x),
            fmt(est_y),
            fmt(robot_x),
            fmt(robot_y),
            fmt(robot_yaw),
            robot_state,
            fmt(distance_m),
            classification,
            note,
        ]

        with open(self.log_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(row)
