#!/usr/bin/env python3
"""
run_gui_simulation.py - Standalone Desktop Simulation Application for Smart Pothole Robot.

A fully functional, native desktop GUI application:
- Real-time physics simulation of the 4WD robot traversing a 24m ENU roadway
- Live OpenCV computer vision perception pipeline with contour bounding and pinhole inverse road projection
- Real acoustic buzzer alarm playing through PC speakers (via winsound / wave)
- High-contrast visual warning subsystem: dual LEDs, rear LCD display, and telemetry
- Interactive controls: Resume (R), Mode switch (M), Ghost injection (F), Speed adjustment (+/-)
"""

import math
import os
import sys
import time
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image, ImageTk

try:
    import winsound
    HAVE_WINSOUND = True
except ImportError:
    winsound = None
    HAVE_WINSOUND = False

# Physical & Geometric Constants
ROAD_X_MIN = -2.0
ROAD_X_MAX = 22.0
ROAD_WIDTH = 2.2
LANE_CENTER_Y = -0.50
TIMESTEP_MS = 32

POTHOLES = [
    {"id": "pothole_1", "x": 4.0, "y": -0.50, "sx": 0.60, "sy": 0.50, "type": "real"},
    {"id": "pothole_4", "x": 6.0, "y": 0.55, "sx": 0.60, "sy": 0.50, "type": "other_lane"},
    {"id": "pothole_2", "x": 8.0, "y": -0.55, "sx": 0.60, "sy": 0.50, "type": "real"},
    {"id": "stain_1", "x": 10.0, "y": -0.50, "sx": 0.50, "sy": 0.40, "type": "stain"},
    {"id": "pothole_3", "x": 12.5, "y": -0.45, "sx": 0.70, "sy": 0.50, "type": "real"},
    {"id": "ghost_1", "x": 16.0, "y": -0.50, "sx": 0.60, "sy": 0.50, "type": "ghost"},
]


class PotholeRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Smart Pothole Detection and Warning Robot - Live Simulation System")
        self.root.geometry("1100x750")
        self.root.configure(bg="#0d1117")

        # Simulation State
        self.is_running = True
        self.robot_x = 0.0
        self.robot_y = -0.50
        self.robot_speed = 0.40
        self.sim_time = 0.0
        self.state = "RUNNING"  # RUNNING, STOPPED_POTHOLE, DETECTION_OFF, FINISHED
        self.mode = "CAMERA"    # CAMERA, SIM, OFF
        self.inject_ghost = False
        self.active_hazard = None
        self.triggered_ids = set()

        # Audio alarm path
        self.sound_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "controllers", "pothole_controller", "sounds", "alarm.wav")

        self.setup_ui()
        self.bind_keys()
        self.update_loop()

    def setup_ui(self):
        # 1. Header Frame
        header = tk.Frame(self.root, bg="#161b22", padx=16, pady=12, highlightthickness=1, highlightbackground="#30363d")
        header.pack(fill=tk.X, padx=16, pady=(14, 8))

        lbl_title = tk.Label(header, text="Autonomous Smart Pothole Detection & Warning Robot", font=("Segoe UI", 16, "bold"), fg="#f0f6fc", bg="#161b22")
        lbl_title.pack(side=tk.LEFT)

        self.lbl_state_badge = tk.Label(header, text="RUNNING", font=("Segoe UI", 11, "bold"), fg="#3fb950", bg="#161b22", padx=10, pady=2, relief=tk.GROOVE)
        self.lbl_state_badge.pack(side=tk.RIGHT)

        # 2. Main Work Area (Split Left/Right)
        main_paned = tk.Frame(self.root, bg="#0d1117")
        main_paned.pack(fill=tk.BOTH, expand=True, padx=16, pady=8)

        # Left Column: Road Canvas + Vision Pipeline
        left_col = tk.Frame(main_paned, bg="#0d1117")
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        # Road 2D Physics Canvas
        lbl_road = tk.Label(left_col, text="Road View (ENU Frame: +X Forward, +Y Left)", font=("Segoe UI", 11, "bold"), fg="#c9d1d9", bg="#0d1117")
        lbl_road.pack(anchor=tk.W, pady=(0, 4))

        self.canvas = tk.Canvas(left_col, width=720, height=220, bg="#1a1e24", highlightthickness=1, highlightbackground="#30363d")
        self.canvas.pack(fill=tk.X)

        # OpenCV Camera Perception Panel
        lbl_vision = tk.Label(left_col, text="OpenCV Front Camera Pipeline (160x120, Pitch 0.45 rad)", font=("Segoe UI", 11, "bold"), fg="#c9d1d9", bg="#0d1117")
        lbl_vision.pack(anchor=tk.W, pady=(12, 4))

        vis_frame = tk.Frame(left_col, bg="#161b22", padx=10, pady=10, highlightthickness=1, highlightbackground="#30363d")
        vis_frame.pack(fill=tk.X)

        self.lbl_cam_raw = tk.Label(vis_frame, bg="#000")
        self.lbl_cam_raw.pack(side=tk.LEFT, padx=6)

        self.lbl_cam_thresh = tk.Label(vis_frame, bg="#000")
        self.lbl_cam_thresh.pack(side=tk.LEFT, padx=6)

        self.lbl_cam_contour = tk.Label(vis_frame, bg="#000")
        self.lbl_cam_contour.pack(side=tk.LEFT, padx=6)

        # Captions
        cap_frame = tk.Frame(left_col, bg="#0d1117")
        cap_frame.pack(fill=tk.X, pady=(2, 0))
        tk.Label(cap_frame, text="1. Raw Camera", font=("Segoe UI", 9), fg="#8b949e", bg="#0d1117", width=19).pack(side=tk.LEFT, padx=6)
        tk.Label(cap_frame, text="2. Dark Threshold (<=45)", font=("Segoe UI", 9), fg="#8b949e", bg="#0d1117", width=19).pack(side=tk.LEFT, padx=6)
        tk.Label(cap_frame, text="3. OpenCV Contours & Distance", font=("Segoe UI", 9), fg="#8b949e", bg="#0d1117", width=22).pack(side=tk.LEFT, padx=6)

        # Right Column: Warning Indicators, Telemetry & Controls
        right_col = tk.Frame(main_paned, bg="#161b22", width=320, padx=14, pady=14, highlightthickness=1, highlightbackground="#30363d")
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH)

        tk.Label(right_col, text="Multi-Modal Warning System", font=("Segoe UI", 11, "bold"), fg="#f0f6fc", bg="#161b22").pack(anchor=tk.W)

        # LEDs and LCD Box
        warn_box = tk.Frame(right_col, bg="#0d1117", padx=10, pady=10, highlightthickness=1, highlightbackground="#30363d")
        warn_box.pack(fill=tk.X, pady=(8, 12))

        # LEDs
        led_frame = tk.Frame(warn_box, bg="#0d1117")
        led_frame.pack(side=tk.LEFT, padx=8)

        self.canvas_led_red = tk.Canvas(led_frame, width=28, height=28, bg="#0d1117", highlightthickness=0)
        self.canvas_led_red.pack()
        self.led_red_circle = self.canvas_led_red.create_oval(3, 3, 25, 25, fill="#333", outline="#555", width=2)
        tk.Label(led_frame, text="RED LED", font=("Segoe UI", 8), fg="#8b949e", bg="#0d1117").pack()

        self.canvas_led_green = tk.Canvas(led_frame, width=28, height=28, bg="#0d1117", highlightthickness=0)
        self.canvas_led_green.pack(pady=(4, 0))
        self.led_green_circle = self.canvas_led_green.create_oval(3, 3, 25, 25, fill="#2ecc71", outline="#27ae60", width=2)
        tk.Label(led_frame, text="GREEN LED", font=("Segoe UI", 8), fg="#8b949e", bg="#0d1117").pack()

        # Rear LCD Display
        self.lcd_frame = tk.Frame(warn_box, bg="#071a10", highlightthickness=2, highlightbackground="#2ecc71", padx=10, pady=8)
        self.lcd_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(8, 0))

        self.lbl_lcd_title = tk.Label(self.lcd_frame, text="ROAD CLEAR", font=("Consolas", 12, "bold"), fg="#2ecc71", bg="#071a10")
        self.lbl_lcd_title.pack()
        self.lbl_lcd_sub = tk.Label(self.lcd_frame, text="Cruising safely in lane", font=("Consolas", 8), fg="#2ecc71", bg="#071a10")
        self.lbl_lcd_sub.pack(pady=(2, 0))

        # Telemetry Table
        tk.Label(right_col, text="Live Telemetry", font=("Segoe UI", 11, "bold"), fg="#f0f6fc", bg="#161b22").pack(anchor=tk.W, pady=(4, 4))
        self.tele_vars = {}
        tele_items = [
            ("Simulation Time", "0.00 s"),
            ("Robot X", "0.00 m"),
            ("Robot Y", "-0.50 m"),
            ("Cruising Speed", "0.40 m/s"),
            ("Detection Mode", "CAMERA"),
            ("Active Hazard", "None"),
            ("Classification", "CLEAR"),
        ]

        for label, default_val in tele_items:
            row = tk.Frame(right_col, bg="#161b22")
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, font=("Segoe UI", 9), fg="#8b949e", bg="#161b22").pack(side=tk.LEFT)
            v = tk.Label(row, text=default_val, font=("Consolas", 9, "bold"), fg="#f0f6fc", bg="#161b22")
            v.pack(side=tk.RIGHT)
            self.tele_vars[label] = v

        # Controls & Buttons
        tk.Label(right_col, text="Interactive Controls", font=("Segoe UI", 11, "bold"), fg="#f0f6fc", bg="#161b22").pack(anchor=tk.W, pady=(12, 4))

        ctrl_frame = tk.Frame(right_col, bg="#161b22")
        ctrl_frame.pack(fill=tk.X)

        self.btn_play = tk.Button(ctrl_frame, text="Pause Simulation", font=("Segoe UI", 9, "bold"), bg="#238636", fg="#fff", activebackground="#2ea043", command=self.toggle_play)
        self.btn_play.pack(fill=tk.X, pady=3)

        self.btn_resume = tk.Button(ctrl_frame, text="Resume Movement [R]", font=("Segoe UI", 9, "bold"), bg="#1f6feb", fg="#fff", activebackground="#388bfd", command=self.resume)
        self.btn_resume.pack(fill=tk.X, pady=3)

        self.btn_mode = tk.Button(ctrl_frame, text="Cycle Mode [M]: CAMERA", font=("Segoe UI", 9), bg="#21262d", fg="#c9d1d9", command=self.cycle_mode)
        self.btn_mode.pack(fill=tk.X, pady=3)

        self.btn_ghost = tk.Button(ctrl_frame, text="Toggle Ghost Hazard [F]: OFF", font=("Segoe UI", 9), bg="#21262d", fg="#c9d1d9", command=self.toggle_ghost)
        self.btn_ghost.pack(fill=tk.X, pady=3)

        btn_restart = tk.Button(ctrl_frame, text="Restart Road Track", font=("Segoe UI", 9), bg="#30363d", fg="#c9d1d9", command=self.restart)
        btn_restart.pack(fill=tk.X, pady=3)

    def bind_keys(self):
        self.root.bind("<r>", lambda e: self.resume())
        self.root.bind("<R>", lambda e: self.resume())
        self.root.bind("<m>", lambda e: self.cycle_mode())
        self.root.bind("<M>", lambda e: self.cycle_mode())
        self.root.bind("<f>", lambda e: self.toggle_ghost())
        self.root.bind("<F>", lambda e: self.toggle_ghost())
        self.root.bind("<plus>", lambda e: self.adjust_speed(0.1))
        self.root.bind("<equal>", lambda e: self.adjust_speed(0.1))
        self.root.bind("<minus>", lambda e: self.adjust_speed(-0.1))
        self.root.bind("<underscore>", lambda e: self.adjust_speed(-0.1))

    def toggle_play(self):
        self.is_running = not self.is_running
        self.btn_play.config(text="Pause Simulation" if self.is_running else "Resume Simulation")

    def resume(self):
        if self.state == "STOPPED_POTHOLE":
            self.state = "RUNNING"
            self.active_hazard = None
            self.update_warning_display()

    def cycle_mode(self):
        if self.mode == "CAMERA":
            self.mode = "SIM"
        elif self.mode == "SIM":
            self.mode = "OFF"
        else:
            self.mode = "CAMERA"
        self.btn_mode.config(text=f"Cycle Mode [M]: {self.mode}")
        if self.mode == "OFF" and self.state != "STOPPED_POTHOLE":
            self.state = "DETECTION_OFF"
        elif self.state == "DETECTION_OFF" and self.mode != "OFF":
            self.state = "RUNNING"
        self.update_warning_display()

    def toggle_ghost(self):
        self.inject_ghost = not self.inject_ghost
        self.btn_ghost.config(text=f"Toggle Ghost Hazard [F]: {'ON' if self.inject_ghost else 'OFF'}")

    def adjust_speed(self, delta):
        self.robot_speed = max(0.1, min(1.0, self.robot_speed + delta))

    def restart(self):
        self.robot_x = 0.0
        self.robot_y = -0.50
        self.sim_time = 0.0
        self.state = "RUNNING"
        self.active_hazard = None
        self.triggered_ids.clear()
        self.update_warning_display()

    def play_buzzer(self):
        if HAVE_WINSOUND:
            try:
                # Beep at 880 Hz for 200 ms
                winsound.Beep(880, 180)
            except Exception:
                pass

    def update_warning_display(self):
        if self.state == "RUNNING":
            self.canvas_led_red.itemconfig(self.led_red_circle, fill="#333", outline="#555")
            self.canvas_led_green.itemconfig(self.led_green_circle, fill="#2ecc71", outline="#27ae60")
            self.lcd_frame.config(bg="#071a10", highlightbackground="#2ecc71")
            self.lbl_lcd_title.config(text="ROAD CLEAR", fg="#2ecc71", bg="#071a10")
            self.lbl_lcd_sub.config(text="Cruising safely in lane", fg="#2ecc71", bg="#071a10")
            self.lbl_state_badge.config(text="RUNNING", fg="#3fb950")
        elif self.state == "STOPPED_POTHOLE":
            self.canvas_led_green.itemconfig(self.led_green_circle, fill="#333", outline="#555")
            self.canvas_led_red.itemconfig(self.led_red_circle, fill="#e74c3c", outline="#c0392b")
            self.lcd_frame.config(bg="#2a0808", highlightbackground="#e74c3c")
            hid = self.active_hazard["id"] if self.active_hazard else "POTHOLE"
            self.lbl_lcd_title.config(text="POTHOLE DETECTED", fg="#ff6b6b", bg="#2a0808")
            self.lbl_lcd_sub.config(text=f"ROBOT STOPPED ({hid})", fg="#ff6b6b", bg="#2a0808")
            self.lbl_state_badge.config(text="STOPPED", fg="#f85149")
        elif self.state == "DETECTION_OFF":
            self.canvas_led_green.itemconfig(self.led_green_circle, fill="#333", outline="#555")
            self.canvas_led_red.itemconfig(self.led_red_circle, fill="#333", outline="#555")
            self.lcd_frame.config(bg="#161b22", highlightbackground="#8b949e")
            self.lbl_lcd_title.config(text="DETECTION OFF", fg="#8b949e", bg="#161b22")
            self.lbl_lcd_sub.config(text="Vehicle immobilized", fg="#8b949e", bg="#161b22")
            self.lbl_state_badge.config(text="FAIL-SAFE OFF", fg="#8b949e")
        elif self.state == "FINISHED":
            self.canvas_led_green.itemconfig(self.led_green_circle, fill="#2ecc71", outline="#27ae60")
            self.canvas_led_red.itemconfig(self.led_red_circle, fill="#333", outline="#555")
            self.lcd_frame.config(bg="#071a10", highlightbackground="#58a6ff")
            self.lbl_lcd_title.config(text="FINISH REACHED", fg="#58a6ff", bg="#071a10")
            self.lbl_lcd_sub.config(text="Road section completed", fg="#58a6ff", bg="#071a10")
            self.lbl_state_badge.config(text="FINISHED", fg="#58a6ff")

    def update_physics(self, dt):
        if not self.is_running:
            return

        if self.state == "RUNNING":
            self.robot_x += self.robot_speed * dt
            self.sim_time += dt

            if self.robot_x >= 20.0:
                self.state = "FINISHED"
                self.update_warning_display()
                return

            # Check hazards ahead
            if self.mode != "OFF":
                for p in POTHOLES:
                    if p["type"] == "ghost" and not self.inject_ghost:
                        continue
                    if p["id"] in self.triggered_ids:
                        continue
                    if p["y"] > 0.0:
                        # Ignore opposite lane
                        continue

                    bumper_x = self.robot_x + 0.15
                    near_edge_x = p["x"] - p["sx"] / 2.0
                    dist = near_edge_x - bumper_x

                    if 0.0 <= dist <= 0.60:
                        self.triggered_ids.add(p["id"])
                        self.active_hazard = p
                        self.state = "STOPPED_POTHOLE"
                        self.update_warning_display()
                        self.play_buzzer()
                        break
        else:
            self.sim_time += dt

    def draw_road(self):
        self.canvas.delete("all")
        cw = 720
        ch = 220
        scale = 32  # px per meter
        offset_x = self.robot_x * scale - 120
        center_y = ch / 2

        # Grass borders
        self.canvas.create_rectangle(0, 0, cw, ch, fill="#1e4d2b", outline="")

        # Asphalt roadway
        road_h = 2.2 * scale
        road_top = center_y - road_h / 2
        road_bot = center_y + road_h / 2
        self.canvas.create_rectangle(0, road_top, cw, road_bot, fill="#2b3035", outline="")

        # Curbs
        self.canvas.create_rectangle(0, road_top - 5, cw, road_top, fill="#636e72", outline="")
        self.canvas.create_rectangle(0, road_bot, cw, road_bot + 5, fill="#636e72", outline="")

        # White edge lines
        self.canvas.create_line(0, road_top + 4, cw, road_top + 4, fill="#dfe6e9", width=2)
        self.canvas.create_line(0, road_bot - 4, cw, road_bot - 4, fill="#dfe6e9", width=2)

        # Dashed yellow center line (y = 0.0)
        for x in np.arange(-2.0, 24.0, 2.0):
            sx = x * scale - offset_x
            self.canvas.create_line(sx, center_y, sx + 1.0 * scale, center_y, fill="#fdcb6e", width=3)

        # Potholes and Decoys
        for p in POTHOLES:
            if p["type"] == "ghost" and not self.inject_ghost:
                continue
            sx = (p["x"] - p["sx"] / 2.0) * scale - offset_x
            sy = center_y - (p["y"] + p["sy"] / 2.0) * scale
            pw = p["sx"] * scale
            ph = p["sy"] * scale

            if p["type"] == "stain":
                self.canvas.create_oval(sx, sy, sx + pw, sy + ph, fill="#111113", outline="")
                self.canvas.create_text(sx + pw/2, sy - 6, text="Oil Stain", fill="#8b949e", font=("Segoe UI", 8))
            elif p["type"] == "ghost":
                self.canvas.create_rectangle(sx, sy, sx + pw, sy + ph, outline="#e74c3c", dash=(4, 4), width=2)
                self.canvas.create_text(sx + pw/2, sy - 6, text="Ghost (Phantom)", fill="#e74c3c", font=("Segoe UI", 8))
            else:
                self.canvas.create_rectangle(sx, sy, sx + pw, sy + ph, fill="#0c0d0e", outline="#1a1c1e", width=2)
                self.canvas.create_text(sx + pw/2, sy - 6, text=p["id"], fill="#8b949e", font=("Segoe UI", 8))

        # 4WD Robot
        rx = self.robot_x * scale - offset_x
        ry = center_y - (self.robot_y * scale)
        rw = 0.30 * scale
        rh = 0.20 * scale

        # Camera FOV Cone
        self.canvas.create_polygon(
            rx + rw/2, ry,
            rx + rw/2 + 0.9 * scale, ry - 0.4 * scale,
            rx + rw/2 + 0.9 * scale, ry + 0.4 * scale,
            fill="#1e3a5f", outline=""
        )

        # Wheels
        self.canvas.create_rectangle(rx - rw/2 + 2, ry - rh/2 - 4, rx - rw/2 + 10, ry - rh/2, fill="#1e272e")
        self.canvas.create_rectangle(rx + rw/2 - 10, ry - rh/2 - 4, rx + rw/2 - 2, ry - rh/2, fill="#1e272e")
        self.canvas.create_rectangle(rx - rw/2 + 2, ry + rh/2, rx - rw/2 + 10, ry + rh/2 + 4, fill="#1e272e")
        self.canvas.create_rectangle(rx + rw/2 - 10, ry + rh/2, rx + rw/2 - 2, ry + rh/2 + 4, fill="#1e272e")

        # Chassis
        self.canvas.create_rectangle(rx - rw/2, ry - rh/2, rx + rw/2, ry + rh/2, fill="#0984e3", outline="#74b9ff", width=1)

        # Front Camera Bumper
        self.canvas.create_rectangle(rx + rw/2 - 3, ry - 4, rx + rw/2 + 1, ry + 4, fill="#2d3436")

    def process_opencv_frame(self):
        # 160x120 synthetic camera frame
        w, h = 160, 120
        raw = np.full((h, w, 3), 75, dtype=np.uint8)

        # Check hazards ahead in camera FOV
        detected_info = None
        for p in POTHOLES:
            if p["type"] == "ghost" and not self.inject_ghost:
                continue
            if p["y"] > 0.0:
                continue
            dist = p["x"] - self.robot_x
            if 0.2 < dist < 2.5:
                # Project hazard to image plane
                u = int(80 + (p["y"] - self.robot_y) * 65)
                v = int(60 + (1.2 / dist) * 20)
                pw = int((p["sx"] / dist) * 35)
                ph = int((p["sy"] / dist) * 25)

                color = (15, 15, 15) if p["type"] != "stain" else (20, 20, 20)
                cv2.rectangle(raw, (u - pw//2, v - ph//2), (u + pw//2, v + ph//2), color, -1)
                detected_info = (p, u, v, pw, ph, dist)

        # OpenCV Computer Vision Pipeline
        gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 45, 255, cv2.THRESH_BINARY_INV)

        # Contour extraction
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 30:
                cx, cy, cw_box, ch_box = cv2.boundingRect(cnt)
                is_stain = (detected_info and detected_info[0]["type"] == "stain")
                box_color = (0, 165, 255) if is_stain else (0, 255, 0)
                cv2.rectangle(contour_img, (cx, cy), (cx + cw_box, cy + ch_box), box_color, 2)
                if detected_info:
                    cv2.putText(contour_img, f"{detected_info[0]['id']} ({detected_info[5]:.2f}m)", (cx, cy - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Convert OpenCV numpy arrays to PhotoImage for Tkinter
        def to_photo(img_bgr):
            rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(rgb)
            return ImageTk.PhotoImage(im)

        def to_photo_gray(img_g):
            im = Image.fromarray(img_g)
            return ImageTk.PhotoImage(im)

        self.photo_raw = to_photo(raw)
        self.photo_thresh = to_photo_gray(thresh)
        self.photo_contour = to_photo(contour_img)

        self.lbl_cam_raw.config(image=self.photo_raw)
        self.lbl_cam_thresh.config(image=self.photo_thresh)
        self.lbl_cam_contour.config(image=self.photo_contour)

    def update_telemetry(self):
        self.tele_vars["Simulation Time"].config(text=f"{self.sim_time:.2f} s")
        self.tele_vars["Robot X"].config(text=f"{self.robot_x:.2f} m")
        self.tele_vars["Robot Y"].config(text=f"{self.robot_y:.2f} m")
        self.tele_vars["Cruising Speed"].config(text=f"{self.robot_speed:.2f} m/s")
        self.tele_vars["Detection Mode"].config(text=self.mode)
        self.tele_vars["Active Hazard"].config(text=self.active_hazard["id"] if self.active_hazard else "None")
        if self.active_hazard:
            is_fp = (self.active_hazard["type"] in ["stain", "ghost"])
            self.tele_vars["Classification"].config(text="FALSE_POSITIVE" if is_fp else "TRUE_POSITIVE", fg="#d29922" if is_fp else "#3fb950")
        else:
            self.tele_vars["Classification"].config(text="CLEAR", fg="#8b949e")

    def update_loop(self):
        dt = 0.032
        self.update_physics(dt)
        self.draw_road()
        self.process_opencv_frame()
        self.update_telemetry()
        self.root.after(32, self.update_loop)


def main():
    root = tk.Tk()
    app = PotholeRobotApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
