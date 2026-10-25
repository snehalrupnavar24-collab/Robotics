# Smart Pothole Detection and Warning Robot (Webots)

An academic, software-only autonomous robotics simulation project for **Webots** using the official Python Controller API. The system autonomously navigates a two-lane roadway in ENU coordinates, detects physical road recesses using dual detection pipelines (Ground-Truth Simulation and OpenCV Computer Vision), triggers multi-modal warning signals (dual LEDs, acoustic buzzer, and rear LCD texture), safely immobilizes the vehicle before hazards, and supports interactive keyboard and automated resumption.

---

## 1. System Architecture

```
+-----------------------------------------------------------------------------------------------+
|                                        WEBOTS WORLD                                           |
|  - Coordinate Frame: ENU (+X Forward, +Y Left, +Z Up)                                         |
|  - Roadway: Length 24.0 m (X: -2.0m to 22.0m), Width 2.2 m (Y: -1.1m to 1.1m)                 |
|  - Asphalt: 6 cm physical structure sliced into solid blocks leaving hollow 3.5 cm recesses    |
|  - Potholes: Pothole.proto (recessed dark floor, wall liners, aggregate rubble)               |
|  - Decoys: Flush flat dark oil stain (stain_1 at x=10.0m)                                     |
|  - Robot: 4WD Differential Chassis ("pothole_robot", 0.30 x 0.20 x 0.06 m)                   |
|  - Actuators: 4x HingeJoint (axis 0 1 0, cylinders rotated 90 deg about X inside Transform)   |
|  - Sensory: GPS ("gps"), IMU ("imu"), Camera ("camera", 160x120, FOV 1.0, pitch -0.45 rad)   |
|  - Signaling: Red LED, Green LED, Speaker ("speaker"), Display ("lcd", 256x128 px on mast)    |
+-----------------------------------------------+-----------------------------------------------+
                                                |
                            Sensory Feedback    | Actuation & Visuals
                                                v
+-----------------------------------------------------------------------------------------------+
|                              POTHOLE CONTROLLER STATE MACHINE                                 |
|                                                                                               |
|         +---------------------------[RUNNING]----------------------------+                    |
|         | Cruising speed (default 0.4 m/s)                               |                    |
|         | Lane keeping via GPS y & IMU yaw                               |                    |
|         | Green LED ON, Red LED OFF, LCD: "ROAD CLEAR"                   |                    |
|         +----------------------------------------------------------------+                    |
|                      |                                       ^                                |
|             Hazard Detected                         Keyboard 'R' /                            |
|             (Sim or Camera)                          --auto-resume                            |
|                      v                                       |                                |
|         +----------------------[STOPPED_POTHOLE]-------------+                                |
|         | All motors commanded 0 velocity immediately                                         |
|         | Red LED ON, Green LED OFF                                                           |
|         | Speaker plays sounds/alarm.wav (pulsed buzzer)                                      |
|         | LCD displays: "POTHOLE DETECTED" / "ROBOT STOPPED"                                  |
|         | Event logged to CSV with ground-truth classification                                |
|         +------------------------------------------------------------+                        |
|                                                                      |                        |
|                      Mode OFF / Vision Unavailable                   | Reaches x >= 20.0m     |
|                      v                                               v                        |
|         +------------------------+                     +------------------------+             |
|         |    [DETECTION_OFF]     |                     |       [FINISHED]       |             |
|         | Fail-safe immobilization|                     | Cruise complete        |             |
|         | LCD: "DETECTION OFF"   |                     | LCD: "FINISH REACHED"  |             |
|         | (--blind: cruises with |                     | Motors braked          |             |
|         |  "NOT MONITORING")     |                     +------------------------+             |
|         +------------------------+                                                            |
+-----------------------------------------------+-----------------------------------------------+
                                                |
                                                v
+-----------------------------------------------------------------------------------------------+
|                                    DETECTION SUBSYSTEM                                        |
|                                                                                               |
|  1. SimulationDetector (Validation Mode):                                                      |
|     - Uses assets/pothole_map.json + GPS pose (robot_x, robot_y, robot_yaw)                   |
|     - Computes longitudinal distance between robot front bumper and pothole near edge         |
|     - Validates lane corridor (filters opposite-lane potholes) and latches 1 trigger per ID   |
|     - NOTE: Designed for state-machine and workflow verification, NOT perception.             |
|                                                                                               |
|  2. CameraDetector (Perception Mode):                                                         |
|     - Pipeline: BGRA -> Grayscale -> Gaussian Blur -> Threshold (<= 45)                       |
|     - Morphological Opening/Closing -> Contour Area & Aspect Ratio Filter                     |
|     - Pinhole camera inverse perspective model: projects pixel (u, v) to road plane (X, Y)    |
|     - 3-frame temporal persistence buffer + spatial Euclidean de-duplication                  |
|     - Ignores startup black frames; distinguishes true recesses from oil stains               |
+-----------------------------------------------+-----------------------------------------------+
                                                |
                                                v
+-----------------------------------------------------------------------------------------------+
|                                LOGGING & TEST VERIFICATION                                    |
|  - results/detection_log.csv: Real-time telemetry, state transitions, classifications        |
|  - tools/summarize_log.py: Automated evaluator for 5 benchmark test cases                     |
|  - results/test_summary.csv: Strict benchmark verdict (PASS / FAIL / NOT RUN)                 |
+-----------------------------------------------------------------------------------------------+
```

### Architectural Decisions: Display and Speaker
- **Webots Display Node as LCD**: Webots does not provide a native `LCD` node. In this project, an official Webots `Display` node (`width 256, height 128`) is attached to the rear mast facing backwards. The controller uses the Webots Display API (`setColor`, `fillRectangle`, `drawText`) to draw high-contrast graphical status cards. To guarantee visibility in headless testing and console logging, every visual LCD change is simultaneously mirrored to `stdout`.
- **Speaker and Audio Generation**: Webots acoustic feedback is provided via the `Speaker` device playing `sounds/alarm.wav`. The audio file is procedurally generated with standard library modules (`wave`, `struct`, `math`) without external binaries. If the simulator is running headless or without audio drivers, a console buzzer message (`[BUZZER SOUND] >>> *BEEP-BEEP* <<<`) acts as an automatic fallback.

---

## 2. Project File Structure

```
Smart-Pothole-Robot/
  ├── worlds/
  │   └── pothole_road.wbt          # Procedural Webots simulation world
  ├── protos/
  │   └── Pothole.proto             # PROTO node for physical recessed pothole
  ├── controllers/
  │   └── pothole_controller/
  │       ├── pothole_controller.py # Main loop & state machine controller
  │       ├── config.py             # Parameters, device declarations, CLI arguments
  │       ├── movement.py           # 4WD drive & proportional lane-keeping
  │       ├── detection.py          # SimulationDetector & CameraDetector
  │       ├── warning_system.py     # LEDs, Speaker (alarm.wav), LCD Display
  │       ├── logger.py             # Structured CSV logger
  │       └── sounds/
  │           └── alarm.wav         # Synthesized audio warning buzzer
  ├── assets/
  │   └── pothole_map.json          # Ground truth: real potholes, decoys, ghosts
  ├── results/
  │   ├── detection_log.csv         # Runtime event telemetry log
  │   └── test_summary.csv          # Benchmark test case summary
  ├── tools/
  │   ├── generate_world.py         # World generator script
  │   ├── check_install.py          # Pre-flight environment & device checker
  │   └── summarize_log.py          # Benchmark log analysis script
  ├── tests/
  │   ├── mock_webots.py            # Complete offline Webots mock API
  │   └── test_offline.py           # Comprehensive unittest test suite
  └── README.md                     # Complete project documentation
```

---

## 3. Installation & Setup

### Prerequisites
- **Python**: Version 3.8 or higher (Python 3.12 verified).
- **Python Libraries**:
  - `numpy` (installed)
  - `opencv-python` (installed; required only for camera perception mode)
  - Python standard library (`wave`, `struct`, `math`, `json`, `csv`, `unittest`, `argparse`).
- **Webots**: Webots R2022a or newer (World header defaults to `R2025a`).

### 1. Pre-flight Compatibility Check
Run the compatibility script from the project root or terminal:
```pwsh
python Smart-Pothole-Robot/tools/check_install.py
```
Expected output:
```
=================================================================
 Smart Pothole Detection Robot - Pre-flight & Install Check
=================================================================
[PASS]  Python Version            : Python 3.12.10 (>= 3.8 requirement met)
[PASS]  NumPy Library             : Found numpy v2.5.3
[PASS]  OpenCV Library            : Found cv2 v5.0.0 (Camera mode ready)
[WARN]  Webots Installation       : Webots is NOT installed or not found in WEBOTS_HOME/PATH...
[PASS]  Project Structure         : All 16 mandatory files present
[PASS]  Device Alignment          : All 11 controller devices found in worlds/pothole_road.wbt
=================================================================
```

### 2. Configuring Webots Python Interpreter
When running inside Webots:
1. Open Webots.
2. Navigate to **Tools > Preferences > Python command** (or **Webots > Preferences > Python command** on macOS).
3. Set the Python command to your Python executable containing NumPy and OpenCV (e.g. `python` or `C:\Users\<User>\AppData\Local\Programs\Python\Python312\python.exe`).
4. Open the world: `File > Open World... > Smart-Pothole-Robot/worlds/pothole_road.wbt`.

---

## 4. World Generation & Ground Truth

The simulation road and physical recesses are generated programmatically:
```pwsh
python Smart-Pothole-Robot/tools/generate_world.py --version R2025a
```

### Coordinate System (ENU)
- $+X$ = Forward (along road length from $-2.0\,\text{m}$ to $+22.0\,\text{m}$).
- $+Y$ = Left (road width $2.2\,\text{m}$; center dividing line at $y = 0.0\,\text{m}$).
- $+Z$ = Up (road surface at $z = 0.06\,\text{m}$, pothole floor at $z = 0.025\,\text{m}$).

### Ground Truth Layout (`assets/pothole_map.json`)
| ID | Type | Lane | Position (X, Y) | Dimensions (LxWxD) | Must Detect? | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `pothole_1` | Pothole | Robot Lane | $(4.0\,\text{m}, -0.50\,\text{m})$ | $0.60 \times 0.50 \times 0.035\,\text{m}$ | **YES** | Primary hazard stop |
| `pothole_4` | Pothole | Opposite Lane | $(6.0\,\text{m}, +0.55\,\text{m})$ | $0.60 \times 0.50 \times 0.035\,\text{m}$ | **NO** | Opposite lane isolation test |
| `pothole_2` | Pothole | Robot Lane | $(8.0\,\text{m}, -0.55\,\text{m})$ | $0.60 \times 0.50 \times 0.035\,\text{m}$ | **YES** | Multi-pothole test (offset) |
| `stain_1` | Oil Stain | Robot Lane | $(10.0\,\text{m}, -0.50\,\text{m})$ | $0.50 \times 0.40 \times 0.000\,\text{m}$ | **NO** | Visual decoy (dark, no depth) |
| `pothole_3` | Pothole | Robot Lane | $(12.5\,\text{m}, -0.45\,\text{m})$ | $0.70 \times 0.50 \times 0.035\,\text{m}$ | **YES** | Long pothole stop |
| `ghost_1` | Ghost | Robot Lane | $(16.0\,\text{m}, -0.50\,\text{m})$ | $0.60 \times 0.50 \times 0.035\,\text{m}$ | **NO** | `--inject-false` test target |

---

## 5. Controls & Command-Line Arguments

### Keyboard Hotkeys (Active in Webots 3D Window)
| Key | Action | Description |
| :---: | :--- | :--- |
| **`R`** | **Resume** | Releases emergency brake and resumes forward cruising after stopping for a pothole |
| **`M`** | **Cycle Mode** | Toggles detection pipeline: `SIM` $\to$ `CAMERA` $\to$ `OFF` $\to$ `SIM` |
| **`F`** | **Toggle Ghost** | Injects/removes virtual phantom pothole `ghost_1` at $x=16.0\,\text{m}$ |
| **`+`** / **`=`** | **Speed Up** | Increases cruising speed by $+0.1\,\text{m/s}$ |
| **`-`** / **`_`** | **Slow Down** | Decreases cruising speed by $-0.1\,\text{m/s}$ |
| **`H`** | **Help** | Prints keyboard control summary to simulation console |

### CLI Arguments (`controllerArgs`)
Arguments can be configured in the robot node's `controllerArgs` field in Webots or passed on CLI:
```bash
--mode [sim|camera|off]       # Detection mode (default: sim)
--speed [FLOAT]               # Cruising speed in m/s (default: 0.4)
--detect-distance [FLOAT]     # Detection distance ahead of bumper in m (default: 0.6)
--inject-false                # Injects phantom ghost pothole at x=16m
--blind                       # Overrides fail-safe stop when detection is off/unavailable
--auto-resume [SECONDS]       # Automatically resumes after N seconds (ideal for regression runs)
--dark-threshold [INT]        # Grayscale threshold for OpenCV detection (default: 45)
--quiet                       # Suppresses repetitive per-timestep debug output
```

---

## 6. Verification of the 5 Benchmark Test Cases

The evaluation tool `tools/summarize_log.py` automatically validates simulation runs against 5 benchmark cases and records results to `results/test_summary.csv` with strict `PASS`, `FAIL`, or `NOT RUN` verdicts:

### Case 1: Clear Road Driving
- **Requirement**: While traversing pristine roadway ($x < 2.0\,\text{m}$), green LED is ON, red LED is OFF, LCD displays `ROAD CLEAR`, and zero false detections are logged.
- **Verification**:
  ```pwsh
  python -c "
  import sys; sys.path.extend(['Smart-Pothole-Robot/controllers/pothole_controller', 'Smart-Pothole-Robot/tests'])
  from mock_webots import MockRobot; from pothole_controller import run_robot_controller
  robot = MockRobot(0.0, -0.5); robot.max_steps = 40
  run_robot_controller(mock_robot=robot, cli_args=['--mode', 'sim'])"
  python Smart-Pothole-Robot/tools/summarize_log.py
  ```

### Case 2: Pothole Detected & Safe Stop
- **Requirement**: Robot detects `pothole_1` within `detect_distance` ($0.6\,\text{m}$), zeros motor velocities immediately, turns red LED ON, green LED OFF, activates buzzer, and displays `POTHOLE DETECTED` / `ROBOT STOPPED`.
- **Verification**:
  ```pwsh
  python -c "
  import sys; sys.path.extend(['Smart-Pothole-Robot/controllers/pothole_controller', 'Smart-Pothole-Robot/tests'])
  from mock_webots import MockRobot; from pothole_controller import run_robot_controller
  robot = MockRobot(2.5, -0.5); robot.max_steps = 150
  run_robot_controller(mock_robot=robot, cli_args=['--mode', 'sim'])"
  python Smart-Pothole-Robot/tools/summarize_log.py
  ```

### Case 3: Multiple Potholes with No Repeated Alerts
- **Requirement**: Robot stops for each pothole in its lane (`pothole_1`, `pothole_2`, `pothole_3`) exactly once, with zero repeated triggers for the same hazard. Pothole in opposite lane (`pothole_4` at $y=+0.55\,\text{m}$) is completely ignored.
- **Verification**:
  ```pwsh
  python -c "
  import sys; sys.path.extend(['Smart-Pothole-Robot/controllers/pothole_controller', 'Smart-Pothole-Robot/tests'])
  from mock_webots import MockRobot; from pothole_controller import run_robot_controller
  robot = MockRobot(0.0, -0.5); robot.max_steps = 1000
  run_robot_controller(mock_robot=robot, cli_args=['--mode', 'sim', '--speed', '0.5', '--auto-resume', '0.3'])"
  python Smart-Pothole-Robot/tools/summarize_log.py
  ```

### Case 4: Reset / Resume Functionality
- **Requirement**: Upon receiving a resume command (keyboard `R` or `--auto-resume`), the robot transitions from `STOPPED_POTHOLE` back to `RUNNING`, restores green LED, and climbs out of the recess to advance.
- **Verification**:
  ```pwsh
  python -m unittest Smart-Pothole-Robot/tests/test_offline.py -k test_case_4_reset_resume_keyboard -v
  ```

### Case 5: Detection Disabled / Unavailable Fail-Safe
- **Requirement**: When detection is disabled (`--mode off`) or vision packages are unavailable, vehicle is immobilized under `DETECTION_OFF` fail-safe with both LEDs off and `DETECTION_UNAVAILABLE` logged. When overridden by `--blind`, robot drives displaying `NOT MONITORING`, and never logs `ROAD CLEAR`.
- **Verification**:
  ```pwsh
  python -c "
  import sys; sys.path.extend(['Smart-Pothole-Robot/controllers/pothole_controller', 'Smart-Pothole-Robot/tests'])
  from mock_webots import MockRobot; from pothole_controller import run_robot_controller
  robot = MockRobot(0.0, -0.5); robot.max_steps = 30
  run_robot_controller(mock_robot=robot, cli_args=['--mode', 'off'])"
  python Smart-Pothole-Robot/tools/summarize_log.py
  ```

### False-Detection & Stain Decoy Demos
- **Ghost Pothole Injection (`--inject-false`)**:
  ```pwsh
  python -m unittest Smart-Pothole-Robot/tests/test_offline.py -k test_ghost_pothole_labeling -v
  ```
  *Result*: Ghost pothole at $x=16.0\,\text{m}$ triggers stopping and is classified as `FALSE_POSITIVE`.
- **Camera Oil Stain Decoy Demo**:
  ```pwsh
  python -m unittest Smart-Pothole-Robot/tests/test_offline.py -k test_camera_mode_detection_and_stain_decoy -v
  ```
  *Result*: Real potholes are classified as `TRUE_POSITIVE`, whereas the dark flat oil stain (`stain_1` at $x=10.0\,\text{m}$) is flagged as `FALSE_POSITIVE` (dark surface patch without physical depth).

---

## 7. Automated Offline Mock Test Suite

Run the complete 13-test suite:
```pwsh
python -m unittest discover -s Smart-Pothole-Robot/tests -p "test_*.py" -v
```

### Test Suite Execution Output (Mock, Not Webots)
```
test_camera_mode_detection_and_stain_decoy (test_offline.TestPotholeRobotOffline) ... ok
test_camera_mode_without_opencv_fallback (test_offline.TestPotholeRobotOffline) ... ok
test_case_1_clear_road_driving (test_offline.TestPotholeRobotOffline) ... ok
test_case_2_pothole_detected_and_stop (test_offline.TestPotholeRobotOffline) ... ok
test_case_3_multiple_potholes_no_repeated_alerts (test_offline.TestPotholeRobotOffline) ... ok
test_case_4_reset_resume_keyboard (test_offline.TestPotholeRobotOffline) ... ok
test_case_5_blind_mode_override (test_offline.TestPotholeRobotOffline) ... ok
test_case_5_detection_off_failsafe (test_offline.TestPotholeRobotOffline) ... ok
test_configurable_detect_distance (test_offline.TestPotholeRobotOffline) ... ok
test_fatal_error_when_wheel_motor_missing (test_offline.TestPotholeRobotOffline) ... ok
test_ghost_pothole_labeling (test_offline.TestPotholeRobotOffline) ... ok
test_missing_optional_sensors_handled_gracefully (test_offline.TestPotholeRobotOffline) ... ok
test_other_lane_pothole_ignored (test_offline.TestPotholeRobotOffline) ... ok

----------------------------------------------------------------------
Ran 13 tests in 0.540s

OK
```
*Note: All 13 test cases were executed against `tests/mock_webots.py` using physical kinematics and synthetic camera rendering ("mock, not Webots").*

---

## 8. Troubleshooting Guide

| Issue / Symptom | Root Cause | Solution |
| :--- | :--- | :--- |
| `FATAL: Required drive motor 'wheel_fl' missing` | World file does not contain required motor name or motor is misspelled | Run `python tools/check_install.py` to verify device alignment. Regenerate world with `python tools/generate_world.py`. |
| `ModuleNotFoundError: No module named 'controller'` when running directly from terminal | Real Webots API is only exposed when executed from within Webots environment | Open the world directly inside Webots GUI, or run tests with `tests/mock_webots.py`. |
| Camera mode displays `DETECTION_UNAVAILABLE` | `opencv-python` or `numpy` is not installed in the active environment | Install OpenCV: `pip install opencv-python numpy`. |
| Robot does not resume after stopping | Robot is waiting for explicit user input `R` | Click inside the Webots 3D simulation window and press `R`, or run with `--auto-resume 1.5`. |
| Robot veers off the lane | High speed causing lateral overshoot | Reduce speed using `-` key or start controller with `--speed 0.3`. |
| Camera detects false positives in shadows | Dark threshold set too aggressively | Adjust threshold via `--dark-threshold 35`. |

---

## 9. 3-Minute Demonstration Script

Follow this step-by-step walkthrough to demonstrate the system:

1. **Minute 0:00 - 0:30 (Launch & Pre-flight)**:
   - Run `python Smart-Pothole-Robot/tools/check_install.py` in the terminal to confirm 100% device and environment compliance.
   - Open Webots, load `Smart-Pothole-Robot/worlds/pothole_road.wbt`.
   - Click the **Play** button in Webots.
2. **Minute 0:30 - 1:15 (First Hazard Detection)**:
   - Observe the robot cruising at $0.4\,\text{m/s}$ in lane $y = -0.5\,\text{m}$. Rear LCD displays green `ROAD CLEAR`, Green LED is illuminated.
   - At $x \approx 3.0\,\text{m}$, the robot detects `pothole_1` ($0.6\,\text{m}$ ahead).
   - The robot brakes to a complete halt before entering the depression.
   - Red LED turns ON, Green LED turns OFF, acoustic buzzer sounds, and the rear LCD displays `POTHOLE DETECTED / ROBOT STOPPED (pothole_1)`.
3. **Minute 1:15 - 2:00 (Interactive Resume & Opposite Lane Isolation)**:
   - Click on the Webots 3D view and press **`R`** on the keyboard.
   - The robot resumes forward movement.
   - At $x = 6.0\,\text{m}$, observe `pothole_4` in the opposite lane ($y = +0.55\,\text{m}$). The robot ignores it and continues cruising without stopping.
4. **Minute 2:00 - 2:30 (Second Hazard & Speed Adjustment)**:
   - At $x \approx 7.0\,\text{m}$, the robot safely stops before `pothole_2`.
   - Press **`R`** to resume. Press **`+`** twice to accelerate to $0.6\,\text{m/s}$.
5. **Minute 2:30 - 3:00 (Vision & Decoy Handling & Finish)**:
   - At $x = 10.0\,\text{m}$, observe the robot crossing the flat dark oil stain (`stain_1`).
   - Stop at `pothole_3` ($x = 12.5\,\text{m}$), press **`R`** to resume.
   - At $x \ge 20.0\,\text{m}$, the robot stops automatically and displays `FINISH REACHED`.
   - Run `python Smart-Pothole-Robot/tools/summarize_log.py` in the terminal to view the test verdict.

---

## 10. Completed Features Checklist

- [x] **ENU Coordinate Alignment**: Procedural world generator (`tools/generate_world.py`) with $+X$ forward, $+Y$ left, $+Z$ up.
- [x] **Physical Asphalt Recesses**: Asphalt sliced into solid blocks leaving hollow $3.5\,\text{cm}$ depressions; grass excluded from under road.
- [x] **Template Pothole PROTO**: `protos/Pothole.proto` with dark recessed floor, dark liners, and aggregate rubble.
- [x] **4WD Chassis & Wheel Geometry**: HingeJoint with axis $0\,1\,0$, wheel cylinders rotated $90^\circ$ about X inside Transform.
- [x] **State Machine Controller**: `RUNNING`, `STOPPED_POTHOLE`, `DETECTION_OFF`, `FINISHED`.
- [x] **Simulation Detection**: Ground-truth validation based on `assets/pothole_map.json` and GPS pose with lane corridor filtering.
- [x] **OpenCV Camera Detection**: Grayscale $\to$ blur $\to$ dark threshold $\to$ morphology $\to$ contour analysis $\to$ pinhole inverse road projection $\to$ 3-frame persistence.
- [x] **Multi-modal Warning System**: Red/Green LEDs, Speaker playing `sounds/alarm.wav`, rear LCD Display via Webots Display API with console mirroring.
- [x] **Opposite Lane Filtering**: `pothole_4` in other lane ($y=+0.55\,\text{m}$) ignored.
- [x] **False Positive Handling**: Ghost pothole injection (`--inject-false`) and oil stain decoy (`stain_1`) classified as `FALSE_POSITIVE`.
- [x] **Interactive & Automated Resumption**: Keyboard `R` and `--auto-resume` command.
- [x] **Fail-Safe Mechanism**: Detection OFF or vision unavailable immobilizes robot; `--blind` cruises under `NOT MONITORING`.
- [x] **Structured Event Logging**: `results/detection_log.csv` recording 15 telemetry fields.
- [x] **Automated Evaluator**: `tools/summarize_log.py` writing `results/test_summary.csv` with strict PASS / FAIL / NOT RUN verdicts.
- [x] **Pre-flight Installation Checker**: `tools/check_install.py` cross-verifying devices against `.wbt` file.
- [x] **Offline Test Suite & Mock**: 13 unit tests in `tests/test_offline.py` covering all features and error modes.

---

## 11. Limitations & Verification Disclosure

### Verified Offline via Mock ("Mock, Not Webots")
- Webots is **not installed** on this machine (`webots --version` returned not found in PATH and default directories).
- Therefore, all controller state transitions, kinematics integration, pinhole inverse projections, camera computer vision filtering, warning device fallbacks, log generation, and unit tests were validated using `tests/mock_webots.py` and `tests/test_offline.py`.
- **No accuracy numbers, trained models, or false claims of running inside the Webots GUI have been made.**

### Limitations of Dark Thresholding Perception
1. **Photometric Dependency**: Thresholding at $\le 45$ grayscale assumes potholes cast a dark shadow and have darker sub-base aggregate than surrounding asphalt. Pale, shallow, dry, or sun-flooded potholes may fall below this contrast threshold.
2. **Visual Decoys / Non-Pothole Anomalies**: A flat dark surface anomaly (such as the oil stain `stain_1`) creates a dark contour indistinguishable from a physical pit using monocular intensity thresholding alone. True geometric depth discrimination requires stereo vision, LiDAR, or structured light.
3. **Lighting & Shadows**: Overcast or uniform lighting is assumed; harsh directional sunlight or tree shadows will produce false positive contours unless specialized deep neural networks or geometric edge filters are employed.

### Items for User Confirmation inside Real Webots GUI
When opening `worlds/pothole_road.wbt` in the Webots GUI, please inspect and confirm:
1. **LCD Texture Appearance**: Confirm that the rear mast Display texture renders clearly in the 3D viewport.
2. **Viewpoint Angle**: Verify that the tracking viewpoint follow mode matches your display resolution.
3. **Contact Dynamics**: Confirm that the $3.5\,\text{cm}$ recessed asphalt depth and wheel friction contact allow the robot to smoothly climb out after resuming.
4. **Camera Threshold Calibration**: If you modify the directional lighting or world brightness in Webots, adjust `--dark-threshold` to match your scene illumination.
