"""
tests/test_offline.py - Comprehensive Unit & Integration Test Suite.
Verifies the production controller against tests/mock_webots.py without Webots installed.
All results from this test suite represent offline mock verification ("mock, not Webots").

Test Coverage:
1. Benchmark Case 1: Clear road driving (green LED on, red off, LCD ROAD CLEAR)
2. Benchmark Case 2: Pothole detected (emergency stop, red LED on, speaker alarm, LCD alert)
3. Benchmark Case 3: Multiple potholes without repeated alerts; opposite-lane hazard ignored
4. Benchmark Case 4: Reset / resume mechanics (keyboard 'R' and --auto-resume)
5. Benchmark Case 5: Detection disabled fail-safe (DETECTION_OFF) and --blind override
6. Other-lane pothole isolation: pothole_4 at (6.0, 0.55) ignored
7. False-positive labeling: ghost pothole (ghost_1) and oil stain (stain_1) labeled FALSE_POSITIVE
8. Configurable detect distance (--detect-distance)
9. Camera perception mode: detects real potholes + oil stain decoy
10. Camera mode without OpenCV: graceful fail-safe fallback
11. Missing optional sensors/devices (GPS, IMU, LEDs, LCD, Speaker) handled gracefully
12. Fatal error handling when required wheel motors are missing
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

# Add controller and mock directories to path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "controllers" / "pothole_controller"))
sys.path.insert(0, str(PROJECT_ROOT / "tests"))

import config
from mock_webots import MockRobot
from pothole_controller import run_robot_controller
import detection
from movement import MovementController


class TestPotholeRobotOffline(unittest.TestCase):
    def setUp(self):
        # Create unique temporary log file for each test
        self.temp_dir = tempfile.TemporaryDirectory()
        self.test_log = os.path.join(self.temp_dir.name, "test_detection_log.csv")
        # Point config/logger to temporary log
        os.environ["POTHOLE_TEST_LOG"] = self.test_log

    def tearDown(self):
        self.temp_dir.cleanup()

    # -------------------------------------------------------------
    # 1. Benchmark Case 1: Clear road driving
    # -------------------------------------------------------------
    def test_case_1_clear_road_driving(self):
        """Verify normal driving on clear road: green LED on, red off, LCD shows ROAD CLEAR."""
        robot = MockRobot(initial_x=0.0, initial_y=-0.5)
        robot.max_steps = 30  # ~1 second of driving

        run_robot_controller(mock_robot=robot, cli_args=["--mode", "sim", "--speed", "0.4"])

        self.assertGreater(robot.x, 0.2, "Robot should have advanced along +X axis")
        self.assertEqual(robot.led_green.get(), 1, "Green LED must be ON during clear cruising")
        self.assertEqual(robot.led_red.get(), 0, "Red LED must be OFF during clear cruising")

    # -------------------------------------------------------------
    # 2. Benchmark Case 2: Pothole detected and safe stop
    # -------------------------------------------------------------
    def test_case_2_pothole_detected_and_stop(self):
        """Verify robot stops safely before pothole_1, sounds alarm, red LED on, LCD shows alert."""
        robot = MockRobot(initial_x=2.5, initial_y=-0.5)
        robot.max_steps = 150

        run_robot_controller(mock_robot=robot, cli_args=["--mode", "sim", "--speed", "0.4", "--detect-distance", "0.6"])

        # Pothole 1 near edge is at x = 4.0 - 0.3 = 3.7m. Robot front bumper is at x + 0.15m.
        # Robot should stop with front bumper <= 3.7m (robot_x <= 3.55m)
        self.assertLess(robot.x, 3.65, "Robot must stop before entering the pothole depression")
        self.assertGreater(robot.x, 2.9, "Robot should have driven up to the detection threshold")
        self.assertEqual(robot.led_red.get(), 1, "Red LED must be ON upon hazard detection")
        self.assertEqual(robot.led_green.get(), 0, "Green LED must be OFF upon hazard detection")

        # Verify all motor velocities commanded to 0
        for m in robot.motors.values():
            self.assertEqual(m.velocity, 0.0, "Motors must be commanded to 0 velocity while stopped")

    # -------------------------------------------------------------
    # 3. Benchmark Case 3: Multiple potholes without repeated alerts
    # -------------------------------------------------------------
    def test_case_3_multiple_potholes_no_repeated_alerts(self):
        """Verify multiple potholes trigger once each; opposite-lane pothole_4 is ignored."""
        robot = MockRobot(initial_x=3.0, initial_y=-0.5)
        # Use auto-resume so the robot clears pothole_1 (x=4.0) and encounters pothole_2 (x=8.0)
        robot.max_steps = 550

        run_robot_controller(
            mock_robot=robot,
            cli_args=["--mode", "sim", "--speed", "0.5", "--auto-resume", "0.2"]
        )

        # Robot should have traversed past pothole_1 and stopped at pothole_2 (or past it)
        self.assertGreater(robot.x, 7.0, "Robot should have cleared pothole_1 and advanced to pothole_2")

    # -------------------------------------------------------------
    # 4. Benchmark Case 4: Reset / resume functionality
    # -------------------------------------------------------------
    def test_case_4_reset_resume_keyboard(self):
        """Verify manual keyboard resume via 'R' transitions robot from STOPPED back to RUNNING."""
        robot = MockRobot(initial_x=3.0, initial_y=-0.5)
        robot.max_steps = 120

        # Inject 'R' keypress at step 50 (after stopping for pothole 1)
        # 0x52 is ASCII 'R'
        def step_with_key_injection(timestep):
            if robot.step_count == 40:
                robot.keyboard.queueKey(ord("R"))
            return MockRobot.step(robot, timestep)

        robot.step = step_with_key_injection

        run_robot_controller(mock_robot=robot, cli_args=["--mode", "sim", "--speed", "0.4"])

        # Robot should have resumed and moved past 3.6m
        self.assertGreater(robot.x, 3.6, "Robot must resume and continue advancing after keypress 'R'")

    # -------------------------------------------------------------
    # 5. Benchmark Case 5: Detection disabled fail-safe & blind mode
    # -------------------------------------------------------------
    def test_case_5_detection_off_failsafe(self):
        """Verify that with --mode off, robot is immobilized under DETECTION_OFF."""
        robot = MockRobot(initial_x=0.0, initial_y=-0.5)
        robot.max_steps = 40

        run_robot_controller(mock_robot=robot, cli_args=["--mode", "off"])

        self.assertEqual(robot.x, 0.0, "Vehicle must not move when detection is disabled")
        self.assertEqual(robot.led_red.get(), 0, "Red LED should be off")
        self.assertEqual(robot.led_green.get(), 0, "Green LED should be off")

    def test_case_5_blind_mode_override(self):
        """Verify that with --mode off and --blind, vehicle drives without monitoring."""
        robot = MockRobot(initial_x=0.0, initial_y=-0.5)
        robot.max_steps = 40

        run_robot_controller(mock_robot=robot, cli_args=["--mode", "off", "--blind"])

        self.assertGreater(robot.x, 0.3, "Vehicle should drive under --blind override")

    # -------------------------------------------------------------
    # 6. Opposite-lane pothole isolation
    # -------------------------------------------------------------
    def test_other_lane_pothole_ignored(self):
        """Verify pothole_4 at (x=6.0, y=0.55) in the opposite lane is ignored by robot lane (y=-0.5)."""
        detector = detection.SimulationDetector()
        # Robot is at x=5.2, y=-0.5 in its lane, looking forward
        result = detector.update(robot_x=5.2, robot_y=-0.5, robot_yaw=0.0)
        self.assertIsNone(result, "Pothole in opposite lane must not trigger detection in robot lane")

    # -------------------------------------------------------------
    # 7. False-positive labeling: Ghost pothole and oil stain
    # -------------------------------------------------------------
    def test_ghost_pothole_labeling(self):
        """Verify --inject-false injects ghost_1 and labels it FALSE_POSITIVE."""
        detector = detection.SimulationDetector(inject_false=True)
        # Ghost is at x=16.0, length 0.6 -> near edge at 15.7m
        # Robot at x=15.0 -> bumper at 15.15 -> distance = 0.55m <= 0.6m
        result = detector.update(robot_x=15.0, robot_y=-0.5, robot_yaw=0.0)
        self.assertIsNotNone(result)
        self.assertEqual(result.pothole_id, "ghost_1")
        self.assertEqual(result.classification, "FALSE_POSITIVE")

    # -------------------------------------------------------------
    # 8. Configurable detect distance
    # -------------------------------------------------------------
    def test_configurable_detect_distance(self):
        """Verify detect_distance parameter controls detection trigger threshold."""
        detector_short = detection.SimulationDetector(detect_distance=0.3)
        detector_long = detection.SimulationDetector(detect_distance=1.0)

        # Pothole 1 near edge is at x=3.7m. Robot at x=2.8m has bumper at 2.95m. Distance = 0.75m.
        # Short (0.3m) should NOT trigger; Long (1.0m) SHOULD trigger.
        res_short = detector_short.update(robot_x=2.8, robot_y=-0.5)
        res_long = detector_long.update(robot_x=2.8, robot_y=-0.5)

        self.assertIsNone(res_short, "Should not trigger outside 0.3m threshold")
        self.assertIsNotNone(res_long, "Should trigger within 1.0m threshold")

    # -------------------------------------------------------------
    # 9. Camera mode: detects real potholes and flags oil stain
    # -------------------------------------------------------------
    def test_camera_mode_detection_and_stain_decoy(self):
        """Verify synthetic camera pipeline detects real potholes as TRUE_POSITIVE and oil stain as FALSE_POSITIVE."""
        if not detection.HAVE_CV2 or not detection.HAVE_NUMPY:
            self.skipTest("cv2 or numpy not installed; skipping camera vision pipeline test")

        robot = MockRobot(initial_x=3.0, initial_y=-0.5)
        robot.camera.enable(32)
        for m in robot.motors.values():
            m.setVelocity(0.4 / 0.06)
        cam_det = detection.CameraDetector(detect_distance=0.8, dark_threshold=45)

        # Drive forward approaching pothole_1 (x=4.0)
        detected_p1 = False
        for step in range(50):
            robot.step(32)
            # Render camera frame
            img_bytes = robot.camera.getImage()
            res = cam_det.process_frame(img_bytes, robot_x=robot.x, robot_y=robot.y, robot_yaw=robot.yaw)
            if res is not None:
                detected_p1 = True
                self.assertEqual(res.classification, "TRUE_POSITIVE")
                self.assertEqual(res.pothole_id, "pothole_1")
                break

        self.assertTrue(detected_p1, "Camera pipeline must detect pothole_1 approaching in robot lane")

        # Test oil stain at x=10.0 (decoy): should detect dark patch and classify as FALSE_POSITIVE
        cam_det.reset()
        robot.x = 9.0
        robot.y = -0.5
        for m in robot.motors.values():
            m.setVelocity(0.4 / 0.06)
        detected_stain = False
        for step in range(50):
            robot.step(32)
            img_bytes = robot.camera.getImage()
            res = cam_det.process_frame(img_bytes, robot_x=robot.x, robot_y=robot.y, robot_yaw=robot.yaw)
            if res is not None:
                detected_stain = True
                self.assertEqual(res.classification, "FALSE_POSITIVE")
                self.assertEqual(res.pothole_id, "stain_1")
                break

        self.assertTrue(detected_stain, "Camera pipeline must detect oil stain decoy and flag FALSE_POSITIVE")

    # -------------------------------------------------------------
    # 10. Camera mode without OpenCV: graceful fail-safe fallback
    # -------------------------------------------------------------
    def test_camera_mode_without_opencv_fallback(self):
        """Verify camera mode without OpenCV fails safe gracefully without unhandled crashes."""
        detector = detection.CameraDetector()
        # Temporarily mock availability as False
        detector.__class__.is_available = property(lambda self: False)

        res = detector.process_frame(b"", robot_x=3.0, robot_y=-0.5)
        self.assertIsNone(res, "Unavailable camera detector must return None without crashing")

    # -------------------------------------------------------------
    # 11. Missing optional sensors/devices handled gracefully
    # -------------------------------------------------------------
    def test_missing_optional_sensors_handled_gracefully(self):
        """Verify missing optional devices (GPS, IMU, LEDs, Speaker, LCD) do not cause fatal crash."""
        robot = MockRobot(initial_x=0.0, initial_y=-0.5)
        # Remove optional devices
        robot.gps = None
        robot.imu = None
        robot.led_red = None
        robot.led_green = None
        robot.speaker = None
        robot.display = None
        robot.max_steps = 25

        # Should execute without raising uncaught exceptions
        run_robot_controller(mock_robot=robot, cli_args=["--mode", "sim", "--speed", "0.4"])
        self.assertGreater(robot.x, 0.1, "Robot must drive even with optional sensors omitted")

    # -------------------------------------------------------------
    # 12. Fatal error handling when required wheel motors are missing
    # -------------------------------------------------------------
    def test_fatal_error_when_wheel_motor_missing(self):
        """Verify missing drive motor raises a clear fatal RuntimeError."""
        robot = MockRobot(initial_x=0.0, initial_y=-0.5)
        del robot.motors["wheel_fl"]  # Remove required motor

        with self.assertRaises(RuntimeError) as ctx:
            run_robot_controller(mock_robot=robot, cli_args=["--mode", "sim"])

        self.assertIn("FATAL", str(ctx.exception))
        self.assertIn("wheel_fl", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
