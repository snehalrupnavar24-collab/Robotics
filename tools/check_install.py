#!/usr/bin/env python3
"""
tools/check_install.py - Pre-flight Compatibility & Installation Verification Script.

Verifies:
1. Python version >= 3.8
2. Required and optional Python libraries (numpy, cv2)
3. WEBOTS_HOME environment variable and executable presence
4. Project file hierarchy completeness
5. Device name consistency between controller/config.py and worlds/pothole_road.wbt
"""

import os
import re
import sys
from pathlib import Path


def print_status(component: str, ok: bool, message: str, warning: bool = False):
    if ok:
        status = "[PASS]"
        color = "\033[92m"
    elif warning:
        status = "[WARN]"
        color = "\033[93m"
    else:
        status = "[FAIL]"
        color = "\033[91m"
    reset = "\033[0m"
    print(f"{color}{status:<7}{reset} {component:<25} : {message}")


def check_python_version() -> bool:
    v = sys.version_info
    ver_str = f"{v.major}.{v.minor}.{v.micro}"
    if v.major > 3 or (v.major == 3 and v.minor >= 8):
        print_status("Python Version", True, f"Python {ver_str} (>= 3.8 requirement met)")
        return True
    else:
        print_status("Python Version", False, f"Python {ver_str} (< 3.8 unsupported)")
        return False


def check_libraries():
    all_ok = True
    # numpy check
    try:
        import numpy as np
        print_status("NumPy Library", True, f"Found numpy v{np.__version__}")
    except ImportError:
        print_status("NumPy Library", False, "numpy not installed. Run: pip install numpy")
        all_ok = False

    # cv2 check (optional for sim mode, required for camera mode)
    try:
        import cv2
        print_status("OpenCV Library", True, f"Found cv2 v{cv2.__version__} (Camera mode ready)")
    except ImportError:
        print_status("OpenCV Library", False, "cv2 not installed. Camera mode will be unavailable. Run: pip install opencv-python", warning=True)

    return all_ok


def check_webots_installation():
    webots_home = os.environ.get("WEBOTS_HOME")
    webots_in_path = False

    # Check where.exe / which
    import shutil
    webots_path = shutil.which("webots")

    if webots_home and Path(webots_home).is_dir():
        print_status("WEBOTS_HOME", True, f"Defined at: {webots_home}")
    elif webots_path:
        print_status("WEBOTS_HOME", True, f"Found in PATH: {webots_path}")
        webots_in_path = True
    else:
        # Check standard default installation paths on Windows / Linux / macOS
        std_paths = [
            Path("C:/Program Files/Webots"),
            Path(os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots")),
            Path("/usr/local/webots"),
            Path("/snap/webots/current/usr/share/webots"),
            Path("/Applications/Webots.app"),
        ]
        found_std = None
        for p in std_paths:
            if p.is_dir():
                found_std = p
                break
        if found_std:
            print_status("Webots Installation", True, f"Found at default path: {found_std} (Set WEBOTS_HOME={found_std})", warning=True)
        else:
            print_status("Webots Installation", False, "Webots is NOT installed or not found in WEBOTS_HOME/PATH. Real simulation requires Webots R2022a+.", warning=True)


def check_project_files(project_root: Path) -> bool:
    required_files = [
        "worlds/pothole_road.wbt",
        "protos/Pothole.proto",
        "controllers/pothole_controller/pothole_controller.py",
        "controllers/pothole_controller/config.py",
        "controllers/pothole_controller/movement.py",
        "controllers/pothole_controller/detection.py",
        "controllers/pothole_controller/warning_system.py",
        "controllers/pothole_controller/logger.py",
        "controllers/pothole_controller/sounds/alarm.wav",
        "assets/pothole_map.json",
        "tools/generate_world.py",
        "tools/check_install.py",
        "tools/summarize_log.py",
        "tests/mock_webots.py",
        "tests/test_offline.py",
        "README.md",
    ]

    all_exist = True
    missing = []
    for rel_path in required_files:
        full_path = project_root / rel_path
        if not full_path.exists():
            all_exist = False
            missing.append(rel_path)

    if all_exist:
        print_status("Project Structure", True, f"All {len(required_files)} mandatory files present")
    else:
        print_status("Project Structure", False, f"Missing {len(missing)} files: {', '.join(missing[:3])}{'...' if len(missing) > 3 else ''}")

    return all_exist


def check_device_names(project_root: Path):
    world_path = project_root / "worlds" / "pothole_road.wbt"
    if not world_path.exists():
        print_status("Device Alignment", False, "worlds/pothole_road.wbt not found yet (generate via tools/generate_world.py)", warning=True)
        return False

    # Read world file contents
    world_text = world_path.read_text(encoding="utf-8", errors="ignore")

    # Import declared device names from config.py
    sys.path.insert(0, str(project_root / "controllers" / "pothole_controller"))
    try:
        import config
    except ImportError as e:
        print_status("Device Alignment", False, f"Could not import config.py: {e}")
        return False

    required_devices = config.REQUIRED_MOTOR_NAMES
    optional_devices = config.OPTIONAL_SENSOR_NAMES

    missing_required = []
    for dev in required_devices:
        # Match name "dev_name"
        pattern = rf'name\s+"{dev}"'
        if not re.search(pattern, world_text):
            missing_required.append(dev)

    missing_optional = []
    for dev in optional_devices:
        pattern = rf'name\s+"{dev}"'
        if not re.search(pattern, world_text):
            missing_optional.append(dev)

    if not missing_required and not missing_optional:
        print_status("Device Alignment", True, f"All {len(required_devices)+len(optional_devices)} controller devices found in worlds/pothole_road.wbt")
        return True
    elif missing_required:
        print_status("Device Alignment", False, f"FATAL: Required motors missing in world: {missing_required}")
        return False
    else:
        print_status("Device Alignment", True, f"Required motors present. Note: Optional sensors not found: {missing_optional}", warning=True)
        return True


def main():
    print("=" * 65)
    print(" Smart Pothole Detection Robot - Pre-flight & Install Check")
    print("=" * 65)

    script_path = Path(__file__).resolve()
    project_root = script_path.parent.parent

    check_python_version()
    check_libraries()
    check_webots_installation()
    check_project_files(project_root)
    check_device_names(project_root)
    print("=" * 65)


if __name__ == "__main__":
    main()
