#!/usr/bin/env python3
"""
tools/generate_world.py - Procedural Webots World Generator.

Generates worlds/pothole_road.wbt in ENU coordinates:
- Road length: x from -2.0 to 22.0 m (24 m total)
- Road width: 2.2 m (-1.1 to +1.1 m in y)
- Two lanes: Robot lane at y = -0.50 m, Opposite lane at y = +0.50 m
- Markings: White edge lines, dashed yellow center line, protective curbs
- Grass borders strictly outside road perimeter (sides |y| > 1.2 m, ends x < -2.0 m and x > 22.0 m)
- Physical Recessed Potholes:
  - Asphalt layer (6 cm total) cut into solid blocks leaving rectangular depressions
  - Recessed floor (3.5 cm deep, floor at z = 0.025 m)
  - Pothole.proto instances with near-black floor, dark wall liners, and aggregate rubble
  - 3 potholes in robot lane: pothole_1 (4.0, -0.50), pothole_2 (8.0, -0.55), pothole_3 (12.5, -0.45)
  - 1 pothole in opposite lane: pothole_4 (6.0, +0.55)
  - 1 flat dark oil stain flush with road: stain_1 (10.0, -0.50)
- Robot "pothole_robot":
  - 4WD differential drive with 4x HingeJoint (axis 0 1 0, cylinders rotated 90 deg about x)
  - Devices: GPS "gps", IMU "imu", Camera "camera" (160x120, FOV 1.0, pitch -0.45 rad),
    LED "led_red", LED "led_green", Speaker "speaker", Display "lcd" (256x128 on rear mast)
- Configurable Webots version header (default R2025a, compatible R2022a+).
"""

import argparse
import json
import math
import sys
from pathlib import Path


def generate_world(version: str = "R2025a", output_path: Path = None):
    if output_path is None:
        output_path = Path(__file__).resolve().parent.parent / "worlds" / "pothole_road.wbt"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Road specifications
    ROAD_X_MIN = -2.0
    ROAD_X_MAX = 22.0
    ROAD_Y_MIN = -1.1
    ROAD_Y_MAX = 1.1
    ROAD_SURFACE_Z = 0.06      # Asphalt top surface
    POTHOLE_DEPTH = 0.035      # 3.5 cm recess depth
    PIT_FLOOR_Z = ROAD_SURFACE_Z - POTHOLE_DEPTH  # 0.025 m

    # Pothole definitions: (id, x, y, size_x, size_y)
    potholes = [
        {"id": "pothole_1", "x": 4.0, "y": -0.50, "sx": 0.60, "sy": 0.50},
        {"id": "pothole_2", "x": 8.0, "y": -0.55, "sx": 0.60, "sy": 0.50},
        {"id": "pothole_3", "x": 12.5, "y": -0.45, "sx": 0.70, "sy": 0.50},
        {"id": "pothole_4", "x": 6.0, "y": 0.55, "sx": 0.60, "sy": 0.50},
    ]

    # Oil stain decoy: flush flat plane
    stain = {"id": "stain_1", "x": 10.0, "y": -0.50, "sx": 0.50, "sy": 0.40}

    lines = []
    # 1. Header
    lines.append(f"#VRML_SIM {version} utf-8")
    lines.append("")
    lines.append("EXTERNPROTO \"../protos/Pothole.proto\"")
    lines.append("")
    lines.append("WorldInfo {")
    lines.append("  coordinateSystem \"ENU\"")
    lines.append("  info [")
    lines.append("    \"Smart Pothole Detection and Warning Robot Simulation\",")
    lines.append("    \"Autonomous road inspection with dual simulation/camera detection\"")
    lines.append("  ]")
    lines.append("  title \"Smart Pothole Detection Road\"")
    lines.append("  basicTimeStep 32")
    lines.append("}")
    lines.append("")

    # 2. Viewpoint (following robot from behind-left)
    lines.append("Viewpoint {")
    lines.append("  orientation 0.28 0.28 -0.92 1.65")
    lines.append("  position -2.5 -0.5 1.8")
    lines.append("  follow \"pothole_robot\"")
    lines.append("  followType \"Tracking Shot\"")
    lines.append("}")
    lines.append("")

    # 3. Lighting & Background (No sun shadows)
    lines.append("Background {")
    lines.append("  skyColor [ 0.4 0.7 0.9 ]")
    lines.append("}")
    lines.append("")
    lines.append("DirectionalLight {")
    lines.append("  ambientIntensity 0.4")
    lines.append("  direction 0.5 0.3 -0.8")
    lines.append("  intensity 0.9")
    lines.append("  castShadows FALSE")
    lines.append("}")
    lines.append("")

    # 4. Grass borders (strictly outside road bounds)
    # Side grass: y in [-6.0, -1.2] and y in [1.2, 6.0]
    # End grass: x in [-6.0, -2.0] and x in [22.0, 26.0]
    grass_patches = [
        # Right grass border
        {"x": 10.0, "y": -3.6, "sx": 32.0, "sy": 4.8, "name": "grass_right"},
        # Left grass border
        {"x": 10.0, "y": 3.6, "sx": 32.0, "sy": 4.8, "name": "grass_left"},
        # Start end grass
        {"x": -4.0, "y": 0.0, "sx": 4.0, "sy": 2.4, "name": "grass_start"},
        # Finish end grass
        {"x": 24.0, "y": 0.0, "sx": 4.0, "sy": 2.4, "name": "grass_finish"},
    ]
    for g in grass_patches:
        lines.append(f"Solid {{")
        lines.append(f"  translation {g['x']} {g['y']} 0.02")
        lines.append(f"  name \"{g['name']}\"")
        lines.append(f"  children [")
        lines.append(f"    Shape {{")
        lines.append(f"      appearance PBRAppearance {{")
        lines.append(f"        baseColor 0.22 0.55 0.18")
        lines.append(f"        roughness 0.9")
        lines.append(f"      }}")
        lines.append(f"      geometry Box {{")
        lines.append(f"        size {g['sx']} {g['sy']} 0.04")
        lines.append(f"      }}")
        lines.append(f"    }}")
        lines.append(f"  ]")
        lines.append(f"  boundingObject Box {{ size {g['sx']} {g['sy']} 0.04 }}")
        lines.append(f"}}")
        lines.append("")

    # 5. Protective Curbs (along edge y = -1.15 and y = +1.15)
    lines.append("Solid {")
    lines.append("  translation 10.0 -1.15 0.07")
    lines.append("  name \"curb_right\"")
    lines.append("  children [")
    lines.append("    Shape {")
    lines.append("      appearance PBRAppearance { baseColor 0.7 0.7 0.7 roughness 0.6 }")
    lines.append("      geometry Box { size 24.0 0.10 0.10 }")
    lines.append("    }")
    lines.append("  ]")
    lines.append("  boundingObject Box { size 24.0 0.10 0.10 }")
    lines.append("}")
    lines.append("")
    lines.append("Solid {")
    lines.append("  translation 10.0 1.15 0.07")
    lines.append("  name \"curb_left\"")
    lines.append("  children [")
    lines.append("    Shape {")
    lines.append("      appearance PBRAppearance { baseColor 0.7 0.7 0.7 roughness 0.6 }")
    lines.append("      geometry Box { size 24.0 0.10 0.10 }")
    lines.append("    }")
    lines.append("  ]")
    lines.append("  boundingObject Box { size 24.0 0.10 0.10 }")
    lines.append("}")
    lines.append("")

    # 6. Road Sub-base Layer (Continuous structural floor under road at z = 0.0125, height 0.025)
    lines.append("Solid {")
    lines.append("  translation 10.0 0.0 0.0125")
    lines.append("  name \"road_subbase\"")
    lines.append("  children [")
    lines.append("    Shape {")
    lines.append("      appearance PBRAppearance {")
    lines.append("        baseColor 0.12 0.12 0.12")
    lines.append("        roughness 0.95")
    lines.append("      }")
    lines.append("      geometry Box { size 24.0 2.2 0.025 }")
    lines.append("    }")
    lines.append("  ]")
    lines.append("  boundingObject Box { size 24.0 2.2 0.025 }")
    lines.append("}")
    lines.append("")

    # 7. Physical Asphalt Recesses (Sliced asphalt blocks)
    # The asphalt upper layer has thickness 0.035 m (from z = 0.025 to z = 0.06).
    # Center of asphalt upper layer is z = 0.0425.
    # To leave real hollow rectangular holes for potholes, we subdivide the road surface.
    # We create longitudinal sections along X:
    # Segments between potholes:
    # -2.0 to 3.7 (L1 = 5.7)
    # Pothole 1 at x in [3.7, 4.3]:
    #   hole in robot lane y in [-0.75, -0.25].
    #   pavement around hole: right strip y in [-1.1, -0.75], left strip y in [-0.25, 1.1]
    # 4.3 to 5.7 (L2 = 1.4)
    # Pothole 4 at x in [5.7, 6.3]:
    #   hole in other lane y in [0.30, 0.80].
    #   pavement around hole: right strip y in [-1.1, 0.30], left strip y in [0.80, 1.1]
    # 6.3 to 7.7 (L3 = 1.4)
    # Pothole 2 at x in [7.7, 8.3]:
    #   hole in robot lane y in [-0.80, -0.30].
    #   pavement around hole: right strip y in [-1.1, -0.80], left strip y in [-0.30, 1.1]
    # 8.3 to 12.15 (L4 = 3.85) (contains flat oil stain at x=10.0, y=-0.50)
    # Pothole 3 at x in [12.15, 12.85]:
    #   hole in robot lane y in [-0.70, -0.20].
    #   pavement around hole: right strip y in [-1.1, -0.70], left strip y in [-0.20, 1.1]
    # 12.85 to 22.0 (L5 = 9.15)

    asphalt_thickness = POTHOLE_DEPTH  # 0.035 m
    asphalt_cz = PIT_FLOOR_Z + asphalt_thickness / 2.0  # 0.0425 m

    def make_box(x_min, x_max, y_min, y_max, name):
        sx = x_max - x_min
        sy = y_max - y_min
        cx = (x_min + x_max) / 2.0
        cy = (y_min + y_max) / 2.0
        return (
            f"Solid {{\n"
            f"  translation {cx:.4f} {cy:.4f} {asphalt_cz:.4f}\n"
            f"  name \"{name}\"\n"
            f"  children [\n"
            f"    Shape {{\n"
            f"      appearance PBRAppearance {{\n"
            f"        baseColor 0.28 0.28 0.28\n"
            f"        roughness 0.85\n"
            f"      }}\n"
            f"      geometry Box {{ size {sx:.4f} {sy:.4f} {asphalt_thickness:.4f} }}\n"
            f"    }}\n"
            f"  ]\n"
            f"  boundingObject Box {{ size {sx:.4f} {sy:.4f} {asphalt_thickness:.4f} }}\n"
            f"}}\n"
        )

    # Add asphalt blocks
    # Block 1: Before pothole 1
    lines.append(make_box(ROAD_X_MIN, 3.70, ROAD_Y_MIN, ROAD_Y_MAX, "asphalt_sec1"))
    # Pothole 1 zone (3.70 to 4.30)
    lines.append(make_box(3.70, 4.30, -1.10, -0.75, "asphalt_p1_right"))
    lines.append(make_box(3.70, 4.30, -0.25, 1.10, "asphalt_p1_left"))
    # Block 2: Between pothole 1 and 4
    lines.append(make_box(4.30, 5.70, ROAD_Y_MIN, ROAD_Y_MAX, "asphalt_sec2"))
    # Pothole 4 zone (5.70 to 6.30)
    lines.append(make_box(5.70, 6.30, -1.10, 0.30, "asphalt_p4_right"))
    lines.append(make_box(5.70, 6.30, 0.80, 1.10, "asphalt_p4_left"))
    # Block 3: Between pothole 4 and 2
    lines.append(make_box(6.30, 7.70, ROAD_Y_MIN, ROAD_Y_MAX, "asphalt_sec3"))
    # Pothole 2 zone (7.70 to 8.30)
    lines.append(make_box(7.70, 8.30, -1.10, -0.80, "asphalt_p2_right"))
    lines.append(make_box(7.70, 8.30, -0.30, 1.10, "asphalt_p2_left"))
    # Block 4: Between pothole 2 and 3
    lines.append(make_box(8.30, 12.15, ROAD_Y_MIN, ROAD_Y_MAX, "asphalt_sec4"))
    # Pothole 3 zone (12.15 to 12.85)
    lines.append(make_box(12.15, 12.85, -1.10, -0.70, "asphalt_p3_right"))
    lines.append(make_box(12.15, 12.85, -0.20, 1.10, "asphalt_p3_left"))
    # Block 5: After pothole 3 to end
    lines.append(make_box(12.85, ROAD_X_MAX, ROAD_Y_MIN, ROAD_Y_MAX, "asphalt_sec5"))

    # 8. Pothole PROTO instances placed inside the recesses
    for p in potholes:
        pz = PIT_FLOOR_Z + POTHOLE_DEPTH / 2.0
        lines.append(f"Pothole {{")
        lines.append(f"  translation {p['x']} {p['y']} {pz:.4f}")
        lines.append(f"  name \"{p['id']}\"")
        lines.append(f"  sizeX {p['sx']}")
        lines.append(f"  sizeY {p['sy']}")
        lines.append(f"  depth {POTHOLE_DEPTH}")
        lines.append(f"}}")
        lines.append("")

    # 9. Flat Oil Stain Decoy (flush with asphalt at z = 0.0605)
    lines.append("Solid {")
    lines.append(f"  translation {stain['x']} {stain['y']} 0.0605")
    lines.append(f"  name \"{stain['id']}\"")
    lines.append("  children [")
    lines.append("    Shape {")
    lines.append("      appearance PBRAppearance {")
    lines.append("        baseColor 0.05 0.05 0.05")
    lines.append("        roughness 0.3")
    lines.append("        metalness 0.1")
    lines.append("      }")
    lines.append(f"      geometry Box {{ size {stain['sx']} {stain['sy']} 0.001 }}")
    lines.append("    }")
    lines.append("  ]")
    lines.append("}")
    lines.append("")

    # 10. Road Markings (White edge lines & dashed yellow center line)
    # White edge lines: y = -1.05 and y = +1.05
    for y_edge, name in [(-1.05, "edge_line_right"), (1.05, "edge_line_left")]:
        lines.append("Solid {")
        lines.append(f"  translation 10.0 {y_edge} 0.061")
        lines.append(f"  name \"{name}\"")
        lines.append("  children [")
        lines.append("    Shape {")
        lines.append("      appearance PBRAppearance { baseColor 0.95 0.95 0.95 roughness 0.5 }")
        lines.append("      geometry Box { size 24.0 0.08 0.001 }")
        lines.append("    }")
        lines.append("  ]")
        lines.append("}")
        lines.append("")

    # Dashed yellow center line (segments along y = 0.0 from -2 to 22)
    # 1.0 m dash, 1.0 m gap
    dash_idx = 0
    curr_x = ROAD_X_MIN + 0.5
    while curr_x + 1.0 <= ROAD_X_MAX:
        lines.append("Solid {")
        lines.append(f"  translation {curr_x + 0.5:.2f} 0.0 0.061")
        lines.append(f"  name \"center_dash_{dash_idx}\"")
        lines.append("  children [")
        lines.append("    Shape {")
        lines.append("      appearance PBRAppearance { baseColor 0.95 0.85 0.10 roughness 0.5 }")
        lines.append("      geometry Box { size 1.0 0.10 0.001 }")
        lines.append("    }")
        lines.append("  ]")
        lines.append("}")
        lines.append("")
        dash_idx += 1
        curr_x += 2.0

    # 11. Robot "pothole_robot"
    # Robot dimensions: body 0.30 x 0.20 x 0.06 m
    # Wheel radius: 0.06 m, cylinder rotated 90 deg about x inside Transform
    # Robot initial position: x = 0.0, y = -0.50, z = 0.12 (road is at z = 0.06, wheel radius 0.06)
    lines.append("Robot {")
    lines.append("  translation 0.0 -0.50 0.12")
    lines.append("  rotation 0 0 1 0")
    lines.append("  name \"pothole_robot\"")
    lines.append("  controller \"pothole_controller\"")
    lines.append("  synchronization TRUE")
    lines.append("  children [")

    # Chassis Shape & Bounding
    lines.append("    # Robot Chassis")
    lines.append("    Transform {")
    lines.append("      translation 0 0 0")
    lines.append("      children [")
    lines.append("        Shape {")
    lines.append("          appearance PBRAppearance {")
    lines.append("            baseColor 0.15 0.35 0.65")
    lines.append("            roughness 0.4")
    lines.append("            metalness 0.5")
    lines.append("          }")
    lines.append("          geometry Box { size 0.30 0.20 0.06 }")
    lines.append("        }")
    lines.append("      ]")
    lines.append("    }")

    # GPS & IMU
    lines.append("    # Navigation Sensors")
    lines.append("    GPS {")
    lines.append("      name \"gps\"")
    lines.append("      translation 0 0 0.03")
    lines.append("    }")
    lines.append("    InertialUnit {")
    lines.append("      name \"imu\"")
    lines.append("      translation 0 0 0.03")
    lines.append("    }")

    # Front Camera: pitched down 0.45 rad, 160x120, FOV 1.0
    # Pitch down in ENU: rotation about Y axis by 0.45 rad
    lines.append("    # Perception Camera")
    lines.append("    Camera {")
    lines.append("      name \"camera\"")
    lines.append("      translation 0.15 0.0 0.06")
    lines.append("      rotation 0 1 0 0.45")
    lines.append("      fieldOfView 1.0")
    lines.append("      width 160")
    lines.append("      height 120")
    lines.append("    }")

    # Warning LEDs (Red and Green)
    lines.append("    # Warning LEDs")
    lines.append("    LED {")
    lines.append("      name \"led_red\"")
    lines.append("      translation 0.14 0.08 0.04")
    lines.append("      children [")
    lines.append("        Shape {")
    lines.append("          appearance PBRAppearance { baseColor 0.8 0.1 0.1 roughness 0.2 }")
    lines.append("          geometry Sphere { radius 0.015 }")
    lines.append("        }")
    lines.append("      ]")
    lines.append("      color [ 1 0 0 ]")
    lines.append("    }")
    lines.append("    LED {")
    lines.append("      name \"led_green\"")
    lines.append("      translation 0.14 -0.08 0.04")
    lines.append("      children [")
    lines.append("        Shape {")
    lines.append("          appearance PBRAppearance { baseColor 0.1 0.8 0.1 roughness 0.2 }")
    lines.append("          geometry Sphere { radius 0.015 }")
    lines.append("        }")
    lines.append("      ]")
    lines.append("      color [ 0 1 0 ]")
    lines.append("    }")

    # Speaker
    lines.append("    # Warning Speaker / Buzzer")
    lines.append("    Speaker {")
    lines.append("      name \"speaker\"")
    lines.append("      translation 0.0 0.0 0.04")
    lines.append("    }")

    # Display (LCD on rear mast facing backwards)
    lines.append("    # Rear LCD Display Mast")
    lines.append("    Transform {")
    lines.append("      translation -0.12 0.0 0.08")
    lines.append("      children [")
    lines.append("        # Mast post")
    lines.append("        Shape {")
    lines.append("          appearance PBRAppearance { baseColor 0.2 0.2 0.2 roughness 0.5 }")
    lines.append("          geometry Box { size 0.02 0.02 0.10 }")
    lines.append("        }")
    lines.append("        # LCD Screen casing facing rear (-x)")
    lines.append("        Transform {")
    lines.append("          translation -0.01 0.0 0.05")
    lines.append("          children [")
    lines.append("            Shape {")
    lines.append("              appearance PBRAppearance { baseColor 0.05 0.05 0.05 roughness 0.3 }")
    lines.append("              geometry Box { size 0.01 0.14 0.08 }")
    lines.append("            }")
    lines.append("          ]")
    lines.append("        }")
    lines.append("      ]")
    lines.append("    }")
    lines.append("    Display {")
    lines.append("      name \"lcd\"")
    lines.append("      width 256")
    lines.append("      height 128")
    lines.append("    }")

    # 4 Wheels using HingeJoint (axis 0 1 0)
    # Wheel geometry: Cylinder rotated 90 deg about X inside Transform
    wheel_configs = [
        {"name": "wheel_fl", "x": 0.10, "y": 0.11, "sensor": "wheel_fl_sensor"},
        {"name": "wheel_fr", "x": 0.10, "y": -0.11, "sensor": "wheel_fr_sensor"},
        {"name": "wheel_rl", "x": -0.10, "y": 0.11, "sensor": "wheel_rl_sensor"},
        {"name": "wheel_rr", "x": -0.10, "y": -0.11, "sensor": "wheel_rr_sensor"},
    ]

    for w in wheel_configs:
        lines.append(f"    # Wheel Joint: {w['name']}")
        lines.append(f"    HingeJoint {{")
        lines.append(f"      jointParameters HingeJointParameters {{")
        lines.append(f"        axis 0 1 0")
        lines.append(f"        anchor {w['x']} {w['y']} 0.0")
        lines.append(f"      }}")
        lines.append(f"      device [")
        lines.append(f"        RotationalMotor {{")
        lines.append(f"          name \"{w['name']}\"")
        lines.append(f"        }}")
        lines.append(f"        PositionSensor {{")
        lines.append(f"          name \"{w['sensor']}\"")
        lines.append(f"        }}")
        lines.append(f"      ]")
        lines.append(f"      endPoint Solid {{")
        lines.append(f"        translation {w['x']} {w['y']} 0.0")
        lines.append(f"        rotation 0 1 0 0")
        lines.append(f"        children [")
        lines.append(f"          Transform {{")
        lines.append(f"            rotation 1 0 0 1.57079632679")
        lines.append(f"            children [")
        lines.append(f"              Shape {{")
        lines.append(f"                appearance PBRAppearance {{")
        lines.append(f"                  baseColor 0.1 0.1 0.1")
        lines.append(f"                  roughness 0.9")
        lines.append(f"                }}")
        lines.append(f"                geometry Cylinder {{")
        lines.append(f"                  height 0.03")
        lines.append(f"                  radius 0.06")
        lines.append(f"                }}")
        lines.append(f"              }}")
        lines.append(f"            ]")
        lines.append(f"          }}")
        lines.append(f"        ]")
        lines.append(f"        boundingObject Transform {{")
        lines.append(f"          rotation 1 0 0 1.57079632679")
        lines.append(f"          children [")
        lines.append(f"            Cylinder {{ height 0.03 radius 0.06 }}")
        lines.append(f"          ]")
        lines.append(f"        }}")
        lines.append(f"        physics Physics {{")
        lines.append(f"          density -1")
        lines.append(f"          mass 0.4")
        lines.append(f"        }}")
        lines.append(f"      }}")
        lines.append(f"    }}")

    lines.append("  ]")
    lines.append("  boundingObject Transform {")
    lines.append("    translation 0 0 0")
    lines.append("    children [ Box { size 0.30 0.20 0.06 } ]")
    lines.append("  }")
    lines.append("  physics Physics {")
    lines.append("    density -1")
    lines.append("    mass 3.5")
    lines.append("    centerOfMass [ 0 0 -0.01 ]")
    lines.append("  }")
    lines.append("}")
    lines.append("")

    content = "\n".join(lines)
    output_path.write_text(content, encoding="utf-8")
    print(f"Generated world file successfully at: {output_path} ({len(lines)} lines, Webots {version})")
    return output_path


def main():
    parser = argparse.ArgumentParser(description="Generate Webots pothole road world file.")
    parser.add_argument("--version", type=str, default="R2025a", help="Webots version header (default: R2025a)")
    parser.add_argument("--output", type=Path, default=None, help="Output .wbt file path")
    args = parser.parse_args()

    generate_world(version=args.version, output_path=args.output)


if __name__ == "__main__":
    main()
