#!/usr/bin/env python3
"""
tools/summarize_log.py - Analyzes results/detection_log.csv and writes results/test_summary.csv.

Evaluates the latest run against 5 benchmark test cases:
1. Clear road (no false alarms on pristine road before first pothole)
2. Pothole detected (stops before pothole, triggers alarm)
3. Multiple potholes with no repeated alert (stops for subsequent potholes, no duplicate alerts, ignores opposite lane)
4. Reset/resume (successfully resumes from STOPPED state upon explicit command or auto-resume)
5. Detection disabled/unavailable (fail-safe stop, never shows ROAD CLEAR or detects hazards)

Outputs strictly: PASS, FAIL, or NOT RUN (never PASS by default).
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from pathlib import Path


def parse_latest_run(log_path: str):
    if not os.path.exists(log_path) or os.path.getsize(log_path) == 0:
        return None, []

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        return None, []

    # Get the run_id of the final row
    latest_run_id = rows[-1].get("run_id")
    latest_rows = [r for r in rows if r.get("run_id") == latest_run_id]
    return latest_run_id, latest_rows


def evaluate_run(rows):
    """
    Evaluates log rows for the 5 benchmark cases.
    Returns list of dicts: [ { "id": 1, "name": ..., "status": ..., "details": ... }, ... ]
    """
    results = [
        {"id": 1, "name": "Clear road driving", "status": "NOT RUN", "details": "No data found"},
        {"id": 2, "name": "Pothole detected & safe stop", "status": "NOT RUN", "details": "No data found"},
        {"id": 3, "name": "Multiple potholes & no duplicates", "status": "NOT RUN", "details": "No data found"},
        {"id": 4, "name": "Reset / resume functionality", "status": "NOT RUN", "details": "No data found"},
        {"id": 5, "name": "Detection disabled fail-safe", "status": "NOT RUN", "details": "No data found"},
    ]

    if not rows:
        return results

    # Helper parses
    def get_float(val, default=0.0):
        try:
            return float(val)
        except (ValueError, TypeError):
            return default

    # Classify events
    start_row = next((r for r in rows if r.get("event") == "RUN_START"), None)
    mode = start_row.get("mode") if start_row else rows[0].get("mode", "sim")

    clear_events = [r for r in rows if r.get("event") == "CLEAR_ROAD"]
    detected_events = [r for r in rows if r.get("event") == "POTHOLE_DETECTED"]
    resume_events = [r for r in rows if r.get("event") == "RESUME"]
    unavail_events = [r for r in rows if r.get("event") == "DETECTION_UNAVAILABLE"]
    blind_events = [r for r in rows if r.get("event") == "NOT_MONITORING"]

    max_x = max((get_float(r.get("robot_x")) for r in rows), default=0.0)

    # -------------------------------------------------------------
    # Case 1: Clear road
    # -------------------------------------------------------------
    # Evaluated if robot drove past x=1.0m
    if max_x >= 1.0:
        # Check if any detection occurred before x = 2.0m (well before first pothole at x=4.0m)
        false_alarms = [
            r for r in detected_events if get_float(r.get("robot_x")) < 2.0
        ]
        if false_alarms:
            results[0]["status"] = "FAIL"
            results[0]["details"] = f"False detection occurred prematurely on clear road at x={false_alarms[0].get('robot_x')}m"
        elif clear_events:
            results[0]["status"] = "PASS"
            results[0]["details"] = f"Pristine road traversed up to x={max_x:.1f}m without false alarms"
        else:
            results[0]["status"] = "FAIL"
            results[0]["details"] = "Traversed road but no clear road events recorded"

    # -------------------------------------------------------------
    # Case 2: Pothole detected
    # -------------------------------------------------------------
    if detected_events:
        first_det = detected_events[0]
        det_dist = get_float(first_det.get("distance_m"))
        stopped_state = first_det.get("robot_state") == "STOPPED_POTHOLE"

        if stopped_state and 0.0 <= det_dist <= 1.2:
            results[1]["status"] = "PASS"
            results[1]["details"] = f"Pothole '{first_det.get('pothole_id')}' detected at dist {det_dist:.2f}m; robot stopped"
        else:
            results[1]["status"] = "FAIL"
            results[1]["details"] = f"Detected but improper stop state or distance: {det_dist}m, state={first_det.get('robot_state')}"
    elif max_x >= 4.5 and mode != "off":
        results[1]["status"] = "FAIL"
        results[1]["details"] = f"Robot passed pothole 1 (x=4.0m, reached x={max_x:.1f}m) without detecting it"

    # -------------------------------------------------------------
    # Case 3: Multiple potholes with no repeated alert
    # -------------------------------------------------------------
    pothole_id_counts = defaultdict(int)
    detected_opposite = False

    for r in detected_events:
        pid = r.get("pothole_id")
        pothole_id_counts[pid] += 1
        if pid == "pothole_4":
            detected_opposite = True

    if detected_opposite:
        results[2]["status"] = "FAIL"
        results[2]["details"] = "FAILED: Opposite-lane hazard 'pothole_4' was erroneously triggered!"
    elif len(pothole_id_counts) >= 2:
        duplicates = [pid for pid, count in pothole_id_counts.items() if count > 1]
        if duplicates:
            results[2]["status"] = "FAIL"
            results[2]["details"] = f"FAILED: Repeated alert for same pothole ID(s): {duplicates}"
        else:
            results[2]["status"] = "PASS"
            results[2]["details"] = f"Successfully detected {len(pothole_id_counts)} distinct hazards with 0 duplicate alerts"
    elif max_x >= 9.0 and mode != "off":
        results[2]["status"] = "FAIL"
        results[2]["details"] = f"Traversed multiple pothole locations (reached x={max_x:.1f}m) but only detected {len(pothole_id_counts)} hazards"

    # -------------------------------------------------------------
    # Case 4: Reset / resume
    # -------------------------------------------------------------
    if resume_events:
        # Check if robot moved after resume
        first_resume = resume_events[0]
        resume_sim_time = get_float(first_resume.get("sim_time_s"))
        subsequent_rows = [r for r in rows if get_float(r.get("sim_time_s")) > resume_sim_time]
        subsequent_x = max((get_float(r.get("robot_x")) for r in subsequent_rows), default=0.0)

        if subsequent_x > get_float(first_resume.get("robot_x")) + 0.2:
            results[3]["status"] = "PASS"
            results[3]["details"] = f"Resumed at sim_time {resume_sim_time:.1f}s and continued advancing to x={subsequent_x:.1f}m"
        else:
            results[3]["status"] = "FAIL"
            results[3]["details"] = "Resume event logged but robot failed to advance"
    elif len(detected_events) >= 1 and max_x < 4.5:
        # Pothole detected and stopped, but resume was never commanded
        results[3]["status"] = "NOT RUN"
        results[3]["details"] = "Robot remains safely stopped at first pothole; waiting for resume"

    # -------------------------------------------------------------
    # Case 5: Detection disabled / unavailable fail-safe
    # -------------------------------------------------------------
    if mode == "off" or unavail_events or blind_events:
        # Check that it NEVER showed ROAD CLEAR or detected a pothole
        has_clear = len(clear_events) > 0
        has_detect = len(detected_events) > 0

        if has_clear or has_detect:
            results[4]["status"] = "FAIL"
            results[4]["details"] = "FAILED: Displayed ROAD CLEAR or detected pothole while detection mode was OFF"
        elif unavail_events:
            results[4]["status"] = "PASS"
            results[4]["details"] = "Fail-safe engaged: vehicle immobilized and DETECTION_UNAVAILABLE logged"
        elif blind_events:
            results[4]["status"] = "PASS"
            results[4]["details"] = "Blind override active: robot cruised under NOT_MONITORING without false clear status"
        else:
            results[4]["status"] = "FAIL"
            results[4]["details"] = "Mode was off but neither fail-safe nor blind event was logged"

    return results


def main():
    parser = argparse.ArgumentParser(description="Evaluate simulation detection log against benchmark test cases.")
    parser.add_argument("--log", type=str, default=None, help="Path to detection_log.csv")
    parser.add_argument("--summary", type=str, default=None, help="Path to write test_summary.csv")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    log_path = args.log or str(project_root / "results" / "detection_log.csv")
    summary_path = args.summary or str(project_root / "results" / "test_summary.csv")

    run_id, rows = parse_latest_run(log_path)
    print("=" * 70)
    print(" Smart Pothole Detection Robot - Benchmark Test Summary")
    print("=" * 70)

    if not run_id:
        print(f"[WARN] No simulation runs found in {log_path}.")
        evaluations = evaluate_run([])
    else:
        print(f"Evaluated Run ID: {run_id} ({len(rows)} logged entries)")
        evaluations = evaluate_run(rows)

    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["test_case_id", "test_case_name", "status", "details"])
        writer.writeheader()
        for ev in evaluations:
            writer.writerow({
                "test_case_id": ev["id"],
                "test_case_name": ev["name"],
                "status": ev["status"],
                "details": ev["details"],
            })

    # Print pretty summary table
    for ev in evaluations:
        st = ev["status"]
        if st == "PASS":
            col = "\033[92m"
        elif st == "FAIL":
            col = "\033[91m"
        else:
            col = "\033[93m"
        reset = "\033[0m"
        print(f"Case {ev['id']}: {ev['name']:<35} -> {col}{st:<8}{reset} | {ev['details']}")

    print("=" * 70)
    print(f"Summary written to: {summary_path}")


if __name__ == "__main__":
    main()
