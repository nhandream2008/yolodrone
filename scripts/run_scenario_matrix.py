#!/usr/bin/env python3
"""Run deterministic offline fault-injection scenarios for the LiDAR-only planner.

This is an evidence generator, not a flight runner: it imports neither MAVSDK,
Gazebo, ROS, YOLO, nor process-control code.  Each scenario uses the existing
offline dynamics model with a fixed seed and records pass/fail metrics in JSON.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from offline_slalom import OfflineDynamics, config_for, simulate, slalom
from reproducibility import environment_manifest


SCENARIOS = {
    "nominal": OfflineDynamics(),
    "lidar_noise": OfflineDynamics(lidar_noise_std_m=0.08),
    "intermittent_dropout": OfflineDynamics(single_scan_drop_probability=0.10),
    "burst_dropout": OfflineDynamics(burst_start_probability=0.025, burst_length=3),
    "jittered_control": OfflineDynamics(update_jitter_fraction=0.25),
    "combined_faults": OfflineDynamics(
        lidar_noise_std_m=0.08, single_scan_drop_probability=0.08,
        burst_start_probability=0.02, burst_length=3, update_jitter_fraction=0.20,
    ),
}


def run_matrix(*, seeds: range, speed: float, gates: int) -> dict:
    """Execute every deterministic scenario/seed pair and aggregate safety metrics."""
    course = slalom(gates=gates)
    config = config_for(course, speed=speed)
    records = []
    for name, dynamics in SCENARIOS.items():
        for seed in seeds:
            result = simulate(course.boxes, config, seed=seed, dynamics=dynamics)
            passed = result.reached_goal and not result.collided
            records.append({
                "scenario": name,
                "seed": seed,
                "passed": passed,
                "reached_goal": result.reached_goal,
                "collided": result.collided,
                "time_to_goal_s": result.time_to_goal,
                "min_segment_clearance_m": result.min_segment_clearance,
                "vehicle_radius_m": dynamics.vehicle_radius_m,
                "clearance_margin_m": result.min_segment_clearance-dynamics.vehicle_radius_m,
                "forward_stalls": result.forward_stalls,
                "direction_flips": result.direction_flips,
                "dropped_scans": result.dropped_scans,
                "max_commanded_yaw_rate_deg_s": result.max_abs_yaw_rate,
                "dynamics": asdict(dynamics),
            })
    grouped = {}
    for record in records:
        grouped.setdefault(record["scenario"], []).append(record)
    summary = {}
    for name, runs in grouped.items():
        clearances = [run["min_segment_clearance_m"] for run in runs]
        durations = [run["time_to_goal_s"] for run in runs if run["time_to_goal_s"] is not None]
        summary[name] = {
            "runs": len(runs),
            "passed": sum(run["passed"] for run in runs),
            "pass_rate": sum(run["passed"] for run in runs)/len(runs),
            "collision_count": sum(run["collided"] for run in runs),
            "worst_segment_clearance_m": min(clearances),
            "mean_time_to_goal_s": sum(durations)/len(durations) if durations else None,
            "max_dropped_scans": max(run["dropped_scans"] for run in runs),
            "max_direction_flips": max(run["direction_flips"] for run in runs),
        }
    return {
        "schema_version": 1,
        "scope": "offline-only; no PX4, Gazebo, MAVSDK, ROS, or YOLO control path",
        "parameters": {"seeds": list(seeds), "speed_m_s": speed, "gates": gates},
        "reproducibility": environment_manifest(ROOT),
        "acceptance": {
            "required": "Every run must reach the goal without a segment collision",
            "all_passed": all(record["passed"] for record in records),
        },
        "scenarios": summary,
        "runs": records,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=10, help="fixed seeds 0..N-1 (1..100)")
    parser.add_argument("--speed", type=float, default=1.5, help="offline cruise cap (0,3]")
    parser.add_argument("--gates", type=int, default=4, help="alternating gates (1..10)")
    parser.add_argument("--output", default="logs/offline-scenario-matrix.json")
    args = parser.parse_args(argv)
    if not 1 <= args.seeds <= 100 or not 0 < args.speed <= 3 or not 1 <= args.gates <= 10:
        parser.error("seeds 1..100, speed (0,3], gates 1..10")
    report = run_matrix(seeds=range(args.seeds), speed=args.speed, gates=args.gates)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(f"Saved: {output.resolve()}")
    print(f"Acceptance: {'PASS' if report['acceptance']['all_passed'] else 'FAIL'}")
    for name, summary in report["scenarios"].items():
        print(f"{name}: {summary['passed']}/{summary['runs']} pass, "
              f"worst clearance {summary['worst_segment_clearance_m']:.3f} m")
    return 0 if report["acceptance"]["all_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
