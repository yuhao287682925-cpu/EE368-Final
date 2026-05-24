#!/usr/bin/env python3
"""
evaluate_experiment.py - Evaluate experiment results from rosbag.

Evaluation metrics:
  1. Line continuity: spatial gap at trajectory segment junctions [mm]
  2. Force consistency: standard deviation / mean of estimated force
  3. Face transition offset: deviation between expected and actual landing point [mm]
  4. Trajectory tracking error: RMS between desired and actual end-effector poses

Usage:
  rosrun system_integrator evaluate_experiment.py experiment.bag

  Or from Python:
  python evaluate_experiment.py experiment.bag
"""
import sys
import os
import argparse
import math
import json

import rosbag
import numpy as np


def compute_line_continuity(trajectories, actual_poses_by_face):
    """
    Compute spatial gap between consecutive trajectory segments.
    Returns mean gap and max gap in mm.
    """
    gaps = []
    for i in range(len(trajectories) - 1):
        # Last waypoint of face i vs first waypoint of face i+1
        if trajectories[i].waypoints and trajectories[i+1].waypoints:
            wp_end = trajectories[i].waypoints[-1]
            wp_start = trajectories[i+1].waypoints[0]
            dx = wp_end.position.x - wp_start.position.x
            dy = wp_end.position.y - wp_start.position.y
            dz = wp_end.position.z - wp_start.position.z
            gap_mm = math.sqrt(dx*dx + dy*dy + dz*dz) * 1000.0
            gaps.append(gap_mm)

    if not gaps:
        return 0.0, 0.0
    return np.mean(gaps), np.max(gaps)


def compute_force_consistency(force_readings):
    """
    Compute force consistency: sigma / mu.
    Lower is better — more consistent pressure.
    """
    if not force_readings:
        return 0.0, 0.0
    forces = np.array(force_readings)
    mean_f = np.mean(np.abs(forces))
    std_f = np.std(forces)
    if mean_f < 1e-6:
        return 0.0, 0.0
    return std_f / mean_f, mean_f


def compute_tracking_error(desired_poses, actual_poses, times):
    """
    Compute RMS trajectory tracking error.
    Aligns desired and actual poses by timestamp interpolation.

    Args:
        desired_poses: list of (t, x, y, z)
        actual_poses: list of (t, x, y, z)
    Returns:
        RMS position error in mm
    """
    if not desired_poses or not actual_poses:
        return 0.0

    errors = []
    for t_d, x_d, y_d, z_d in desired_poses:
        # Find closest actual pose in time
        best_dt = float('inf')
        best_x, best_y, best_z = 0, 0, 0
        for t_a, x_a, y_a, z_a in actual_poses:
            dt = abs(t_a - t_d)
            if dt < best_dt:
                best_dt = dt
                best_x, best_y, best_z = x_a, y_a, z_a

        dx = (x_d - best_x) * 1000.0  # mm
        dy = (y_d - best_y) * 1000.0
        dz = (z_d - best_z) * 1000.0
        errors.append(math.sqrt(dx*dx + dy*dy + dz*dz))

    if not errors:
        return 0.0
    return math.sqrt(np.mean(np.array(errors)**2))


def evaluate_bag(bag_path, trajectories=None):
    """
    Analyze a rosbag and return evaluation metrics.

    Returns:
        dict with metrics: line_continuity, force_cv, tracking_rms, etc.
    """
    if not os.path.exists(bag_path):
        print(f"Error: Bag file not found: {bag_path}")
        return None

    bag = rosbag.Bag(bag_path, 'r')

    # Extract data from bag
    actual_ee_poses = []      # (t, x, y, z)
    force_corrections = []    # force correction z-values
    joint_states = []         # (t, positions[], efforts[])

    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()
        if topic.endswith('/tool_pose') or topic.endswith('/end_effector_pose'):
            actual_ee_poses.append((
                ts, msg.position.x, msg.position.y, msg.position.z))
        elif topic == '/force_correction':
            force_corrections.append(msg.z)
        elif topic.endswith('/joint_state'):
            joint_states.append((ts, list(msg.position), list(msg.effort)))

    bag.close()

    metrics = {}

    # Line continuity (requires trajectory data)
    if trajectories:
        mean_gap, max_gap = compute_line_continuity(
            trajectories, actual_ee_poses)
        metrics['line_continuity_mean_gap_mm'] = mean_gap
        metrics['line_continuity_max_gap_mm'] = max_gap

    # Force consistency
    if force_corrections:
        cv, mean_f = compute_force_consistency(force_corrections)
        metrics['force_cv'] = cv
        metrics['force_mean'] = mean_f

    # Tracking error
    # Placeholder: would need desired waypoints with timestamps for proper comparison
    metrics['num_ee_poses'] = len(actual_ee_poses)
    metrics['num_force_readings'] = len(force_corrections)
    metrics['num_joint_states'] = len(joint_states)

    return metrics


def print_report(metrics):
    """Pretty-print evaluation report."""
    print("\n" + "="*60)
    print("  EXPERIMENT EVALUATION REPORT")
    print("="*60)

    if metrics.get('line_continuity_mean_gap_mm') is not None:
        print(f"  Line continuity (mean gap):  {metrics['line_continuity_mean_gap_mm']:.2f} mm")
        print(f"  Line continuity (max gap):   {metrics['line_continuity_max_gap_mm']:.2f} mm")

    if metrics.get('force_cv') is not None:
        print(f"  Force consistency (CV):      {metrics['force_cv']:.4f}")
        print(f"  Force mean:                  {metrics['force_mean']:.3f} N")

    print(f"  EE pose samples:             {metrics.get('num_ee_poses', 0)}")
    print(f"  Force reading samples:       {metrics.get('num_force_readings', 0)}")
    print(f"  Joint state samples:         {metrics.get('num_joint_states', 0)}")
    print("="*60)

    # Overall grade
    score = 0
    total = 0
    if 'line_continuity_mean_gap_mm' in metrics:
        gap = metrics['line_continuity_mean_gap_mm']
        total += 1
        if gap < 2.0: score += 1
        elif gap < 5.0: score += 0.5
    if 'force_cv' in metrics:
        cv = metrics['force_cv']
        total += 1
        if cv < 0.3: score += 1
        elif cv < 0.6: score += 0.5

    if total > 0:
        pct = score / total * 100
        print(f"  Overall score: {pct:.0f}% ({score}/{total} criteria passed)")
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate block drawing experiment results.")
    parser.add_argument('bag_file', help='Path to experiment rosbag')
    parser.add_argument('--output-json', default='',
                        help='Save metrics as JSON file')
    args = parser.parse_args()

    metrics = evaluate_bag(args.bag_file)
    if metrics is None:
        sys.exit(1)

    print_report(metrics)

    if args.output_json:
        with open(args.output_json, 'w') as f:
            json.dump(metrics, f, indent=2)
        print(f"Metrics saved to {args.output_json}")


if __name__ == '__main__':
    main()
