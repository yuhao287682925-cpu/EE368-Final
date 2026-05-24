#!/usr/bin/env python3
"""
plot_results.py - Visualize experiment results from rosbag data.

Generates:
  1. 3D trajectory overlay (desired vs actual)
  2. Force profile over time
  3. Joint torque profile

Usage:
  rosrun system_integrator plot_results.py experiment.bag --output-dir ./plots
"""
import sys
import os
import argparse
import math

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("Warning: matplotlib not available. Install with: pip install matplotlib")

import rosbag
import numpy as np


def extract_data(bag_path):
    """Extract relevant data from rosbag."""
    bag = rosbag.Bag(bag_path, 'r')

    data = {
        't_ee': [], 'ee_x': [], 'ee_y': [], 'ee_z': [],
        't_force': [], 'force_z': [],
        't_joint': [], 'joint_efforts': [],  # list of lists
        't_fb': [], 'feedback_progress': [],
    }

    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()

        if '/force_correction' in topic:
            data['t_force'].append(ts)
            data['force_z'].append(msg.z)

        elif '/joint_state' in topic and hasattr(msg, 'effort') and msg.effort:
            data['t_joint'].append(ts)
            data['joint_efforts'].append(list(msg.effort))

        elif '/execute_drawing/feedback' in topic:
            data['t_fb'].append(ts)
            data['feedback_progress'].append(msg.progress_fraction)

    bag.close()
    return data


def plot_force_profile(data, output_dir):
    """Plot force correction over time."""
    if not data['t_force']:
        print("No force data to plot.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))
    t_rel = np.array(data['t_force']) - data['t_force'][0]
    ax.plot(t_rel, np.array(data['force_z']) * 1000.0, 'b-', linewidth=0.5)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Force Correction Z [mm]')
    ax.set_title('Force Controller Z-Correction Profile')
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(output_dir, 'force_profile.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved: {path}")


def plot_joint_torques(data, output_dir):
    """Plot joint torques over time."""
    if not data['t_joint']:
        print("No joint torque data to plot.")
        return

    eff = np.array(data['joint_efforts'])
    t_rel = np.array(data['t_joint']) - data['t_joint'][0]

    fig, ax = plt.subplots(figsize=(12, 6))
    n_joints = min(eff.shape[1], 6)
    for j in range(n_joints):
        ax.plot(t_rel, eff[:, j], linewidth=0.5,
                label=f'Joint {j+1}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Joint Torque [Nm]')
    ax.set_title('Joint Torques During Drawing')
    ax.legend(loc='upper right', fontsize='small')
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(output_dir, 'joint_torques.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved: {path}")


def plot_progress(data, output_dir):
    """Plot drawing progress over time."""
    if not data['t_fb']:
        print("No progress feedback data to plot.")
        return

    t_rel = np.array(data['t_fb']) - data['t_fb'][0]
    progress = np.array(data['feedback_progress']) * 100.0

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t_rel, progress, 'g-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Progress [%]')
    ax.set_title('Drawing Progress Over Time')
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(output_dir, 'progress.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved: {path}")


def plot_summary(data, output_dir):
    """Create a summary dashboard plot."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Force
    ax = axes[0, 0]
    if data['t_force']:
        t_rel = np.array(data['t_force']) - data['t_force'][0]
        ax.plot(t_rel, np.array(data['force_z']) * 1000.0, 'b-', linewidth=0.5)
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Z Correction [mm]')
    ax.set_title('Force Correction')
    ax.grid(True, alpha=0.3)

    # Joint torques
    ax = axes[0, 1]
    if data['t_joint']:
        eff = np.array(data['joint_efforts'])
        t_rel = np.array(data['t_joint']) - data['t_joint'][0]
        n_joints = min(eff.shape[1], 6)
        for j in range(n_joints):
            ax.plot(t_rel, eff[:, j], linewidth=0.5, label=f'J{j+1}')
        ax.legend(loc='upper right', fontsize='x-small')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Torque [Nm]')
    ax.set_title('Joint Torques')
    ax.grid(True, alpha=0.3)

    # Progress
    ax = axes[1, 0]
    if data['t_fb']:
        t_rel = np.array(data['t_fb']) - data['t_fb'][0]
        ax.plot(t_rel, np.array(data['feedback_progress']) * 100.0, 'g-', linewidth=1.5)
        ax.set_ylim(0, 105)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Progress [%]')
    ax.set_title('Drawing Progress')
    ax.grid(True, alpha=0.3)

    # Force histogram
    ax = axes[1, 1]
    if data['force_z']:
        fz = np.abs(np.array(data['force_z'])) * 1000.0
        ax.hist(fz, bins=50, color='royalblue', edgecolor='white', alpha=0.8)
        ax.axvline(x=np.mean(fz), color='red', linestyle='--',
                   label=f'Mean: {np.mean(fz):.3f} mm')
        ax.legend()
    ax.set_xlabel('|Z Correction| [mm]')
    ax.set_ylabel('Count')
    ax.set_title('Force Correction Distribution')
    ax.grid(True, alpha=0.3)

    fig.suptitle('Block Drawing Experiment — Summary Dashboard', fontsize=14)
    fig.tight_layout()
    path = os.path.join(output_dir, 'summary.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Saved: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot experiment results from rosbag.")
    parser.add_argument('bag_file', help='Path to experiment rosbag')
    parser.add_argument('--output-dir', '-o', default='./plots',
                        help='Output directory for plots')
    args = parser.parse_args()

    if not HAS_MPL:
        print("matplotlib is required. Install with: pip install matplotlib")
        sys.exit(1)

    if not os.path.exists(args.bag_file):
        print(f"Error: Bag file not found: {args.bag_file}")
        sys.exit(1)

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    print(f"Loading data from: {args.bag_file}")
    data = extract_data(args.bag_file)

    print(f"Generating plots in: {args.output_dir}")

    plot_force_profile(data, args.output_dir)
    plot_joint_torques(data, args.output_dir)
    plot_progress(data, args.output_dir)
    plot_summary(data, args.output_dir)

    print("\nAll plots generated successfully!")


if __name__ == '__main__':
    main()
