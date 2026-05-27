#!/usr/bin/env python3
"""Visualize generated cube drawing trajectories in 3D."""

import argparse
import csv
import os
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from cube_circle_trajectory import build_axis_aligned_cube_surfaces  # noqa: E402


Point3 = Tuple[float, float, float]


FACE_COLORS = {
    "top": "#1f77b4",
    "front": "#ff7f0e",
    "right": "#2ca02c",
    "back": "#d62728",
    "left": "#9467bd",
    "bottom": "#8c564b",
}


def read_waypoints(path: Path) -> List[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_point(row: dict) -> Point3:
    return float(row["x_m"]), float(row["y_m"]), float(row["z_m"])


def cube_vertices(top_center: Point3, side_length: float) -> List[Point3]:
    half = side_length / 2.0
    x, y, z_top = top_center
    z_bottom = z_top - side_length
    return [
        (x - half, y - half, z_bottom),
        (x + half, y - half, z_bottom),
        (x + half, y + half, z_bottom),
        (x - half, y + half, z_bottom),
        (x - half, y - half, z_top),
        (x + half, y - half, z_top),
        (x + half, y + half, z_top),
        (x - half, y + half, z_top),
    ]


def draw_cube(ax, top_center: Point3, side_length: float) -> None:
    vertices = cube_vertices(top_center, side_length)
    faces = [
        [vertices[i] for i in [4, 5, 6, 7]],
        [vertices[i] for i in [0, 1, 2, 3]],
        [vertices[i] for i in [0, 1, 5, 4]],
        [vertices[i] for i in [1, 2, 6, 5]],
        [vertices[i] for i in [2, 3, 7, 6]],
        [vertices[i] for i in [3, 0, 4, 7]],
    ]
    collection = Poly3DCollection(
        faces,
        facecolors="#d9e6f2",
        edgecolors="#555555",
        linewidths=0.8,
        alpha=0.16,
    )
    ax.add_collection3d(collection)

    surfaces = build_axis_aligned_cube_surfaces(top_center, side_length)
    for name, surface in surfaces.items():
        ax.text(
            surface.origin[0],
            surface.origin[1],
            surface.origin[2],
            name,
            fontsize=8,
            color="#333333",
        )


def set_axes_equal(ax, points: List[Point3], margin: float = 0.02) -> None:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    x_mid = (min(xs) + max(xs)) / 2.0
    y_mid = (min(ys) + max(ys)) / 2.0
    z_mid = (min(zs) + max(zs)) / 2.0
    radius = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) / 2.0
    radius = max(radius + margin, margin)
    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(z_mid - radius, z_mid + radius)
    ax.set_box_aspect((1, 1, 1))


def group_points(rows: List[dict]) -> Tuple[Dict[tuple, List[Point3]], Dict[tuple, List[Point3]]]:
    draw_groups: Dict[tuple, List[Point3]] = defaultdict(list)
    motion_groups: Dict[tuple, List[Point3]] = defaultdict(list)
    for row in rows:
        key = (row["face"], row.get("pattern", ""), row["stroke_id"])
        point = as_point(row)
        motion_groups[key].append(point)
        if row["phase"] == "draw" and row["pen_down"] == "1":
            draw_groups[key].append(point)
    return draw_groups, motion_groups


def plot_trajectory(
    rows: List[dict],
    output: Path,
    top_center: Point3,
    side_length: float,
    title: str,
) -> None:
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    draw_cube(ax, top_center, side_length)

    draw_groups, motion_groups = group_points(rows)
    all_points = cube_vertices(top_center, side_length)

    for key, points in motion_groups.items():
        if len(points) < 2:
            continue
        xs, ys, zs = zip(*points)
        ax.plot(xs, ys, zs, color="#888888", linewidth=0.6, linestyle="--", alpha=0.28)
        all_points.extend(points)

    labels_seen = set()
    for (face, pattern, _stroke_id), points in draw_groups.items():
        if len(points) < 2:
            continue
        color = FACE_COLORS.get(face, "#111111")
        label = f"{face}:{pattern}"
        xs, ys, zs = zip(*points)
        ax.plot(
            xs,
            ys,
            zs,
            color=color,
            linewidth=2.2,
            label=None if label in labels_seen else label,
        )
        labels_seen.add(label)
        ax.scatter(xs[0], ys[0], zs[0], color=color, s=18, marker="o")

    set_axes_equal(ax, all_points)
    ax.set_title(title)
    ax.set_xlabel("X base (m)")
    ax.set_ylabel("Y base (m)")
    ax.set_zlabel("Z base (m)")
    ax.view_init(elev=24, azim=-48)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot generated cube drawing trajectories as a 3D PNG."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("arm_drawing/generated/all_faces_patterns.csv"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("arm_drawing/generated/all_faces_patterns_3d.png"),
    )
    parser.add_argument("--top-center-x", type=float, default=0.280)
    parser.add_argument("--top-center-y", type=float, default=-0.032)
    parser.add_argument("--top-center-z", type=float, default=0.119)
    parser.add_argument("--side-length", type=float, default=0.060)
    parser.add_argument("--title", default="Cube Surface Drawing Trajectory")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = read_waypoints(args.input)
    if not rows:
        raise SystemExit(f"No trajectory rows found in {args.input}")

    plot_trajectory(
        rows=rows,
        output=args.output,
        top_center=(args.top_center_x, args.top_center_y, args.top_center_z),
        side_length=args.side_length,
        title=args.title,
    )
    draw_count = sum(1 for row in rows if row["phase"] == "draw" and row["pen_down"] == "1")
    print(f"Plotted {draw_count} draw points from {args.input}")
    print(f"Saved 3D visualization to {args.output}")


if __name__ == "__main__":
    main()
