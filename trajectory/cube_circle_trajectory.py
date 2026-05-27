#!/usr/bin/env python3
"""Generate drawing waypoints for 2D patterns on a cube face.

This is the first member-1 module: it owns the geometric side of the task.
It does not depend on EGO-Planner or ROS. The output CSV can be consumed by a
later ROS2 execution node or by an IK script.
"""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


FACE_NAMES = ("top", "front", "right", "back", "left", "bottom")
PATTERN_NAMES = ("circle", "triangle", "square", "star", "sun", "spiral")
DEFAULT_FACE_PATTERNS = {
    "top": "circle",
    "front": "triangle",
    "right": "star",
    "back": "spiral",
    "left": "sun",
    "bottom": "square",
}

Point2 = Tuple[float, float]
Polyline2D = List[Point2]
Pattern2D = List[Polyline2D]
Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class PlaneSurface:
    name: str
    origin: Vector3
    u_axis: Vector3
    v_axis: Vector3
    normal: Vector3
    u_min: float
    u_max: float
    v_min: float
    v_max: float

    def point_from_uv(self, u: float, v: float, normal_offset: float) -> Vector3:
        return (
            self.origin[0] + u * self.u_axis[0] + v * self.v_axis[0]
            + normal_offset * self.normal[0],
            self.origin[1] + u * self.u_axis[1] + v * self.v_axis[1]
            + normal_offset * self.normal[1],
            self.origin[2] + u * self.u_axis[2] + v * self.v_axis[2]
            + normal_offset * self.normal[2],
        )

    def contains_uv(self, u: float, v: float) -> bool:
        return self.u_min <= u <= self.u_max and self.v_min <= v <= self.v_max


@dataclass(frozen=True)
class Waypoint:
    index: int
    stroke_id: int
    phase: str
    face: str
    pattern: str
    xyz: Vector3
    normal: Vector3
    rpy_deg: Vector3
    pen_down: int


def build_axis_aligned_cube_surfaces(
    top_center: Vector3,
    side_length: float,
) -> dict:
    """Build a cube model whose top face center is known in the robot frame.

    The first experiment assumes the cube edges are parallel to the robot base
    frame axes. That matches the simplest known-pose case and keeps member 2's
    execution interface clean.
    """
    half = side_length / 2.0
    x, y, z_top = top_center

    return {
        "top": PlaneSurface(
            name="top",
            origin=(x, y, z_top),
            u_axis=(1.0, 0.0, 0.0),
            v_axis=(0.0, 1.0, 0.0),
            normal=(0.0, 0.0, 1.0),
            u_min=-half,
            u_max=half,
            v_min=-half,
            v_max=half,
        ),
        "front": PlaneSurface(
            name="front",
            origin=(x, y - half, z_top - half),
            u_axis=(1.0, 0.0, 0.0),
            v_axis=(0.0, 0.0, 1.0),
            normal=(0.0, -1.0, 0.0),
            u_min=-half,
            u_max=half,
            v_min=-half,
            v_max=half,
        ),
        "right": PlaneSurface(
            name="right",
            origin=(x + half, y, z_top - half),
            u_axis=(0.0, 1.0, 0.0),
            v_axis=(0.0, 0.0, 1.0),
            normal=(1.0, 0.0, 0.0),
            u_min=-half,
            u_max=half,
            v_min=-half,
            v_max=half,
        ),
        "back": PlaneSurface(
            name="back",
            origin=(x, y + half, z_top - half),
            u_axis=(-1.0, 0.0, 0.0),
            v_axis=(0.0, 0.0, 1.0),
            normal=(0.0, 1.0, 0.0),
            u_min=-half,
            u_max=half,
            v_min=-half,
            v_max=half,
        ),
        "left": PlaneSurface(
            name="left",
            origin=(x - half, y, z_top - half),
            u_axis=(0.0, -1.0, 0.0),
            v_axis=(0.0, 0.0, 1.0),
            normal=(-1.0, 0.0, 0.0),
            u_min=-half,
            u_max=half,
            v_min=-half,
            v_max=half,
        ),
        "bottom": PlaneSurface(
            name="bottom",
            origin=(x, y, z_top - side_length),
            u_axis=(1.0, 0.0, 0.0),
            v_axis=(0.0, -1.0, 0.0),
            normal=(0.0, 0.0, -1.0),
            u_min=-half,
            u_max=half,
            v_min=-half,
            v_max=half,
        ),
    }


def sample_circle_uv(radius: float, samples: int) -> Iterable[Tuple[float, float]]:
    for i in range(samples):
        angle = 2.0 * math.pi * i / samples
        yield radius * math.cos(angle), radius * math.sin(angle)


def close_polyline(points: Polyline2D) -> Polyline2D:
    if not points:
        return []
    if points[0] == points[-1]:
        return points
    return points + [points[0]]


def regular_polygon(
    sides: int,
    radius: float,
    rotation_rad: float = math.pi / 2.0,
) -> Polyline2D:
    if sides < 3:
        raise ValueError("regular polygon needs at least 3 sides")
    return close_polyline(
        [
            (
                radius * math.cos(rotation_rad + 2.0 * math.pi * i / sides),
                radius * math.sin(rotation_rad + 2.0 * math.pi * i / sides),
            )
            for i in range(sides)
        ]
    )


def star_polyline(
    outer_radius: float,
    inner_radius: float,
    points: int = 5,
    rotation_rad: float = math.pi / 2.0,
) -> Polyline2D:
    if points < 3:
        raise ValueError("star needs at least 3 points")
    vertices = []
    for i in range(points * 2):
        radius = outer_radius if i % 2 == 0 else inner_radius
        angle = rotation_rad + math.pi * i / points
        vertices.append((radius * math.cos(angle), radius * math.sin(angle)))
    return close_polyline(vertices)


def make_pattern(
    name: str,
    radius: float,
    samples: int,
    ray_count: int,
) -> Pattern2D:
    """Return a 2D pattern in face-local coordinates.

    Each polyline is a separate pen stroke. The projection layer will lift the
    pen between strokes, which lets us draw discontinuous figures such as a sun.
    """
    if radius <= 0.0:
        raise ValueError("pattern radius must be positive")
    if samples < 12:
        raise ValueError("samples must be at least 12")

    name = name.lower()
    if name == "circle":
        return [close_polyline(list(sample_circle_uv(radius, samples)))]
    if name == "triangle":
        return [regular_polygon(3, radius)]
    if name == "square":
        return [regular_polygon(4, radius, rotation_rad=math.pi / 4.0)]
    if name == "star":
        return [star_polyline(radius, radius * 0.42)]
    if name == "sun":
        inner_radius = radius * 0.48
        ray_start = radius * 0.64
        strokes: Pattern2D = [
            close_polyline(list(sample_circle_uv(inner_radius, samples)))
        ]
        for i in range(ray_count):
            angle = 2.0 * math.pi * i / ray_count
            strokes.append(
                [
                    (ray_start * math.cos(angle), ray_start * math.sin(angle)),
                    (radius * math.cos(angle), radius * math.sin(angle)),
                ]
            )
        return strokes
    if name == "spiral":
        turns = 2.5
        points = []
        for i in range(samples):
            t = i / max(samples - 1, 1)
            angle = 2.0 * math.pi * turns * t
            local_radius = radius * t
            points.append(
                (local_radius * math.cos(angle), local_radius * math.sin(angle))
            )
        return [points]

    raise ValueError(
        f"unknown pattern {name!r}; choose {', '.join(PATTERN_NAMES)}"
    )


def resample_polyline(polyline: Polyline2D, spacing: float) -> Polyline2D:
    """Make point spacing more uniform for later robot execution."""
    if spacing <= 0.0 or len(polyline) < 2:
        return polyline

    resampled = [polyline[0]]
    for start, end in zip(polyline, polyline[1:]):
        du = end[0] - start[0]
        dv = end[1] - start[1]
        length = math.hypot(du, dv)
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(1, steps + 1):
            ratio = step / steps
            resampled.append((start[0] + ratio * du, start[1] + ratio * dv))
    return resampled


def validate_pattern_fits(surface: PlaneSurface, pattern: Pattern2D) -> None:
    for stroke_id, polyline in enumerate(pattern):
        for u, v in polyline:
            if not surface.contains_uv(u, v):
                raise ValueError(
                    f"pattern point ({u:.4f}, {v:.4f}) in stroke {stroke_id} "
                    f"does not fit inside face {surface.name} with limits "
                    f"u=[{surface.u_min:.4f},{surface.u_max:.4f}], "
                    f"v=[{surface.v_min:.4f},{surface.v_max:.4f}]"
                )


def generate_pattern_on_face(
    surface: PlaneSurface,
    pattern: Pattern2D,
    pattern_name: str,
    rpy_deg: Vector3,
    normal_offset: float,
    approach_height: float,
    index_start: int = 0,
    stroke_id_start: int = 0,
) -> List[Waypoint]:
    validate_pattern_fits(surface, pattern)
    waypoints: List[Waypoint] = []
    for stroke_id, polyline in enumerate(pattern):
        if not polyline:
            continue

        first_u, first_v = polyline[0]
        first_contact = surface.point_from_uv(first_u, first_v, normal_offset)
        first_approach = surface.point_from_uv(
            first_u, first_v, normal_offset + approach_height
        )

        waypoints.append(
            Waypoint(
                len(waypoints),
                stroke_id_start + stroke_id,
                "approach",
                surface.name,
                pattern_name,
                first_approach,
                surface.normal,
                rpy_deg,
                0,
            )
        )
        waypoints.append(
            Waypoint(
                len(waypoints),
                stroke_id_start + stroke_id,
                "touch_down",
                surface.name,
                pattern_name,
                first_contact,
                surface.normal,
                rpy_deg,
                0,
            )
        )

        for u, v in polyline:
            xyz = surface.point_from_uv(u, v, normal_offset)
            waypoints.append(
                Waypoint(
                    len(waypoints),
                    stroke_id_start + stroke_id,
                    "draw",
                    surface.name,
                    pattern_name,
                    xyz,
                    surface.normal,
                    rpy_deg,
                    1,
                )
            )

        waypoints.append(
            Waypoint(
                len(waypoints),
                stroke_id_start + stroke_id,
                "lift",
                surface.name,
                pattern_name,
                first_approach,
                surface.normal,
                rpy_deg,
                0,
            )
        )
    if index_start == 0:
        return waypoints

    return [
        Waypoint(
            index_start + waypoint.index,
            waypoint.stroke_id,
            waypoint.phase,
            waypoint.face,
            waypoint.pattern,
            waypoint.xyz,
            waypoint.normal,
            waypoint.rpy_deg,
            waypoint.pen_down,
        )
        for waypoint in waypoints
    ]


def parse_faces(raw_faces: str) -> List[str]:
    raw_faces = raw_faces.strip().lower()
    if raw_faces == "all":
        return list(FACE_NAMES)

    faces = [face.strip() for face in raw_faces.split(",") if face.strip()]
    if not faces:
        raise ValueError("at least one face must be selected")

    invalid = [face for face in faces if face not in FACE_NAMES]
    if invalid:
        raise ValueError(
            f"unknown face(s): {', '.join(invalid)}; choose {', '.join(FACE_NAMES)}"
        )
    return faces


def parse_face_patterns(raw_mapping: str) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    if not raw_mapping.strip():
        return mapping

    for item in raw_mapping.split(","):
        if not item.strip():
            continue
        if ":" not in item:
            raise ValueError(
                f"invalid face-pattern item {item!r}; expected face:pattern"
            )
        face, pattern = [part.strip().lower() for part in item.split(":", 1)]
        if face not in FACE_NAMES:
            raise ValueError(
                f"unknown face {face!r} in --face-patterns; choose {', '.join(FACE_NAMES)}"
            )
        if pattern not in PATTERN_NAMES:
            raise ValueError(
                f"unknown pattern {pattern!r} in --face-patterns; "
                f"choose {', '.join(PATTERN_NAMES)}"
            )
        mapping[face] = pattern
    return mapping


def resampled_pattern(
    pattern_name: str,
    radius: float,
    samples: int,
    ray_count: int,
    point_spacing: float,
) -> Pattern2D:
    pattern = make_pattern(
        name=pattern_name,
        radius=radius,
        samples=samples,
        ray_count=ray_count,
    )
    return [
        resample_polyline(polyline, point_spacing)
        for polyline in pattern
    ]


def write_waypoints_csv(path: Path, waypoints: Sequence[Waypoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "index",
                "stroke_id",
                "phase",
                "face",
                "pattern",
                "x_m",
                "y_m",
                "z_m",
                "nx",
                "ny",
                "nz",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "pen_down",
            ]
        )
        for waypoint in waypoints:
            roll, pitch, yaw = waypoint.rpy_deg
            writer.writerow(
                [
                    waypoint.index,
                    waypoint.stroke_id,
                    waypoint.phase,
                    waypoint.face,
                    waypoint.pattern,
                    f"{waypoint.xyz[0]:.6f}",
                    f"{waypoint.xyz[1]:.6f}",
                    f"{waypoint.xyz[2]:.6f}",
                    f"{waypoint.normal[0]:.6f}",
                    f"{waypoint.normal[1]:.6f}",
                    f"{waypoint.normal[2]:.6f}",
                    f"{roll:.6f}",
                    f"{pitch:.6f}",
                    f"{yaw:.6f}",
                    f"{math.radians(roll):.9f}",
                    f"{math.radians(pitch):.9f}",
                    f"{math.radians(yaw):.9f}",
                    waypoint.pen_down,
                ]
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate 2D pattern drawing trajectories on a known cube face."
    )
    parser.add_argument("--top-center-x", type=float, default=0.280)
    parser.add_argument("--top-center-y", type=float, default=-0.032)
    parser.add_argument("--top-center-z", type=float, default=0.119)
    parser.add_argument("--side-length", type=float, default=0.060)
    parser.add_argument("--face", choices=FACE_NAMES, default="top")
    parser.add_argument(
        "--faces",
        default=None,
        help="Comma-separated faces to draw, or 'all'. Overrides --face.",
    )
    parser.add_argument(
        "--pattern",
        choices=PATTERN_NAMES,
        default="circle",
    )
    parser.add_argument(
        "--face-patterns",
        default="",
        help=(
            "Optional comma-separated mapping, e.g. "
            "top:circle,front:triangle,right:star,left:sun,back:spiral,bottom:square"
        ),
    )
    parser.add_argument("--radius", type=float, default=0.020)
    parser.add_argument("--samples", type=int, default=96)
    parser.add_argument("--point-spacing", type=float, default=0.002)
    parser.add_argument("--ray-count", type=int, default=8)
    parser.add_argument("--normal-offset", type=float, default=0.0)
    parser.add_argument("--approach-height", type=float, default=0.020)
    parser.add_argument("--roll-deg", type=float, default=22.688)
    parser.add_argument("--pitch-deg", type=float, default=175.755)
    parser.add_argument("--yaw-deg", type=float, default=83.736)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    surfaces = build_axis_aligned_cube_surfaces(
        top_center=(args.top_center_x, args.top_center_y, args.top_center_z),
        side_length=args.side_length,
    )
    selected_faces = parse_faces(args.faces or args.face)
    face_pattern_overrides = parse_face_patterns(args.face_patterns)

    waypoints: List[Waypoint] = []
    next_stroke_id = 0
    pattern_cache: Dict[str, Pattern2D] = {}
    used_patterns = []

    for face in selected_faces:
        if args.faces == "all" and not face_pattern_overrides:
            pattern_name = DEFAULT_FACE_PATTERNS[face]
        else:
            pattern_name = face_pattern_overrides.get(face, args.pattern)

        if pattern_name not in pattern_cache:
            pattern_cache[pattern_name] = resampled_pattern(
                pattern_name=pattern_name,
                radius=args.radius,
                samples=args.samples,
                ray_count=args.ray_count,
                point_spacing=args.point_spacing,
            )

        face_waypoints = generate_pattern_on_face(
            surface=surfaces[face],
            pattern=pattern_cache[pattern_name],
            pattern_name=pattern_name,
            rpy_deg=(args.roll_deg, args.pitch_deg, args.yaw_deg),
            normal_offset=args.normal_offset,
            approach_height=args.approach_height,
            index_start=len(waypoints),
            stroke_id_start=next_stroke_id,
        )
        waypoints.extend(face_waypoints)
        next_stroke_id = max(waypoint.stroke_id for waypoint in waypoints) + 1
        used_patterns.append((face, pattern_name))

    if args.output:
        output = args.output
    elif len(selected_faces) == 1:
        output = Path(
            f"arm_drawing/generated/{used_patterns[0][1]}_{selected_faces[0]}_face.csv"
        )
    else:
        output = Path("arm_drawing/generated/all_faces_patterns.csv")

    write_waypoints_csv(output, waypoints)
    print(f"Wrote {len(waypoints)} waypoints to {output}")
    print(
        "Face jobs: "
        + ", ".join(f"{face}:{pattern}" for face, pattern in used_patterns)
    )
    print(f"Total strokes: {next_stroke_id}")
    print(
        "First draw point: "
        f"({waypoints[2].xyz[0]:.3f}, {waypoints[2].xyz[1]:.3f}, "
        f"{waypoints[2].xyz[2]:.3f}) m"
    )


if __name__ == "__main__":
    main()
