#!/usr/bin/env python3
"""
pattern_designer.py - Helper script to visually design 2D drawing patterns.

Allows the user to define patterns as sequences of (x, y) points in mm,
then saves them as SVG files or directly as ROS parameters.

Usage:
    rosrun trajectory_generator pattern_designer.py --output pattern.svg
"""
import sys
import os
import argparse
import math


def create_square_svg(width_mm, height_mm, output_path):
    """Create a simple square frame SVG."""
    hw = width_mm / 2.0
    hh = height_mm / 2.0
    svg_content = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg"
     viewBox="{-hw} {-hh} {width_mm} {height_mm}"
     width="{width_mm}mm" height="{height_mm}mm">
  <path d="M {-hw} {-hh} L {hw} {-hh} L {hw} {hh} L {-hw} {hh} Z"
        fill="none" stroke="black" stroke-width="1"/>
</svg>'''
    with open(output_path, 'w') as f:
        f.write(svg_content)
    print(f"Square pattern saved to {output_path}")


def create_circle_svg(radius_mm, output_path, segments=72):
    """Create a circle approximated by line segments."""
    points = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        x = radius_mm * math.cos(angle)
        y = radius_mm * math.sin(angle)
        points.append(f"{x:.2f} {y:.2f}")

    d = f"M {points[0]} " + " ".join(f"L {p}" for p in points[1:]) + " Z"
    svg_content = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg"
     viewBox="{-radius_mm} {-radius_mm} {2*radius_mm} {2*radius_mm}"
     width="{2*radius_mm}mm" height="{2*radius_mm}mm">
  <path d="{d}" fill="none" stroke="black" stroke-width="1"/>
</svg>'''
    with open(output_path, 'w') as f:
        f.write(svg_content)
    print(f"Circle pattern saved to {output_path}")


def create_star_svg(outer_r, inner_r, points, output_path):
    """Create a star pattern."""
    coords = []
    for i in range(2 * points):
        angle = math.pi * i / points - math.pi / 2
        r = outer_r if i % 2 == 0 else inner_r
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        coords.append(f"{x:.2f} {y:.2f}")

    d = f"M {coords[0]} " + " ".join(f"L {c}" for c in coords[1:]) + " Z"
    size = 2 * outer_r
    svg_content = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg"
     viewBox="{-outer_r} {-outer_r} {size} {size}"
     width="{size}mm" height="{size}mm">
  <path d="{d}" fill="none" stroke="black" stroke-width="1"/>
</svg>'''
    with open(output_path, 'w') as f:
        f.write(svg_content)
    print(f"Star pattern saved to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Design 2D drawing patterns for the block drawing system.")
    parser.add_argument('--output', '-o', default='pattern.svg',
                        help='Output SVG file path')
    parser.add_argument('--type', '-t', choices=['square', 'circle', 'star'],
                        default='square', help='Pattern type')
    parser.add_argument('--width', type=float, default=100.0,
                        help='Pattern width in mm (for square)')
    parser.add_argument('--height', type=float, default=100.0,
                        help='Pattern height in mm (for square)')
    parser.add_argument('--radius', type=float, default=50.0,
                        help='Radius in mm (for circle/star outer)')
    parser.add_argument('--inner-radius', type=float, default=20.0,
                        help='Inner radius in mm (for star)')
    parser.add_argument('--star-points', type=int, default=5,
                        help='Number of star points')

    args = parser.parse_args()

    if args.type == 'square':
        create_square_svg(args.width, args.height, args.output)
    elif args.type == 'circle':
        create_circle_svg(args.radius, args.output)
    elif args.type == 'star':
        create_star_svg(args.radius, args.inner_radius, args.star_points, args.output)


if __name__ == '__main__':
    main()
