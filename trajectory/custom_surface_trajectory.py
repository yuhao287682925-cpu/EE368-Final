#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import json
import argparse
from pathlib import Path

# 动态添加路径以兼容导入 cube_circle_trajectory
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from trajectory.cube_circle_trajectory import (
    resampled_pattern,
    write_waypoints_csv,
    Waypoint
)

def generate_custom_trajectory(config_file, pattern_name, radius, output_file):
    # 1. 载入真实物理平面的三点标定数据
    if not os.path.exists(config_file):
        print(f"❌ 找不到标定文件: {config_file}")
        print("请先运行 tools/spatial_draw/auto_surface_calibrator.py 进行物理接触标定！")
        sys.exit(1)
        
    with open(config_file, 'r') as f:
        data = json.load(f)
        
    center = np.array(data["center_point"])
    normal = np.array(data["normal_vector"])
    u_axis = np.array(data["u_axis"])
    v_axis = np.array(data["v_axis"])
    
    print(f"📦 已加载物理标定面 [{data.get('surface_name', 'custom_surface')}]")
    print(f"   法线 (Normal): {np.round(normal, 3)}")
    print(f"   原点 (Center): {np.round(center, 3)}")
    
    # 2. 生成 2D 基础图案 (默认点间距 2mm)
    print(f"🎨 正在生成并重采样基础 2D 图形 [{pattern_name}]...")
    pattern_2d = resampled_pattern(
        pattern_name=pattern_name,
        radius=radius,
        samples=int(math.ceil(2 * math.pi * radius / 0.002)),
        ray_count=8,
        point_spacing=0.002
    )
    
    # 3. 将 2D 图案严丝合缝地投影到真实的 3D 物理斜面上
    waypoints = []
    approach_height = 0.05 # 下探前距离表面 5cm 的安全悬空高度
    stroke_id = 0
    
    for stroke in pattern_2d:
        if not stroke:
            continue
            
        # (A) 移动到起点的悬空位置 (沿法线正方向抬升)
        u_start, v_start = stroke[0]
        start_3d = center + u_start * u_axis + v_start * v_axis
        hover_pt = start_3d + approach_height * normal
        waypoints.append(Waypoint(
            xyz=tuple(hover_pt),
            normal=tuple(normal),
            stroke_id=stroke_id,
            phase="hover"
        ))
        
        # (B) 开始接触 (此时 auto_contact_draw 会触发寻面)
        waypoints.append(Waypoint(
            xyz=tuple(start_3d),
            normal=tuple(normal),
            stroke_id=stroke_id,
            phase="touch_down"
        ))
        
        # (C) 绘制整个笔画轨迹
        for (u, v) in stroke:
            pt_3d = center + u * u_axis + v * v_axis
            waypoints.append(Waypoint(
                xyz=tuple(pt_3d),
                normal=tuple(normal),
                stroke_id=stroke_id,
                phase="draw"
            ))
            
        # (D) 画完后垂直拔出 (沿法线正方向抬升)
        u_end, v_end = stroke[-1]
        end_3d = center + u_end * u_axis + v_end * v_axis
        lift_pt = end_3d + approach_height * normal
        waypoints.append(Waypoint(
            xyz=tuple(lift_pt),
            normal=tuple(normal),
            stroke_id=stroke_id,
            phase="lift"
        ))
        
        stroke_id += 1
        
    # 4. 保存为兼容底层架构的 CSV
    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    write_waypoints_csv(Path(output_file), waypoints)
    
    print(f"✅ 成功生成带物理姿态投影的真实 3D 轨迹！")
    print(f"   图形配置: {pattern_name} (半径 {radius}m)")
    print(f"   总航点数: {len(waypoints)} (分为 {stroke_id} 笔)")
    print(f"   导出路径: {output_file}")
    print("\n🚀 下一步：运行 auto_contact_draw_3d.py 载入此 CSV 文件即可完美自适应绘制！")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a 3D trajectory perfectly mapped onto a physically calibrated custom surface")
    parser.add_argument("--config", type=str, default="tools/spatial_draw/custom_surface.json", help="三点物理标定生成的 json 文件路径")
    parser.add_argument("--pattern", type=str, default="star", help="要绘制的图案 (e.g. star, circle, spiral, square, triangle)")
    parser.add_argument("--radius", type=float, default=0.04, help="图案的半径尺寸 (单位: 米)")
    parser.add_argument("--output", type=str, default="trajectory/custom_star.csv", help="输出的 CSV 文件路径")
    
    args = parser.parse_args()
    
    generate_custom_trajectory(
        config_file=args.config,
        pattern_name=args.pattern,
        radius=args.radius,
        output_file=args.output
    )
