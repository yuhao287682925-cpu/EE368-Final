import numpy as np
import json
import csv
import os
import sys

def generate_square_on_plane(config_file, side_length=0.1, points_per_side=20):
    with open(config_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    normal = np.array(data["normal_vector"])
    center = np.array(data["center_point"])
    
    # 构造平面上的两个正交基底向量 (u, v) 以便将 2D 坐标映射上来
    if abs(normal[0]) > abs(normal[1]):
        up = np.array([0, 1, 0])
    else:
        up = np.array([1, 0, 0])
        
    u = np.cross(up, normal)
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    v = v / np.linalg.norm(v)
    
    # 在 2D 坐标系生成正方形轮廓点
    half = side_length / 2.0
    corners = [
        np.array([-half, -half]),
        np.array([half, -half]),
        np.array([half, half]),
        np.array([-half, half])
    ]
    
    points_2d = []
    for i in range(4):
        p_start = corners[i]
        p_end = corners[(i+1)%4]
        for t in np.linspace(0, 1, points_per_side, endpoint=False):
            points_2d.append(p_start + t * (p_end - p_start))
            
    # 将 2D 点贴回 3D 物理平面
    points_3d = []
    for p in points_2d:
        p3d = center + p[0]*u + p[1]*v
        points_3d.append(p3d)
        
    # 保存为理论轨迹 CSV 供后续对比使用
    out_csv = os.path.join(os.path.dirname(config_file), 'theoretical_mock_square.csv')
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z'])
        for p in points_3d:
            writer.writerow([p[0], p[1], p[2]])
            
    print(f"\n✅ 理论基准图案(10cm 正方形)已生成至: {out_csv}")
    print(f"一共包含 {len(points_3d)} 个三维空间点，完美贴合你刚才标定的平面！")
    print("你可以直接把这个 CSV 喂给你的 analyse_error.py 脚本作为参考理论线，或者直接发给成员A/B用于他们初期的闭环运动测试！")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        config = sys.argv[1]
    else:
        # 如果没有指定，自动找当前目录下刚刚标定生成的 json
        files = [f for f in os.listdir(os.path.dirname(__file__)) if f.endswith('_config.json')]
        if files:
            config = os.path.join(os.path.dirname(__file__), files[0])
            print(f"自动加载最近的标定文件: {config}")
        else:
            print("❌ 找不到配置文件！请先运行 surface_calibrator.py 进行物理标定。")
            sys.exit(1)
            
    generate_square_on_plane(config)
