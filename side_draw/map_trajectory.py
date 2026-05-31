"""
离线示例：读取一个三维轨迹 CSV，使用若干采样点拟合目标面，输出拟合结果并将轨迹投影/映射到拟合平面。

用法:
    python3 map_trajectory.py <input_csv> [output_mapped.csv]

CSV 要求：第一行为表头，包含 x,y,z 三列。
"""
import sys
import csv
import numpy as np
from plane_fit import fit_plane, make_plane_frame, project_point_to_plane

def read_xyz_csv(path):
    pts = []
    with open(path, 'r') as f:
        r = csv.DictReader(f)
        for row in r:
            pts.append([float(row['x']), float(row['y']), float(row['z'])])
    return np.array(pts)

def write_xyz_csv(path, pts):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x','y','z'])
        writer.writeheader()
        for p in pts:
            writer.writerow({'x':float(p[0]), 'y':float(p[1]), 'z':float(p[2])})

def main():
    if len(sys.argv) < 2:
        print('用法: python3 map_trajectory.py <input.csv> [output_mapped.csv]')
        sys.exit(1)

    inp = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) >= 3 else 'mapped_trajectory.csv'

    pts = read_xyz_csv(inp)
    print(f'读取点数量: {len(pts)}')

    # 采样若干点用于拟合（这里均匀抽样最多 50 个点）
    N = min(len(pts), 50)
    idx = np.linspace(0, len(pts)-1, N, dtype=int)
    sample = pts[idx]

    normal, centroid, rms = fit_plane(sample)
    print('拟合平面结果:')
    print(f'  法向: {normal}')
    print(f'  质心: {centroid}')
    print(f'  残差 RMS: {rms:.6f} m')

    # 构造平面局部坐标系并将所有点投影到平面上
    frame = make_plane_frame(normal)
    projected = np.array([project_point_to_plane(p, normal, centroid) for p in pts])

    # 另外，计算每点到质心坐标在平面基下的 (u,v) 坐标，供映射验证
    rel = projected - centroid
    uvz = rel.dot(frame)  # 列: [u, v, z_along_normal]

    # 保存投影结果与基于平面的 u,v 表示（这里只输出投影三维坐标）
    write_xyz_csv(out, projected)
    print(f'已将投影轨迹保存到: {out}')

if __name__ == '__main__':
    main()
