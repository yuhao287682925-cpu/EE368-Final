import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

def analyze_and_plot(actual_csv, theoretical_csv=None):
    if not os.path.exists(actual_csv):
        print(f"找不到记录文件: {actual_csv}")
        return

    # 读取实际记录的数据
    actual_df = pd.read_csv(actual_csv)
    act_x, act_y, act_z = actual_df['x'].values, actual_df['y'].values, actual_df['z'].values
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制实际跑出来的物理轨迹 (蓝色实线)
    ax.plot(act_x, act_y, act_z, label='Actual Trajectory', color='b', linewidth=2)
    
    if theoretical_csv and os.path.exists(theoretical_csv):
        # 读取成员A生成的理论轨迹点
        theo_df = pd.read_csv(theoretical_csv)
        theo_x, theo_y, theo_z = theo_df['x'].values, theo_df['y'].values, theo_df['z'].values
        
        # 绘制理论轨迹 (红色虚线)
        ax.plot(theo_x, theo_y, theo_z, label='Theoretical Trajectory', color='r', linestyle='--')
        
        # 简单均方根误差 (RMSE) 计算
        # 注意：严格意义上的误差需要使用最近邻(KDTree)或动态时间规整(DTW)对齐。这里展示最基础的点对点近似计算。
        min_len = min(len(act_x), len(theo_x))
        if min_len > 0:
            error = np.sqrt(np.mean((act_x[:min_len] - theo_x[:min_len])**2 + 
                                    (act_y[:min_len] - theo_y[:min_len])**2 + 
                                    (act_z[:min_len] - theo_z[:min_len])**2))
            print(f"平移轨迹均方根误差 (RMSE): {error:.5f} 米")
    
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('3D Trajectory Visualization & Error Analysis')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="分析机械臂轨迹误差")
    parser.add_argument('--actual', type=str, default='actual_trajectory.csv', help='实际录制的轨迹CSV路径')
    parser.add_argument('--theo', type=str, default=None, help='理论参考轨迹CSV路径 (可选)')
    args = parser.parse_args()
    
    analyze_and_plot(args.actual, args.theo)
