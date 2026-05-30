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

    actual_df = pd.read_csv(actual_csv)
    act_x, act_y, act_z = actual_df['x'].values, actual_df['y'].values, actual_df['z'].values
    
    fig = plt.figure(figsize=(16, 7))
    
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(act_x, act_y, act_z, label='Actual Trajectory', color='b', linewidth=2)
    
    ax2 = fig.add_subplot(122)
    
    if theoretical_csv and os.path.exists(theoretical_csv):
        theo_df = pd.read_csv(theoretical_csv)
        theo_x, theo_y, theo_z = theo_df['x'].values, theo_df['y'].values, theo_df['z'].values
        
        ax1.plot(theo_x, theo_y, theo_z, label='Theoretical Trajectory', color='r', linestyle='--')
        
        min_len = min(len(act_x), len(theo_x))
        if min_len > 0:
            error = np.sqrt(np.mean((act_x[:min_len] - theo_x[:min_len])**2 + 
                                    (act_y[:min_len] - theo_y[:min_len])**2 + 
                                    (act_z[:min_len] - theo_z[:min_len])**2))
            print(f"平移轨迹均方根误差 (RMSE): {error:.5f} 米")
            
        # 自动推断目标平面并绘制 2D 投影
        std_x = np.std(theo_x)
        std_y = np.std(theo_y)
        std_z = np.std(theo_z)
        
        stds = [('X', std_x, theo_x, act_x), ('Y', std_y, theo_y, act_y), ('Z', std_z, theo_z, act_z)]
        # 按照标准差排序，方差最小的轴是平面的法向量
        stds.sort(key=lambda item: item[1])
        plane_axes = stds[1:] # 后两个方差大的轴就是目标平面所在的两个轴
        
        label1, _, data1_theo, data1_act = plane_axes[0]
        label2, _, data2_theo, data2_act = plane_axes[1]
        
        ax2.plot(data1_act, data2_act, label='Actual 2D Projection', color='b', linewidth=2)
        ax2.plot(data1_theo, data2_theo, label='Theoretical 2D Projection', color='r', linestyle='--')
        ax2.set_xlabel(f'{label1} (meters)')
        ax2.set_ylabel(f'{label2} (meters)')
        ax2.set_title(f'2D Projection on {label1}-{label2} Plane')
        ax2.legend()
        ax2.axis('equal') # 保证图形的横纵比例 1:1，这样画出来的形状才不会失真变形
        
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.set_zlabel('Z (meters)')
    ax1.set_title('3D Trajectory Visualization')
    ax1.legend()
    
    plt.savefig('trajectory_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="分析机械臂轨迹误差")
    parser.add_argument('--actual', type=str, default='actual_trajectory.csv', help='实际录制的轨迹CSV路径')
    parser.add_argument('--theo', type=str, default=None, help='理论参考轨迹CSV路径 (可选)')
    args = parser.parse_args()
    
    analyze_and_plot(args.actual, args.theo)
