#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import json
import rospy
from sensor_msgs.msg import JointState

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
root_dir = os.path.abspath(os.path.join(parent_dir, '..'))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from jacobian import NLinkArm

class AutoCalibrator:
    def __init__(self):
        rospy.init_node('auto_surface_calibrator', anonymous=True)
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        self.current_pos = None
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        if len(thetas) < 6:
            return
        # 使用高精度底层正运动学实时解算末端坐标
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_pos = np.array([tool_pose[0], tool_pose[1], tool_pose[2]])
        
def calculate_plane(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    if norm < 1e-6:
        raise ValueError("三个点共线，无法构成平面！请拉开三点距离重试。")
    normal = normal / norm
    
    center = (p1 + p2 + p3) / 3.0
    
    # 启发式规则：法向量应指向外部（即指向机械臂基座）。
    # 机械臂基座在原点 (0,0,0)，所以从 Center 指向原点的向量大致是 -Center。
    # 我们希望 Normal 与 -Center 的夹角小于 90 度，即 dot(Normal, -Center) > 0 => dot(Normal, Center) < 0
    if np.dot(normal, center) > 0:
        normal = -normal
        
    return normal, center

def main():
    print("====================================================")
    print("   🧱 全自动物理平面 3 点标定工具 (Auto Calibrator) ")
    print("====================================================")
    print("操作步骤：")
    print("1. 解锁机械臂关节（进入示教模式）或使用手柄控制。")
    print("2. 将笔尖贴近你要绘画的纸板表面的左下、右下、上方（任意三个尽量分开的点）。")
    print("3. 每贴好一个点，在此终端按下【回车键】进行坐标抓取。")
    print("====================================================\n")
    
    calibrator = AutoCalibrator()
    print("正在连接 ROS 等待获取机械臂位姿...")
    while calibrator.current_pos is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    print("✅ 已成功连接并获取实时坐标！\n")
        
    points = []
    for i in range(1, 4):
        input(f"👉 请将笔尖贴紧目标表面的第 {i} 个点，然后按【回车键】确认...")
        pos = calibrator.current_pos.copy()
        print(f"   ✅ 抓取成功: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}\n")
        points.append(pos)
        
    try:
        normal, center = calculate_plane(points[0], points[1], points[2])
        
        # 构建 U 和 V 基底供轨迹生成使用
        # 找一个与 Normal 不平行的临时向量计算 U
        temp_vec = np.array([0, 0, 1])
        if abs(np.dot(normal, temp_vec)) > 0.99:
            temp_vec = np.array([0, 1, 0])
            
        u_axis = np.cross(temp_vec, normal)
        u_axis = u_axis / np.linalg.norm(u_axis)
        v_axis = np.cross(normal, u_axis)
        
        data = {
            "surface_name": "custom_physical_surface",
            "normal_vector": normal.tolist(),
            "center_point": center.tolist(),
            "u_axis": u_axis.tolist(),
            "v_axis": v_axis.tolist(),
            "p1": points[0].tolist(),
            "p2": points[1].tolist(),
            "p3": points[2].tolist()
        }
        
        out_file = os.path.join(current_dir, 'custom_surface.json')
        with open(out_file, 'w') as f:
            json.dump(data, f, indent=4)
            
        print("🎉 平面物理姿态计算成功！")
        print(f"   法线 (Normal) : {np.round(normal, 4)}")
        print(f"   中心 (Center) : {np.round(center, 4)}")
        print(f"   U 轴基底      : {np.round(u_axis, 4)}")
        print(f"   V 轴基底      : {np.round(v_axis, 4)}")
        print(f"\n🗂️ 标定数据已保存至: {out_file}")
        print("下一步：请运行 custom_surface_trajectory.py 生成对齐该平面的专属绘制轨迹！")
        
    except Exception as e:
        print(f"\n❌ 计算失败: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
