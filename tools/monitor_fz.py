#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState

# 动态添加路径以兼容导入 jacobian
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm

class ForceMonitor:
    def __init__(self):
        rospy.init_node('force_monitor', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        self.calibrated = False
        self.calibration_samples = []
        self.fz_bias = 0.0
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        rospy.loginfo("🔍 Fz 实时监控节点已启动，正在采集零点偏置...")

    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6:
            return
            
        # 1. 计算雅可比
        J = self.arm_model.basic_jacobian(thetas)
        
        # 2. 伪逆解算原始接触力 (包含机械臂重力分量)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        raw_fz = tool_force[2]
        
        # 自动校准
        if not self.calibrated:
            self.calibration_samples.append(raw_fz)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 校准完成！初始 Z轴力偏置 (包含重力): {self.fz_bias:.3f} N")
                rospy.loginfo("--------------------------------------------------")
            return

        # 减去初始偏置后的净受力 (假设姿态不变)
        net_fz = raw_fz - self.fz_bias
        
        # 打印实时数据
        # \r 使得输出在同一行刷新，方便观察
        sys.stdout.write(f"\r📡 实时监控 | 原始推算 Fz: {raw_fz:8.3f} N | 去皮净受力(Net): {net_fz:8.3f} N | 绝对值(Abs): {abs(net_fz):8.3f} N    ")
        sys.stdout.flush()

if __name__ == '__main__':
    try:
        monitor = ForceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("\n监控结束")
