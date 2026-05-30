#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from kortex_driver.msg import TwistCommand

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from jacobian import NLinkArm
from scipy.spatial.transform import Rotation as R

class WristAlignerSide:
    def __init__(self):
        rospy.init_node('align_wrist_side', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        self.R_curr = np.eye(3)
        self.thetas = np.zeros(6)
        self.received_state = False
        self.msg_count = 0
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)

    def joint_states_callback(self, msg):
        self.msg_count += 1
        thetas = msg.position[0:6]
        if len(thetas) < 6: return
        self.thetas = np.array(thetas)
            
        trans = self.arm_model.transformation_matrix(thetas)
        self.R_curr = trans[0:3, 0:3]
        self.received_state = True
        
        if self.msg_count % 40 == 0:
            rospy.loginfo("📊 状态正常 | 正在进行动态位置释放对齐...")

    def run(self):
        rospy.loginfo("⏳ 等待接收关节状态话题数据...")
        while not self.received_state and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rospy.loginfo("✅ 成功连接上 /my_gen3_lite/joint_states 话题！")
        rate = rospy.Rate(40) # 40Hz
        
        k_rot = 1.0          
        max_ang_vel = 0.2    
        stable_count = 0     
        align_success = False
        
        rospy.loginfo("🔄 开始自适应侧面对齐 (释放位置约束，仅对齐笔尖至 +X 轴，允许自然偏移)...")
        
        while not rospy.is_shutdown():
            # 1. 纯指向误差计算 (对应 θY 任意)
            v_curr = self.R_curr[:, 2] # 当前工具 Z 轴 (笔尖指向)
            v_target = np.array([1.0, 0.0, 0.0]) # 目标指向正前方 +X 轴
            
            dot = np.clip(np.dot(v_curr, v_target), -1.0, 1.0)
            angle = math.acos(dot)
            
            if angle < 0.015: # 约 0.85 度
                stable_count += 1
                if stable_count >= 12: 
                    align_success = True
                    break
                omega_cmd = np.zeros(3)
            else:
                stable_count = 0
                axis = np.cross(v_curr, v_target)
                if np.linalg.norm(axis) < 1e-5:
                    axis = np.array([0.0, 1.0, 0.0])
                else:
                    axis = axis / np.linalg.norm(axis)
                    
                omega_cmd = k_rot * angle * axis
                omega_cmd = np.clip(omega_cmd, -max_ang_vel, max_ang_vel)
                
            # 2. 位置释放控制核心算法 (避免卡死)
            # 通过提取角速度雅可比，计算最小关节速度，并前馈线速度，让位置自然偏移
            J = self.arm_model.basic_jacobian(self.thetas)
            # 假设标准雅可比: 0:3 为线速度, 3:6 为角速度
            J_linear = J[0:3, :]
            J_angular = J[3:6, :]
            
            # 使用伪逆计算实现该角速度所需的最小关节角速度
            q_dot = np.linalg.pinv(J_angular).dot(omega_cmd)
            
            # 伴随产生的自然位置偏移速度
            v_linear = J_linear.dot(q_dot)
            
            # 3. 发布 TwistCommand
            cmd = TwistCommand()
            cmd.reference_frame = 3 # 基座坐标系
            cmd.duration = 0
            
            # 允许线速度跟随关节运动自然释放，而不是强行锁定在 0 (强行锁定 0 极易导致运动学奇异/卡死)
            if np.linalg.norm(omega_cmd) < 1e-4:
                cmd.twist.linear_x = 0.0
                cmd.twist.linear_y = 0.0
                cmd.twist.linear_z = 0.0
            else:
                # 给一定的阻尼系数，防止偏移过快，0.8 经验值
                cmd.twist.linear_x = 0.8 * v_linear[0]
                cmd.twist.linear_y = 0.8 * v_linear[1]
                cmd.twist.linear_z = 0.8 * v_linear[2]
            
            cmd.twist.angular_x = omega_cmd[0]
            cmd.twist.angular_y = omega_cmd[1]
            cmd.twist.angular_z = omega_cmd[2]
            
            self.vel_pub.publish(cmd)
            rate.sleep()
            
        # 发送零速度指令锁定机械臂
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.005)
            
        if align_success:
            rospy.loginfo("✅ 侧面姿态自动调直成功！笔尖现已直指 +X 轴 (正前方)！")
            rospy.loginfo("👉 提示：虽然位置发生了微小偏移，但没关系，现在请将笔尖平移到纸板起点，然后运行 side_contact_draw.py 开始作画！")
        else:
            rospy.logerr("❌ 姿态调直超时或异常终止。")

if __name__ == '__main__':
    try:
        aligner = WristAlignerSide()
        aligner.run()
    except rospy.ROSInterruptException:
        pass
