#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

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
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/joint_velocity", Base_JointSpeeds, queue_size=1)

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
        
        rospy.loginfo("🔄 开始自适应侧面对齐 (仅对齐笔尖至 +Y 轴，允许自然偏移)...")
        
        while not rospy.is_shutdown():
            # 1. 纯指向误差计算 (对应 θY 任意)
            v_curr = self.R_curr[:, 2] # 当前工具 Z 轴 (笔尖指向)
            v_target = np.array([0.0, 1.0, 0.0]) # 目标指向 +Y 轴
            
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
                    axis = np.array([1.0, 0.0, 0.0]) # 若刚好反向，换个正交轴
                else:
                    axis = axis / np.linalg.norm(axis)
                    
                omega_cmd = k_rot * angle * axis
                omega_cmd = np.clip(omega_cmd, -max_ang_vel, max_ang_vel)
                
            # 2. 位置释放控制核心算法：直接解算角速度雅可比逆
            J = self.arm_model.basic_jacobian(self.thetas)
            J_angular = J[3:6, :]
            
            # 使用带阻尼的伪逆，求解所需关节速度
            q_dot = np.linalg.pinv(J_angular, rcond=1e-3).dot(omega_cmd)
            
            # 3. 绕过笛卡尔限位，直接下发底层关节角速度
            max_q_dot = 0.5
            q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
            
            cmd = Base_JointSpeeds()
            for j in range(6):
                speed = JointSpeed()
                speed.joint_identifier = j
                if np.linalg.norm(omega_cmd) < 1e-4:
                    speed.value = 0.0
                else:
                    speed.value = q_dot[j]
                cmd.joint_speeds.append(speed)
                
            self.vel_pub.publish(cmd)
            rate.sleep()
            
        # 停机指令
        stop_cmd = Base_JointSpeeds()
        for j in range(6):
            speed = JointSpeed()
            speed.joint_identifier = j
            speed.value = 0.0
            stop_cmd.joint_speeds.append(speed)
            
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.005)
            
        if align_success:
            rospy.loginfo("✅ 侧面姿态自动调直成功！笔尖现已直指 +Y 轴！")
            rospy.loginfo("👉 提示：现在请将笔尖平移到纸板起点，然后运行 side_contact_draw_with_log.py 开始作画！")
        else:
            rospy.logerr("❌ 姿态调直超时或异常终止。")

if __name__ == '__main__':
    try:
        aligner = WristAlignerSide()
        aligner.run()
    except rospy.ROSInterruptException:
        pass
