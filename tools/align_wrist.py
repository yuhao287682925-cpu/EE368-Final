#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from kortex_driver.msg import TwistCommand
from scipy.spatial.transform import Rotation as R

# 动态添加路径以导入已有的 jacobian.py
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm

def wrap_angle(angle):
    """
    将角度限制在 [-pi, pi] 之间
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

class WristAligner:
    def __init__(self):
        rospy.init_node('align_wrist', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型的机械臂
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 当前末端状态缓存
        self.alpha = 0.0
        self.beta = 0.0
        self.gamma = 0.0
        self.R_curr = np.identity(3)
        self.received_state = False
        self.msg_count = 0
        
        # 订阅关节状态话题
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 发布给 Kortex 的速度控制话题
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        
        rospy.loginfo("🟢 闭环三维姿态调直节点已启动！")
        rospy.loginfo("   >> 目标 RPY 姿态: [0.0, 180.0, 0.0] 度")
        rospy.loginfo("   >> 基于三维轴角反馈控制，同时调直并校准第一个角")
        
    def joint_states_callback(self, msg):
        self.msg_count += 1
        thetas = msg.position[0:6]
        if len(thetas) < 6:
            return
            
        # 实时正运动学解算当前旋转矩阵
        trans = self.arm_model.transformation_matrix(thetas)
        self.R_curr = trans[0:3, 0:3]
        
        # 解算欧拉角用于诊断打印
        self.alpha, self.beta, self.gamma = self.arm_model.euler_angle(thetas)
        self.received_state = True
        
        if self.msg_count % 40 == 0:
            rospy.loginfo(f"📊 关节回调正常 | Beta倾斜角: {math.degrees(self.beta):.2f}° | Alpha角: {math.degrees(self.alpha):.2f}°")

    def run(self):
        rospy.loginfo("⏳ 等待接收关节状态话题数据...")
        # 确保已接收到机器人状态
        while not self.received_state and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rospy.loginfo("✅ 成功连接上 /my_gen3_lite/joint_states 话题！")
        rate = rospy.Rate(40) # 40Hz
        
        # 设定目标姿态：笔身完全垂直向下，且第一个角对齐 0 度
        # 对应 RPY 旋转 [0.0, 180.0, 0.0] 度
        r_target = R.from_euler('xyz', [0.0, 180.0, 0.0], degrees=True)
        
        k_rot = 0.8          # 控制增益降低，防止在终点来回跳动
        max_ang_vel = 0.15   # 限制最大角速度为 0.15 rad/s，平滑逼近
        stable_count = 0     # 连续到位计数器
        align_success = False
        
        rospy.loginfo("🔄 开始基于三维轴角闭环姿态调直与首角校正控制...")
        
        while not rospy.is_shutdown():
            # 计算当前与目标的旋转偏差
            r_curr = R.from_matrix(self.R_curr)
            r_err = r_target * r_curr.inv()
            omega_err = r_err.as_rotvec()
            
            err_norm = np.linalg.norm(omega_err)
            
            # 判断是否连续稳定到位 (偏差小于 0.015 弧度，约 0.85 度)
            if err_norm < 0.015:
                stable_count += 1
                if stable_count >= 12: # 持续 12 个周期 (0.3 秒) 稳定在死区内，安全退出
                    align_success = True
                    break
            else:
                stable_count = 0
                
            cmd = TwistCommand()
            cmd.reference_frame = 3 # 基座坐标系
            cmd.duration = 0
            
            # 位置保持绝对不动
            cmd.twist.linear_x = 0.0
            cmd.twist.linear_y = 0.0
            cmd.twist.linear_z = 0.0
            
            # 计算角速度
            ang_vel = k_rot * omega_err
            
            # 引入极小偏差死区，彻底避免目标位置高频抖动
            if err_norm < 0.008: # 约 0.45 度
                ang_vel = np.zeros(3)
                
            # 裁切角速度范围，平滑限制
            ang_vel = np.clip(ang_vel, -max_ang_vel, max_ang_vel)
            
            cmd.twist.angular_x = ang_vel[0]
            cmd.twist.angular_y = ang_vel[1]
            cmd.twist.angular_z = ang_vel[2]
            
            self.vel_pub.publish(cmd)
            rate.sleep()
            
        # 调平完毕后发送零速度指令锁定机械臂
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.005)
            
        if align_success:
            rospy.loginfo(f"✅ 姿态与首角调直成功！当前: Beta倾斜={math.degrees(self.beta):.2f}°, Alpha={math.degrees(self.alpha):.2f}°")
            rospy.loginfo("👉 提示：现在你可以用手柄将垂直状态的机械臂挪到你想画图的纸箱起点正上方，然后启动 auto_contact_draw.py 开始绘制！")
        else:
            rospy.logerr("❌ 姿态调直超时或异常终止。")

if __name__ == '__main__':
    try:
        aligner = WristAligner()
        aligner.run()
    except rospy.ROSInterruptException:
        pass
