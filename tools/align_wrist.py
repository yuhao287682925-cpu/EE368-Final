#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from kortex_driver.msg import TwistCommand

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
    将角度限制在 [-pi, pi] 之间，防止旋转突变
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
        
        # 当前末端欧拉角缓存 (弧度)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.received_state = False
        
        # 订阅关节状态话题以实时解算正运动学欧拉角
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 发布给 Kortex 的速度控制话题
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        
        rospy.loginfo("🟢 闭环速度姿态调直节点已启动！")
        rospy.loginfo("   >> 目标 RPY 弧度: Roll=0.0, Pitch=3.1416 (180度), Yaw=0.0")
        rospy.loginfo("   >> 采用直接速度通道驱动，完全免疫 Session Not In Control 控制权拦截")
        
    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        if len(thetas) < 6:
            return
        # 实时正运动学解算当前旋转欧拉角 (弧度)
        pose = self.arm_model.forward_kinematics(thetas)
        self.roll = pose[3]
        self.pitch = pose[4]
        self.yaw = pose[5]
        self.received_state = True

    def run(self):
        # 确保已接收到机器人状态
        while not self.received_state and not rospy.is_shutdown():
            rospy.sleep(0.05)
            
        rate = rospy.Rate(40) # 40Hz
        
        # 目标垂直朝下姿态 (0, 180, 0)
        target_roll = 0.0
        target_pitch = math.pi
        target_yaw = 0.0
        
        k_rot = 1.8 # 旋转控制增益
        align_success = False
        
        rospy.loginfo("🔄 正在执行闭环速度调直姿态...")
        
        while not rospy.is_shutdown():
            # 计算欧拉角偏差并做 [-pi, pi] 包裹
            e_roll = wrap_angle(target_roll - self.roll)
            e_pitch = wrap_angle(target_pitch - self.pitch)
            e_yaw = wrap_angle(target_yaw - self.yaw)
            
            error_norm = math.sqrt(e_roll**2 + e_pitch**2 + e_yaw**2)
            
            # 偏差小于 0.015 rad (约 0.8 度) 时认为到位，安全退出
            if error_norm < 0.015:
                align_success = True
                break
                
            cmd = TwistCommand()
            cmd.reference_frame = 0 # 基座坐标系
            cmd.duration = 0
            
            # 位置保持绝对不动，XY 和 Z 速度设为 0 (解决原位漂移问题)
            cmd.twist.linear_x = 0.0
            cmd.twist.linear_y = 0.0
            cmd.twist.linear_z = 0.0
            
            # 闭环发布姿态角速度
            cmd.twist.angular_x = np.clip(k_rot * e_roll, -0.25, 0.25) # 限制角速度防止狂甩
            cmd.twist.angular_y = np.clip(k_rot * e_pitch, -0.25, 0.25)
            cmd.twist.angular_z = np.clip(k_rot * e_yaw, -0.25, 0.25)
            
            self.vel_pub.publish(cmd)
            rate.sleep()
            
        # 调平完毕后发送零速度指令锁定机械臂
        stop_cmd = TwistCommand()
        for _ in range(10):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.005)
            
        if align_success:
            rospy.loginfo("✅ 姿态已完美调整至笔直向下！无任何坐标偏移。")
            rospy.loginfo("👉 提示：现在你可以用手柄将垂直状态的机械臂挪到你想画图的纸箱起点正上方，然后启动 auto_contact_draw.py 开始绘制！")
        else:
            rospy.logerr("❌ 姿态调直超时或异常终止。")

if __name__ == '__main__':
    try:
        aligner = WristAligner()
        aligner.run()
    except rospy.ROSInterruptException:
        pass
