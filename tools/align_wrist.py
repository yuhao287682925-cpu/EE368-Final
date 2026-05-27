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
        
        # 当前末端 ZYZ 欧拉角 beta 缓存 (弧度)
        self.beta = 0.0
        self.received_state = False
        self.msg_count = 0
        
        # 订阅关节状态话题
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 发布给 Kortex 的速度控制话题
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        
        rospy.loginfo("🟢 闭环姿态调直节点已启动！")
        rospy.loginfo("   >> 目标 ZYZ Beta 角: 180度 (即 math.pi)")
        rospy.loginfo("   >> 只需控制 Y 轴倾斜，绕过其他轴的奇异性")
        
    def joint_states_callback(self, msg):
        self.msg_count += 1
        thetas = msg.position[0:6]
        if len(thetas) < 6:
            return
            
        # 实时正运动学解算当前旋转欧拉角 (ZYZ 格式)
        pose = self.arm_model.forward_kinematics(thetas)
        self.beta = pose[4] # pose[4] 对应 ZYZ 的 beta (倾斜角)
        self.received_state = True
        
        if self.msg_count % 40 == 0:
            rospy.loginfo(f"📊 关节回调正常运行中 | 当前 Beta 倾斜角: {math.degrees(self.beta):.2f} 度")

    def run(self):
        rospy.loginfo("⏳ 等待接收关节状态话题数据...")
        # 确保已接收到机器人状态
        while not self.received_state and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rospy.loginfo("✅ 成功连接上 /my_gen3_lite/joint_states 话题！")
        rate = rospy.Rate(40) # 40Hz
        
        # 目标倾斜角：笔身垂直向下对应 beta = pi
        target_beta = math.pi
        k_rot = 1.8
        align_success = False
        
        rospy.loginfo("🔄 开始闭环向 Y 轴发布角速度调直...")
        
        while not rospy.is_shutdown():
            # 计算 beta 角偏差并做包裹
            e_beta = wrap_angle(target_beta - self.beta)
            
            # 偏差小于 0.015 rad (约 0.8 度) 时认为到位，安全退出
            if abs(e_beta) < 0.015:
                align_success = True
                break
                
            cmd = TwistCommand()
            cmd.reference_frame = 3 # 基座坐标系
            cmd.duration = 0
            
            # 位置保持绝对不动
            cmd.twist.linear_x = 0.0
            cmd.twist.linear_y = 0.0
            cmd.twist.linear_z = 0.0
            
            # 仅在 Y 轴施加角速度，其余轴设为 0
            cmd.twist.angular_x = 0.0
            cmd.twist.angular_y = np.clip(k_rot * e_beta, -0.25, 0.25)
            cmd.twist.angular_z = 0.0
            
            self.vel_pub.publish(cmd)
            rate.sleep()
            
        # 调平完毕后发送零速度指令锁定机械臂
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.005)
            
        if align_success:
            rospy.loginfo(f"✅ 姿态调直成功！当前 Beta 倾斜角: {math.degrees(self.beta):.2f} 度")
            rospy.loginfo("👉 提示：现在你可以用手柄将垂直状态的机械臂挪到你想画图的纸箱起点正上方，然后启动 auto_contact_draw.py 开始绘制！")
        else:
            rospy.logerr("❌ 姿态调直超时或异常终止。")

if __name__ == '__main__':
    try:
        aligner = WristAligner()
        aligner.run()
    except rospy.ROSInterruptException:
        pass
