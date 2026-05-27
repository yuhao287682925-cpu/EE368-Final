#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from kortex_driver.msg import TwistCommand, Twist as KortexTwist

# 动态添加路径以兼容各种运行方式导入 jacobian
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm

class IndependentForceController:
    def __init__(self):
        rospy.init_node('independent_force_controller', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型的机械臂
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 力控参数设定 (目标力改为 4N)
        self.target_force = 4.0      # 目标接触力 4N
        self.kp = 0.003              # 比例增益 (速度控制下微调)
        self.kd = 0.0005             # 微分增益
        self.contact_threshold = 1.0 # 接触门槛 1N
        self.max_speed_z = 0.03      # 单周期最大 Z 轴速度 0.03 m/s
        
        # 内部状态变量
        self.prev_force_error = 0.0  # 上一时刻力偏差，用于微分计算
        self.current_fz = 0.0
        
        # 手柄输入缓存
        self.teleop_twist = KortexTwist()
        self.last_teleop_time = rospy.Time(0)
        
        # 订阅关节状态话题以实时进行力矩估计与位姿解算
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 兼容两种常见的手柄遥控话题 (Twist / TwistCommand)
        rospy.Subscriber("/my_gen3_lite/teleop/cmd_vel", Twist, self.twist_teleop_callback)
        rospy.Subscriber("/my_gen3_lite/teleop/cartesian_velocity", TwistCommand, self.twist_command_teleop_callback)
        
        # 发布给真实 Kortex 驱动控制的话题
        self.control_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        
        # 发布力控调试信息
        self.force_fz_pub = rospy.Publisher("/force_control/teleop/estimated_fz", Float64, queue_size=1)
        self.force_cmd_z_pub = rospy.Publisher("/force_control/teleop/cmd_vel_z", Float64, queue_size=1)
        
        rospy.loginfo("🟢 独立手柄力控补偿节点已启动！")
        rospy.loginfo("   >> 目标力: 4.0 N")
        rospy.loginfo("   >> 请将手柄发出的速度指令发布至: /my_gen3_lite/teleop/cmd_vel 或 /my_gen3_lite/teleop/cartesian_velocity")
        
    def joint_states_callback(self, msg):
        """
        基于雅可比矩阵转置从关节力矩实时估计末端受力
        """
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        # 防御性设计：在仿真或者驱动初始化时，effort 数组可能为空，需跳过计算防止崩溃
        if len(thetas) < 6 or len(torques) < 6:
            return
            
        # 求解基础雅可比矩阵并计算估计力
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        
        # 估计末端 Z 轴向力（取绝对值代表接触力大小）
        self.current_fz = abs(tool_force[2])
        self.force_fz_pub.publish(Float64(self.current_fz))

    def twist_teleop_callback(self, msg):
        """
        处理普通的 geometry_msgs/Twist 遥控输入
        """
        self.teleop_twist.linear_x = msg.linear.x
        self.teleop_twist.linear_y = msg.linear.y
        self.teleop_twist.linear_z = msg.linear.z
        self.teleop_twist.angular_x = msg.angular.x
        self.teleop_twist.angular_y = msg.angular.y
        self.teleop_twist.angular_z = msg.angular.z
        self.last_teleop_time = rospy.Time.now()

    def twist_command_teleop_callback(self, msg):
        """
        处理 kortex_driver/TwistCommand 遥控输入
        """
        self.teleop_twist = msg.twist
        self.last_teleop_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(40) # 40Hz 稳定控制环
        dt = 0.025
        
        while not rospy.is_shutdown():
            # 检查遥控指令是否超时 (0.5秒内无指令则认为手柄无输入)
            teleop_active = (rospy.Time.now() - self.last_teleop_time).to_sec() < 0.5
            
            cmd = TwistCommand()
            cmd.reference_frame = 0 # 基座坐标系
            cmd.duration = 0
            
            # 初始化速度为手柄遥控输入值
            if teleop_active:
                cmd.twist.linear_x = self.teleop_twist.linear_x
                cmd.twist.linear_y = self.teleop_twist.linear_y
                cmd.twist.linear_z = self.teleop_twist.linear_z
                cmd.twist.angular_x = self.teleop_twist.angular_x
                cmd.twist.angular_y = self.teleop_twist.angular_y
                cmd.twist.angular_z = self.teleop_twist.angular_z
            else:
                # 若无遥控输入，XY及旋转速度置零
                cmd.twist.linear_x = 0.0
                cmd.twist.linear_y = 0.0
                cmd.twist.linear_z = 0.0
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0
                
            # 执行 Z 轴力控
            if self.current_fz >= self.contact_threshold:
                # 接触力误差计算 (目标 4N - 实际估计力)
                force_error = self.target_force - self.current_fz
                
                # 计算微分项
                d_error = (force_error - self.prev_force_error) / dt
                self.prev_force_error = force_error
                
                # PD 输出补偿速度 (下压速度为负，因此需取负号)
                v_z_comp = -(self.kp * force_error + self.kd * d_error)
                
                # 速度保护限制
                v_z_comp = np.clip(v_z_comp, -self.max_speed_z, self.max_speed_z)
                
                # 强制覆盖手柄的 Z 轴输入，实施主动力控
                cmd.twist.linear_z = v_z_comp
            else:
                # 未触碰平面时，重置力控微分项
                self.prev_force_error = 0.0
                # Z 轴完全跟随手柄速度输入 (如果没有输入就是 0.0)
            
            # 发布最终控制指令
            self.control_pub.publish(cmd)
            self.force_cmd_z_pub.publish(Float64(cmd.twist.linear_z))
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = IndependentForceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
