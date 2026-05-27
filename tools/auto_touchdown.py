#!/usr/bin/env python3
import sys
import os
import math
import csv
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
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
from scipy.spatial.transform import Rotation as R

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(0.0, 180.0, 0.0)):
    r_default = R.from_euler('xyz', default_rpy_deg, degrees=True)
    v_from = np.array([0.0, 0.0, 1.0])
    v_to = np.array([nx, ny, nz])
    
    if np.allclose(v_from, v_to):
        q = r_default.as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
    axis = np.cross(v_from, v_to)
    axis_len = np.linalg.norm(axis)
    
    if axis_len < 1e-6:
        r_align = R.from_euler('x', 180, degrees=True)
    else:
        axis = axis / axis_len
        angle = np.arccos(np.clip(np.dot(v_from, v_to), -1.0, 1.0))
        r_align = R.from_rotvec(axis * angle)
        
    r_final = r_align * r_default
    q = r_final.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class AutoTouchdown:
    def __init__(self):
        rospy.init_node('auto_touchdown', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 对刀接触判定阈值 (Z轴估计力 3.0N 且末端手腕关节力矩 0.06N.m)
        self.contact_threshold = 4.0
        self.wrist_torque_threshold = 0.06
        
        # 内部力估计与校准变量
        self.fz_bias = 0.0
        self.calibration_samples = []
        self.calibrated = False
        
        self.current_fz = 0.0
        self.wrist_torque = 0.0
        
        # 订阅关节状态话题
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 速度发布话题
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_fz_pub = rospy.Publisher("/force_control/touchdown/estimated_fz", Float64, queue_size=1)
        
        # MoveIt 控制器接口
        import moveit_commander
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
        self.move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6:
            return
            
        # 1. 求解末端正运动学位置 (用于 MoveIt 获取失败时的绝对备用基准)
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
            
        # 2. 求解基础雅可比矩阵并计算估计力
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        raw_fz = tool_force[2]
        
        # 提取手腕末端关节 (第 6 关节) 原始力矩
        self.wrist_torque = abs(torques[5])
        
        # 自动零点校准
        if not self.calibrated:
            self.calibration_samples.append(raw_fz)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！偏置 (Z Bias): {self.fz_bias:.2f} N")
            return
            
        self.current_fz = abs(raw_fz - self.fz_bias)
        self.force_fz_pub.publish(Float64(self.current_fz))

    def align_wrist_to_vertical(self):
        """
        动作 1：仅限制姿态为垂直，调用 MoveIt 调直手腕
        """
        rospy.loginfo("🔄 正在调整末端手腕至完全垂直姿态...")
        vertical_quat = get_orientation_for_normal(0, 0, 1) # 垂直向下 (0, 180, 0)
        self.move_group.set_orientation_target([vertical_quat.x, vertical_quat.y, vertical_quat.z, vertical_quat.w])
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("✅ 末端手腕已成功翻转到完全垂直朝下姿态！")
        else:
            rospy.logerr("❌ 手腕姿态校准失败！")
            raise RuntimeError("手腕姿态初始化失败")

    def run(self):
        # 1. 第一步：将手腕对齐到垂直
        self.align_wrist_to_vertical()
        
        # 2. 第二步：强迫机械臂在垂直姿态静止 2 秒，平息惯性残留
        rospy.loginfo("⏸️ 机械臂静止中 (2秒)，正在平息残留力矩并重新进行力估计去皮...")
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(2.0)
        
        # 确保偏置校准完成
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        # 3. 第三步：向下慢速对刀下探
        rospy.loginfo("🚀 开始自动慢速下探寻面...")
        rate = rospy.Rate(40)
        
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 0
        down_cmd.twist.linear_z = -0.005 # 极慢的 5mm/s 下落
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 0
        
        contact_detected = False
        
        while not rospy.is_shutdown():
            # 双重接触判定：第 6 关节力矩跳变且 Z 轴估计力达到阈值
            if self.wrist_torque >= self.wrist_torque_threshold and self.current_fz >= 3.0:
                rospy.loginfo("🟢 检测到接触面！物理接触力判定达标。")
                rospy.loginfo(f"   >> 末端力矩 (Joint_6 Effort): {self.wrist_torque:.3f} N.m (阈值: {self.wrist_torque_threshold} N.m)")
                rospy.loginfo(f"   >> Z轴估计力 (Fz): {self.current_fz:.2f} N (阈值: 3.0 N)")
                
                # 连续发送 10 次零速度指令确保刹车刹死
                for _ in range(10):
                    self.vel_pub.publish(stop_cmd)
                    rospy.sleep(0.005)
                contact_detected = True
                break
                
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if contact_detected:
            rospy.sleep(0.5) # 等待机械臂完全静止
            
            # 由于已完全静止，我们可以非常稳定且 100% 成功地获取真实坐标
            try:
                current_pose = self.move_group.get_current_pose().pose
            except Exception as e:
                rospy.logerr(f"无法从 MoveIt 读取姿态: {e}")
                # 使用备用方案：如果 MoveIt 依然报错，我们这里直接使用正运动学估算的绝对坐标！
                # 这完全不依赖 MoveIt 的 tf 缓冲，极其可靠！
                rospy.logwarn("⚠️ MoveIt 获取坐标失败，启用底层正运动学高精度坐标计算...")
                current_pose = Pose()
                current_pose.position.x = self.current_x if hasattr(self, 'current_x') else 0.0
                current_pose.position.y = self.current_y if hasattr(self, 'current_y') else 0.0
                current_pose.position.z = self.current_z if hasattr(self, 'current_z') else 0.0
                
            rospy.loginfo(f"📍 寻面接触坐标成功锁定: X={current_pose.position.x:.4f}, Y={current_pose.position.y:.4f}, Z={current_pose.position.z:.4f}")
            
            # 将基准接触坐标写入本地文件，传递给绘图脚本
            pose_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'contact_pose.csv')
            with open(pose_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z'])
                writer.writerow([current_pose.position.x, current_pose.position.y, current_pose.position.z])
                
            rospy.loginfo(f"💾 对刀基准坐标已缓存至: {pose_file}")
            rospy.loginfo("🎉 对刀对准已完成！你可以关闭当前脚本并运行 auto_contact_draw.py 开启绘图了！")
        else:
            rospy.logerr("❌ 下探对刀失败。")

if __name__ == '__main__':
    try:
        touchdown = AutoTouchdown()
        touchdown.run()
    except rospy.ROSInterruptException:
        pass
