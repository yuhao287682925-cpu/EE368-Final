#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Painting Control Node (Python 替代 painting.cpp)
根据 Jacobian 节点的力估计，进行 PD 力控轨迹跟踪
"""

import rospy
import numpy as np
import math
import csv
import sys
from geometry_msgs.msg import Point
from kortex_driver.msg import TwistCommand, BaseCyclic_Feedback

class PaintingController:
    def __init__(self):
        rospy.init_node("painting_control", anonymous=True)
        
        # 参数设定
        self.target_force = 13.0      # 目标接触力 13N
        self.k_p = 0.005              # P 增益
        self.k_d = 0.001              # D 增益
        self.z_nominal = 0.028        # 落笔标称高度
        self.z_safe = 0.15            # 抬笔安全高度
        self.rot_angle = math.radians(-50) # 旋转 -50 度
        
        # 状态变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.fz = 0.0
        
        self.z_offset = 0.0
        self.prev_error_force = 0.0
        
        # 发布与订阅
        self.twist_pub = rospy.Publisher('/my_gen3_lite/in/cartesian_velocity', TwistCommand, queue_size=1)
        rospy.Subscriber('/tool_force_cartesian', Point, self.force_callback)
        rospy.Subscriber('/my_gen3_lite/base_feedback', BaseCyclic_Feedback, self.feedback_callback)
        
        rospy.loginfo("Painting Control Node 初始化完成。")

    def force_callback(self, msg):
        self.fz = msg.z

    def feedback_callback(self, msg):
        self.current_x = msg.base.tool_pose_x
        self.current_y = msg.base.tool_pose_y
        self.current_z = msg.base.tool_pose_z

    def send_velocity(self, vx, vy, vz):
        cmd = TwistCommand()
        cmd.reference_frame = 0 # BASE frame
        cmd.twist.linear_x = vx
        cmd.twist.linear_y = vy
        cmd.twist.linear_z = vz
        cmd.twist.angular_x = 0.0
        cmd.twist.angular_y = 0.0
        cmd.twist.angular_z = 0.0
        self.twist_pub.publish(cmd)

    def stop_robot(self):
        self.send_velocity(0.0, 0.0, 0.0)

    def parse_trajectory(self, filename):
        waypoints = []
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            # 跳过表头(如果有)
            header = next(reader, None)
            if header and 'x' not in header[0].lower():
                # 如果第一行不是表头而是数据，重新处理
                f.seek(0)
                reader = csv.reader(f)
                
            for row in reader:
                if not row or len(row) < 2:
                    continue
                    
                x_str, y_str = row[0].strip().upper(), row[1].strip().upper()
                if x_str == 'BREAK' or y_str == 'BREAK':
                    waypoints.append({'phase': 'break'})
                else:
                    try:
                        x_norm = float(row[0])
                        y_norm = float(row[1])
                        # 1. 缩放
                        x_mapped = (x_norm + 0.1) / 2.5
                        y_mapped = (y_norm + 0.1) / 2.5
                        # 2. 旋转 -50 度
                        x_rot = x_mapped * math.cos(self.rot_angle) - y_mapped * math.sin(self.rot_angle)
                        y_rot = x_mapped * math.sin(self.rot_angle) + y_mapped * math.cos(self.rot_angle)
                        
                        waypoints.append({
                            'phase': 'draw',
                            'x': x_rot,
                            'y': y_rot
                        })
                    except ValueError:
                        continue
        return waypoints

    def execute_trajectory(self, waypoints):
        rospy.loginfo(f"开始执行轨迹，总航点数: {len(waypoints)}")
        rate = rospy.Rate(40) # 40Hz
        
        k_pos = 5.0 # 位置追踪的P比例系数
        
        for idx, wp in enumerate(waypoints):
            if rospy.is_shutdown():
                break
                
            if wp['phase'] == 'break':
                rospy.loginfo("检测到 BREAK 标记，抬笔至安全高度...")
                # 抬笔移动：保持当前的 XY，抬高 Z
                target_x = self.current_x
                target_y = self.current_y
                target_z = self.z_safe
                self.z_offset = 0.0 # 抬笔时重置力控偏置
                self.prev_error_force = 0.0
                
                # 简单循环直到到达安全高度
                while not rospy.is_shutdown() and abs(self.current_z - target_z) > 0.01:
                    vz = 2.0 * (target_z - self.current_z)
                    vz = np.clip(vz, -0.05, 0.05)
                    self.send_velocity(0.0, 0.0, vz)
                    rate.sleep()
                continue
                
            # 正常画图点
            target_x = wp['x']
            target_y = wp['y']
            
            # 等待到达目标 XY 点
            while not rospy.is_shutdown():
                # 1. PD 力控计算
                if abs(self.fz) > 1.0:
                    error_force = self.target_force - abs(self.fz)
                    d_error = error_force - self.prev_error_force
                    
                    delta_z = self.k_p * error_force + self.k_d * d_error
                    # 饱和限制：单周期调整量不超过 0.01m
                    delta_z = np.clip(delta_z, -0.01, 0.01)
                    
                    self.z_offset += delta_z
                    self.prev_error_force = error_force
                
                target_z = self.z_nominal + self.z_offset
                
                # 2. 笛卡尔位置追踪
                err_x = target_x - self.current_x
                err_y = target_y - self.current_y
                err_z = target_z - self.current_z
                
                dist_xy = math.hypot(err_x, err_y)
                if dist_xy < 0.002:
                    break # 已到达当前航点，切换下一个
                    
                vx = k_pos * err_x
                vy = k_pos * err_y
                vz = k_pos * err_z
                
                # 速度限幅
                max_speed = 0.04
                v_mag = math.hypot(vx, vy)
                if v_mag > max_speed:
                    vx = vx * (max_speed / v_mag)
                    vy = vy * (max_speed / v_mag)
                    
                vz = np.clip(vz, -0.02, 0.02)
                
                self.send_velocity(vx, vy, vz)
                rate.sleep()

        rospy.loginfo("轨迹执行完毕，抬笔归位。")
        self.stop_robot()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 pd_force_painting.py <path_to_normalized_csv>")
        sys.exit(1)
        
    controller = PaintingController()
    # 稍等订阅器连接
    rospy.sleep(1.0)
    
    wps = controller.parse_trajectory(sys.argv[1])
    controller.execute_trajectory(wps)
