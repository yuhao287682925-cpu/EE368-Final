#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
from geometry_msgs.msg import Point
from kortex_driver.msg import TwistCommand
from std_msgs.msg import Float64

class PaintingControlNode:
    def __init__(self):
        rospy.init_node('painting_control', anonymous=True)
        
        # PD 力控参数
        self.target_force = 13.0  # 维持与纸张的接触力 13N
        self.contact_threshold = 1.0 # 激活力控的力阈值 1N
        self.Kp = 0.005
        self.Kd = 0.001
        self.max_delta_z = 0.01  # 单周期 Z 轴调整量限制 0.01m
        
        # 状态变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_fz = 0.0
        
        self.z_offset = 0.0      # 累积的 Z 轴偏移量
        self.prev_e = 0.0        # 上一周期的力误差
        self.is_drawing = False  # 是否正在绘画
        
        # 位姿与力话题订阅
        rospy.Subscriber("/tool_pose_cartesian", Point, self.pose_callback)
        rospy.Subscriber("/tool_force_cartesian", Point, self.force_callback)
        
        # 控制指令发布
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.z_offset_pub = rospy.Publisher("/force_control/z_offset", Float64, queue_size=1)
        
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        
    def force_callback(self, msg):
        # 我们主要关注 Z 轴的力
        self.current_fz = msg.z

    def update_force_control(self, dt):
        """
        PD 力控核心计算，返回当前的补偿量并累加
        """
        fz_abs = abs(self.current_fz)
        
        if fz_abs > self.contact_threshold:
            # 误差 = 目标力 - 当前力
            e_f = self.target_force - fz_abs
            d_e = (e_f - self.prev_e) / dt if dt > 0 else 0
            self.prev_e = e_f
            
            # 基础控制律
            delta_z = self.Kp * e_f + self.Kd * d_e
            
            # 饱和限制
            delta_z = np.clip(delta_z, -self.max_delta_z, self.max_delta_z)
            
            # 极性处理：如果力偏小 (e_f > 0)，需要下压（机械臂 Z 轴减小）。
            # 因此，这里的 z_offset 需要向相反方向累加
            self.z_offset -= delta_z
            
            # 可选：限制总的 z_offset 范围，防止异常情况导致笔尖撞击过深
            self.z_offset = np.clip(self.z_offset, -0.05, 0.02)
        else:
            # 未接触纸张时，重置误差，并且可以视情况选择缓慢清零或保持 offset
            self.prev_e = 0.0
            
        self.z_offset_pub.publish(Float64(self.z_offset))
        return self.z_offset

    def parse_path_file(self, csv_file):
        """
        解析路径文件，并进行坐标缩放和旋转映射
        """
        waypoints = []
        rospy.loginfo(f"正在加载轨迹文件: {csv_file}")
        
        phi = math.radians(-50.0) # -50 度转换为弧度
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # 兼容不同列名
                if 'x_norm' in row and 'y_norm' in row:
                    raw_x = float(row['x_norm'])
                    raw_y = float(row['y_norm'])
                elif 'x_m' in row and 'y_m' in row:
                    raw_x = float(row['x_m'])
                    raw_y = float(row['y_m'])
                else:
                    raw_x = float(row.get('x', 0))
                    raw_y = float(row.get('y', 0))
                
                # 1. 缩放平移映射
                x_scale = (raw_x + 0.1) / 2.5
                y_scale = (raw_y + 0.1) / 2.5
                
                # 2. 绕基座标旋转
                x_base = x_scale * cos_phi - y_scale * sin_phi
                y_base = x_scale * sin_phi + y_scale * cos_phi
                
                phase = row.get('phase', 'draw')
                
                waypoints.append({
                    'x': x_base,
                    'y': y_base,
                    'phase': phase
                })
        
        rospy.loginfo(f"共加载并映射 {len(waypoints)} 个路径点。")
        return waypoints

    def execute_path(self, waypoints):
        """
        执行轨迹
        """
        rospy.loginfo("请手动将笔尖移动并接触到纸面。")
        input("准备就绪后，请按 [Enter] 键开始绘画...")
        
        # 初始静态参数
        nominal_z = 0.028   # 标称落笔高度
        safe_z = 0.15       # 抬笔安全高度
        
        rate_hz = 40.0
        dt = 1.0 / rate_hz
        rate = rospy.Rate(rate_hz)
        
        rospy.loginfo("开始执行绘制...")
        
        for i, wp in enumerate(waypoints):
            if rospy.is_shutdown():
                break
                
            target_x = wp['x']
            target_y = wp['y']
            
            # 判断动作
            is_break = (wp['phase'] == 'BREAK' or wp['phase'] == 'lift')
            
            if is_break:
                target_z = safe_z
                # 在悬空抬笔时重置接触控制参数
                self.z_offset = 0.0 
            else:
                # 正常绘图，更新 PD 力控累加偏移
                self.update_force_control(dt)
                target_z = nominal_z + self.z_offset
                
            k_pos = 1.0 # 伺服比例系数
            
            while not rospy.is_shutdown():
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dz = target_z - self.current_z
                
                dist_xy = math.hypot(dx, dy)
                dist_z = abs(dz)
                
                # 如果水平到达目标且高度也差不多了，就切换下一个点
                if dist_xy < 0.005 and (is_break and dist_z < 0.005 or not is_break):
                    break
                    
                cmd = TwistCommand()
                cmd.reference_frame = 3 # 基座坐标系
                
                # 速度输出硬限幅，保证绘图平稳
                cmd.twist.linear_x = np.clip(k_pos * dx, -0.03, 0.03)
                cmd.twist.linear_y = np.clip(k_pos * dy, -0.03, 0.03)
                cmd.twist.linear_z = np.clip(k_pos * dz, -0.03, 0.03)
                
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            if i % 10 == 0:
                rospy.loginfo(f"进度: {i}/{len(waypoints)} | 状态: {'BREAK' if is_break else 'DRAW'} | Fz: {abs(self.current_fz):.2f}N | Target Z: {target_z:.4f}m")
                
        # 完成后抬笔停机
        rospy.loginfo("绘制完成！竖直抬起机械臂至安全高度...")
        self.lift_arm(safe_z, rate_hz)

    def lift_arm(self, safe_z, rate_hz):
        """
        竖直抬升机械臂至 safe_z 高度
        """
        rate = rospy.Rate(rate_hz)
        k_pos = 1.0
        while not rospy.is_shutdown():
            dz = safe_z - self.current_z
            if abs(dz) < 0.005:
                break
                
            cmd = TwistCommand()
            cmd.reference_frame = 3
            cmd.twist.linear_x = 0.0
            cmd.twist.linear_y = 0.0
            cmd.twist.linear_z = np.clip(k_pos * dz, -0.05, 0.05)
            self.vel_pub.publish(cmd)
            rate.sleep()
            
        # 发送 0 速度刹车
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(10):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
        
        rospy.loginfo("安全归位完成。")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 painting_control.py <path_to_csv>")
        sys.exit(1)
        
    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"找不到文件: {csv_path}")
        sys.exit(1)
        
    try:
        controller = PaintingControlNode()
        # 在 ROS 初始化并订阅数据后，给一点时间接收数据
        rospy.sleep(1.0)
        
        waypoints = controller.parse_path_file(csv_path)
        controller.execute_path(waypoints)
    except rospy.ROSInterruptException:
        pass
