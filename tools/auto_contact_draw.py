#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from jacobian import NLinkArm

class PureAdmittanceDrawer:
    """
    纯粹的尖端力控混合控制器 (Pure Admittance Controller)
    彻底抛弃之前基于位置卡死检测的修补逻辑。
    通过精确更新的 DH 参数计算雅可比，实现完美的 TCP 力感知与笛卡尔速度映射。
    Z轴纯导纳力控，XY轴纯位置追踪。
    """
    def __init__(self):
        rospy.init_node('pure_admittance_draw', anonymous=True)
        
        # 1. 精确的 DH 参数：最后一杆更换为最新测量值 245mm (彻底解决笔尖跑偏问题)
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 245.0/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 实时状态缓存
        self.current_thetas = np.zeros(6)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 力控状态
        self.raw_fz = 0.0
        self.fz_bias = 0.0
        self.current_net_fz = 0.0
        self.calibrated = False
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/joint_velocity", Base_JointSpeeds, queue_size=1)
        self.force_pub = rospy.Publisher("/force_control/auto/estimated_fz", Float64, queue_size=1)

    def joint_states_callback(self, msg):
        thetas = np.array(msg.position[0:6])
        torques = np.array(msg.effort[0:6])
        if len(thetas) < 6 or len(torques) < 6: return
        
        self.current_thetas = thetas
        
        # 1. 实时正运动学 (基于精确 245mm TCP)
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
        
        # 2. 实时静力学力矩映射 (基于精确 245mm 雅可比的转置伪逆)
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        self.raw_fz = tool_force[2]

    def send_cartesian_velocity(self, vx, vy, vz):
        """将笛卡尔平移速度转换为底层关节角速度，完全锁定姿态"""
        # 为了避免全零导致的奇异或微小抖动，确保雅可比是最新的
        J = self.arm_model.basic_jacobian(self.current_thetas)
        V_cart = np.array([vx, vy, vz, 0.0, 0.0, 0.0])
        J_pinv = np.linalg.pinv(J, rcond=1e-3)
        q_dot = J_pinv.dot(V_cart)
        
        max_q_dot = 0.5
        q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
        
        cmd = Base_JointSpeeds()
        for j in range(6):
            speed = JointSpeed()
            speed.joint_identifier = j
            speed.value = q_dot[j]
            cmd.joint_speeds.append(speed)
        self.vel_pub.publish(cmd)

    def execute_drawing(self, csv_file):
        # 1. 解析轨迹点
        raw_waypoints = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'phase': row['phase']
                })
        if not raw_waypoints:
            rospy.logerr("轨迹文件为空！")
            return
            
        rospy.loginfo(f"📊 成功加载轨迹点: {len(raw_waypoints)} 个")
                
        # 2. 悬空静态传感器去皮校准
        rospy.loginfo("🔄 正在半空中自动标定重力与关节固有偏置...")
        rospy.sleep(1.0)
        samples = []
        for _ in range(40):
            samples.append(self.raw_fz)
            rospy.sleep(0.025)
        self.fz_bias = np.mean(samples)
        self.calibrated = True
        rospy.loginfo(f"✅ 偏置去皮完成 (Z Bias: {self.fz_bias:.2f} N)")
        
        # 3. 初始软着陆寻面
        rospy.loginfo("🚀 开始自动寻面 (导纳着陆)...")
        rate = rospy.Rate(40)
        
        for _ in range(20):
            self.send_cartesian_velocity(0.0, 0.0, 0.0)
            rate.sleep()
            
        highest_draw_z = -999.0
        
        while not rospy.is_shutdown():
            self.current_net_fz = abs(self.raw_fz - self.fz_bias)
            self.force_pub.publish(Float64(self.current_net_fz))
            
            if self.current_net_fz > 8.0:
                rospy.loginfo(f"🟢 寻面完成，确认接触物理表面！(接触力: {self.current_net_fz:.2f} N)")
                highest_draw_z = self.current_z
                for _ in range(10):
                    self.send_cartesian_velocity(0.0, 0.0, 0.0)
                    rate.sleep()
                break
            else:
                self.send_cartesian_velocity(0.0, 0.0, -0.005) # 恒定匀速盲探
                rate.sleep()
                
        # 4. 轨迹原点自动对齐物理接触点
        first_draw_wp = next(w for w in raw_waypoints if w['phase'] in ['draw', 'touch_down'])
        u_ref_x = first_draw_wp['x']
        u_ref_y = first_draw_wp['y']
        
        aligned_waypoints = []
        for wp in raw_waypoints:
            aligned_waypoints.append({
                'x': self.current_x + (wp['x'] - u_ref_x),
                'y': self.current_y + (wp['y'] - u_ref_y),
                'phase': wp['phase']
            })
            
        # 5. 启动终极混合位置/力控制器 (Hybrid Position/Force Controller)
        rospy.loginfo("✍️ 启动纯混合导纳控制器，开始自适应贴面绘图！")
        
        target_force = 3.5 # 目标下压恒定力 3.5N
        k_force = 0.003    # Z轴力控导纳系数 (m/s / N) - 极其柔顺的弹簧
        k_pos = 12.0       # XY 轴位置追踪刚度
        max_xy_speed = 0.045 # 巡航最高线速度 45mm/s
        
        wp_idx = 0
        total_wps = len(aligned_waypoints)
        filtered_fz = self.current_net_fz
        
        while wp_idx < total_wps and not rospy.is_shutdown():
            wp = aligned_waypoints[wp_idx]
            
            # --- 1. X/Y 轴：精准位置追踪 ---
            dx = wp['x'] - self.current_x
            dy = wp['y'] - self.current_y
            dist = math.hypot(dx, dy)
            
            if dist < 0.0005: # 到达当前航点 (误差<0.5mm)，自动无缝流转至下一个点
                wp_idx += 1
                if wp_idx % 20 == 0:
                    rospy.loginfo(f"📍 进度: {wp_idx}/{total_wps} | 当前力: {filtered_fz:.2f} N")
                continue
                
            cmd_vx = k_pos * dx
            cmd_vy = k_pos * dy
            v_mag = math.hypot(cmd_vx, cmd_vy)
            if v_mag > max_xy_speed:
                cmd_vx = cmd_vx * (max_xy_speed / v_mag)
                cmd_vy = cmd_vy * (max_xy_speed / v_mag)
                
            # --- 2. Z 轴：纯粹的导纳力控 ---
            raw_net_fz = abs(self.raw_fz - self.fz_bias)
            filtered_fz = 0.8 * filtered_fz + 0.2 * raw_net_fz # 低通平滑滤波
            self.force_pub.publish(Float64(filtered_fz))
            
            if wp['phase'] in ['draw', 'touch_down']:
                # 更新最高安全海拔，供抬笔使用
                highest_draw_z = max(highest_draw_z, self.current_z)
                
                force_error = target_force - filtered_fz
                
                if force_error > 2.5: 
                    # 悬空状态 (误差过大)，恒定慢速下探找纸，防止砸向纸板
                    cmd_vz = -0.005 
                else:
                    # 导纳核心：将力误差映射为虚拟速度！像弹簧一样自动起伏
                    # 若受力大 (error<0) -> 速度为正 -> 向上退缩泄力
                    # 若受力小 (error>0) -> 速度为负 -> 向下深压追力
                    cmd_vz = force_error * (-k_force) 
                    cmd_vz = np.clip(cmd_vz, -0.015, 0.015) # 安全限幅 15mm/s
            else:
                # 抬笔移动阶段：彻底放弃力控，强制向最高安全海拔抬升 15mm
                if self.current_z < highest_draw_z + 0.015:
                    cmd_vz = 0.02
                else:
                    cmd_vz = 0.0
                    
            self.send_cartesian_velocity(cmd_vx, cmd_vy, cmd_vz)
            rate.sleep()

        rospy.loginfo("🛑 轨迹全部执行完毕！准备抬笔收尾...")
        for _ in range(40):
            self.send_cartesian_velocity(0.0, 0.0, 0.03)
            rospy.sleep(0.025)
        for _ in range(15):
            self.send_cartesian_velocity(0.0, 0.0, 0.0)
            rospy.sleep(0.01)
        rospy.loginfo("🎉 全自动导纳力控绘制圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 auto_contact_draw.py <path_to_csv>")
        sys.exit(1)
    try:
        drawer = PureAdmittanceDrawer()
        drawer.execute_drawing(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
