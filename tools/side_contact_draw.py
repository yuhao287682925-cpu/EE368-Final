#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
from std_msgs.msg import Float64 as StdFloat64

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from jacobian import NLinkArm
from scipy.spatial.transform import Rotation as R

class SideContactDrawer:
    def __init__(self):
        rospy.init_node('side_contact_draw', anonymous=True)
        
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.f3d_bias = np.zeros(3)
        self.calibration_samples = []
        self.calibrated = False
        self.current_f_total = 0.0
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/joint_velocity", Base_JointSpeeds, queue_size=1)
        self.force_pub = rospy.Publisher("/force_control/auto/estimated_f_normal", StdFloat64, queue_size=1)

    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6: return
        
        # 记录当前关节角供底层刚性逆解使用
        self.current_thetas = thetas
            
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
        
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        
        raw_f3d = tool_force[0:3]
        self.raw_f3d = raw_f3d # 保存原始力供寻面时去皮使用
        
        if not self.calibrated:
            self.calibration_samples.append(raw_f3d)
            if len(self.calibration_samples) >= 40:
                self.f3d_bias = np.mean(self.calibration_samples, axis=0)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除 3D 静力偏置: [{self.f3d_bias[0]:.2f}, {self.f3d_bias[1]:.2f}, {self.f3d_bias[2]:.2f}] N")
            return
            
        f_net = raw_f3d - self.f3d_bias
        self.current_f_total = np.linalg.norm(f_net)
        self.force_pub.publish(StdFloat64(self.current_f_total))

    def send_cartesian_velocity(self, vx, vy, vz):
        """
        利用自研 DH 雅可比矩阵将末端线速度映射为关节角速度，完全接管底层控制
        """
        if not hasattr(self, 'current_thetas'):
            return
            
        # 1. 计算当前雅可比矩阵
        J = self.arm_model.basic_jacobian(self.current_thetas)
        
        # 2. 构建目标笛卡尔速度向量 (仅平移，保证笔尖姿态绝对不变)
        V_cart = np.array([vx, vy, vz, 0.0, 0.0, 0.0])
        
        # 3. 伪逆解算目标关节速度 q_dot = J_pinv * V_cart
        J_pinv = np.linalg.pinv(J, rcond=1e-3)
        q_dot = J_pinv.dot(V_cart)
        
        # 4. 安全硬限幅 (极严苛限制: 0.5 rad/s)
        max_q_dot = 0.5
        q_dot = np.clip(q_dot, -max_q_dot, max_q_dot)
        
        # 5. 打包并发送 JointSpeed 消息
        cmd = Base_JointSpeeds()
        for j in range(6):
            speed = JointSpeed()
            speed.joint_identifier = j
            speed.value = q_dot[j]
            cmd.joint_speeds.append(speed)
            
        self.vel_pub.publish(cmd)

    def run_auto_touchdown(self):
        rospy.loginfo("🚀 开始沿 -Y 轴纯运动学软着陆寻面...")
        
        rospy.sleep(1.5) # 等待机械臂平稳
        
        rate = rospy.Rate(40)
        dt = 0.025
        contact_detected = False
        loop_cnt = 0
        stuck_cnt = 0
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            prev_y = self.current_y
            
            # 沿 -Y 方向恒速前探 5mm/s
            self.send_cartesian_velocity(0.0, -0.005, 0.0)
            rate.sleep()
            
            # 前 1.5 秒为加速平稳期，屏蔽检测
            if loop_cnt > 60:
                # 使用物理编码器反馈计算真实 Y 轴速度
                actual_speed = abs(self.current_y - prev_y) / dt
                
                # 预期速度为 0.005 m/s，若实际速度跌破 0.001 m/s (20%)，即判定为物理碰壁失速！
                if actual_speed < 0.001:
                    stuck_cnt += 1
                else:
                    stuck_cnt = 0
                    
                # 连续 4 帧（0.1秒）确认失速
                if stuck_cnt >= 4:
                    rospy.loginfo(f"🟢 运动学失速触发！精准判定触壁 (当前移速: {actual_speed:.4f} m/s)")
                    # 立即发送 5 次 0 速度，确保驱动层彻底刹停
                    for _ in range(5):
                        self.send_cartesian_velocity(0.0, 0.0, 0.0)
                        rospy.sleep(0.01)
                    contact_detected = True
                    break
            else:
                if loop_cnt % 15 == 0:
                    rospy.loginfo("⏳ 启动加速平稳期，屏蔽接触判定...")
            
        if contact_detected:
            rospy.sleep(0.5)
            rospy.loginfo(f"📍 侧面寻面起点锁定: X={self.current_x:.4f}, Y={self.current_y:.4f}, Z={self.current_z:.4f}")
            return self.current_x, self.current_y, self.current_z
        else:
            raise RuntimeError("寻面程序异常终止")

    def execute_and_draw(self, csv_file):
        rospy.loginfo(f"读取 2D 轨迹文件: {csv_file}")
        raw_waypoints = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append({
                    'x': float(row['x_m']),
                    'y': float(row['y_m']),
                    'phase': row['phase'],
                    'stroke_id': int(row['stroke_id'])
                })
        
        if not raw_waypoints: return
            
        contact_x, contact_y, contact_z = self.run_auto_touchdown()
        
        first_draw_idx = 0
        for idx, wp in enumerate(raw_waypoints):
            if wp['phase'] in ['draw', 'touch_down']:
                first_draw_idx = idx
                break
                
        # 取平面的二维坐标作为映射基准
        u_ref = raw_waypoints[first_draw_idx]['x']
        v_ref = raw_waypoints[first_draw_idx]['y']
        
        aligned_waypoints = []
        for wp in raw_waypoints:
            du = wp['x'] - u_ref
            dv = wp['y'] - v_ref
            
            # X 控制左右，Z 控制上下 (将轨迹映射到侧面)
            aligned_waypoints.append({
                'x': contact_x - du,
                'y': contact_y, 
                'z': contact_z - dv, 
                'phase': wp['phase']
            })
            
        rate = rospy.Rate(40)
        dt = 0.025
        
        rospy.loginfo("✍️ 寻面完成，开始启动纯运动学啄木鸟实战侧绘...")
        
        # 侧向安全泄压补偿量 (+Y方向)
        y_offset_relief = 0.0
        macro_y_shift = 0.0 # 宏观平面自适应积分器
        relief_cooldown = 0
        stuck_cnt = 0
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown(): break
                
            target_x = wp['x']
            target_z = wp['z']
            
            prev_servo_x = self.current_x
            prev_servo_z = self.current_z
            
            actual_speed = 0.0
            expected_speed = 0.0
            
            while not rospy.is_shutdown():
                curr_pos = np.array([self.current_x, self.current_y, self.current_z])
                
                # 绘制平面在 X-Z，计算面内误差
                err_x = target_x - curr_pos[0]
                err_z = target_z - curr_pos[2]
                dist_to_target = math.hypot(err_x, err_z)
                
                if dist_to_target < 0.001:
                    break
                    
                movement = math.hypot(curr_pos[0] - prev_servo_x, curr_pos[2] - prev_servo_z)
                actual_speed = movement / dt if dt > 0 else 0.0
                
                # 面内巡航速度生成
                max_speed = 0.025 # 降低侧向最高限速，提升抗重力跟随精度
                k_pos_near = 4.0  # 降低P控制刚度，防止物理柔性形变
                
                cmd_vx_raw = k_pos_near * err_x
                cmd_vz_raw = k_pos_near * err_z
                v_mag = math.hypot(cmd_vx_raw, cmd_vz_raw)
                
                if v_mag > max_speed:
                    scale = max_speed / v_mag
                    cmd_vx = cmd_vx_raw * scale
                    cmd_vz = cmd_vz_raw * scale
                else:
                    cmd_vx = cmd_vx_raw
                    cmd_vz = cmd_vz_raw
                    
                expected_speed = math.hypot(cmd_vx, cmd_vz)
                
                # 失速卡死检测 (免疫侧壁重力与摩擦干扰的运动学神技)
                if wp['phase'] in ['draw', 'touch_down']:
                    if expected_speed > 0.005 and relief_cooldown == 0:
                        if actual_speed < 0.25 * expected_speed or actual_speed < 0.002:
                            stuck_cnt += 1
                        else:
                            stuck_cnt = 0
                    else:
                        stuck_cnt = 0
                else:
                    stuck_cnt = 0
                    
                prev_servo_x = curr_pos[0]
                prev_servo_z = curr_pos[2]
                
                # 触发侧边抬升
                if wp['phase'] in ['draw', 'touch_down']:
                    if stuck_cnt > 8:
                        y_offset_relief += 0.0035 # 侧边方向拔出 (+Y)
                        y_offset_relief = min(y_offset_relief, 0.015)
                        
                        macro_y_shift += 0.0004
                        
                        relief_cooldown = 10
                        stuck_cnt = 0
                        rospy.logwarn(f"⚠️ 物理受阻 (Act/Exp={actual_speed:.3f}/{expected_speed:.3f})！瞬发退缩且基准外移 -> {macro_y_shift:.4f}m")
                else:
                    y_offset_relief = 0.0
                    relief_cooldown = 0
                    stuck_cnt = 0
                    
                fixed_press_depth = 0.001
                if wp['phase'] in ['draw', 'touch_down']:
                    target_y = wp['y'] - fixed_press_depth + y_offset_relief + macro_y_shift
                else:
                    target_y = wp['y'] + 0.015 + macro_y_shift
                    
                dy = target_y - curr_pos[1]
                
                if relief_cooldown > 0:
                    relief_cooldown -= 1
                    cmd_vx = 0.0
                    cmd_vz = 0.0
                else:
                    if wp['phase'] in ['draw', 'touch_down']:
                        y_offset_relief -= 0.005 * dt # 平滑降落优化 (5mm/s)
                        y_offset_relief = max(0.0, y_offset_relief)
                        
                        if y_offset_relief < 0.0001:
                            macro_y_shift -= 0.0001 * dt
                        
                if dy > 0:
                    cmd_vy = np.clip(8.0 * dy, 0.0, 0.05) # 向外退缩可快速
                else:
                    if y_offset_relief > 0.0001:
                        cmd_vy = np.clip(4.0 * dy, -0.04, 0.0) # 快速恢复
                    else:
                        cmd_vy = np.clip(1.0 * dy, -0.01, 0.0) # 轻柔压入
                        
                # 将 X, Y, Z 的速度统一通过底层刚性下发
                self.send_cartesian_velocity(cmd_vx, cmd_vy, cmd_vz)
                rate.sleep()
                
            rospy.loginfo(f"侧绘进度: {i+1}/{len(aligned_waypoints)} | 面内速度: {actual_speed:.3f}/{expected_speed:.3f} | Y轴退缩: {y_offset_relief:.4f}m")
            
        rospy.loginfo("🛑 绘制到达终点，沿 +Y 轴向外拔出...")
        for _ in range(40):
            if rospy.is_shutdown(): break
            self.send_cartesian_velocity(0.0, 0.03, 0.0)
            rospy.sleep(0.025)
            
        for _ in range(15):
            self.send_cartesian_velocity(0.0, 0.0, 0.0)
            rospy.sleep(0.01)
            
        rospy.loginfo("🎉 -Y 轴侧面绘制任务圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 side_contact_draw.py <path_to_2d_csv>")
        sys.exit(1)
        
    try:
        drawer = SideContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
