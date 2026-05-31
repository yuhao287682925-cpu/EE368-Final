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
        rospy.init_node('side_contact_draw_with_log', anonymous=True)
        
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
        
        self.current_thetas = thetas
            
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
        
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        
        raw_f3d = tool_force[0:3]
        self.raw_f3d = raw_f3d
        
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
        if not hasattr(self, 'current_thetas'):
            return
            
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

    def run_auto_touchdown(self):
        rospy.loginfo("🚀 开始沿 -Y 轴软着陆寻面...")
        
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40)
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 4
        
        local_f3d_bias = self.f3d_bias.copy()
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            
            raw_f3d = self.raw_f3d
            if loop_cnt < 60:
                local_f3d_bias = 0.90 * local_f3d_bias + 0.10 * raw_f3d
            elif np.linalg.norm(raw_f3d - local_f3d_bias) < 5.0:
                local_f3d_bias = 0.98 * local_f3d_bias + 0.02 * raw_f3d
                
            current_net_f = np.linalg.norm(raw_f3d - local_f3d_bias)
            
            recent_forces.append(current_net_f)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
                
            if loop_cnt > 60:
                if current_net_f > 2.0:
                    speed_factor = max(0.0, (10.0 - current_net_f) / 8.0)
                    down_speed = -0.005 * speed_factor
                else:
                    down_speed = -0.005
                    
                if len(recent_forces) >= verify_size and all(f >= 10.0 for f in recent_forces):
                    rospy.loginfo(f"🟢 判定触及立式纸箱表面！接触合力: {current_net_f:.2f} N")
                    for _ in range(5):
                        self.send_cartesian_velocity(0.0, 0.0, 0.0)
                        rospy.sleep(0.01)
                    contact_detected = True
                    break
            else:
                down_speed = -0.005
                if loop_cnt % 15 == 0:
                    rospy.loginfo("⏳ 启动加速平稳期，屏蔽接触判定...")
                    
            self.send_cartesian_velocity(0.0, down_speed, 0.0)
            rate.sleep()
            
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
                
        u_ref = raw_waypoints[first_draw_idx]['x']
        v_ref = raw_waypoints[first_draw_idx]['y']
        
        aligned_waypoints = []
        for wp in raw_waypoints:
            du = wp['x'] - u_ref
            dv = wp['y'] - v_ref
            
            aligned_waypoints.append({
                'x': contact_x - du,
                'y': contact_y, 
                'z': contact_z - dv, 
                'phase': wp['phase']
            })
            
        rate = rospy.Rate(40)
        dt = 0.025
        
        rospy.loginfo("✍️ 寻面完成，开始启动纯运动学啄木鸟实战侧绘 (带日志记录)...")
        
        y_offset_relief = 0.0
        relief_cooldown = 0
        stuck_cnt = 0
        
        actual_log = []
        theo_log = []
        
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
                
                err_x = target_x - curr_pos[0]
                err_z = target_z - curr_pos[2]
                dist_to_target = math.hypot(err_x, err_z)
                
                if dist_to_target < 0.001:
                    break
                    
                movement = math.hypot(curr_pos[0] - prev_servo_x, curr_pos[2] - prev_servo_z)
                actual_speed = movement / dt if dt > 0 else 0.0
                
                max_speed = 0.035
                k_pos_near = 6.0
                
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
                
                if wp['phase'] in ['draw', 'touch_down']:
                    if stuck_cnt > 8:
                        y_offset_relief += 0.0035
                        y_offset_relief = min(y_offset_relief, 0.015)
                        relief_cooldown = 10
                        stuck_cnt = 0
                        rospy.logwarn(f"⚠️ 侧面物理受阻 (Actual/Exp={actual_speed:.3f}/{expected_speed:.3f})，触发极速退缩防卡死！")
                else:
                    y_offset_relief = 0.0
                    relief_cooldown = 0
                    stuck_cnt = 0
                    
                fixed_press_depth = 0.001
                if wp['phase'] in ['draw', 'touch_down']:
                    target_y = wp['y'] - fixed_press_depth + y_offset_relief
                else:
                    target_y = wp['y'] + 0.015
                    
                dy = target_y - curr_pos[1]
                
                if relief_cooldown > 0:
                    relief_cooldown -= 1
                    cmd_vx = 0.0
                    cmd_vz = 0.0
                else:
                    if wp['phase'] in ['draw', 'touch_down']:
                        y_offset_relief -= 0.005 * dt
                        y_offset_relief = max(0.0, y_offset_relief)
                        
                if dy > 0:
                    cmd_vy = np.clip(8.0 * dy, 0.0, 0.05)
                else:
                    if y_offset_relief > 0.0001:
                        cmd_vy = np.clip(4.0 * dy, -0.04, 0.0)
                    else:
                        cmd_vy = np.clip(1.0 * dy, -0.01, 0.0)
                        
                # 记录高频日志
                if wp['phase'] == 'draw':
                    actual_log.append({'x': curr_pos[0], 'y': curr_pos[1], 'z': curr_pos[2]})
                    theo_log.append({'x': target_x, 'y': target_y, 'z': target_z})
                        
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
            
        with open('actual_executed_trajectory.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z'])
            writer.writeheader()
            writer.writerows(actual_log)
        with open('theo_mapped_trajectory.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z'])
            writer.writeheader()
            writer.writerows(theo_log)
            
        rospy.loginfo("📊 轨迹日志已保存至 actual_executed_trajectory.csv 和 theo_mapped_trajectory.csv")
        rospy.loginfo("🎉 -Y 轴侧面绘制任务圆满完成！")
        
        try:
            import subprocess
            subprocess.Popen(["python3", "tools/analyze_error.py", "--actual", "actual_executed_trajectory.csv", "--theo", "theo_mapped_trajectory.csv"])
        except Exception as e:
            rospy.logerr(f"启动自动分析失败: {e}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 side_contact_draw_with_log.py <path_to_2d_csv>")
        sys.exit(1)
        
    try:
        drawer = SideContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
