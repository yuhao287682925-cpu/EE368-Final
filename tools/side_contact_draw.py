#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from kortex_driver.msg import TwistCommand
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
        
        self.fy_bias = 0.0
        self.calibration_samples = []
        self.calibrated = False
        self.current_fy = 0.0
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_pub = rospy.Publisher("/force_control/auto/estimated_f_normal", StdFloat64, queue_size=1)

    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6: return
            
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
        
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        
        # 面向 -Y 寻面，主要受力轴为 Y
        raw_fy = tool_force[1]
        
        if not self.calibrated:
            self.calibration_samples.append(raw_fy)
            if len(self.calibration_samples) >= 40:
                self.fy_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除偏置 (Y Bias): {self.fy_bias:.2f} N")
            return
            
        # -Y 方向推进，取绝对值作为阻力
        self.current_fy = abs(raw_fy - self.fy_bias)
        self.force_pub.publish(StdFloat64(self.current_fy))

    def run_auto_touchdown(self):
        rospy.loginfo("🚀 开始沿 -Y 轴直线寻面...")
        
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        # -Y 轴前移探测
        down_cmd.twist.linear_x = 0.0 
        down_cmd.twist.linear_y = -0.015
        down_cmd.twist.linear_z = 0.0
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 7
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            recent_forces.append(self.current_fy)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
                
            if loop_cnt > 60:
                if len(recent_forces) >= verify_size and all(f >= 12.0 for f in recent_forces):
                    rospy.loginfo(f"🟢 判定触及纸箱表面！Y 轴接触力: {self.current_fy:.2f} N")
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
                    
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if contact_detected:
            rospy.sleep(0.5)
            rospy.loginfo(f"📍 寻面起点锁定: X={self.current_x:.4f}, Y={self.current_y:.4f}, Z={self.current_z:.4f}")
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
        
        # 让寻面的接触点成为整个轨迹的“正下方最低点”
        draw_wps = [wp for wp in raw_waypoints if wp['phase'] in ['draw', 'touch_down']]
        if not draw_wps:
            draw_wps = raw_waypoints
            
        min_y = min(wp['y'] for wp in draw_wps)
        max_x = max(wp['x'] for wp in draw_wps)
        min_x = min(wp['x'] for wp in draw_wps)
        
        # u_ref 取轨迹的横向中心，保证画作左右居中在接触点上方
        u_ref = (max_x + min_x) / 2.0
        # v_ref 取轨迹的最低点，保证 dv 始终 >= 0，即 Z 始终大于接触点高度
        v_ref = min_y
        
        aligned_waypoints = []
        for wp in raw_waypoints:
            du = wp['x'] - u_ref
            dv = wp['y'] - v_ref
            
            # Y 控制深度 (往 -Y 压入)。
            # X 控制左右：面向 -Y 时，右边是 -X 轴。如果轨迹右边增加(x增大)，X轴要减小。
            # Z 控制上下：Z 轴朝上。y 增大时，Z增加。
            aligned_waypoints.append({
                'x': contact_x - du,
                'y': contact_y, 
                'z': contact_z + dv, 
                'phase': wp['phase']
            })
            
        rate = rospy.Rate(40)
        dt = 0.025
        y_offset_relief = 0.0
        draw_force_window = []
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown(): break
                
            k_pos = 1.2
            stuck_cnt = 0
            prev_pos = np.array([self.current_x, self.current_y, self.current_z])
            
            while not rospy.is_shutdown():
                curr_pos = np.array([self.current_x, self.current_y, self.current_z])
                
                # XZ 平面的位移误差
                err_x = wp['x'] - curr_pos[0]
                err_z = wp['z'] - curr_pos[2]
                dist_to_target = math.hypot(err_x, err_z)
                
                if dist_to_target < 0.005:
                    break
                    
                draw_force_window.append(self.current_fy)
                if len(draw_force_window) > 4: draw_force_window.pop(0)
                fy_filtered = np.mean(draw_force_window)
                
                if wp['phase'] in ['draw', 'touch_down'] and dist_to_target > 0.01:
                    if np.linalg.norm(curr_pos - prev_pos) < 0.0001:
                        stuck_cnt += 1
                    else:
                        stuck_cnt = max(0, stuck_cnt - 1)
                else:
                    stuck_cnt = 0
                prev_pos = curr_pos
                
                # 安全泄压：如果在 -Y 轴上遇到大阻力，往 +Y 退缩
                if wp['phase'] in ['draw', 'touch_down']:
                    if fy_filtered > 10.0 or stuck_cnt > 8:
                        y_offset_relief += 0.005 * dt
                    elif fy_filtered < 5.0:
                        y_offset_relief -= 0.002 * dt
                    y_offset_relief = np.clip(y_offset_relief, 0.0, 0.015)
                else:
                    y_offset_relief = 0.0
                    
                # 固定深度：向 -Y 压入 3mm
                fixed_depth = 0.003
                if wp['phase'] in ['draw', 'touch_down']:
                    depth_offset = fixed_depth - y_offset_relief
                else:
                    depth_offset = -0.015 # hover阶段，往 +Y 方向拔出 15mm 避免刮擦
                
                # 目标 Y 为接触面减去定深偏移 (往 -Y 压，所以减去正数)
                
                # 目标 Y 为接触面往里压 (- depth_offset)
                target_y = wp['y'] - depth_offset
                err_y = target_y - curr_pos[1]
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                cmd.duration = 0
                cmd.twist.linear_x = np.clip(k_pos * err_x, -0.06, 0.06)
                cmd.twist.linear_y = np.clip(k_pos * err_y, -0.06, 0.06)
                cmd.twist.linear_z = np.clip(k_pos * err_z, -0.06, 0.06)
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            rospy.loginfo(f"进度: {i+1}/{len(aligned_waypoints)} | Y轴向力: {self.current_fy:.2f}N | Y退缩: {y_offset_relief:.4f}m")
            
        rospy.loginfo("🛑 绘制到达终点，沿 +Y 轴向外拔出...")
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        # Y 轴后退拔出 3cm (往 +Y)
        lift_cmd.twist.linear_x = 0.0
        lift_cmd.twist.linear_y = 0.03
        lift_cmd.twist.linear_z = 0.0
        
        for _ in range(40):
            if rospy.is_shutdown(): break
            self.vel_pub.publish(lift_cmd)
            rospy.sleep(0.025)
            
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
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
