#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TwistCommand
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
        
        # 初始化 Gen3-lite DH 模型
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 实时坐标
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 侧面受力 (Fx) 的零点校准
        self.fx_bias = 0.0
        self.calibration_samples = []
        self.calibrated = False
        self.current_fx = 0.0
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_pub = rospy.Publisher("/force_control/auto/estimated_f_normal", StdFloat64, queue_size=1)
        
        # MoveIt 用于对准姿态
        import moveit_commander
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
        self.move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

    def align_wrist_side(self, target_rpy_deg=(0.0, 90.0, 0.0)):
        rospy.loginfo(f"🔄 正在自动对齐笔尖至侧面姿态: {target_rpy_deg} (准备面朝+X轴)...")
        self.move_group.set_num_planning_attempts(3)
        self.move_group.set_planning_time(2.0)
        
        current_pose = self.move_group.get_current_pose().pose
        target_pose = Pose()
        target_pose.position = current_pose.position
        
        r = R.from_euler('xyz', target_rpy_deg, degrees=True)
        q = r.as_quat()
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        
        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("✅ 姿态已对准，笔尖直指正前方！")
        else:
            rospy.logerr("❌ 姿态对准失败！请检查是否有奇异点或碰撞。")
            sys.exit(1)

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
        
        # 侧面碰撞主要看 X 轴方向的力
        raw_fx = tool_force[0]
        
        if not self.calibrated:
            self.calibration_samples.append(raw_fx)
            if len(self.calibration_samples) >= 40:
                self.fx_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除偏置 (X Bias): {self.fx_bias:.2f} N")
            return
            
        # 机械臂向前推进，受到纸板反作用力，取绝对值作为正压力
        self.current_fx = abs(raw_fx - self.fx_bias)
        self.force_pub.publish(StdFloat64(self.current_fx))

    def run_auto_touchdown(self):
        rospy.loginfo("🚀 开始沿 X 轴正向直线寻面...")
        
        # 重新去皮
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        # X 轴前移 15mm/s 探测表面
        down_cmd.twist.linear_x = 0.015 
        down_cmd.twist.linear_y = 0.0
        down_cmd.twist.linear_z = 0.0
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 7
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            recent_forces.append(self.current_fx)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
                
            if loop_cnt > 60:
                if len(recent_forces) >= verify_size and all(f >= 12.0 for f in recent_forces):
                    rospy.loginfo(f"🟢 判定触及纸箱侧表面！X 轴接触力: {self.current_fx:.2f} N")
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
            
        # 1. 自动对齐姿态 (笔尖朝 +X 轴，即 Pitch=90)
        self.align_wrist_side(target_rpy_deg=(0.0, 90.0, 0.0))
        
        # 2. X 轴前移寻面
        contact_x, contact_y, contact_z = self.run_auto_touchdown()
        
        # 3. YZ 坐标转换 (将 2D 轨迹硬编码映射到机器人的正前方立面上)
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
            
            # X 控制深度
            # 机器人 Y 轴朝左。我们希望轨迹 x 增大时往右画，所以减去 du。
            # 机器人 Z 轴朝上。我们希望轨迹 y 增大时往上画，所以加上 dv。
            aligned_waypoints.append({
                'x': contact_x,
                'y': contact_y - du, 
                'z': contact_z + dv, 
                'phase': wp['phase']
            })
            
        # 4. 高频位置伺服控制
        rate = rospy.Rate(40)
        dt = 0.025
        x_offset_relief = 0.0
        draw_force_window = []
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown(): break
                
            k_pos = 1.2
            stuck_cnt = 0
            prev_pos = np.array([self.current_x, self.current_y, self.current_z])
            
            while not rospy.is_shutdown():
                curr_pos = np.array([self.current_x, self.current_y, self.current_z])
                
                # YZ 平面的位移误差
                err_y = wp['y'] - curr_pos[1]
                err_z = wp['z'] - curr_pos[2]
                dist_to_target = math.hypot(err_y, err_z)
                
                if dist_to_target < 0.005:
                    break
                    
                draw_force_window.append(self.current_fx)
                if len(draw_force_window) > 4: draw_force_window.pop(0)
                fx_filtered = np.mean(draw_force_window)
                
                if wp['phase'] in ['draw', 'touch_down'] and dist_to_target > 0.01:
                    if np.linalg.norm(curr_pos - prev_pos) < 0.0001:
                        stuck_cnt += 1
                    else:
                        stuck_cnt = max(0, stuck_cnt - 1)
                else:
                    stuck_cnt = 0
                prev_pos = curr_pos
                
                # 安全泄压：如果在 X 轴上遇到大阻力，往后退缩
                if wp['phase'] in ['draw', 'touch_down']:
                    if fx_filtered > 10.0 or stuck_cnt > 8:
                        x_offset_relief += 0.005 * dt
                    elif fx_filtered < 5.0:
                        x_offset_relief -= 0.002 * dt
                    x_offset_relief = np.clip(x_offset_relief, 0.0, 0.015)
                else:
                    x_offset_relief = 0.0
                    
                # 固定深度：向 +X 压入 3mm
                fixed_depth = 0.003
                depth_offset = fixed_depth - x_offset_relief if wp['phase'] in ['draw', 'touch_down'] else 0.0
                
                # 目标 X 为接触面加上定深偏移
                target_x = wp['x'] + depth_offset
                err_x = target_x - curr_pos[0]
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                cmd.duration = 0
                cmd.twist.linear_x = np.clip(k_pos * err_x, -0.06, 0.06)
                cmd.twist.linear_y = np.clip(k_pos * err_y, -0.06, 0.06)
                cmd.twist.linear_z = np.clip(k_pos * err_z, -0.06, 0.06)
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            rospy.loginfo(f"进度: {i+1}/{len(aligned_waypoints)} | X轴向力: {self.current_fx:.2f}N | X退缩: {x_offset_relief:.4f}m")
            
        rospy.loginfo("🛑 绘制到达终点，沿 X 轴向后拔出...")
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        # X 轴后退拔出 3cm
        lift_cmd.twist.linear_x = -0.03
        lift_cmd.twist.linear_y = 0.0
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
            
        rospy.loginfo("🎉 X 轴纯净版侧面绘制任务圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 side_contact_draw.py <path_to_2d_csv>")
        sys.exit(1)
        
    try:
        drawer = SideContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
