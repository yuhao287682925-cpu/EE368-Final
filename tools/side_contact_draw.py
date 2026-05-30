#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64 as StdFloat64
from kortex_driver.msg import TwistCommand

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
        
        # 实时笛卡尔坐标缓存
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 零点力校准状态
        self.fz_bias = 0.0
        self.calibration_samples = []
        self.calibrated = False
        self.current_f_normal = 0.0
        self.is_static = True
        
        # 笔尖指向 (向纸里扎的方向)
        self.pen_dir = np.array([1.0, 0.0, 0.0]) # 默认正前方
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_pub = rospy.Publisher("/force_control/auto/estimated_f_normal", StdFloat64, queue_size=1)
        
        # MoveIt 控制器接口 (仅用于获取高精度位姿和停止，不用于运动规划)
        import moveit_commander
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
        self.move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        velocities = msg.velocity[0:6] if msg.velocity else []
        
        if len(thetas) < 6 or len(torques) < 6 or len(velocities) < 6:
            return
            
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
        
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        self.is_static = all(abs(v) < 0.005 for v in velocities)
        
        # 计算沿 pen_dir 的受力
        f_normal_raw = tool_force[0]*self.pen_dir[0] + tool_force[1]*self.pen_dir[1] + tool_force[2]*self.pen_dir[2]
        
        if not self.calibrated:
            self.calibration_samples.append(f_normal_raw)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除偏置 (Bias): {self.fz_bias:.2f} N")
            return
            
        self.current_f_normal = abs(f_normal_raw - self.fz_bias)
        self.force_pub.publish(StdFloat64(self.current_f_normal))

    def run_auto_touchdown(self):
        rospy.loginfo("🚀 开始沿笔尖方向直线寻面...")
        
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        # 沿笔尖方向下探 15mm/s
        down_cmd.twist.linear_x = 0.015 * self.pen_dir[0]
        down_cmd.twist.linear_y = 0.015 * self.pen_dir[1]
        down_cmd.twist.linear_z = 0.015 * self.pen_dir[2]
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 7
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            recent_forces.append(self.current_f_normal)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
                
            if loop_cnt > 60:
                if len(recent_forces) >= verify_size and all(f >= 12.0 for f in recent_forces):
                    rospy.loginfo(f"🟢 判定触及纸箱表面！接触力: {self.current_f_normal:.2f} N")
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
                    
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if contact_detected:
            rospy.sleep(0.5)
            rospy.loginfo(f"📍 寻面接触起点锁定: X={self.current_x:.4f}, Y={self.current_y:.4f}, Z={self.current_z:.4f}")
            return np.array([self.current_x, self.current_y, self.current_z])
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
        
        if not raw_waypoints:
            return
            
        # 1. 提取当前真实笔尖朝向
        rospy.loginfo("🚗 获取当前物理笔尖朝向作为绘制深度方向...")
        pose = self.move_group.get_current_pose().pose
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        r = R.from_quat(q)
        
        # 对于 Kinova Gen3 lite，末端 Z 轴是向外的 (往纸里扎的方向)
        pen_dir = r.apply([0, 0, 1])
        self.pen_dir = pen_dir / np.linalg.norm(pen_dir)
        rospy.loginfo(f"🌐 笔尖前进方向 (Normal): [{self.pen_dir[0]:.3f}, {self.pen_dir[1]:.3f}, {self.pen_dir[2]:.3f}]")
        
        # 2. 构造局部 YZ 映射平面
        # 尽量让 2D 的 +y 对应世界的 +Z (上方)
        global_up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(self.pen_dir, global_up)) > 0.95:
            # 如果是往下扎，那就给个朝前的 UP
            global_up = np.array([1.0, 0.0, 0.0])
            
        y_axis = global_up - np.dot(global_up, self.pen_dir) * self.pen_dir
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # 确保 (x_axis, y_axis, pen_dir) 是右手系: x cross y = pen_dir => x = y cross pen_dir
        x_axis = np.cross(y_axis, self.pen_dir)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        rospy.loginfo(f"🌐 局部 X 轴 (对应轨迹左右): [{x_axis[0]:.3f}, {x_axis[1]:.3f}, {x_axis[2]:.3f}]")
        rospy.loginfo(f"🌐 局部 Y 轴 (对应轨迹上下): [{y_axis[0]:.3f}, {y_axis[1]:.3f}, {y_axis[2]:.3f}]")

        # 3. 自动寻面
        contact_pt = self.run_auto_touchdown()
        
        # 4. 原点对齐与 3D 轨迹重生成
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
            
            # 计算 3D 坐标：contact_pt 为原点，横向走 x_axis，纵向走 y_axis
            pt_3d = contact_pt + du * x_axis + dv * y_axis
            aligned_waypoints.append({
                'x': pt_3d[0],
                'y': pt_3d[1],
                'z': pt_3d[2],
                'phase': wp['phase']
            })
            
        # 5. 高频位置伺服控制
        rate = rospy.Rate(40)
        dt = 0.025
        z_offset_relief = 0.0
        draw_force_window = []
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break
                
            k_pos = 1.2
            stuck_cnt = 0
            prev_pos = np.array([self.current_x, self.current_y, self.current_z])
            
            while not rospy.is_shutdown():
                curr_pos = np.array([self.current_x, self.current_y, self.current_z])
                
                # 计算目标点在局部平面的距离
                err_vec = np.array([wp['x'], wp['y'], wp['z']]) - curr_pos
                err_depth = np.dot(err_vec, self.pen_dir)
                err_plane = err_vec - err_depth * self.pen_dir
                dist_to_target = np.linalg.norm(err_plane)
                
                if dist_to_target < 0.005:
                    break
                    
                draw_force_window.append(self.current_f_normal)
                if len(draw_force_window) > 4: draw_force_window.pop(0)
                fz_filtered = np.mean(draw_force_window)
                
                if wp['phase'] in ['draw', 'touch_down'] and dist_to_target > 0.01:
                    if np.linalg.norm(curr_pos - prev_pos) < 0.0001:
                        stuck_cnt += 1
                    else:
                        stuck_cnt = max(0, stuck_cnt - 1)
                else:
                    stuck_cnt = 0
                prev_pos = curr_pos
                
                if wp['phase'] in ['draw', 'touch_down']:
                    if fz_filtered > 10.0 or stuck_cnt > 8:
                        z_offset_relief += 0.005 * dt
                    elif fz_filtered < 5.0:
                        z_offset_relief -= 0.002 * dt
                    z_offset_relief = np.clip(z_offset_relief, 0.0, 0.015)
                else:
                    z_offset_relief = 0.0
                    
                fixed_depth = 0.003
                depth_offset = fixed_depth - z_offset_relief if wp['phase'] in ['draw', 'touch_down'] else 0.0
                
                target_3d = np.array([wp['x'], wp['y'], wp['z']]) + depth_offset * self.pen_dir
                err_3d = target_3d - curr_pos
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                cmd.duration = 0
                cmd.twist.linear_x = np.clip(k_pos * err_3d[0], -0.06, 0.06)
                cmd.twist.linear_y = np.clip(k_pos * err_3d[1], -0.06, 0.06)
                cmd.twist.linear_z = np.clip(k_pos * err_3d[2], -0.06, 0.06)
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            rospy.loginfo(f"点进度: {i+1}/{len(aligned_waypoints)} | 接触力: {self.current_f_normal:.2f}N | 泄压量: {z_offset_relief:.4f}m")
            
        rospy.loginfo("🛑 绘制到达终点，拔出画笔...")
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        # 沿笔尖反方向拔出 3cm
        lift_cmd.twist.linear_x = -0.03 * self.pen_dir[0]
        lift_cmd.twist.linear_y = -0.03 * self.pen_dir[1]
        lift_cmd.twist.linear_z = -0.03 * self.pen_dir[2]
        
        for _ in range(40):
            if rospy.is_shutdown(): break
            self.vel_pub.publish(lift_cmd)
            rospy.sleep(0.025)
            
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        self.move_group.stop()
        rospy.loginfo("🎉 自动笔尖投影绘制任务圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 side_contact_draw.py <path_to_2d_csv>")
        sys.exit(1)
        
    try:
        drawer = SideContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
