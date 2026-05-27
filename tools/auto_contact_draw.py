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
from kortex_driver.msg import TwistCommand

# 动态添加路径以兼容各种运行方式导入 jacobian
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm
from scipy.spatial.transform import Rotation as R

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(0.0, 180.0, 0.0)):
    """
    计算末端姿态四元数，确保笔尖垂直于所绘平面
    """
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

class AutoContactDrawer:
    def __init__(self):
        rospy.init_node('auto_contact_draw', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 核心力控参数
        self.target_force = 7.0       # 目标接触力 7N
        self.contact_threshold = 4.0  # 接触与寻面判定阈值 4.0N
        self.wrist_torque_threshold = 0.06 # 末端关节 (第 6 关节) 扭矩接触跳变阈值 0.06 N.m
        
        self.kp_up = 0.005            # 过度按压抬升增益 (快速向上抬)
        self.kd_up = 0.001
        self.kp_down = 0.0008         # 接触不足下压增益 (缓慢向下压)
        self.kd_down = 0.0001
        
        self.max_step = 0.01          # 单周期最大调整量
        self.z_offset = 0.0           # 虚拟 Z 轴力控累积位移 (用于防飞车限位)
        self.max_z_offset = 0.015     # 最大抬升位移限制 (1.5 cm)
        self.min_z_offset = -0.03     # 最大下压位移限制 (-3.0 cm)
        
        # 零点力校准状态
        self.fz_bias = 0.0
        self.calibration_samples = []
        self.calibrated = False
        self.current_fz = 0.0
        self.wrist_torque = 0.0
        self.prev_force_error = 0.0
        
        # 实时笛卡尔坐标缓存 (由正运动学在回调中解算更新，频率极高)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 订阅关节状态话题以实时进行力矩估计与位姿解算
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 控制指令发布话题
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_fz_pub = rospy.Publisher("/force_control/auto/estimated_fz", Float64, queue_size=1)
        self.z_offset_pub = rospy.Publisher("/force_control/auto/z_offset", Float64, queue_size=1)
        
        # MoveIt 控制器接口
        import moveit_commander
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
        self.move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
    def joint_states_callback(self, msg):
        """
        基于雅可比转置和消除零偏后的力估计，同时更新实时正运动学末端位置
        """
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6:
            return
            
        # 1. 求解末端正运动学位置 (高频更新，不需要卡顿的 TF)
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
            
        # 2. 求解基础雅可比矩阵并计算估计力
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        raw_fz = tool_force[2]
        
        # 提取第 6 关节 (末端手腕) 的力矩绝对值
        self.wrist_torque = abs(torques[5])
        
        # 自动零点校准
        if not self.calibrated:
            self.calibration_samples.append(raw_fz)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除重力偏差 (Z Bias): {self.fz_bias:.2f} N")
            return
            
        # 估计末端 Z 轴向力（减去零点偏差并取绝对值）
        self.current_fz = abs(raw_fz - self.fz_bias)
        self.force_fz_pub.publish(Float64(self.current_fz))

    def align_wrist_to_vertical(self):
        """
        强制将机械臂设置到完全垂直朝下的旋转角
        """
        rospy.loginfo("🔄 正在调整末端手腕至完全垂直姿态...")
        
        # 仅指定姿态目标，不限制位置 (允许 MoveIt 在解算时自动微调 XY 坐标以获取可行解)
        vertical_quat = get_orientation_for_normal(0, 0, 1) # 垂直向下
        self.move_group.set_orientation_target([vertical_quat.x, vertical_quat.y, vertical_quat.z, vertical_quat.w])
        
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            rospy.loginfo("✅ 末端手腕已成功翻转到完全垂直朝下姿态！")
        else:
            rospy.logerr("❌ 手腕翻转垂直姿态规划失败！请检查是否有碰撞或奇异点。")
            raise RuntimeError("手腕姿态初始化失败")

    def run_auto_touchdown(self):
        """
        全自动双阈值下探寻面：控制机械臂向下慢移，直到第6关节力矩跳变且Z轴力达标时刹车
        """
        # 1. 首先确保手腕完全垂直
        self.align_wrist_to_vertical()
        
        rospy.loginfo("🚀 开始自动下探寻面程序...")
        # 确保已校准
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40) # 40Hz 控制循环
        
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 0 # 基座坐标系
        down_cmd.twist.linear_z = -0.005 # -5mm/s 慢速下落
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 0
        
        contact_detected = False
        
        while not rospy.is_shutdown():
            # 双重保险接触判定：第 6 关节力矩跳变 (>=0.06 N.m) 且 Z 轴总估算力大于等于 3.0N
            # 这样能 100% 避免因机械臂运动导致的空气中力矩虚假跳变
            if self.wrist_torque >= self.wrist_torque_threshold and self.current_fz >= 3.0:
                rospy.loginfo(f"🟢 判定触及物体表面！")
                rospy.loginfo(f"   >> 末端力矩 (Joint_6 Effort): {self.wrist_torque:.3f} N.m (阈值: {self.wrist_torque_threshold} N.m)")
                rospy.loginfo(f"   >> 估计接触力 (Fz): {self.current_fz:.2f} N (阈值: 3.0 N)")
                
                # 瞬间停止并锁死
                for _ in range(10):
                    self.vel_pub.publish(stop_cmd)
                    rospy.sleep(0.005)
                contact_detected = True
                break
                
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if contact_detected:
            rospy.sleep(0.5) # 延时确保机械臂彻底静止
            current_pose = self.move_group.get_current_pose().pose
            rospy.loginfo(f"📍 接触坐标基准锁定: X={current_pose.position.x:.3f}, Y={current_pose.position.y:.3f}, Z={current_pose.position.z:.3f}")
            return current_pose
        else:
            raise RuntimeError("寻面程序异常终止")

    def update_force_control(self, dt=0.025):
        """
        非对称三段式 PD 力控律
        """
        if not self.calibrated or self.current_fz < self.contact_threshold:
            # 未接触或未完成校准时，重置力控状态，力控不施加位移补偿
            self.prev_force_error = 0.0
            return self.z_offset
            
        force_error = self.target_force - self.current_fz
        d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
        self.prev_force_error = force_error
        
        # 实施非对称控制
        if force_error < 0:
            # 1. 过度按压区 (力 > 7.0N)：快速向上抬笔
            delta_z = self.kp_up * force_error + self.kd_up * d_error
        elif self.current_fz >= 5.0:
            # 2. 允许浮动区 (5.0N <= 力 <= 7.0N)：允许浮动的死区，不调整位移
            delta_z = 0.0
        else:
            # 3. 即将脱离区 (4.0N <= 力 < 5.0N)：极慢速度向下补偿压紧
            delta_z = self.kp_down * force_error + self.kd_down * d_error
            
        # 裁切单周期位移
        delta_z = np.clip(delta_z, -self.max_step, self.max_step)
        self.z_offset += delta_z
        
        # 触发防飞车保护
        if self.z_offset >= self.max_z_offset and delta_z > 0:
            self.z_offset = self.max_z_offset
        elif self.z_offset <= self.min_z_offset and delta_z < 0:
            self.z_offset = self.min_z_offset
            
        self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
        
        self.force_fz_pub.publish(Float64(self.current_fz))
        self.z_offset_pub.publish(Float64(self.z_offset))
        
        return self.z_offset

    def execute_and_draw(self, csv_file):
        """
        全自动寻面对齐，并在高频速度伺服 + 非对称力控下运行绘制
        """
        # 1. 读取原始轨迹
        raw_waypoints = []
        rospy.loginfo(f"正在载入原始轨迹文件: {csv_file}")
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append({
                    'x': float(row['x_m']),
                    'y': float(row['y_m']),
                    'z_nominal': float(row['z_m']),
                    'nx': float(row['nx']),
                    'ny': float(row['ny']),
                    'nz': float(row['nz']),
                    'stroke_id': int(row['stroke_id']),
                    'phase': row['phase']
                })
        
        # 2. 自动下探寻面 (包含预先校正手腕为垂直姿态)
        contact_pose = self.run_auto_touchdown()
        
        # 3. 动态轨迹原点对齐
        first_draw_idx = 0
        for idx, wp in enumerate(raw_waypoints):
            if wp['phase'] in ['draw', 'touch_down']:
                first_draw_idx = idx
                break
                
        u_ref_x = raw_waypoints[first_draw_idx]['x']
        u_ref_y = raw_waypoints[first_draw_idx]['y']
        u_ref_z = raw_waypoints[first_draw_idx]['z_nominal']
        
        rospy.loginfo("🔄 正在基于实际物理接触点在线重生成轨迹...")
        aligned_waypoints = []
        for wp in raw_waypoints:
            aligned_wp = {
                'x': contact_pose.position.x + (wp['x'] - u_ref_x),
                'y': contact_pose.position.y + (wp['y'] - u_ref_y),
                'z_nominal': contact_pose.position.z + (wp['z_nominal'] - u_ref_z),
                'nx': wp['nx'],
                'ny': wp['ny'],
                'nz': wp['nz'],
                'phase': wp['phase'],
                'stroke_id': wp['stroke_id']
            }
            aligned_waypoints.append(aligned_wp)
            
        rospy.loginfo("✅ 轨迹对准成功！开始启动高频速度伺服绘图...")
        
        # 4. 高频速度伺服跟踪与力控循环
        rate = rospy.Rate(40) # 40Hz
        dt = 0.025
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break
                
            quat = get_orientation_for_normal(wp['nx'], wp['ny'], wp['nz'])
            
            # 计算轨迹点期望的 3D 绝对坐标 (融合静态坐标与力控 Z 轴累积偏移)
            if wp['phase'] in ['draw', 'touch_down']:
                self.update_force_control(dt)
            else:
                self.z_offset = 0.0
                self.prev_force_error = 0.0
                
            target_x = wp['x']
            target_y = wp['y']
            target_z = wp['z_nominal'] + self.z_offset
            
            # 比例位置控制跟踪算法 (用于 XY 轨迹伺服)
            k_pos = 1.2
            
            # 在控制循环中持续朝向目标点发布速度，直至到达目标点 5mm 范围内
            while not rospy.is_shutdown():
                # 实时计算当前点与目标点的偏差
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dz = target_z - self.current_z
                
                # 距离判断：小于 5mm 则切换至下一个航点
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.005:
                    break
                    
                # 速度伺服计算
                cmd = TwistCommand()
                cmd.reference_frame = 0 # 基座坐标系
                cmd.duration = 0
                
                # XY 方向速度伺服
                cmd.twist.linear_x = np.clip(k_pos * dx, -0.025, 0.025) # 限幅 2.5 cm/s
                cmd.twist.linear_y = np.clip(k_pos * dy, -0.025, 0.025)
                
                # Z 方向受力控接管
                if wp['phase'] in ['draw', 'touch_down'] and self.current_fz >= self.contact_threshold:
                    # 接触状态下：使用力控计算出的速度修正量
                    # 力控 PD 速度：v = - (Kp * e_f + Kd * d_e_f) 
                    force_error = self.target_force - self.current_fz
                    d_error = (force_error - self.prev_force_error) / dt
                    self.prev_force_error = force_error
                    
                    if force_error < 0:
                        v_z_comp = -(self.kp_up * force_error + self.kd_up * d_error)
                    elif self.current_fz >= 5.0:
                        v_z_comp = 0.0
                    else:
                        v_z_comp = -(self.kp_down * force_error + self.kd_down * d_error)
                        
                    # 物理防飞车截断
                    self.z_offset += v_z_comp * dt
                    if self.z_offset >= self.max_z_offset and v_z_comp > 0:
                        v_z_comp = 0.0
                    elif self.z_offset <= self.min_z_offset and v_z_comp < 0:
                        v_z_comp = 0.0
                    self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
                    
                    cmd.twist.linear_z = np.clip(v_z_comp, -0.02, 0.02)
                else:
                    # 悬空/过渡状态下：直接进行 Z 轴位置比例伺服，以 1.5 cm/s 逼近
                    cmd.twist.linear_z = np.clip(k_pos * dz, -0.015, 0.015)
                
                # 手腕姿态直接赋值（防止旋转飞车，由 Twist 保持当前目标面的垂直角度，Gen3-Lite底层会自动规划姿态过渡）
                # 这里我们保持角速度为 0，因为姿态已在前置 move_group 中完成对齐，微小的路径偏差只需平移即可
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0
                
                self.vel_pub.publish(cmd)
                self.z_offset_pub.publish(Float64(self.z_offset))
                rate.sleep()
                
            rospy.loginfo(f"点进度: {i+1}/{len(aligned_waypoints)} | Fz: {self.current_fz:.2f}N | Offset Z: {self.z_offset:.4f}m")
            
        # 5. 绘制结束，抬笔停机
        rospy.loginfo("🛑 绘制完毕，垂直提笔并停机...")
        stop_cmd = TwistCommand()
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        self.move_group.stop()
        rospy.loginfo("🎉 全自动寻面与伺服力控绘制任务圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 auto_contact_draw.py <path_to_csv>")
        sys.exit(1)
        
    try:
        drawer = AutoContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
