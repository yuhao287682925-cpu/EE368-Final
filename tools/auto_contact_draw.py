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
        
        # 核心力控与对刀判定参数
        self.target_force = 7.0       # 目标接触力 7N
        self.contact_threshold = 4.0  # 接触与力控激活判定阈值 4.0N
        self.wrist_torque_threshold = 0.06 # 末端关节 (第 6 关节) 扭矩接触跳变阈值 0.06 N.m
        
        self.kp_up = 0.005            # 过度按压抬升增益 (快速向上抬)
        self.kd_up = 0.001
        self.kp_down = 0.0008         # 接触不足下压增益 (缓慢向下压)
        self.kd_down = 0.0001
        
        self.max_step = 0.01          # 单周期最大位移微调量
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
        
        # 实时笛卡尔坐标缓存
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
            
        # 1. 求解末端正运动学位置
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
                rospy.loginfo(f"✅ 传感器零点校准完成！消除偏置 (Z Bias): {self.fz_bias:.2f} N")
            return
            
        # 估计末端 Z 轴向力（减去零点偏差并取绝对值）
        self.current_fz = abs(raw_fz - self.fz_bias)
        self.force_fz_pub.publish(Float64(self.current_fz))

    def run_auto_touchdown(self):
        """
        动作 1：全自动下探寻面。控制机械臂以 5mm/s 速度向下移动，直到双阈值判定接触时停机。
        """
        rospy.loginfo("🚀 开始自动下探寻面程序...")
        
        # 强迫机械臂在启动前静止 1.5 秒，重新去皮校零，完全平息之前移动带来的残余力矩
        rospy.loginfo("⏸️ 机械臂静止中 (1.5秒)，正在平息关节残留力矩并执行高精度校零...")
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.5)
        
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40) # 40Hz
        
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3 # 基座坐标系
        down_cmd.twist.linear_z = -0.015 # -15mm/s 向下，跳出超低速黏滑爬行区
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        
        # 引入接触判定滑动窗口 (40Hz 下 10个周期约 0.25 秒)
        force_window = []
        window_size = 10
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            
            # 滑动窗口维护
            force_window.append(self.current_fz)
            if len(force_window) > window_size:
                force_window.pop(0)
            
            # 计算窗口内的平均估计力
            avg_force = np.mean(force_window) if len(force_window) >= window_size else 0.0
            
            if loop_cnt % 15 == 0:
                rospy.loginfo(f"⏳ 正在直线下探... 瞬时 Fz: {self.current_fz:.2f} N | 平均 Fz(0.25s): {avg_force:.2f} N (阈值: 7.0 N)")
                
            # 起步前 0.5 秒 (约 20 个周期) 内屏蔽判定，避开加速瞬间的惯性力波动
            if loop_cnt > 20:
                # 使用平均估计力进行稳定接触判定
                if len(force_window) >= window_size and avg_force >= 7.0:
                    rospy.loginfo(f"🟢 判定触及纸箱表面！")
                    rospy.loginfo(f"   >> 窗口内平均接触力 (Avg Fz): {avg_force:.2f} N (阈值: 7.0 N)")
                    rospy.loginfo(f"   >> 瞬时接触力 (Inst Fz): {self.current_fz:.2f} N")
                    
                    # 发送 10 次 0 速度，确保驱动层刹停
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
            else:
                if loop_cnt % 10 == 0:
                    rospy.loginfo("⏳ 启动加速平稳期，屏蔽接触判定...")
                    
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if contact_detected:
            rospy.sleep(0.5) # 等待彻底静止
            
            # 使用高精度底层正运动学估算的绝对坐标 (防止 MoveIt 获取超时)
            current_pose = Pose()
            current_pose.position.x = self.current_x
            current_pose.position.y = self.current_y
            current_pose.position.z = self.current_z
            
            rospy.loginfo(f"📍 寻面接触起点锁定: X={current_pose.position.x:.4f}, Y={current_pose.position.y:.4f}, Z={current_pose.position.z:.4f}")
            return current_pose
        else:
            raise RuntimeError("寻面程序异常终止")

    def update_force_control(self, dt=0.025):
        """
        非对称三段式 PD 力控律
        """
        if not self.calibrated or self.current_fz < self.contact_threshold:
            # 未进入接触状态时，重置力控状态与位移量
            self.prev_force_error = 0.0
            self.z_offset = 0.0
            return self.z_offset
            
        force_error = self.target_force - self.current_fz
        d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
        self.prev_force_error = force_error
        
        # 非对称控制增益分配
        if force_error < 0:
            # 1. 过度按压区 (力 > 7.0N)：快速向上抬笔
            delta_z = self.kp_up * force_error + self.kd_up * d_error
        elif self.current_fz >= 5.0:
            # 2. 允许浮动区 (5.0N <= 力 <= 7.0N)：死区不动作
            delta_z = 0.0
        else:
            # 3. 即将脱离区 (4.0N <= 力 < 5.0N)：极慢速度下压
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
        自动寻面对刀对齐，随后高频速度伺服绘制
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
        
        # 2. 全自动下探寻面 (就地直接下探，不进行可能产生水平偏移的姿态校正)
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
            
            # 位置控制的目标点 XY
            target_x = wp['x']
            target_y = wp['y']
            
            k_pos = 1.2
            
            # 位置伺服走点循环
            while not rospy.is_shutdown():
                # 在控制周期内部，基于当前的力控偏移 z_offset 动态更新目标高度
                target_z = wp['z_nominal'] + self.z_offset
                
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dz = target_z - self.current_z
                
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.005: # 到位距离 5mm
                    break
                    
                cmd = TwistCommand()
                cmd.reference_frame = 3 # 基座坐标系
                cmd.duration = 0
                
                # XY 方向伺服速度
                cmd.twist.linear_x = np.clip(k_pos * dx, -0.025, 0.025)
                cmd.twist.linear_y = np.clip(k_pos * dy, -0.025, 0.025)
                
                # Z 方向受力控接管
                if wp['phase'] in ['draw', 'touch_down'] and self.current_fz >= self.contact_threshold:
                    force_error = self.target_force - self.current_fz
                    d_error = (force_error - self.prev_force_error) / dt
                    self.prev_force_error = force_error
                    
                    if force_error < 0:
                        v_z_comp = -(self.kp_up * force_error + self.kd_up * d_error)
                    elif self.current_fz >= 5.0:
                        v_z_comp = 0.0
                    else:
                        v_z_comp = -(self.kp_down * force_error + self.kd_down * d_error)
                        
                    # 虚拟限位限制与累加量更新
                    self.z_offset += v_z_comp * dt
                    if self.z_offset >= self.max_z_offset and v_z_comp > 0:
                        v_z_comp = 0.0
                    elif self.z_offset <= self.min_z_offset and v_z_comp < 0:
                        v_z_comp = 0.0
                    self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
                    
                    cmd.twist.linear_z = np.clip(v_z_comp, -0.02, 0.02)
                else:
                    # 如果发生脱离（力小于 4N）或处于抬笔区，立即将偏置 z_offset 归零，重置力控状态，迫使机械臂向下压紧重新寻面
                    self.z_offset = 0.0
                    self.prev_force_error = 0.0
                    
                    # 重新计算无偏置时的标称高度差，发布下探贴紧速度
                    target_z_nominal = wp['z_nominal']
                    dz_nominal = target_z_nominal - self.current_z
                    cmd.twist.linear_z = np.clip(k_pos * dz_nominal, -0.015, 0.015)
                
                # 保持姿态稳定，角速度设为 0
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
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        self.move_group.stop()
        rospy.loginfo("🎉 全自动伺服力控绘制任务圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 auto_contact_draw.py <path_to_csv>")
        sys.exit(1)
        
    try:
        drawer = AutoContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
