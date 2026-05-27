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

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(22.688, 175.755, 83.736)):
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
        
        # 非对称力控核心参数
        self.target_force = 7.0       # 目标接触力 7N
        self.contact_threshold = 4.0  # 接触与寻面判定阈值 4.0N
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
        self.prev_force_error = 0.0
        
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
        基于雅可比转置和消除零偏后的力估计
        """
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6:
            return
            
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        raw_fz = tool_force[2]
        
        if not self.calibrated:
            self.calibration_samples.append(raw_fz)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除重力偏差 (Z Bias): {self.fz_bias:.2f} N")
            return
            
        self.current_fz = abs(raw_fz - self.fz_bias)
        self.force_fz_pub.publish(Float64(self.current_fz))

    def run_auto_touchdown(self):
        """
        全自动下探寻面：控制机械臂以 5mm/s 速度向下移动，直到接触力达 4N 时停止
        """
        rospy.loginfo("🚀 开始自动下探寻面程序...")
        # 确保已校准
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40) # 40Hz 控制循环
        
        # 缓慢下探速度定义
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 0 # 基座坐标系
        down_cmd.twist.linear_z = -0.005 # -5mm/s 向下
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 0
        
        contact_detected = False
        
        while not rospy.is_shutdown():
            # 检测力是否达到寻面阈值 (4.0N)
            if self.current_fz >= self.contact_threshold:
                rospy.loginfo(f"🟢 检测到平面接触！当前受力: {self.current_fz:.2f} N >= 4.0 N")
                # 瞬间停止并锁死
                for _ in range(5):
                    self.vel_pub.publish(stop_cmd)
                    rospy.sleep(0.01)
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

    def update_force_control(self, dt=0.05):
        """
        非对称三段式 PD 力控律
        """
        if not self.calibrated or self.current_fz < self.contact_threshold:
            # 未进入接触状态或未完成校准，不计算 PD，重置状态
            self.prev_force_error = 0.0
            return self.z_offset
            
        # 接触力误差计算 (目标 7.0N)
        force_error = self.target_force - self.current_fz
        d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
        self.prev_force_error = force_error
        
        # 实施非对称增益分配
        if force_error < 0:
            # 1. 过度按压区 (力 > 7.0N)：快速向上抬笔
            delta_z = self.kp_up * force_error + self.kd_up * d_error
        elif self.current_fz >= 5.0:
            # 2. 允许浮动区 (5.0N <= 力 <= 7.0N)：浮动死区，不作位移调整
            delta_z = 0.0
        else:
            # 3. 即将脱离区 (4.0N <= 力 < 5.0N 或完全悬空边缘)：采用极小增益极慢下压
            delta_z = self.kp_down * force_error + self.kd_down * d_error
            
        # 饱和限幅
        delta_z = np.clip(delta_z, -self.max_step, self.max_step)
        
        # 积分虚拟位移偏移
        self.z_offset += delta_z
        
        # 防飞车限制器
        if self.z_offset >= self.max_z_offset and delta_z > 0:
            self.z_offset = self.max_z_offset
        elif self.z_offset <= self.min_z_offset and delta_z < 0:
            self.z_offset = self.min_z_offset
            
        # 重新裁切范围
        self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
        
        # 发布调试话题
        self.force_fz_pub.publish(Float64(self.current_fz))
        self.z_offset_pub.publish(Float64(self.z_offset))
        
        return self.z_offset

    def execute_and_draw(self, csv_file):
        """
        载入轨迹文件，下探寻面，动态重映射，并在非对称力控下执行绘制
        """
        # 1. 读取原始 CSV 轨迹
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
        
        # 2. 全自动下探寻面
        contact_pose = self.run_auto_touchdown()
        
        # 3. 动态原点平移和轨迹对齐
        # 我们假设首个 phase 为 'draw' 或是 'touch_down' 的航点为轨迹绘制起点
        first_draw_idx = 0
        for idx, wp in enumerate(raw_waypoints):
            if wp['phase'] in ['draw', 'touch_down']:
                first_draw_idx = idx
                break
                
        # 提取首个绘制点的原始几何坐标
        u_ref_x = raw_waypoints[first_draw_idx]['x']
        u_ref_y = raw_waypoints[first_draw_idx]['y']
        u_ref_z = raw_waypoints[first_draw_idx]['z_nominal']
        
        rospy.loginfo("🔄 正在基于实际物理接触点在线重生成轨迹...")
        aligned_waypoints = []
        for wp in raw_waypoints:
            # 以物理接触坐标为起点进行整体平移偏置
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
            
        rospy.loginfo("✅ 轨迹在线动态对齐完成！开始按规划路径绘图。")
        
        # 4. 执行轨迹并叠加实时非对称力控
        rate = rospy.Rate(20) # 20Hz 频率执行
        dt = 0.05
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break
                
            quat = get_orientation_for_normal(wp['nx'], wp['ny'], wp['nz'])
            
            # 只在落笔 (touch_down) 和绘制 (draw) 阶段介入非对称力控
            if wp['phase'] in ['draw', 'touch_down']:
                self.update_force_control(dt)
            else:
                # 自由空间悬空时重置偏移
                self.z_offset = 0.0
                self.prev_force_error = 0.0
                
            # 融合静态目标点与力控累积位移偏移量
            target_pose = Pose()
            target_pose.position.x = wp['x']
            target_pose.position.y = wp['y']
            target_pose.position.z = wp['z_nominal'] + self.z_offset
            target_pose.orientation = quat
            
            # 使用 MoveIt 控制机械臂跟进到带力控补偿后的新坐标
            self.move_group.set_pose_target(target_pose)
            self.move_group.go(wait=True)
            
            rospy.loginfo(f"进度: {i+1}/{len(aligned_waypoints)} | Phase: {wp['phase']} | Fz: {self.current_fz:.2f}N | Offset Z: {self.z_offset:.4f}m")
            rate.sleep()
            
        rospy.loginfo("🎉 全自动寻面与力控绘制任务执行完毕！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 auto_contact_draw.py <path_to_csv>")
        sys.exit(1)
        
    try:
        drawer = AutoContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
