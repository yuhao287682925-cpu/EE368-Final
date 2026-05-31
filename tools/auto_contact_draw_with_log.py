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

# 接触状态机状态常量
FREE_SPACE = 0
SOFT_CONTACT = 1
HARD_CONTACT = 2

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
        
        # 接触状态机初始化
        self.state = FREE_SPACE
        self.state_counter = 0 # 状态迟滞校验帧计数器
        
        # 核心力控与对刀判定参数
        self.base_target_force = 6.0  # 标称绘制压力 6.0N
        self.target_force = 6.0       # 动态目标接触力 (可衰减防卡阻)
        self.contact_threshold = 4.0  # 接触与力控激活判定阈值 4.0N
        self.wrist_torque_threshold = 0.25 # 末端关节 (第 6 关节) 扭矩接触跳变阈值 0.25 N.m (调高以防止频繁误触发抬升)
        
        self.kp_up = 0.005            # 过度按压抬升增益 (快速向上抬)
        self.kd_up = 0.001
        self.kp_down = 0.0008         # 接触不足下压增益 (缓慢向下压)
        self.kd_down = 0.0001
        
        self.max_step = 0.01          # 单周期最大位移微调量
        self.z_offset = 0.0           # 虚拟 Z 轴力控累积位移 (用于防飞车限位)
        self.max_z_offset = 0.008     # 最大抬升位移限制 (0.8 cm，防止悬空)
        self.min_z_offset = -0.03     # 最大下压位移限制 (-3.0 cm)
        
        # 零点力校准状态
        self.fz_bias = 0.0
        self.wrist_torque_bias = 0.0
        self.calibration_samples = []
        self.torque_calibration_samples = []
        self.calibrated = False
        self.current_fz = 0.0
        self.raw_fz = 0.0
        self.is_static = True
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
        velocities = msg.velocity[0:6] if msg.velocity else []
        
        if len(thetas) < 6 or len(torques) < 6 or len(velocities) < 6:
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
        raw_wrist_torque = torques[5]
        
        # 判定关节是否静止
        self.is_static = all(abs(v) < 0.005 for v in velocities)
        
        # 自动零点校准
        if not self.calibrated:
            self.calibration_samples.append(raw_fz)
            self.torque_calibration_samples.append(raw_wrist_torque)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.wrist_torque_bias = np.mean(self.torque_calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 传感器零点校准完成！消除偏置 (Z Bias): {self.fz_bias:.2f} N, (Torque Bias): {self.wrist_torque_bias:.3f} Nm")
            return
        
        # 估计末端 Z 轴向力和手腕力矩（减去零点偏差并取绝对值）
        self.current_fz = abs(raw_fz - self.fz_bias)
        self.wrist_torque = abs(raw_wrist_torque - self.wrist_torque_bias)
        
        # 在空闲悬空且静止状态下进行温漂自动去皮（超低通偏置更新）
        if self.state == FREE_SPACE and self.is_static and self.current_fz < 2.0:
            self.fz_bias = 0.9995 * self.fz_bias + 0.0005 * raw_fz
            self.wrist_torque_bias = 0.9995 * self.wrist_torque_bias + 0.0005 * raw_wrist_torque
            self.current_fz = abs(raw_fz - self.fz_bias)
            self.wrist_torque = abs(raw_wrist_torque - self.wrist_torque_bias)
        
        self.force_fz_pub.publish(Float64(self.current_fz))

    def run_auto_touchdown(self):
        """
        动作 1：全自动下探寻面。控制机械臂以 5mm/s 速度向下移动，直到双阈值判定接触时停机。
        """
        rospy.loginfo("🚀 开始自动下探寻面程序...")
        self.state = FREE_SPACE  # 强制处于 FREE_SPACE 以使去皮逻辑生效
        
        # 强迫机械臂在启动前静止 1.5 秒，重新去皮校零，完全平息之前移动带来的残余力矩
        rospy.loginfo("⏸️ 机械臂静止中 (1.5秒)，正在平息关节残留力矩并执行高精度校零...")
        self.calibrated = False
        self.calibration_samples = []
        self.torque_calibration_samples = []
        rospy.sleep(1.5)
        
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rate = rospy.Rate(40) # 40Hz
        
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3 # 基座坐标系
        down_cmd.twist.linear_z = -0.010 # -10mm/s 向下，放慢速度以配合更长的确认窗口
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        
        # 引入接触判定缓存序列 (40Hz 下 7个周期约 0.175 秒)
        recent_forces = []
        verify_size = 7
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            
            # 维护最新力数据缓存序列
            recent_forces.append(self.current_fz)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
            
            if loop_cnt % 15 == 0:
                rospy.loginfo(f"⏳ 正在直线下探... 瞬时 Fz: {self.current_fz:.2f} N | 缓存序列: {[round(f, 2) for f in recent_forces]}")
            
            # 起步前 1.5 秒 (约 60 个周期) 内屏蔽判定，避开加速及克服静摩擦瞬间的电机电流剧烈抖动
            if loop_cnt > 60:
                # 只有当缓存数足够，且最后连续 verify_size 个采样周期都大于等于 12.0 N 时，才判定触及表面
                if len(recent_forces) >= verify_size and all(f >= 12.0 for f in recent_forces):
                    rospy.loginfo("🟢 判定触及纸箱表面！")
                    rospy.loginfo(f"   >> 触发确认序列: {[round(f, 2) for f in recent_forces]} N (连续 {verify_size} 次均 >= 12.0 N)")
                    rospy.loginfo(f"   >> 瞬时接触力 (Inst Fz): {self.current_fz:.2f} N")
                    
                    # 发送 10 次 0 速度，确保驱动层刹停
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
            else:
                if loop_cnt % 15 == 0:
                    rospy.loginfo("⏳ 启动加速平稳期，屏蔽接触判定...")
            
            self.vel_pub.publish(down_cmd)
            rate.sleep()
        
        if contact_detected:
            rospy.loginfo("⬆️ 执行就近重力校准：抬升 10mm 脱离接触...")
            lift_cmd = TwistCommand()
            lift_cmd.reference_frame = 3
            lift_cmd.twist.linear_z = 0.015 # 15mm/s 抬升
            for _ in range(30): # 抬升约 0.75秒，即 11mm
                self.vel_pub.publish(lift_cmd)
                rospy.sleep(0.025)
            
            for _ in range(10):
                self.vel_pub.publish(stop_cmd)
                rospy.sleep(0.025)
                
            rospy.loginfo("⏸️ 重新执行高精度就近零点校准 (1.5秒)...")
            self.calibrated = False
            self.calibration_samples = []
            self.torque_calibration_samples = []
            rospy.sleep(1.5)
            
            while not self.calibrated and not rospy.is_shutdown():
                rospy.sleep(0.1)
                
            rospy.loginfo("⬇️ 二次轻柔下探...")
            down_cmd.twist.linear_z = -0.005 # 非常轻柔地下探 5mm/s
            recent_forces.clear()
            contact_detected_again = False
            loop_cnt = 0
            
            while not rospy.is_shutdown():
                loop_cnt += 1
                recent_forces.append(self.current_fz)
                if len(recent_forces) > 5:
                    recent_forces.pop(0)
                    
                # 屏蔽前 0.5 秒加速抖动
                if loop_cnt > 20:
                    if len(recent_forces) >= 5 and all(f >= 2.0 for f in recent_forces): # 二次下探阈值降低为 2.0N
                        rospy.loginfo("🟢 二次接触锁定！")
                        for _ in range(10):
                            self.vel_pub.publish(stop_cmd)
                            rospy.sleep(0.005)
                        contact_detected_again = True
                        break
                        
                self.vel_pub.publish(down_cmd)
                rate.sleep()
                
            rospy.sleep(0.5) # 等待彻底静止
            
            # 使用高精度底层正运动学估算的绝对坐标
            current_pose = Pose()
            current_pose.position.x = self.current_x
            current_pose.position.y = self.current_y
            current_pose.position.z = self.current_z
            
            rospy.loginfo(f"📍 寻面接触起点锁定: X={current_pose.position.x:.4f}, Y={current_pose.position.y:.4f}, Z={current_pose.position.z:.4f}")
            return current_pose
        else:
            raise RuntimeError("寻面程序异常终止")

    def update_force_control(self, fz_val, state, dt=0.025):
        """
        基于接触状态机状态的高级速度型力控外环与 Leaky 积分器
        """
        if not self.calibrated:
            self.z_offset = 0.0
            self.prev_force_error = 0.0
            return 0.0, 0.0
        
        v_z_comp = 0.0
        
        if state == FREE_SPACE:
            # 1. 悬空状态下，z_offset 以较快的漏损系数平滑收敛归零，不计算力控速度
            self.z_offset = 0.95 * self.z_offset
            self.prev_force_error = 0.0
            
        elif state == SOFT_CONTACT:
            # 2. 软接触状态下，给定极慢的向下贴合速度，漏损系数也设为 0.95 限制偏置
            v_z_comp = -0.002
            self.z_offset = 0.95 * self.z_offset + v_z_comp * dt
            self.prev_force_error = 0.0
            
        elif state == HARD_CONTACT:
            # 3. 稳定接触状态下，执行非对称 PD 控制
            force_error = self.target_force - fz_val
            
            # 【水平力矩抬升判定 (已根据要求完全禁用)】
            # 不再使用手腕扭矩干预控制，仅依靠 Z 轴纯压力 (Fz)
            d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
            self.prev_force_error = force_error
            
            if force_error < -1.0:
                # 超过目标力 1.0N 时才允许快速抬升，极力避免因摩擦等干扰导致误判悬空
                rospy.loginfo_throttle(0.5, f"⬆️ 接触压力过大 (Fz={fz_val:.2f}N > 目标{self.target_force:.1f}N)，执行常规抬升")
                v_z_comp = -(self.kp_up * (force_error + 1.0) + self.kd_up * d_error)
            elif force_error < 0:
                # 处于 [目标力, 目标力+1.0N] 的冗余过度按压区间内，不抬升，维持当前高度
                v_z_comp = 0.0
            else:
                # 压力不足，缓慢下压
                v_z_comp = -(self.kp_down * force_error + self.kd_down * d_error)
                
            # 速度硬限幅 8mm/s
            v_z_comp = np.clip(v_z_comp, -0.008, 0.008)
            
            # Leaky 积分更新，漏损系数 0.995
            self.z_offset = 0.995 * self.z_offset + v_z_comp * dt
            
        # 触发防飞车保护（限制 z_offset 范围）
        self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
        
        self.z_offset_pub.publish(Float64(self.z_offset))
        
        return self.z_offset, v_z_comp

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
        
        # 将接触状态机、滤波器缓存外提，实现跨点连续状态管理
        self.state = FREE_SPACE
        self.state_counter = 0
        draw_force_window = []
        draw_window_size = 4
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break
            
            phase = wp['phase']
            target_x = wp['x']
            target_y = wp['y']
            
            k_pos = 0.8  # XY运动刚度
            
            # === 阶段 1：水平盲走 (锁定 Z) ===
            while not rospy.is_shutdown():
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.001:
                    break # 水平到位
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                
                cmd.twist.linear_x = np.clip(k_pos * dx, -0.03, 0.03)
                cmd.twist.linear_y = np.clip(k_pos * dy, -0.03, 0.03)
                
                if phase not in ['draw', 'touch_down']:
                    # 抬笔移动时，向目标高度移动
                    target_z_nominal = wp['z_nominal']
                    dz_nominal = target_z_nominal - self.current_z
                    cmd.twist.linear_z = np.clip(k_pos * dz_nominal, -0.015, 0.015)
                    
                    # 明显抬升时，强制锁死 X/Y 速度
                    if dz_nominal > 0.002:
                        cmd.twist.linear_x = 0.0
                        cmd.twist.linear_y = 0.0
                else:
                    # 绘制阶段：移动时锁定Z高度，绝对不产生额外下压，防止摩擦力伪装成受力
                    cmd.twist.linear_z = 0.0
                    
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            # === 阶段 2：刹车静止 ===
            stop_cmd = TwistCommand()
            stop_cmd.reference_frame = 3
            for _ in range(3): # 停顿约 0.075 秒，等待电机摩擦力耗散
                self.vel_pub.publish(stop_cmd)
                rospy.sleep(0.025)
                
            # === 阶段 3：静止测力与微调 ===
            if phase in ['draw', 'touch_down']:
                adjust_cnt = 0
                while not rospy.is_shutdown():
                    adjust_cnt += 1
                    static_fz = self.current_fz
                    
                    if 4.0 <= static_fz <= 6.5:
                        # 压力在完美区间，停止调整
                        break
                        
                    adjust_cmd = TwistCommand()
                    adjust_cmd.reference_frame = 3
                    adjust_cmd.twist.linear_x = 0.0
                    adjust_cmd.twist.linear_y = 0.0
                    
                    if static_fz < 4.0:
                        # 受力不足，轻微下探
                        adjust_cmd.twist.linear_z = -0.001
                    elif static_fz > 6.5:
                        # 受力过大，明显抬起
                        adjust_cmd.twist.linear_z = 0.003
                        
                    self.vel_pub.publish(adjust_cmd)
                    rate.sleep()
                    
                    if adjust_cnt > 12: # 最多微调约 0.3秒，防止卡死
                        break
                
                # 微调结束后彻底停住 Z
                self.vel_pub.publish(stop_cmd)
                
            if i % 5 == 0 or i == len(aligned_waypoints) - 1:
                rospy.loginfo(f"点进度: {i+1}/{len(aligned_waypoints)} | 阶段: {phase} | 静态Fz: {self.current_fz:.2f}N | 高度 Z: {self.current_z:.4f}m")
        
        # 绘制结束
        rospy.loginfo("🛑 绘制到达终点，稍作停顿以平息抖动...")
        
        pause_cmd = TwistCommand()
        pause_cmd.reference_frame = 3
        for _ in range(40):
            if rospy.is_shutdown():
                break
            self.vel_pub.publish(pause_cmd)
            rospy.sleep(0.025)
        
        rospy.loginfo("⬆️ 开始垂直抬升画笔...")
        
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        lift_cmd.twist.linear_x = 0.0
        lift_cmd.twist.linear_y = 0.0
        lift_cmd.twist.angular_x = 0.0
        lift_cmd.twist.angular_y = 0.0
        lift_cmd.twist.angular_z = 0.0
        
        # 严格执行用户要求：采用多段控制来进行抬升
        # 分三段加速抬升，平滑脱离
        for speed in [0.005, 0.015, 0.030]:
            lift_cmd.twist.linear_z = speed
            for _ in range(15):
                if rospy.is_shutdown():
                    break
                self.vel_pub.publish(lift_cmd)
                rospy.sleep(0.025)
        
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
