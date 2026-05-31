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
from kortex_driver.msg import TwistCommand, Base_JointSpeeds, JointSpeed

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
        self.base_target_force = 4.0  # 标称绘制压力 4.0N
        self.target_force = 4.0       # 动态目标接触力 (可衰减防卡阻)
        self.contact_threshold = 2.5  # 接触与力控激活判定阈值 2.5N
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
        
        # 控制指令发布话题 (改为底层的关节速度控制)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/joint_velocity", Base_JointSpeeds, queue_size=1)
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
            
        # 0. 记录当前关节角度供逆解使用
        self.current_thetas = thetas
        
        # 1. 求解末端正运动学位置
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
            
        # 2. 求解基础雅可比矩阵并计算估计力
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        raw_fz = tool_force[2]
        self.raw_fz = raw_fz  # 保存为实例变量，供其他函数调用
        
        # 提取手腕末端关节 (第 6 关节) 原始力矩
        self.wrist_torque = abs(torques[5])
        
        # 判定关节是否静止
        self.is_static = all(abs(v) < 0.005 for v in velocities)
        
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
        
        # 在空闲悬空且静止状态下进行温漂自动去皮（超低通偏置更新）
        if self.state == FREE_SPACE and self.is_static and self.current_fz < 2.0:
            self.fz_bias = 0.9995 * self.fz_bias + 0.0005 * raw_fz
            self.current_fz = abs(raw_fz - self.fz_bias)
            
            
        self.force_fz_pub.publish(Float64(self.current_fz))

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
        # 考虑到接近奇异点时数值不稳定，这里加一个极小的阻尼
        J_pinv = np.linalg.pinv(J, rcond=1e-3)
        q_dot = J_pinv.dot(V_cart)
        
        # 4. 安全硬限幅 (极严苛限制: 0.5 rad/s，约 28度/秒，防止奇异点暴走)
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
        rospy.loginfo("🚀 开始沿 -Z 轴阻抗软着陆寻面 (原版力监测)...")
        self.state = FREE_SPACE
        
        # 强迫机械臂在启动前彻底静止 1.5 秒，完全平息之前移动带来的残余力矩
        rospy.sleep(1.5)
        
        self.calibrated = False
        self.calibration_samples = []
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40)
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 4
        
        local_fz_bias = self.fz_bias
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            
            raw_f = self.raw_fz
            # 前 1.5 秒内盲跑，同时强行更新基准以吸收恒速运动时的动态摩擦力底噪
            if loop_cnt < 60:
                local_fz_bias = 0.90 * local_fz_bias + 0.10 * raw_f
            elif abs(raw_f - local_fz_bias) < 5.0:
                local_fz_bias = 0.98 * local_fz_bias + 0.02 * raw_f
                
            current_net_fz = abs(raw_f - local_fz_bias)
            
            recent_forces.append(current_net_fz)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
                
            # 起步的 1.5 秒内盲跑（屏蔽判定），避开加速瞬间的爆表电流
            if loop_cnt > 60:
                # 关键修复：完全移除动态减速机制！
                # 之前只要 force > 2.0 就会减速，而减速会产生极大的惯性力，导致 force 进一步飙升，
                # 形成“减速->力变大->更减速->力爆表”的正反馈死锁，从而在半空中产生假接触！
                # 现在强制保持恒速下探，彻底切断正反馈！
                down_speed = -0.005
                
                if len(recent_forces) >= verify_size and all(f >= 10.0 for f in recent_forces):
                    rospy.loginfo(f"🟢 判定触及桌面纸板！接触力: {current_net_fz:.2f} N")
                    for _ in range(5):
                        self.send_cartesian_velocity(0.0, 0.0, 0.0)
                        rospy.sleep(0.01)
                    contact_detected = True
                    break
            else:
                down_speed = -0.005
                if loop_cnt % 15 == 0:
                    rospy.loginfo("⏳ 启动加速平稳期，屏蔽接触判定...")
                    
            self.send_cartesian_velocity(0.0, 0.0, down_speed)
            rate.sleep()
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
            d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
            self.prev_force_error = force_error
            
            if force_error < -1.5:
                # 超过目标力 1.5N 时才允许快速抬升，极力避免因摩擦等干扰导致误判悬空
                v_z_comp = -(self.kp_up * (force_error + 1.5) + self.kd_up * d_error)
            elif force_error < 0:
                # 处于 [目标力, 目标力+1.5N] 的冗余过度按压区间内，不抬升，维持当前高度
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
            
        rospy.loginfo("✅ 轨迹对准成功！")
        
        rate = rospy.Rate(40) # 40Hz
        dt = 0.025
        
        rospy.loginfo("✍️ 寻面完成，开始启动高频速度伺服纯物理贴合盲探绘图...")
        
        # --- 阶段 2：正式绘制 (盲探避障绘制 Phase 2: Kinematic Woodpecker) ---
        z_offset_relief = 0.0  # 单向安全泄压补偿量
        macro_z_shift = 0.0    # 宏观平面自适应积分器
        relief_cooldown = 0    # 停滞等待帧数
        stuck_cnt = 0          # 卡死连续帧数
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break
                
            target_x = wp['x']
            target_y = wp['y']
            
            prev_servo_x = self.current_x
            prev_servo_y = self.current_y
            
            actual_speed = 0.0
            expected_speed = 0.0
            
            while not rospy.is_shutdown():
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dist_to_target = math.hypot(dx, dy)
                
                if dist_to_target < 0.0005: # 减小允许误差，强制逼近笔尖真实位置
                    break
                    
                # 1. 速度生成与运动学防卡死检测
                movement = math.hypot(self.current_x - prev_servo_x, self.current_y - prev_servo_y)
                actual_speed = movement / dt if dt > 0 else 0.0
                
                # 【全功率恒速推土机】计算
                max_speed = 0.045  # 巡航限速 45mm/s
                k_pos_near = 12.0  # 极高刚度，防粘滑卡死
                
                cmd_vx_raw = k_pos_near * dx
                cmd_vy_raw = k_pos_near * dy
                v_mag = math.hypot(cmd_vx_raw, cmd_vy_raw)
                
                if v_mag > max_speed:
                    # 距离较远，处于恒速推土机模式 (饱和截断以维持直线方向)
                    scale = max_speed / v_mag
                    cmd_vx = cmd_vx_raw * scale
                    cmd_vy = cmd_vy_raw * scale
                else:
                    # 距离极近 (<5mm左右)，转为高刚度P控制，确保精准收敛至 1mm 不抖动
                    cmd_vx = cmd_vx_raw
                    cmd_vy = cmd_vy_raw
                    
                expected_speed = math.hypot(cmd_vx, cmd_vy)
                
                if wp['phase'] in ['draw', 'touch_down']:
                    # 只要预期下发速度大于 5mm/s 且不在抬升冷却期，就启用防戳破卡死检测
                    if expected_speed > 0.003 and relief_cooldown == 0:
                        # 恢复瞬时重置机制，过滤掉由于电机刚启动/转向时的加速度迟滞导致的“假受阻”
                        if actual_speed < 0.3 * expected_speed or actual_speed < 0.002:
                            stuck_cnt += 1
                        else:
                            stuck_cnt = 0
                    else:
                        stuck_cnt = 0
                else:
                    stuck_cnt = 0
                    
                prev_servo_x = self.current_x
                prev_servo_y = self.current_y
                
                # 2. 状态机转移逻辑
                if wp['phase'] in ['draw', 'touch_down']:
                    if stuck_cnt > 8: # 必须连续卡死 8 帧 (0.2秒)，确保是真的陷入了坑洼，而不是在加速
                        z_offset_relief += 0.0035 # 考虑到笔尖形变缓冲，大幅拔高 3.5mm 以确保笔尖完全脱离障碍物
                        z_offset_relief = min(z_offset_relief, 0.015) # 最大允许拔高 15mm
                        
                        macro_z_shift += 0.0004
                        
                        relief_cooldown = 10 # 停滞 10 帧 (0.25秒)，确保有时间完成物理拔出
                        stuck_cnt = 0
                        rospy.logwarn(f"⚠️ 物理受阻 (Act/Exp={actual_speed:.3f}/{expected_speed:.3f})！瞬发退缩且基准抬升 -> {macro_z_shift:.4f}m")
                else:
                    # 提笔移动阶段，清空状态
                    z_offset_relief = 0.0
                    relief_cooldown = 0
                    stuck_cnt = 0
                
                # 3. 目标高度计算
                # 关键修复：既然 10N 探面时纸箱已被深深压陷，这里设为负值(-0.001)，等于基准向上抬高 1mm！大幅释放压力！
                fixed_press_depth = -0.001 
                if wp['phase'] in ['draw', 'touch_down']:
                    target_z = wp['z_nominal'] - fixed_press_depth + z_offset_relief + macro_z_shift
                else:
                    target_z = wp['z_nominal'] + 0.015 + macro_z_shift
                    
                dz = target_z - self.current_z
                
                # 4. 指令生成与状态执行
                if relief_cooldown > 0:
                    relief_cooldown -= 1
                    # 停滞状态下，强制水平静止
                    cmd_vx = 0.0
                    cmd_vy = 0.0
                    # 停滞状态依然允许极速向上
                else:
                    # 正常移动，并极速下压恢复接触 (5mm/s平滑下落)
                    if wp['phase'] in ['draw', 'touch_down']:
                        z_offset_relief -= 0.005 * dt
                        z_offset_relief = max(0.0, z_offset_relief)
                        
                        if z_offset_relief < 0.0001:
                            macro_z_shift -= 0.0001 * dt
                        
                # Z轴改为“动态非对称刚度”：兼顾起步轻柔防戳与避障极速恢复
                if dz > 0:
                    # 需要向上抬升 (拔出)
                    cmd_vz = np.clip(8.0 * dz, 0.0, 0.05) # 极速拔出，最高 50mm/s
                else:
                    # 需要向下压入 (下探与恢复)
                    if z_offset_relief > 0.0001:
                        # 正在进行避障后的恢复下压，允许高速猛扎以防断墨太长
                        cmd_vz = np.clip(4.0 * dz, -0.04, 0.0) # 快速恢复，最高 40mm/s
                    else:
                        # 正常起步贴合纸面，轻柔慢压防止戳破纸箱
                        cmd_vz = np.clip(1.0 * dz, -0.01, 0.0) # 轻柔贴合，最高 10mm/s
                
                # 直接通过自研关节速度映射下发到底层
                self.send_cartesian_velocity(cmd_vx, cmd_vy, cmd_vz)
                rate.sleep()
                
            rospy.loginfo(f"点进度: {i+1}/{len(aligned_waypoints)} | 位移状态: {actual_speed:.3f}/{expected_speed:.3f} | Relief: {z_offset_relief:.4f}m")
            
        # 5. 绘制结束，到达终点后稍作停顿，平息机械臂末端抖动
        rospy.loginfo("🛑 绘制到达终点，稍作停顿以平息抖动...")
        
        for _ in range(40): # 停顿 1.0 秒 (40Hz)
            if rospy.is_shutdown():
                break
            self.send_cartesian_velocity(0.0, 0.0, 0.0)
            rospy.sleep(0.025)
            
        rospy.loginfo("⬆️ 开始垂直抬升画笔...")
        
        # 40Hz 频率下持续 40 次循环 (1.0 秒)，总共抬升 3.0cm
        for _ in range(40):
            if rospy.is_shutdown():
                break
            self.send_cartesian_velocity(0.0, 0.0, 0.03) # 以 3cm/s 速度垂直向上抬笔
            rospy.sleep(0.025)
            
        # 发送 15 次 0 速度锁定机械臂，确保驱动层刹停
        for _ in range(15):
            self.send_cartesian_velocity(0.0, 0.0, 0.0)
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
