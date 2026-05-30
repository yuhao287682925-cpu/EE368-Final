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
        self.wrist_torque_threshold = 0.8  # 末端关节 (第 6 关节) 扭矩接触跳变阈值由0.06提高至0.8 N.m
        
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
        self.allow_dynamic_calibration = True  # 允许动态温漂去皮的开关
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
        
    def reset_calibration(self):
        """
        重置偏置校准状态，便于多阶段在不同姿态下重复去皮
        """
        self.calibrated = False
        self.calibration_samples = []
        self.torque_calibration_samples = []
        
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
            return
            
        # 估计末端 Z 轴向力和手腕力矩（减去零点偏差并取绝对值）
        self.current_fz = abs(raw_fz - self.fz_bias)
        self.wrist_torque = abs(raw_wrist_torque - self.wrist_torque_bias)
        
        # 在空闲悬空且静止状态下进行温漂自动去皮（超低通偏置更新）
        if self.allow_dynamic_calibration and self.state == FREE_SPACE and self.is_static and self.current_fz < 2.0:
            self.fz_bias = 0.9995 * self.fz_bias + 0.0005 * raw_fz
            self.wrist_torque_bias = 0.9995 * self.wrist_torque_bias + 0.0005 * raw_wrist_torque
            self.current_fz = abs(raw_fz - self.fz_bias)
            self.wrist_torque = abs(raw_wrist_torque - self.wrist_torque_bias)
            
        self.force_fz_pub.publish(Float64(self.current_fz))

    def run_auto_touchdown(self):
        """
        动作 1：全自动下探寻面。
        采用“两阶段自适应寻面与原位高精度去皮校准”：
        - 阶段 1：在 Ready 状态校准，随后以 10mm/s 速度向下进行粗探寻面（瞬时 17N，连续 5 周期 15N 判定接触）。
        - 阶段 2：刹停后上抬 5mm 悬空，静止 1.5 秒，重新执行高精度原位去皮校零，消除姿态改变带来的重力偏置失效。
        - 阶段 3：以极慢速度 3mm/s 向下精细探测贴合（连续 5 周期 5N 判定接触），精确定位最终对刀原点。
        """
        rospy.loginfo("🚀 开始自动下探寻面程序 (两阶段原位去皮重构版)...")
        self.state = FREE_SPACE  # 强制处于 FREE_SPACE
        
        # --- 阶段 1：Ready 姿态初始校零 ---
        rospy.loginfo("⏸️ [阶段1-Ready校零] 机械臂静止中 (1.5秒)，正在进行 Ready 姿态偏置估计...")
        self.reset_calibration()
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo(f"✅ Ready 校零完成！(Z Bias): {self.fz_bias:.2f} N, (Torque Bias): {self.wrist_torque_bias:.3f} Nm")
            
        rate = rospy.Rate(40) # 40Hz
        
        # 粗探速度 (10mm/s)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3 # 基座坐标系
        down_cmd.twist.linear_z = -0.010 
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        rough_contact_detected = False
        loop_cnt = 0
        
        # 粗探检测参数（Request 2）
        recent_forces = []
        verify_size = 5
        inst_limit = 17.0
        seq_limit = 15.0
        
        # 粗下探循环
        while not rospy.is_shutdown():
            loop_cnt += 1
            recent_forces.append(self.current_fz)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
            
            if loop_cnt % 15 == 0:
                rospy.loginfo(f"⏳ [粗探中] Fz 瞬时: {self.current_fz:.2f} N | 缓存序列: {[round(f, 2) for f in recent_forces]}")
                
            # 前 60 个周期屏蔽起步抖动
            if loop_cnt > 60:
                cond_seq = len(recent_forces) >= verify_size and all(f >= seq_limit for f in recent_forces)
                cond_inst = self.current_fz >= inst_limit
                if cond_seq or cond_inst:
                    rospy.loginfo(f"🟢 [粗探触发] 判定触及纸箱表面！原因: {'瞬时力限制' if cond_inst else '连续序列触发'}")
                    rospy.loginfo(f"   >> 确认序列: {[round(f, 2) for f in recent_forces]} N (连续 {verify_size} 次均 >= {seq_limit} N)")
                    
                    # 发送 10 次 0 速度，确保刹停
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    rough_contact_detected = True
                    break
            else:
                if loop_cnt % 15 == 0:
                    rospy.loginfo("⏳ 粗探启动加速平稳期，屏蔽接触判定...")
                    
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if not rough_contact_detected:
            raise RuntimeError("粗下探程序异常终止")
            
        rospy.sleep(0.5) # 等待彻底静止
        z_rough = self.current_z
        rospy.loginfo(f"📍 粗探测接触位置 Z: {z_rough:.4f} m")
        
        # --- 阶段 2：原位回抬悬空与高精度校零 ---
        rospy.loginfo("⬆️ [阶段2-原位抬升] 正在向上垂直回抬 5mm 悬空...")
        lift_calib_cmd = TwistCommand()
        lift_calib_cmd.reference_frame = 3
        lift_calib_cmd.twist.linear_z = 0.010 # 10mm/s
        target_calib_z = z_rough + 0.005
        
        start_lift_time = rospy.get_time()
        while not rospy.is_shutdown():
            # 到达目标高度或超时则退出
            if self.current_z >= target_calib_z - 0.001 or (rospy.get_time() - start_lift_time) > 2.5:
                break
            self.vel_pub.publish(lift_calib_cmd)
            rospy.sleep(0.025)
            
        # 刹停并静止
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        rospy.loginfo("⏸️ 机械臂静止中 (1.5秒)，正在进行原位高精度去皮校零...")
        self.reset_calibration()
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo(f"✅ 原位校零完成！(Z Bias): {self.fz_bias:.2f} N, (Torque Bias): {self.wrist_torque_bias:.3f} Nm")
        
        # --- 阶段 3：极慢速精细二次贴合 ---
        rospy.loginfo("🚀 [阶段3-精细贴合] 开始慢速二次下探...")
        
        fine_down_cmd = TwistCommand()
        fine_down_cmd.reference_frame = 3
        fine_down_cmd.twist.linear_z = -0.003 # 3mm/s 慢速下探
        
        fine_contact_detected = False
        loop_cnt_fine = 0
        recent_forces_fine = []
        verify_size_fine = 5
        fine_limit = 3.0  # 降低微小对刀接触力阈值以防过度挤压纸箱
        
        while not rospy.is_shutdown():
            loop_cnt_fine += 1
            recent_forces_fine.append(self.current_fz)
            if len(recent_forces_fine) > verify_size_fine:
                recent_forces_fine.pop(0)
                
            if loop_cnt_fine % 20 == 0:
                rospy.loginfo(f"⏳ [精探中] Fz 瞬时: {self.current_fz:.2f} N | 缓存序列: {[round(f, 2) for f in recent_forces_fine]}")
                
            # 同样前 20 个周期屏蔽起步微弱扰动
            if loop_cnt_fine > 20:
                if len(recent_forces_fine) >= verify_size_fine and all(f >= fine_limit for f in recent_forces_fine):
                    rospy.loginfo(f"🟢 [精细贴合触发] 二次接触确认！")
                    rospy.loginfo(f"   >> 确认序列: {[round(f, 2) for f in recent_forces_fine]} N (连续 {verify_size_fine} 次均 >= {fine_limit} N)")
                    
                    # 强力刹停
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    fine_contact_detected = True
                    break
            self.vel_pub.publish(fine_down_cmd)
            rate.sleep()
            
        if fine_contact_detected:
            rospy.sleep(0.5) # 等待彻底静止
            current_pose = Pose()
            current_pose.position.x = self.current_x
            current_pose.position.y = self.current_y
            current_pose.position.z = self.current_z
            rospy.loginfo(f"📍 最终高精度接触起点锁定: X={current_pose.position.x:.4f}, Y={current_pose.position.y:.4f}, Z={current_pose.position.z:.4f}")
            return current_pose
        else:
            raise RuntimeError("精细下探程序异常终止")

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
            # 1. 悬空状态下，z_offset 以较缓的漏损系数平滑收敛归零，防止抖动导致剧烈缩回
            self.z_offset = 0.98 * self.z_offset
            self.prev_force_error = 0.0
            
        elif state == SOFT_CONTACT:
            # 2. 软接触状态下，给定极慢的向下贴合速度，漏损系数也设为 0.95 限制偏置
            v_z_comp = -0.002
            self.z_offset = 0.95 * self.z_offset + v_z_comp * dt
            self.prev_force_error = 0.0
            
        elif state == HARD_CONTACT:
            # 3. 稳定接触状态下，执行非对称 PD 控制
            force_error = self.target_force - fz_val
            
            # 【水平力矩抬升判定】
            # 在水平方向遇到阻力（如陷入纸箱凹陷）时，比竖直方向更容易在腕部产生力矩
            # 若检测到力矩异常突变，强行注入巨大负向误差，骗过控制器立刻触发最大速度抬升
            if self.wrist_torque > self.wrist_torque_threshold:
                force_error = -10.0
                rospy.logwarn(f"⚠️ 手腕力矩异常 (Torque={self.wrist_torque:.3f} Nm > {self.wrist_torque_threshold:.3f} Nm)，强行触发保护性快速抬升！")
                
            d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
            self.prev_force_error = force_error
            
            if force_error < -1.0:
                # 超过目标力 1.0N 时才允许快速抬升，极力避免因摩擦等干扰导致误判悬空
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
        
        # 2. 全自动下探寻面 (两阶段高精度对刀)
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
        self.allow_dynamic_calibration = False  # 开始绘制，锁定零偏，防止拐角静止时污染偏置
        rospy.loginfo("🔒 偏置锁定生效，禁止动态去皮更新。")
        
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
                
            quat = get_orientation_for_normal(wp['nx'], wp['ny'], wp['nz'])
            
            # 位置控制的目标点 XY
            target_x = wp['x']
            target_y = wp['y']
            
            k_pos = 1.5  # 刚度系数设为 1.5，加快轨迹运动响应速度
            
            stuck_cnt = 0
            prev_servo_x = self.current_x
            prev_servo_y = self.current_y
            
            # 位置伺服走点循环
            while not rospy.is_shutdown():
                # 1. 强制在非绘制/非下探阶段使状态回到 FREE_SPACE 状态
                if wp['phase'] not in ['draw', 'touch_down']:
                    if self.state != FREE_SPACE:
                        self.state = FREE_SPACE
                        self.state_counter = 0
                        rospy.loginfo("🔵 阶段转换: 强制切换至 FREE_SPACE")
                        
                # 在控制周期内部，基于当前的力控偏移 z_offset 动态更新目标高度
                target_z = wp['z_nominal'] + self.z_offset
                
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dz = target_z - self.current_z
                
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.001: # 到位距离 1mm，确保误差极小
                    break
                    
                # 绘图力滑动窗口维护与平滑
                draw_force_window.append(self.current_fz)
                if len(draw_force_window) > draw_window_size:
                    draw_force_window.pop(0)
                fz_filtered = np.mean(draw_force_window) if len(draw_force_window) >= draw_window_size else self.current_fz
                
                # 2. 接触状态机跳转逻辑 (引入更宽的迟滞区间，消除临界受力点的状态横跳)
                if self.state == FREE_SPACE:
                     if fz_filtered > 2.2:
                         self.state_counter += 1
                         if self.state_counter >= 5:
                             self.state = SOFT_CONTACT
                             self.state_counter = 0
                             rospy.loginfo(f"🟠 状态转移: FREE_SPACE -> SOFT_CONTACT (Fz={fz_filtered:.2f}N)")
                     else:
                         self.state_counter = 0
                         
                elif self.state == SOFT_CONTACT:
                     if fz_filtered > 4.0:
                         self.state_counter += 1
                         if self.state_counter >= 5:
                             self.state = HARD_CONTACT
                             self.state_counter = 0
                             rospy.loginfo(f"🔴 状态转移: SOFT_CONTACT -> HARD_CONTACT (Fz={fz_filtered:.2f}N)")
                     elif fz_filtered < 1.2:
                         self.state = FREE_SPACE
                         self.state_counter = 0
                         draw_force_window = []  # 悬空时清空窗口
                         rospy.loginfo(f"🔵 状态转移: SOFT_CONTACT -> FREE_SPACE (完全悬空, Fz={fz_filtered:.2f}N)")
                     else:
                         self.state_counter = 0
                         
                elif self.state == HARD_CONTACT:
                     if fz_filtered < 3.0:
                         self.state_counter += 1
                         if self.state_counter >= 5:
                             self.state = SOFT_CONTACT
                             self.state_counter = 0
                             rospy.loginfo(f"🟠 状态转移: HARD_CONTACT -> SOFT_CONTACT (力不足退回, Fz={fz_filtered:.2f}N)")
                     else:
                         self.state_counter = 0
                    
                # 3. 判定卡阻逻辑 (在绘制阶段且离目标点较远时)
                if wp['phase'] in ['draw', 'touch_down'] and dist_to_target > 0.005:
                    movement = math.hypot(self.current_x - prev_servo_x, self.current_y - prev_servo_y)
                    if movement < 0.0003: # 单周期位移小于 0.3mm (说明可能被卡在纸箱凹陷里)
                        stuck_cnt += 1
                    else:
                        stuck_cnt = max(0, stuck_cnt - 1)
                else:
                    stuck_cnt = 0
                    
                prev_servo_x = self.current_x
                prev_servo_y = self.current_y
                
                # 4. 动态调整目标压力
                if self.state == HARD_CONTACT and stuck_cnt >= 8:
                    # 目标力自适应衰减 (从第 8 个卡阻周期开始平滑衰减，直到 2.5N)
                    self.target_force = max(2.5, self.base_target_force - 0.44 * (stuck_cnt - 7))
                    if stuck_cnt % 8 == 0:
                        rospy.logwarn(f"⚠️ 末端可能卡阻 (stuck_cnt={stuck_cnt})，降低目标压力至 {self.target_force:.2f}N")
                else:
                    self.target_force = self.base_target_force
                
                # 5. 调用力控律，更新 z_offset 并获取偏置和控制速度
                z_offset_val, v_z_comp = self.update_force_control(fz_filtered, self.state, dt)
                
                cmd = TwistCommand()
                cmd.reference_frame = 3 # 基座坐标系
                cmd.duration = 0
                
                # XY 方向伺服速度 (限速放宽至 0.06 m/s 以加快绘制速度)
                cmd.twist.linear_x = np.clip(k_pos * dx, -0.06, 0.06)
                cmd.twist.linear_y = np.clip(k_pos * dy, -0.06, 0.06)
                
                # 6. Z 方向速度指令根据接触状态机来决定
                if self.state == FREE_SPACE:
                    # 悬空状态下，朝着标称高度运动下探，限速 15mm/s
                    target_z_nominal = wp['z_nominal']
                    dz_nominal = target_z_nominal - self.current_z
                    cmd.twist.linear_z = np.clip(k_pos * dz_nominal, -0.015, 0.015)
                elif self.state == SOFT_CONTACT:
                    # 软接触状态下，固定以 -2mm/s 缓慢下压
                    cmd.twist.linear_z = -0.002
                elif self.state == HARD_CONTACT:
                    # 稳定接触下，直接把速度型 PD 力的外环速度发给关节 (限幅限制在 [-0.008, 0.008]m/s)
                    cmd.twist.linear_z = v_z_comp
                    
                    # 在上升时停止横向移动
                    if v_z_comp > 0.0:
                        cmd.twist.linear_x = 0.0
                        cmd.twist.linear_y = 0.0
                
                # 保持姿态稳定，角速度设为 0
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            rospy.loginfo(f"点进度: {i+1}/{len(aligned_waypoints)} | 状态: {self.state} | Fz: {self.current_fz:.2f}N | Offset Z: {self.z_offset:.4f}m")
            
        # 5. 绘制结束，到达终点后稍作停顿，平息机械臂末端抖动
        rospy.loginfo("🛑 绘制到达终点，稍作停顿以平息抖动...")
        
        pause_cmd = TwistCommand()
        pause_cmd.reference_frame = 3
        for _ in range(40): # 停顿 1.0 秒 (40Hz)
            if rospy.is_shutdown():
                break
            self.vel_pub.publish(pause_cmd)
            rospy.sleep(0.025)
            
        rospy.loginfo("⬆️ 开始垂直抬升画笔...")
        
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3  # 基座坐标系
        lift_cmd.twist.linear_x = 0.0
        lift_cmd.twist.linear_y = 0.0
        lift_cmd.twist.linear_z = 0.03  # 以 3cm/s 速度垂直向上抬笔
        lift_cmd.twist.angular_x = 0.0
        lift_cmd.twist.angular_y = 0.0
        lift_cmd.twist.angular_z = 0.0
        
        # 40Hz 频率下持续 40 次循环 (1.0 秒)，总共抬升 3.0cm
        for _ in range(40):
            if rospy.is_shutdown():
                break
            self.vel_pub.publish(lift_cmd)
            rospy.sleep(0.025)
            
        # 发送 15 次 0 速度锁定机械臂，确保驱动层刹停
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        self.move_group.stop()
        rospy.loginfo("🎉 全自动伺服力控绘制任务圆满完成！")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 auto_contact_draw_v2_refactored.py <path_to_csv>")
        sys.exit(1)
        
    try:
        drawer = AutoContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
