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
        
        # 核心力控参数：13N 目标压力，直接阈值激活门槛 1.0N，力控死区 2.0N
        self.desired_force = 13.0
        self.activation_force_threshold = 1.0
        self.force_deadzone = 2.0
        self.wrist_torque_threshold = 1.5  # 手腕力矩避障阈值调高至 1.5 N.m
        
        # 非对称 PD 增益：抬起快 (0.005)，下压慢 (0.0008)
        self.kp_up = 0.005
        self.kd_up = 0.001
        self.kp_down = 0.0008
        self.kd_down = 0.0001
        
        self.z_offset = 0.0           # 虚拟 Z 轴力控累积位移 (稳态补偿)
        self.max_z_offset = 0.008     # 最大抬升位移限制 (0.8 cm，防悬空)
        self.min_z_offset = -0.010    # 最大下压位移限制收紧为 -1.0 cm (防止压坏纸箱)
        
        # 避障逃逸状态机变量
        self.is_escaping = False
        self.escape_start_time = 0.0
        self.escape_duration = 1.5    # 逃逸期挂起 XY 轨迹 1.5 秒
        self.escape_z_lift = 0.010     # 逃逸期间垂直向上抬升 10mm
        
        # 零点力校准状态
        self.fz_bias = 0.0
        self.wrist_torque_bias = 0.0
        self.calibration_samples = []
        self.torque_calibration_samples = []
        self.calibrated = False
        self.allow_dynamic_calibration = True  # 允许去皮的开关
        
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
        
        # 在去皮允许且静止状态下进行温漂自动去皮（超低通偏置更新，用于空载）
        if self.allow_dynamic_calibration and self.is_static and self.current_fz < 2.0:
            self.fz_bias = 0.9995 * self.fz_bias + 0.0005 * raw_fz
            self.wrist_torque_bias = 0.9995 * self.wrist_torque_bias + 0.0005 * raw_wrist_torque
            self.current_fz = abs(raw_fz - self.fz_bias)
            self.wrist_torque = abs(raw_wrist_torque - self.wrist_torque_bias)
            
        self.force_fz_pub.publish(Float64(self.current_fz))

    def run_auto_touchdown(self):
        """
        动作 1：全自动下探寻面。
        采用“带下行匀速动态校零与原位高精度静态校零”的两阶段下探：
        - 阶段 1：先匀速下落 (10mm/s)。在下落中途（启动 1.5 秒后）动态校零，吸收下行运动摩擦力。
        - 阶段 2：以 2.0N 的极灵敏阈值进行接触探测，确保极度平稳贴合，刹停后回抬 5mm 悬空。
        - 阶段 3：静止 1.5 秒后，重新校准获取超高精度静态偏置。
        """
        rospy.loginfo("🚀 开始全自动下探寻面程序 (带下行去皮与原位重置版)...")
        self.allow_dynamic_calibration = True
        
        # 阶段 1：Ready 状态初始静态校零
        rospy.loginfo("⏸️ [静态初校零] 机械臂静止中 (1.0秒)...")
        self.reset_calibration()
        rospy.sleep(1.0)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.05)
            
        rate = rospy.Rate(40) # 40Hz
        
        # 下行匀速 10mm/s
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        down_cmd.twist.linear_z = -0.010 
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        
        recent_forces = []
        verify_size = 5
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            recent_forces.append(self.current_fz)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
            
            if loop_cnt % 15 == 0:
                rospy.loginfo(f"⏳ [下探中] Fz 瞬时: {self.current_fz:.2f} N | 缓存序列: {[round(f, 2) for f in recent_forces]}")
                
            # 在匀速下行 1.5 秒时（约第 60 个控制周期），重新执行一次偏置采样！
            # 这一步极其关键！能在匀速运动中自动吸收关节向下运动的库伦摩擦力矩！
            if loop_cnt == 60:
                rospy.loginfo("⏸️ [下行动态去皮] 开启匀速下行力偏置重估，滤除运动关节摩擦...")
                self.reset_calibration()
                
            # 在 100 个周期后（此时动态偏置已在匀速下行中收集完毕），使用极灵敏的 2.0N 阈值探测接触
            if loop_cnt > 100:
                if len(recent_forces) >= verify_size and all(f >= 2.0 for f in recent_forces):
                    rospy.loginfo(f"🟢 [粗探触发] 判定触及纸箱表面！")
                    rospy.loginfo(f"   >> 确认序列: {[round(f, 2) for f in recent_forces]} N (连续 5 次均 >= 2.0 N)")
                    
                    # 刹停
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if not contact_detected:
            raise RuntimeError("粗下探程序异常终止")
            
        rospy.sleep(0.5) # 等待彻底静止
        z_rough = self.current_z
        rospy.loginfo(f"📍 粗定位接触高度 Z: {z_rough:.4f} m")
        
        # 阶段 2：向上回抬 5mm 悬空进行最高精度的静态重力偏置去皮
        rospy.loginfo("⬆️ 正在向上回抬 5mm 悬空...")
        lift_calib_cmd = TwistCommand()
        lift_calib_cmd.reference_frame = 3
        lift_calib_cmd.twist.linear_z = 0.010 # 10mm/s
        target_calib_z = z_rough + 0.005
        
        start_lift_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self.current_z >= target_calib_z - 0.001 or (rospy.get_time() - start_lift_time) > 2.0:
                break
            self.vel_pub.publish(lift_calib_cmd)
            rospy.sleep(0.025)
            
        # 刹停并静止
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        rospy.loginfo("⏸️ 机械臂静止中 (1.5秒)，正在执行原位高精度静态去皮校零...")
        self.reset_calibration()
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.05)
        rospy.loginfo(f"✅ 原位静态校零完成！(Z Bias): {self.fz_bias:.2f} N, (Torque Bias): {self.wrist_torque_bias:.3f} Nm")
        
        # 阶段 3：以极慢速度 3mm/s 二次精细贴合寻面，锁定高精度对刀原点
        rospy.loginfo("🚀 开始二次精细贴合寻面...")
        fine_down_cmd = TwistCommand()
        fine_down_cmd.reference_frame = 3
        fine_down_cmd.twist.linear_z = -0.003 # 3mm/s
        
        fine_contact_detected = False
        loop_cnt_fine = 0
        recent_forces_fine = []
        fine_limit = 3.0  # 二次寻面力阈值定为更小的 3.0N，防止预压挤压纸面
        
        while not rospy.is_shutdown():
            loop_cnt_fine += 1
            recent_forces_fine.append(self.current_fz)
            if len(recent_forces_fine) > verify_size:
                recent_forces_fine.pop(0)
                
            if loop_cnt_fine > 20:
                if len(recent_forces_fine) >= verify_size and all(f >= fine_limit for f in recent_forces_fine):
                    rospy.loginfo("🟢 [二次贴合触发] 二次对刀接触锁定！")
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

    def update_force_control(self, fz_val, dt=0.025):
        """
        基于接触检测激活、大死区解耦、非对称PD的 Admittance 力控律
        """
        if not self.calibrated:
            self.z_offset = 0.0
            self.prev_force_error = 0.0
            return 0.0, 0.0
            
        v_z_comp = 0.0
        
        # 1. 接触激活门槛判定
        if fz_val <= self.activation_force_threshold:
            # 悬空状态下：位置补偿以较缓系数收敛归零
            self.z_offset = 0.98 * self.z_offset
            self.prev_force_error = 0.0
            v_z_comp = 0.0
        else:
            # 2. 稳定接触下，根据死区容错和非对称PD计算补偿速度
            force_error = self.desired_force - fz_val  # ef = F_desired - Fz
            
            # 手腕力矩避障保护 (大于 1.5 Nm 强制触发逃逸)
            if self.wrist_torque > self.wrist_torque_threshold:
                # 注入大负误差，强制触发最大速度抬升
                force_error = -10.0
                rospy.logwarn(f"⚠️ 手腕力矩异常 (Torque={self.wrist_torque:.3f} Nm > {self.wrist_torque_threshold:.3f} Nm)")
                
            if abs(force_error) <= self.force_deadzone:
                # 处于 13N +- 2.0N 力控死区内，锁定当前位置补偿，彻底消解水平摩擦带来的波动
                v_z_comp = 0.0
                self.prev_force_error = 0.0
            else:
                # 偏离死区，执行非对称 PD 力控 (抬升用大增益，下压用小增益)
                d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
                self.prev_force_error = force_error
                
                if force_error < 0.0:
                    # 压力过大：需要向上快速抬升
                    v_z_comp = -(self.kp_up * force_error + self.kd_up * d_error)
                else:
                    # 压力不足：需要向下缓慢试探 (缓慢下行克服关节摩擦力矩)
                    v_z_comp = -(self.kp_down * force_error + self.kd_down * d_error)
            
            # 速度硬限幅 8mm/s，防止大幅上下震荡
            v_z_comp = np.clip(v_z_comp, -0.008, 0.008)
            
            # ⚠️ 【边界速度截断保护】
            # 如果 z_offset 已经顶满边界，且速度指令还在继续朝边界输出，强制截断速度为 0！
            if self.z_offset >= self.max_z_offset and v_z_comp > 0.0:
                v_z_comp = 0.0
            elif self.z_offset <= self.min_z_offset and v_z_comp < 0.0:
                v_z_comp = 0.0
            
            # 计算该周期的位置改变量并累积，消除稳态力误差
            dz = v_z_comp * dt
            dz = np.clip(dz, -0.01, 0.01)
            self.z_offset = self.z_offset + dz
            
        # 触发防飞车硬限幅保护 (下压限制在 -10mm 以内，安全防线)
        self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
        self.z_offset_pub.publish(Float64(self.z_offset))
        
        return self.z_offset, v_z_comp

    def execute_and_draw(self, csv_file):
        """
        自动对刀对齐，随后高频速度伺服绘制
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
        
        # 2. 全自动下探寻面 (匀速下行去皮 + 原位静态重校准)
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
        self.allow_dynamic_calibration = False  # 锁死零偏，防止拐角停顿时外界反力污染偏置
        rospy.loginfo("🔒 偏置锁定生效，禁止动态去皮更新。")
        
        # 4. 高频速度伺服跟踪与力控循环
        rate = rospy.Rate(40) # 40Hz
        dt = 0.025
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        draw_force_window = []
        draw_window_size = 8  # 增加滤波窗口至 8 帧，滤除高频水平滑动摩擦突变
        
        # 避障逃逸状态初始化
        self.is_escaping = False
        self.escape_start_time = 0.0
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break
                
            quat = get_orientation_for_normal(wp['nx'], wp['ny'], wp['nz'])
            
            # 位置控制的目标点 XY
            target_x = wp['x']
            target_y = wp['y']
            
            k_pos = 1.5  # 刚度系数设为 1.5，加快轨迹运动响应速度
            
            # 位置伺服走点循环
            while not rospy.is_shutdown():
                is_drawing_phase = wp['phase'] in ['draw', 'touch_down']
                
                # 绘图力滑动窗口维护与平滑
                draw_force_window.append(self.current_fz)
                if len(draw_force_window) > draw_window_size:
                    draw_force_window.pop(0)
                fz_filtered = np.mean(draw_force_window) if len(draw_force_window) >= draw_window_size else self.current_fz
                
                # 【手腕扭矩碰撞检测】 (仅在绘制阶段触发)
                if is_drawing_phase and not self.is_escaping:
                    if self.wrist_torque > self.wrist_torque_threshold:
                        self.is_escaping = True
                        self.escape_start_time = rospy.get_time()
                        rospy.logwarn("🚨 [避障逃逸激活] 触发手腕扭矩碰撞！垂直向上抬升 10mm 并挂起轨迹 1.5 秒...")
                        
                # 智能避障逃逸状态分支
                if self.is_escaping:
                    elapsed_time = rospy.get_time() - self.escape_start_time
                    if elapsed_time < self.escape_duration:
                        # 逃逸挂起期内：完全禁止水平移动，Z 轴持续垂直抬升 10mm 避障逃逸
                        cmd = TwistCommand()
                        cmd.reference_frame = 3
                        cmd.twist.linear_x = 0.0
                        cmd.twist.linear_y = 0.0
                        cmd.twist.linear_z = 0.008  # 以 8mm/s 垂直向上安全抬起
                        
                        # 维持 z_offset 更新与发布
                        self.z_offset = np.clip(self.z_offset + 0.008 * dt, self.min_z_offset, self.max_z_offset)
                        self.z_offset_pub.publish(Float64(self.z_offset))
                        self.vel_pub.publish(cmd)
                        rate.sleep()
                        continue
                    else:
                        # 逃逸挂起期满：开始重新贴合寻纸，贴合时轨迹继续保持挂起
                        rospy.loginfo("🔄 逃逸挂起期满，开始执行重新缓慢下探贴合...")
                        while not rospy.is_shutdown():
                            draw_force_window.append(self.current_fz)
                            if len(draw_force_window) > draw_window_size:
                                draw_force_window.pop(0)
                            fz_filtered = np.mean(draw_force_window) if len(draw_force_window) >= draw_window_size else self.current_fz
                            
                            if fz_filtered >= self.activation_force_threshold:
                                rospy.loginfo("🟢 重新贴合纸面成功，恢复轨迹绘制。")
                                break
                            
                            down_check_cmd = TwistCommand()
                            down_check_cmd.reference_frame = 3
                            down_check_cmd.twist.linear_z = -0.003 # 以 3mm/s 慢速向下试探
                            self.vel_pub.publish(down_check_cmd)
                            rate.sleep()
                        
                        # 逃逸状态彻底重置并退出
                        self.is_escaping = False
                        self.prev_force_error = 0.0
                        # 刹停瞬间
                        for _ in range(5):
                            self.vel_pub.publish(stop_cmd)
                            rospy.sleep(0.01)
                        continue
                
                # 正常控制逻辑：
                if is_drawing_phase:
                    # 绘制阶段下，调用 PD 力控外环，计算位置补偿和速度
                    z_offset_val, v_z_comp = self.update_force_control(fz_filtered, dt)
                else:
                    # transition 悬空过渡段，不激活力控，强制 z_offset 收敛归零
                    self.z_offset = 0.95 * self.z_offset
                    self.prev_force_error = 0.0
                    z_offset_val = self.z_offset
                    v_z_comp = 0.0
                    
                # 动态计算目标 Z 轴坐标
                target_z = wp['z_nominal'] + z_offset_val
                
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dz = target_z - self.current_z
                
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.0005:  # 到位距离收紧至 0.5mm，确保高精度轨迹绘制
                    break
                    
                cmd = TwistCommand()
                cmd.reference_frame = 3 # 基座坐标系
                cmd.duration = 0
                
                # XY 方向位置追踪速度 (限速 0.06 m/s)
                cmd.twist.linear_x = np.clip(k_pos * dx, -0.06, 0.06)
                cmd.twist.linear_y = np.clip(k_pos * dy, -0.06, 0.06)
                
                # Z 方向力控伺服速度输出
                if not is_drawing_phase:
                    # 悬空非绘制状态下，直接追踪目标 Z 轴高度，限速 15mm/s
                    cmd.twist.linear_z = np.clip(k_pos * dz, -0.015, 0.015)
                else:
                    if fz_filtered <= self.activation_force_threshold:
                        # 接触未建立 (力 <= 1N)，执行慢速下探寻找纸面 (-3mm/s)
                        cmd.twist.linear_z = -0.003
                    else:
                        # 建立接触 (力 > 1N)，输出力控 PD 补偿速度
                        cmd.twist.linear_z = v_z_comp
                        
                        # 抬升时 (v_z_comp > 0.0) 停止 XY 平面移动，确保安全 (防止卡阻拖拽)
                        if v_z_comp > 0.0:
                            cmd.twist.linear_x = 0.0
                            cmd.twist.linear_y = 0.0
                
                # 保持姿态稳定，角速度设为 0
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            rospy.loginfo(f"点进度: {i+1}/{len(aligned_waypoints)} | Fz: {self.current_fz:.2f}N | Offset Z: {self.z_offset:.4f}m")
            
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
