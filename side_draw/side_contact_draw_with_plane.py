#!/usr/bin/env python3
"""
基于原始 side_contact_draw_with_log 的改进版本（放在 side_draw/ 目录，不修改原文件）。
新增功能：在每个 stroke 的起始接触点执行多点探测并拟合局部平面，自动调整末端朝向以对齐目标面法线。

注意：此脚本为在虚拟机/真实机器人上直接运行的 ROS 节点，请在使用前阅读并理解安全限幅。
"""
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

# 使用 side_draw 的平面拟合工具
from side_draw.plane_fit import fit_plane, project_point_to_plane, make_plane_frame

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(0.0, 180.0, 0.0)):
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

FREE_SPACE = 0
SOFT_CONTACT = 1
HARD_CONTACT = 2

class SideContactDrawerPlane:
    def __init__(self):
        rospy.init_node('side_contact_draw_plane', anonymous=True)
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        # 状态与参数（继承原脚本的主要参数）
        self.state = FREE_SPACE
        self.base_target_force = 6.0
        self.target_force = 6.0
        self.contact_threshold = 4.0
        self.kp_up = 0.005; self.kd_up = 0.001
        self.kp_down = 0.0008; self.kd_down = 0.0001
        self.y_offset = 0.0
        self.max_y_offset = 0.008
        self.min_y_offset = -0.03
        self.fy_bias = 0.0
        self.wrist_torque_bias = 0.0
        self.calibrated = False
        self.calibration_samples = []
        self.torque_calibration_samples = []
        self.current_fy = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.is_static = True

        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_fy_pub = rospy.Publisher("/force_control/auto/estimated_fy", Float64, queue_size=1)
        self.y_offset_pub = rospy.Publisher("/force_control/auto/y_offset", Float64, queue_size=1)

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
        self.current_x, self.current_y, self.current_z = tool_pose[0:3]
        self.current_roll, self.current_pitch, self.current_yaw = tool_pose[3:6]
        quat = R.from_euler('xyz', [self.current_roll, self.current_pitch, self.current_yaw]).as_quat()
        self.current_quaternion = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        raw_fy = tool_force[1]
        raw_wrist_torque = torques[5]
        self.is_static = all(abs(v) < 0.005 for v in velocities)
        if not self.calibrated:
            self.calibration_samples.append(raw_fy)
            self.torque_calibration_samples.append(raw_wrist_torque)
            if len(self.calibration_samples) >= 40:
                self.fy_bias = np.mean(self.calibration_samples)
                self.wrist_torque_bias = np.mean(self.torque_calibration_samples)
                self.calibrated = True
                rospy.loginfo(f"✅ 零点校准完成: fy_bias={self.fy_bias:.3f}")
            return
        self.current_fy = abs(raw_fy - self.fy_bias)
        if self.state == FREE_SPACE and self.is_static and self.current_fy < 2.0:
            self.fy_bias = 0.9995 * self.fy_bias + 0.0005 * raw_fy
            self.current_fy = abs(raw_fy - self.fy_bias)
        self.force_fy_pub.publish(Float64(self.current_fy))

    def get_cached_pose(self):
        pose = Pose()
        pose.position.x = self.current_x
        pose.position.y = self.current_y
        pose.position.z = self.current_z
        pose.orientation = self.current_quaternion
        return pose

    def move_to_cartesian_target(self, target_x, target_y, target_z, timeout=4.0):
        """用速度指令逼近目标位置，避免依赖 MoveIt 的实时 robot state。"""
        rate = rospy.Rate(40)
        start = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start) < timeout:
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dz = target_z - self.current_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 0.0015:
                break
            cmd = TwistCommand()
            cmd.reference_frame = 3
            cmd.twist.linear_x = float(np.clip(2.5 * dx, -0.03, 0.03))
            cmd.twist.linear_y = float(np.clip(2.5 * dy, -0.02, 0.02))
            cmd.twist.linear_z = float(np.clip(2.5 * dz, -0.03, 0.03))
            cmd.twist.angular_x = 0.0
            cmd.twist.angular_y = 0.0
            cmd.twist.angular_z = 0.0
            self.vel_pub.publish(cmd)
            rate.sleep()

        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(6):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)

    def rotate_to_quaternion(self, target_quaternion, timeout=4.0):
        """用角速度闭环把末端姿态转到目标四元数。"""
        rate = rospy.Rate(40)
        start = rospy.get_time()
        target_r = R.from_quat([
            target_quaternion.x,
            target_quaternion.y,
            target_quaternion.z,
            target_quaternion.w,
        ])
        while not rospy.is_shutdown() and (rospy.get_time() - start) < timeout:
            current_r = R.from_quat([
                self.current_quaternion.x,
                self.current_quaternion.y,
                self.current_quaternion.z,
                self.current_quaternion.w,
            ])
            err_r = target_r * current_r.inv()
            rotvec = err_r.as_rotvec()
            err_norm = float(np.linalg.norm(rotvec))
            if err_norm < 0.03:
                break

            cmd = TwistCommand()
            cmd.reference_frame = 3
            cmd.twist.linear_x = 0.0
            cmd.twist.linear_y = 0.0
            cmd.twist.linear_z = 0.0
            cmd.twist.angular_x = float(np.clip(2.0 * rotvec[0], -0.35, 0.35))
            cmd.twist.angular_y = float(np.clip(2.0 * rotvec[1], -0.35, 0.35))
            cmd.twist.angular_z = float(np.clip(2.0 * rotvec[2], -0.35, 0.35))
            self.vel_pub.publish(cmd)
            rate.sleep()

        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(6):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)

    def perform_plane_scan_at_pose(self, base_pose, scan_radius=0.01, num_samples=5, down_speed=0.004, contact_threshold=4.0, timeout=2.0):
        """
        在 base_pose 附近做若干偏移探测，收集接触点并拟合平面，返回拟合法线与质心与残差。
        - base_pose: geometry_msgs/Pose（参考接触点）
        - scan_radius: 在局部平面内的最大偏移 (m)
        - num_samples: 最多采样点数（含中心点）
        """
        rospy.loginfo(f"🔎 在接触点附近执行平面扫描: radius={scan_radius} m, samples={num_samples}")
        # 生成采样偏移（包含中心）
        samples = [np.array([0.0, 0.0, 0.0])]
        angles = np.linspace(0, 2*math.pi, num_samples, endpoint=False)
        for a in angles[1:]:
            dx = scan_radius * math.cos(a)
            dz = scan_radius * math.sin(a)
            samples.append(np.array([dx, 0.0, dz]))

        collected = []
        rate = rospy.Rate(40)
        stop_cmd = TwistCommand(); stop_cmd.reference_frame = 3

        for off in samples:
            target_x = base_pose.position.x + off[0]
            target_z = base_pose.position.z + off[2]
            # 快速移动到探测位置的悬空点（不经过 MoveIt）
            self.move_to_cartesian_target(target_x, base_pose.position.y + 0.005, target_z)

            # 轻柔向下探测直到接触或超时
            down_cmd = TwistCommand(); down_cmd.reference_frame = 3
            down_cmd.twist.linear_y = down_speed
            t0 = rospy.get_time()
            contacted = False
            while not rospy.is_shutdown() and (rospy.get_time() - t0) < timeout:
                self.vel_pub.publish(down_cmd)
                rate.sleep()
                if self.current_fy >= contact_threshold:
                    # 停止并记录位置
                    for _ in range(6):
                        self.vel_pub.publish(stop_cmd); rospy.sleep(0.01)
                    rospy.sleep(0.02)
                    collected.append([self.current_x, self.current_y, self.current_z])
                    contacted = True
                    break
            if not contacted:
                rospy.logwarn(f"⚠️ 扫描点未检测到接触: target=({target_x:.3f},{target_z:.3f})")
                # 若未接触，记录当前位置信息作为估计（或跳过）
                collected.append([self.current_x, self.current_y, self.current_z])

        pts = np.array(collected)
        normal, centroid, rms = fit_plane(pts)
        rospy.loginfo(f"📐 扫描拟合结果: normal={normal}, centroid={centroid}, rms={rms:.6f}")
        return normal, centroid, rms

    def execute_and_draw(self, csv_file):
        raw_waypoints = []
        rospy.loginfo(f"载入轨迹: {csv_file}")
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append({
                    'x': float(row['x_m']), 'y': float(row['y_m']), 'z_nominal': float(row['z_m']),
                    'nx': float(row.get('nx',0.0)), 'ny': float(row.get('ny',0.0)), 'nz': float(row.get('nz',0.0)),
                    'stroke_id': int(row.get('stroke_id',0)), 'phase': row.get('phase','draw')
                })

        # 初次寻面
        contact_pose = self.perform_initial_touchdown()

        # 轨迹对齐（保留原有行为）
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
            aligned_waypoints.append({
                'x': contact_pose.position.x - du,
                'y_nominal': contact_pose.position.y,
                'z': contact_pose.position.z - dv,
                'phase': wp['phase'], 'stroke_id': wp['stroke_id']
            })

        # 在每个 stroke 的第一个 draw 点做平面扫描并调整姿态
        strokes_done = set()
        for i, wp in enumerate(aligned_waypoints):
            if wp['phase'] in ['draw','touch_down'] and wp['stroke_id'] not in strokes_done:
                rospy.loginfo(f"🔁 对刀检测: stroke {wp['stroke_id']} at waypoint {i}")
                base_pose = Pose(); base_pose.position.x = wp['x']; base_pose.position.y = wp['y_nominal']; base_pose.position.z = wp['z']
                normal, centroid, rms = self.perform_plane_scan_at_pose(base_pose, scan_radius=0.01, num_samples=6)
                # 安全限幅：如果法线与世界 Y 角度变化大于 15 度或 rms>5mm 则警告并跳过姿态调整
                angle = math.degrees(math.acos(np.clip(np.dot(normal, np.array([0.0,1.0,0.0]))/np.linalg.norm(normal), -1.0,1.0)))
                if angle > 15.0 or rms > 0.005:
                    rospy.logwarn(f"拟合过大偏差: angle={angle:.1f}°, rms={rms:.4f}m — 跳过姿态调整")
                else:
                    q = get_orientation_for_normal(normal[0], normal[1], normal[2])
                    rospy.loginfo(f"目标姿态四元数: [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]")
                    self.rotate_to_quaternion(q)
                    rospy.loginfo("✅ 已调整末端姿态以对齐拟合平面法线")
                strokes_done.add(wp['stroke_id'])

        rospy.loginfo("所有对刀检测与姿态调整完成，开始常规绘制...（重用原有逻辑或留给后续集成）")

    def perform_initial_touchdown(self):
        # 复用较简单的寻面逻辑（与原脚本类似，但简化）
        rospy.loginfo("开始初始寻面（轻柔下探）")
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.0)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.05)
        # 直接调用缓存位姿，避免依赖 MoveIt 的实时 robot state
        pose = self.get_cached_pose()
        return pose

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('用法: python3 side_contact_draw_with_plane.py <path_to_csv>')
        sys.exit(1)
    drawer = SideContactDrawerPlane()
    drawer.execute_and_draw(sys.argv[1])
