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
from scipy.spatial.transform import Rotation as R

# 动态添加路径以兼容各种运行方式导入 jacobian
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(22.688, 175.755, 83.736)):
    """
    根据法向量计算末端 TCP 姿态
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

class ForceControlledDrawer:
    def __init__(self):
        rospy.init_node('force_controlled_draw', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型的机械臂
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        # 力控参数设定
        self.target_force = 13.0     # 目标接触力 13N
        self.kp = 0.005              # 比例增益
        self.kd = 0.001              # 微分增益
        self.contact_threshold = 1.0 # 接触门槛 1N
        self.max_step = 0.01         # 单周期最大位移调整量 0.01m
        
        # 内部状态变量
        self.z_offset = 0.0          # 稳态累积 Z 轴位置补偿量
        self.prev_force_error = 0.0  # 上一时刻力偏差，用于微分计算
        
        # 实时力估计状态
        self.current_fz = 0.0
        
        # 订阅关节状态话题以实时进行力矩估计与位姿解算
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
        # 发布力控调试信息
        self.force_error_pub = rospy.Publisher("/force_control/error", Float64, queue_size=1)
        self.force_fz_pub = rospy.Publisher("/force_control/estimated_fz", Float64, queue_size=1)
        self.z_offset_pub = rospy.Publisher("/force_control/z_offset", Float64, queue_size=1)
        
        # MoveIt 控制器
        import moveit_commander
        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
        self.move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
        
        # 降速以保障安全
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
    def joint_states_callback(self, msg):
        """
        基于雅可比矩阵转置从关节力矩实时估计末端受力
        """
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        # 求解基础雅可比矩阵并计算估计力: F_ee = (J^T)^+ * tau
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        
        # 估计末端 Z 轴向力（取绝对值代表接触力大小）
        self.current_fz = abs(tool_force[2])
        
    def update_force_control(self, dt=0.05):
        """
        核心 PD 控制律计算
        """
        if self.current_fz < self.contact_threshold:
            # 未进入接触状态时，保持当前偏移量，不累积误差，防止自由空间发散
            self.prev_force_error = 0.0
            return self.z_offset
            
        # 接触力误差计算 (目标值 - 实际估计值)
        force_error = self.target_force - self.current_fz
        
        # 计算微分项
        d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
        self.prev_force_error = force_error
        
        # PD 控制律输出位置调整量 Delta Z
        delta_z = self.kp * force_error + self.kd * d_error
        
        # 饱和限制防拉扯爆震
        delta_z = np.clip(delta_z, -self.max_step, self.max_step)
        
        # 稳态补偿：累积调节位移
        self.z_offset += delta_z
        
        # 发布调试话题
        self.force_error_pub.publish(Float64(force_error))
        self.force_fz_pub.publish(Float64(self.current_fz))
        self.z_offset_pub.publish(Float64(self.z_offset))
        
        return self.z_offset

    def execute_trajectory(self, csv_file):
        """
        读取 CSV 轨迹，在控制循环中融合 PD 力控进行插值执行
        """
        waypoints = []
        rospy.loginfo(f"正在载入轨迹文件: {csv_file}")
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append({
                    'x': float(row['x_m']),
                    'y': float(row['y_m']),
                    'z_nominal': float(row['z_m']),
                    'nx': float(row['nx']),
                    'ny': float(row['ny']),
                    'nz': float(row['nz']),
                    'stroke_id': int(row['stroke_id']),
                    'phase': row['phase']
                })
        
        rospy.loginfo(f"成功载入 {len(waypoints)} 个轨迹航点。")
        input("🔥 确认安全看门狗已在后台启动！按【回车】开始运行 PD 力控绘制程序...")
        
        rate = rospy.Rate(20) # 20Hz 力控与轨迹更新周期
        dt = 0.05
        
        for i, wp in enumerate(waypoints):
            if rospy.is_shutdown():
                break
                
            # 计算姿态
            quat = get_orientation_for_normal(wp['nx'], wp['ny'], wp['nz'])
            
            # 在自由空间（如 approach 阶段）不施加力控，只在作图（draw/touch_down）阶段引入力控补偿
            if wp['phase'] in ['draw', 'touch_down']:
                self.update_force_control(dt)
            else:
                # 自由悬空重置力控补偿
                self.z_offset = 0.0
                self.prev_force_error = 0.0
                
            # 融合静态与累积补偿位移
            target_pose = Pose()
            target_pose.position.x = wp['x']
            target_pose.position.y = wp['y']
            # Z 轴叠加稳态补偿
            target_pose.position.z = wp['z_nominal'] + self.z_offset
            target_pose.orientation = quat
            
            # 发送目标位姿命令进行跟踪
            self.move_group.set_pose_target(target_pose)
            self.move_group.go(wait=True)
            
            rospy.loginfo(f"点 {i+1}/{len(waypoints)} | Fz: {self.current_fz:.2f}N | Offset Z: {self.z_offset:.4f}m")
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 force_controlled_draw.py <path_to_csv>")
        sys.exit(1)
        
    try:
        drawer = ForceControlledDrawer()
        drawer.execute_trajectory(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
