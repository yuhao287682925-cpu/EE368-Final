#!/usr/bin/env python3
import sys
import csv
import copy
import time
import math
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation as R
from kortex_driver.msg import BaseCyclic_Feedback

class ForceMonitor:
    """
    实时订阅并缓存 Kinova 机械臂末端的力传感器数据
    """
    def __init__(self, topic="/my_gen3_lite/base_feedback"):
        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.received = False
        self.sub = rospy.Subscriber(topic, BaseCyclic_Feedback, self._callback, queue_size=1)

    def _callback(self, msg):
        try:
            self.fx = msg.base.tool_external_wrench_force_x
            self.fy = msg.base.tool_external_wrench_force_y
            self.fz = msg.base.tool_external_wrench_force_z
            self.received = True
        except AttributeError:
            pass

    def get_contact_force(self, nx, ny, nz):
        """
        计算外力在给定面外法向量 (nx, ny, nz) 上的投影值。
        当笔尖被表面顶住时，由于纸面对笔的阻力与外法向一致，该投影值为正。
        """
        if not self.received:
            return 0.0
        return self.fx * nx + self.fy * ny + self.fz * nz

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(22.688, 175.755, 83.736)):
    """
    核心物理姿态对齐：
    根据生成的表面的法向量 (nx, ny, nz)，动态旋转 TCP 姿态。
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

def main():
    if len(sys.argv) < 2:
        print("用法: python3 draw_csv_trajectory.py <path_to_csv>")
        sys.exit(1)
        
    csv_file = sys.argv[1]
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_csv_trajectory', anonymous=True)
    
    robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
    move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
    
    # 初始化力传感器监控
    force_monitor = ForceMonitor()
    rospy.loginfo("正在等待力传感器数据接收...")
    timeout = 5.0
    start_t = time.time()
    while not force_monitor.received and not rospy.is_shutdown():
        if time.time() - start_t > timeout:
            rospy.logwarn("未能接收到力传感器反馈。请确认 /my_gen3_lite/base_feedback 话题正常发布！")
            break
        rospy.sleep(0.1)
    
    # 1. 提取笔画 (Strokes)
    waypoints_by_stroke = []
    current_stroke = []
    current_stroke_id = -1
    
    rospy.loginfo(f"正在读取轨迹文件: {csv_file}...")
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            stroke_id = int(row['stroke_id'])
            if stroke_id != current_stroke_id:
                if current_stroke:
                    waypoints_by_stroke.append(current_stroke)
                current_stroke = []
                current_stroke_id = stroke_id
                
            x, y, z = float(row['x_m']), float(row['y_m']), float(row['z_m'])
            nx, ny, nz = float(row['nx']), float(row['ny']), float(row['nz'])
            phase = row['phase']
            
            quat = get_orientation_for_normal(nx, ny, nz)
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation = quat
            
            current_stroke.append({
                'pose': pose,
                'phase': phase,
                'nx': nx,
                'ny': ny,
                'nz': nz
            })
            
    if current_stroke:
        waypoints_by_stroke.append(current_stroke)
        
    rospy.loginfo(f"成功加载了 {len(waypoints_by_stroke)} 个连续笔画。")
    print("\n👉 请手动将机械臂移动（拖动）到目标纸面正上方安全高度，然后再按回车开始下探！")
    input("🔥 确认当前位置安全后，按回车键开始下探探测与绘制！")
    
    for i, stroke in enumerate(waypoints_by_stroke):
        rospy.loginfo(f"==== 正在执行笔画 {i+1}/{len(waypoints_by_stroke)} ====")
        
        # 寻找本笔画的 touch_down 阶段点作为理论参考坐标
        touch_down_point = None
        for pt in stroke:
            if pt['phase'] == 'touch_down':
                touch_down_point = pt
                break
        
        if not touch_down_point:
            rospy.logwarn("未在该笔画中找到 touch_down 点，跳过自适应下探，直接用理论轨迹执行！")
            offset_x, offset_y, offset_z = 0.0, 0.0, 0.0
            nx, ny, nz = 0.0, 0.0, 1.0
        else:
            theory_pose = touch_down_point['pose']
            nx, ny, nz = touch_down_point['nx'], touch_down_point['ny'], touch_down_point['nz']
            
            # 获取当前实际位姿（带重试与防全零安全验证）
            probe_start_pose = None
            for retry in range(10):
                pose_stamped = move_group.get_current_pose()
                if pose_stamped is not None:
                    p = pose_stamped.pose
                    if abs(p.position.x) > 1e-4 or abs(p.position.y) > 1e-4 or abs(p.position.z) > 1e-4:
                        probe_start_pose = p
                        break
                rospy.logwarn("机械臂状态数据尚未同步，正在重试获取当前位姿...")
                rospy.sleep(0.5)
                
            if probe_start_pose is None:
                rospy.logerr("🚨 错误: 无法获取机械臂当前的有效物理位姿！跳过本笔画以保护设备。")
                continue
                
            rospy.loginfo(f"成功获取下探起点位姿 -> X:{probe_start_pose.position.x:.3f}, Y:{probe_start_pose.position.y:.3f}, Z:{probe_start_pose.position.z:.3f}")
            
            # 使用笛卡尔直线路径规划单向向下探测路径
            probe_distance = 0.035  # 最大下探 35mm
            probe_target_pose = copy.deepcopy(probe_start_pose)
            probe_target_pose.position.x -= probe_distance * nx
            probe_target_pose.position.y -= probe_distance * ny
            probe_target_pose.position.z -= probe_distance * nz
            
            # 采用关键字参数调用以防止 Noetic 底层 C++ 包装的签名类型匹配冲突
            (probe_plan, fraction) = move_group.compute_cartesian_path(
                waypoints=[probe_target_pose],
                eef_step=0.001,
                jump_threshold=0.0
            )
            
            if fraction < 0.90:
                rospy.logerr("下探直线路径规划失败，无法安全执行探测！")
                continue
                
            # 极慢速安全Retiming控制 (限制在2%速度)
            probe_plan = move_group.retime_trajectory(
                robot.get_current_state(),
                probe_plan,
                velocity_scaling_factor=0.02,
                acceleration_scaling_factor=0.02
            )
            
            target_force = 2.0  # 2N 接触力
            detected = False
            
            rospy.loginfo(f"开始连续自适应下探。目标法向按压力: {target_force} N")
            
            # 异步非阻塞执行下探运动
            move_group.execute(probe_plan, wait=False)
            
            start_probe_time = rospy.Time.now()
            max_probe_duration = 20.0  # 最多下探运行 20 秒
            
            # 100Hz 高频循环监听力矩反馈
            rate = rospy.Rate(100)
            while not rospy.is_shutdown():
                if (rospy.Time.now() - start_probe_time).to_sec() > max_probe_duration:
                    move_group.stop()
                    rospy.logerr("🚨 下探动作超时，未能接触到物理表面！")
                    break
                    
                # 计算最新的外法向投影按压力
                f_contact = force_monitor.get_contact_force(nx, ny, nz)
                
                # 实时打断条件
                if f_contact >= target_force:
                    # 瞬时刹车叫停！
                    move_group.stop()
                    rospy.loginfo(f"🎉 触碰成功！当前法向按压力为 {f_contact:.2f} N >= {target_force} N。已实施保护性停机。")
                    detected = True
                    rospy.sleep(0.5)  # 等待机械臂平稳静止
                    break
                    
                rate.sleep()
                
            if not detected:
                rospy.logerr("🚨 下探未触碰，为了安全已中止本笔画！")
                continue
                
            # 探测成功，获取当前的末端实际坐标并计算 Offset
            actual_probe_pose = move_group.get_current_pose().pose
            offset_x = actual_probe_pose.position.x - theory_pose.position.x
            offset_y = actual_probe_pose.position.y - theory_pose.position.y
            offset_z = actual_probe_pose.position.z - theory_pose.position.z
            rospy.loginfo(f"表面偏差 (Offset) -> X: {offset_x*1000:.2f}mm, Y: {offset_y*1000:.2f}mm, Z: {offset_z*1000:.2f}mm")
        
        # 步骤C: 对该笔画后续所有的 draw 轨迹点应用偏移量进行补偿
        draw_waypoints = []
        for point in stroke:
            if point['phase'] == 'draw':
                compensated_pose = copy.deepcopy(point['pose'])
                compensated_pose.position.x += offset_x
                compensated_pose.position.y += offset_y
                compensated_pose.position.z += offset_z
                draw_waypoints.append(compensated_pose)
                
        if draw_waypoints:
            rospy.loginfo(f"规划精准贴面绘制轨迹 (共 {len(draw_waypoints)} 个点)...")
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints=draw_waypoints,
                eef_step=0.005,
                jump_threshold=0.0
            )
            if fraction < 0.95:
                rospy.logwarn(f"规划残缺 (仅 {fraction*100:.1f}%)，跳过该笔画的绘制。")
                continue
                
            # 重采样时间参数以确保绘制速度均匀安全
            plan = move_group.retime_trajectory(
                robot.get_current_state(),
                plan,
                velocity_scaling_factor=0.1,
                acceleration_scaling_factor=0.1
            )
            
            rospy.loginfo("正在平稳绘制...")
            move_group.execute(plan, wait=True)
            rospy.loginfo("绘制完毕。")
            
        # 步骤D: 安全抬笔
        rospy.loginfo("正在安全抬笔...")
        if draw_waypoints:
            last_draw_pose = draw_waypoints[-1]
        else:
            last_draw_pose = stroke[-1]['pose']
            
        lift_pose = copy.deepcopy(last_draw_pose)
        lift_pose.position.x += 0.020 * nx
        lift_pose.position.y += 0.020 * ny
        lift_pose.position.z += 0.020 * nz
        
        move_group.set_max_velocity_scaling_factor(0.3)
        move_group.set_pose_target(lift_pose)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo("==== 当前笔画执行结束 ====\n")

if __name__ == '__main__':
    main()
