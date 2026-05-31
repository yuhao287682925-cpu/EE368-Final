#!/usr/bin/env python3
import sys
import csv
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation as R

def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(22.688, 175.755, 83.736)):
    """
    核心物理补偿逻辑：
    根据生成的表面的法向量 (nx, ny, nz)，动态旋转 TCP 姿态。
    原理：默认姿态是垂直向下的(法向量为 0,0,1)。当在其它面上画图时，
    我们计算出一个能够把 [0,0,1] 转到 [nx,ny,nz] 的旋转矩阵，
    然后叠加在原有的基础姿态上，保证笔尖永远垂直指向目标面。
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
        # 如果是反方向 (底部画图)
        r_align = R.from_euler('x', 180, degrees=True)
    else:
        axis = axis / axis_len
        angle = np.arccos(np.clip(np.dot(v_from, v_to), -1.0, 1.0))
        r_align = R.from_rotvec(axis * angle)
        
    # 叠加旋转：先旋转基础姿态，再进行法向对齐补偿
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
            
            # 【关键】应用魔法姿态补偿
            quat = get_orientation_for_normal(nx, ny, nz)
            
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation = quat
            
            current_stroke.append({'pose': pose, 'phase': phase})
            
    if current_stroke:
        waypoints_by_stroke.append(current_stroke)
        
    rospy.loginfo(f"成功加载了 {len(waypoints_by_stroke)} 个连续笔画。")
    input("🔥 按回车键开始全自动物理绘制！请确认安全看门狗已在后台运行并手握急停按钮！")
    
    for i, stroke in enumerate(waypoints_by_stroke):
        rospy.loginfo(f"==== 正在执行笔画 {i+1}/{len(waypoints_by_stroke)} ====")
        
        # 步骤A: Approach (自由空间运动到该笔画上方)
        approach_pose = stroke[0]['pose']
        move_group.set_max_velocity_scaling_factor(0.5) # 空中可以稍微快一点
        move_group.set_pose_target(approach_pose)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        if not success:
            rospy.logerr("无法安全移动到起始点！提前终止。")
            break
            
        # 步骤B: 下笔、作图、抬笔 (完全由 Cartesian Path 严格保持姿态)
        draw_waypoints = [point['pose'] for point in stroke[1:]]
                
        if draw_waypoints:
            rospy.loginfo("规划精准贴面绘制轨迹...")
            (plan, fraction) = move_group.compute_cartesian_path(draw_waypoints, 0.005, 0.0)
            if fraction < 0.95:
                rospy.logwarn(f"规划残缺 (仅 {fraction*100:.1f}%)，可能是机械臂极限死角导致跳过该笔画。")
                continue
                
            # 将绘制速度大幅度降低，确保安全和力矩感知的灵敏度
            plan = move_group.retime_trajectory(robot.get_current_state(), plan, velocity_scaling_factor=0.1, acceleration_scaling_factor=0.1)
            
            rospy.loginfo("正在绘制...")
            move_group.execute(plan, wait=True)
            rospy.loginfo("单次笔画执行完毕并抬笔。")

if __name__ == '__main__':
    main()
