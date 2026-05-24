#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander

def main():
    print("=======================================")
    print("   🖌️ 单水平面正方形闭环绘制测试   ")
    print("=======================================")
    
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('draw_square_test', anonymous=True)

    # 连接到 Kinova 的 "arm" 规划组
    group_name = "arm"
    try:
        move_group = moveit_commander.MoveGroupCommander(group_name)
    except Exception as e:
        rospy.logerr(f"无法连接到 MoveGroup '{group_name}'。")
        rospy.logerr("如果你看到这个报错，请按 Ctrl+C 退出，并在终端运行: export ROS_NAMESPACE=/my_gen3_lite 然后再重新运行脚本！")
        return

    # 设置极慢的运行速度，确保实机第一次画图安全 (10% 速度)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    print("\n【操作指南】")
    print("你不必再单独把 XYZ 发给我了！这个脚本会自动获取机械臂现在的姿态。")
    print("1. 请用手柄将机械臂挪到你想画图的起始点（让笔尖顶住桌面或箱子顶面）。")
    print("2. 保持笔直朝下的姿态。")
    input("确认笔尖已就位后，请按【回车键】开始提取当前坐标并规划路径...")

    # 核心：直接获取物理世界当前真实的末端位姿
    current_pose = move_group.get_current_pose().pose
    
    print(f"\n✅ 当前起点已自动获取: X={current_pose.position.x:.3f}, Y={current_pose.position.y:.3f}, Z={current_pose.position.z:.3f}")
    
    waypoints = []
    # 我们先画一个小一点的，边长 5 厘米 (0.05 米) 的正方形
    side_length = 0.05 
    
    wpose = copy.deepcopy(current_pose)
    
    # 路径 1: 沿 X 轴正向移动
    wpose.position.x += side_length
    waypoints.append(copy.deepcopy(wpose))
    
    # 路径 2: 沿 Y 轴正向移动
    wpose.position.y += side_length
    waypoints.append(copy.deepcopy(wpose))
    
    # 路径 3: 沿 X 轴反向移动
    wpose.position.x -= side_length
    waypoints.append(copy.deepcopy(wpose))
    
    # 路径 4: 沿 Y 轴反向移动回到原点，闭合正方形
    wpose.position.y -= side_length
    waypoints.append(copy.deepcopy(wpose))

    print("\n正在用 MoveIt 规划严格保持笔尖垂直的直线笛卡尔轨迹...")
    # eef_step = 0.005 (插值精度 5mm), jump_threshold = 0.0 (禁用关节跳跃)
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, 0.0)

    if fraction < 0.95:
        print(f"\n⚠️ 警告：轨迹规划不完整，只算出了 {fraction*100:.2f}% 的路线。")
        print("原因可能是：画图区域超出了机械臂的运动极限边界，或者当前的关节角度由于奇异点被卡住了。")
        print("建议：把箱子往机械臂中心挪一点，或者改变一下当前的肘部关节角度，再重新运行。")
        return
        
    print(f"\n✅ 轨迹规划完美成功！一共生成了 {len(plan.joint_trajectory.points)} 个平滑的插值控制点。")
    input("🔥 请准备好你的手！将手放在急停按钮上，按【回车键】立刻开始物理绘制！")

    # 执行绘画
    move_group.execute(plan, wait=True)
    
    # 确保没有残余指令
    move_group.stop()
    
    print("\n🎉 绘制完成！你可以用之前写的误差分析脚本看看画得直不直。")

if __name__ == '__main__':
    main()
