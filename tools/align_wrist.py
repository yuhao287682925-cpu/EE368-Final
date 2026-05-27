#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R

# 动态添加路径以导入已有的 jacobian.py
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

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

def main():
    rospy.init_node('align_wrist', anonymous=True)
    
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
    move_group = moveit_commander.MoveGroupCommander("arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite")
    
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    
    rospy.loginfo("🔄 正在旋转手腕至完全垂直状态 [0.0, 180.0, 0.0]...")
    vertical_quat = get_orientation_for_normal(0, 0, 1)
    move_group.set_orientation_target([vertical_quat.x, vertical_quat.y, vertical_quat.z, vertical_quat.w])
    
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    if success:
        rospy.loginfo("✅ 姿态调直成功！")
        rospy.loginfo("👉 提示：现在你可以用手柄将机械臂挪到你想画图的纸箱起点正上方，然后启动 auto_contact_draw.py！")
    else:
        rospy.logerr("❌ 姿态调直规划失败。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
