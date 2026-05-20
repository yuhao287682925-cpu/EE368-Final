#!/usr/bin/env python3
import rospy
import csv
import os
# 如果你在这台电脑上/虚拟机里有 kortex_driver 相关的包，可以取消下面的注释直接订阅话题
from kortex_driver.msg import BaseCyclic_Feedback

class TrajectoryRecorderTopic:
    def __init__(self):
        rospy.init_node('trajectory_recorder_topic', anonymous=True)
        
        # 根据 Startup.md，你的 robot_name 是 my_gen3_lite
        self.topic_name = rospy.get_param('~topic', '/my_gen3_lite/base_feedback')
        self.output_file = rospy.get_param('~output_file', 'actual_trajectory_feedback.csv')
        
        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'x', 'y', 'z', 'theta_x', 'theta_y', 'theta_z'])
        
        rospy.loginfo(f"等待订阅话题: {self.topic_name} ...")
        self.sub = rospy.Subscriber(self.topic_name, BaseCyclic_Feedback, self.callback)
        rospy.loginfo(f"开始记录轨迹数据，保存至: {os.path.abspath(self.output_file)}")

    def callback(self, msg):
        # kortex_driver/BaseCyclic_Feedback 包含了底座实时解算的笛卡尔位姿
        t = rospy.Time.now().to_sec()
        x = msg.base.tool_pose_x
        y = msg.base.tool_pose_y
        z = msg.base.tool_pose_z
        tx = msg.base.tool_pose_theta_x
        ty = msg.base.tool_pose_theta_y
        tz = msg.base.tool_pose_theta_z
        
        self.writer.writerow([t, x, y, z, tx, ty, tz])

    def __del__(self):
        if hasattr(self, 'file'):
            self.file.close()

if __name__ == '__main__':
    try:
        recorder = TrajectoryRecorderTopic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
