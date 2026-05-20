#!/usr/bin/env python3
import rospy
import csv
import tf
import os

class TrajectoryRecorder:
    def __init__(self):
        rospy.init_node('trajectory_recorder', anonymous=True)
        
        # 帧名称（注意：需要根据Kinova实际的TF树进行修改）
        # 'base_link' 通常是机械臂底座
        # 'tool_frame' 或 'end_effector_link' 通常是末端
        # 从 Startup.md 分析出你的默认 robot_name 是 my_gen3_lite
        # 但因为 prefix 为空，TF树的帧名应该没有前缀，通常为 'base_link' 和 'end_effector_link' 或 'tool_frame'
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.tool_frame = rospy.get_param('~tool_frame', 'end_effector_link') # Gen3 lite 默认末端通常是 end_effector_link
        self.output_file = rospy.get_param('~output_file', 'actual_trajectory.csv')
        self.record_rate = rospy.get_param('~rate', 40) # 你的 base_feedback 频率是 40Hz (从 Startup.md 获取)
        
        self.listener = tf.TransformListener()
        
        # 打开CSV文件准备写入
        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        
        rospy.loginfo(f"开始记录轨迹数据，保存至: {os.path.abspath(self.output_file)}")

    def record(self):
        rate = rospy.Rate(self.record_rate)
        # 等待TF树建立
        try:
            rospy.loginfo("等待TF变换...")
            self.listener.waitForTransform(self.base_frame, self.tool_frame, rospy.Time(), rospy.Duration(4.0))
        except tf.Exception as e:
            rospy.logwarn(f"等待TF超时，请检查 base_frame 和 tool_frame 是否正确: {e}")

        rospy.loginfo("TF连接成功！正在高频记录坐标...")
        
        while not rospy.is_shutdown():
            try:
                # 获取当前最新的末端坐标变换
                (trans, rot) = self.listener.lookupTransform(self.base_frame, self.tool_frame, rospy.Time(0))
                
                # 写入当前时间戳、位置(XYZ)和姿态四元数(XYZW)
                t = rospy.Time.now().to_sec()
                self.writer.writerow([t, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
            rate.sleep()

    def __del__(self):
        if hasattr(self, 'file'):
            self.file.close()

if __name__ == '__main__':
    try:
        recorder = TrajectoryRecorder()
        recorder.record()
    except rospy.ROSInterruptException:
        pass
