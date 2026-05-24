#!/usr/bin/env python3
import rospy
from kortex_driver.msg import BaseCyclic_Feedback
# Kinova 的 Stop 服务，请根据实际工作空间中的 srv 名称进行调整，如果不确定可以取消这行并在 callback 中只做报警
# from kortex_driver.srv import Stop

class SafetyWatchdog:
    def __init__(self):
        rospy.init_node('safety_watchdog', anonymous=True)
        # 安全底线：如果 Z 坐标低于桌面以上 5厘米 (0.05米)，视为危险！
        # 这个值你需要根据刚才实测的桌面高度来自己修改！
        self.z_min = rospy.get_param('~z_min', 0.05) 
        self.topic_name = rospy.get_param('~topic', '/my_gen3_lite/base_feedback')
        self.stop_service = '/my_gen3_lite/base/stop'
        
        rospy.loginfo(f"🛡️ 安全看门狗已上线！严格监控 Z < {self.z_min}m 的情况。")
        self.sub = rospy.Subscriber(self.topic_name, BaseCyclic_Feedback, self.callback)
        self.stop_triggered = False

    def callback(self, msg):
        z = msg.base.tool_pose_z
        if z < self.z_min and not self.stop_triggered:
            rospy.logerr(f"🚨 危险！Z坐标低于安全底线 (当前Z: {z:.4f}m)。正在触发急停报警！")
            self.trigger_stop()
            self.stop_triggered = True

    def trigger_stop(self):
        # 如果你确定有 kortex_driver.srv.Stop 可以取消这里的注释，实现物理急停
        '''
        rospy.wait_for_service(self.stop_service, timeout=2.0)
        try:
            stop_func = rospy.ServiceProxy(self.stop_service, Stop)
            stop_func()
            rospy.logerr("🛑 机械臂已自动紧急停止！")
        except Exception as e:
            rospy.logerr(f"急停调用失败: {e}")
        '''
        rospy.logerr("🔔（这里是占位报警，请确保你手里捏紧了真实的急停开关！）")

if __name__ == '__main__':
    try:
        SafetyWatchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
