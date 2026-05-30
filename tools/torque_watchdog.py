#!/usr/bin/env python3
import rospy
import numpy as np
from kortex_driver.msg import BaseCyclic_Feedback
from kortex_driver.srv import Stop

class TorqueWatchdog:
    def __init__(self):
        rospy.init_node('torque_watchdog', anonymous=True)
        
        # 【双阈值设定】 
        self.force_z_min = rospy.get_param('~force_z_min', 2.0)   # 牛顿(N)：大于此值认为笔尖已【接触】到纸面
        self.force_z_max = rospy.get_param('~force_z_max', 15.0)  # 牛顿(N)：大于此值认为存在【弯折风险】
        self.joint_torque_limit = rospy.get_param('~joint_torque_limit', 10.0) 
        
        self.topic_name = rospy.get_param('~topic', '/my_gen3_lite/base_feedback')
        self.stop_service = '/my_gen3_lite/base/stop'
        
        rospy.loginfo(f"🛡️ 智能触觉监控已上线！频率: 40Hz")
        rospy.loginfo(f" [悬空/未触碰] Z轴外力 < {self.force_z_min}N")
        rospy.loginfo(f" [正常画图中] {self.force_z_min}N <= Z轴外力 <= {self.force_z_max}N")
        rospy.loginfo(f" [弯折/炸机风险] Z轴外力 > {self.force_z_max}N (将触发急停!)")
        
        self.sub = rospy.Subscriber(self.topic_name, BaseCyclic_Feedback, self.callback)
        self.stop_triggered = False
        self.last_state = ""

    def callback(self, msg):
        if self.stop_triggered:
            return
            
        try:
            fz = abs(msg.base.tool_external_wrench_force_z)
        except AttributeError:
            fz = 0.0 
            
        torques = [abs(actuator.torque) for actuator in msg.actuators]
        max_torque = max(torques) if torques else 0.0
        
        # 1. 自动状态机判断
        current_state = ""
        if fz < self.force_z_min:
            current_state = "⚪ 未触碰 (悬空)"
        elif fz <= self.force_z_max:
            current_state = "🟢 正常画图 (接触良好)"
        else:
            current_state = "🔴 弯折风险 (超载)"

        # 2. 状态改变时，或者大概每秒打印一次
        if current_state != self.last_state or np.random.rand() < 0.02: 
            rospy.loginfo(f"状态: {current_state} | Z轴受力: {fz:.2f} N | 最大关节力矩: {max_torque:.2f} Nm")
            self.last_state = current_state

        # 3. 触发物理急停逻辑 (保护硬件)
        if fz > self.force_z_max or max_torque > self.joint_torque_limit:
            rospy.logerr(f"🚨 危险！检测到极度异常的机械对抗力！机械臂可能要被掰坏了！")
            rospy.logerr(f"当前笔尖 Z 轴受力: {fz:.2f} N (限值: {self.force_z_max} N)")
            rospy.logerr(f"当前最大关节力矩: {max_torque:.2f} Nm (限值: {self.joint_torque_limit} Nm)")
            rospy.logerr("正在触发物理急停服务...")
            
            self.trigger_stop()
            self.stop_triggered = True

    def trigger_stop(self):
        rospy.wait_for_service(self.stop_service, timeout=2.0)
        try:
            stop_func = rospy.ServiceProxy(self.stop_service, Stop)
            stop_func()
            rospy.logerr("🛑 机械臂底座已执行急停，电机已被锁死！请排查碰撞原因后重启驱动。")
        except Exception as e:
            rospy.logerr(f"急停调用失败: {e}")

if __name__ == '__main__':
    try:
        TorqueWatchdog()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
