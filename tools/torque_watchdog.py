#!/usr/bin/env python3
import rospy
import numpy as np
from kortex_driver.msg import BaseCyclic_Feedback
from kortex_driver.srv import Stop

class TorqueWatchdog:
    def __init__(self):
        rospy.init_node('torque_watchdog', anonymous=True)
        
        # 【阈值设定】 (具体数值需要在运行中观察正常画图时的受力并微调)
        # Gen3 lite 是靠电流估算力矩的，静态下可能就不为 0，所以阈值要设得比空载时稍大一些
        self.force_z_limit = rospy.get_param('~force_z_limit', 15.0) # 笔尖 Z 轴受力上限 (牛顿 N)
        self.joint_torque_limit = rospy.get_param('~joint_torque_limit', 10.0) # 各关节力矩上限 (牛米 Nm)
        
        self.topic_name = rospy.get_param('~topic', '/my_gen3_lite/base_feedback')
        self.stop_service = '/my_gen3_lite/base/stop'
        
        rospy.loginfo(f"🛡️ 力矩安全看门狗已上线！正在以 40Hz 的频率监控异常碰撞...")
        rospy.loginfo(f"触发条件: 末端 Z 轴外力 > {self.force_z_limit}N 或 任意关节力矩 > {self.joint_torque_limit}Nm")
        
        self.sub = rospy.Subscriber(self.topic_name, BaseCyclic_Feedback, self.callback)
        self.stop_triggered = False

    def callback(self, msg):
        if self.stop_triggered:
            return
            
        # 1. 获取末端估计受力 (Gen3 lite 依靠关节电流和动力学模型反解出的末端受力)
        try:
            fz = abs(msg.base.tool_external_wrench_force_z)
        except AttributeError:
            fz = 0.0 
            
        # 2. 获取各个关节的实时力矩 (这是非常底层的硬指标，任何碰撞都会导致它飙升)
        torques = [abs(actuator.torque) for actuator in msg.actuators]
        max_torque = max(torques) if torques else 0.0
        
        # 降频打印状态，方便你观察并调节阈值 (大概每秒打印一次)
        if np.random.rand() < 0.02: 
            rospy.loginfo(f"📊 实时监控 | 笔尖 Z 轴受力: {fz:.2f} N | 最大关节力矩: {max_torque:.2f} Nm")

        # 3. 触发物理急停逻辑
        if fz > self.force_z_limit or max_torque > self.joint_torque_limit:
            rospy.logerr(f"🚨 危险！检测到极度异常的机械对抗力！机械臂可能要被掰坏了！")
            rospy.logerr(f"当前笔尖 Z 轴受力: {fz:.2f} N (限值: {self.force_z_limit} N)")
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
