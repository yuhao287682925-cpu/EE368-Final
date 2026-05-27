#!/usr/bin/env python3
import rospy
import math
from kortex_driver.msg import BaseCyclic_Feedback

class ForceTorqueViewer:
    def __init__(self):
        rospy.init_node('force_torque_viewer', anonymous=True)
        rospy.Subscriber("/my_gen3_lite/base_feedback", BaseCyclic_Feedback, self.feedback_callback, queue_size=1)
        self.last_print_time = rospy.Time.now()
        rospy.loginfo("开始纯粹的力与力矩数据监测... (按 Ctrl+C 退出)")

    def feedback_callback(self, msg):
        # 控制打印频率 (每 0.5 秒打印一次，避免刷屏太快看不清)
        current_time = rospy.Time.now()
        if (current_time - self.last_print_time).to_sec() < 0.5:
            return
        self.last_print_time = current_time

        try:
            fx = msg.base.tool_external_wrench_force_x
            fy = msg.base.tool_external_wrench_force_y
            fz = msg.base.tool_external_wrench_force_z
            f_total = math.sqrt(fx**2 + fy**2 + fz**2)
        except AttributeError:
            fx, fy, fz, f_total = 0.0, 0.0, 0.0, 0.0
            
        torques = [abs(actuator.torque) for actuator in msg.actuators]
        max_torque = max(torques) if torques else 0.0

        # 清屏效果 (可选，让终端看起来更整洁)
        print("\033[2J\033[H", end="")
        print("====== 实时力矩传感器监测面板 ======")
        print(f"X 轴受力: {fx:6.2f} N")
        print(f"Y 轴受力: {fy:6.2f} N")
        print(f"Z 轴受力: {fz:6.2f} N")
        print("-" * 34)
        print(f"🌟 笔尖总合力 (F_total) : {f_total:6.2f} N")
        print(f"💪 最大关节力矩 (Torque): {max_torque:6.2f} Nm")
        print("====================================")

if __name__ == '__main__':
    try:
        viewer = ForceTorqueViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
