#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState

# 动态添加路径以兼容导入 jacobian
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm

class ForceZMonitor:
    def __init__(self):
        rospy.init_node('force_z_monitor', anonymous=True)
        
        # 初始化 Gen3-lite DH 模型
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        self.fz_bias = 0.0
        self.raw_fz = 0.0
        self.current_fz = 0.0
        self.is_static = True
        self.calibrated = False
        self.calibration_samples = []
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        
    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        velocities = msg.velocity[0:6] if msg.velocity else []
        
        if len(thetas) < 6 or len(torques) < 6 or len(velocities) < 6:
            return
            
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        self.raw_fz = tool_force[2]
        
        self.is_static = all(abs(v) < 0.005 for v in velocities)
        
        if not self.calibrated:
            self.calibration_samples.append(self.raw_fz)
            if len(self.calibration_samples) >= 40:
                self.fz_bias = np.mean(self.calibration_samples)
                self.calibrated = True
            return
            
        self.current_fz = abs(self.raw_fz - self.fz_bias)

    def trigger_calibration(self):
        print("\n⏳ 正在重新校准去皮零偏，请保持机械臂完全静止且悬空...")
        self.calibrated = False
        self.calibration_samples = []
        # 等待校准完成
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(f"✅ 校准完成！新 Z 轴 Bias 偏置为: {self.fz_bias:.3f} N\n")

    def run(self):
        print("====================================================")
        print("        Kinova Gen3-lite Z 轴估计力实时监控工具")
        print("====================================================")
        print("程序启动，正在进行首次自动去皮...")
        
        # 等待首次自动去皮完成
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        print(f"✅ 首次去皮完成！初始 Z 轴 Bias 偏置为: {self.fz_bias:.3f} N")
        print("提示：在终端中按 [回车键(Enter)] 可随时重新去皮校零！\n")
        
        rate = rospy.Rate(10) # 10Hz 打印刷新率
        
        # 启动主线程的键盘监听和定时刷新
        import threading
        
        def print_loop():
            while not rospy.is_shutdown():
                if self.calibrated:
                    # 制作简易的 ASCII 柱状图表示力大小
                    bar_length = int(min(20, self.current_fz * 2))
                    bar_str = "█" * bar_length + "░" * (20 - bar_length)
                    static_status = "静止" if self.is_static else "运动"
                    
                    sys.stdout.write(
                        f"\r[Estimated Fz]: {self.current_fz:5.2f} N | [Raw Fz]: {self.raw_fz:6.2f} N | "
                        f"Bias: {self.fz_bias:6.2f} N | 状态: {static_status} | 强度: |{bar_str}|"
                    )
                    sys.stdout.flush()
                rate.sleep()
                
        t = threading.Thread(target=print_loop)
        t.daemon = True
        t.start()
        
        try:
            while not rospy.is_shutdown():
                # 阻塞等待用户按回车重新标定
                input()
                self.trigger_calibration()
        except (KeyboardInterrupt, SystemExit):
            pass

if __name__ == '__main__':
    try:
        monitor = ForceZMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
