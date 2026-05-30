#!/usr/bin/env python3
import sys
import os
import csv
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from kortex_driver.msg import TwistCommand
from std_msgs.msg import Float64 as StdFloat64

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from jacobian import NLinkArm
from scipy.spatial.transform import Rotation as R

class SideContactDrawer:
    def __init__(self):
        rospy.init_node('side_contact_draw', anonymous=True)
        
        dh_params_list = np.array([[0, 0, 243.3/1000, 0],
                                   [math.pi/2, 0, 10/1000, 0+math.pi/2],
                                   [math.pi, 280/1000, 0, 0+math.pi/2],
                                   [math.pi/2, 0, 245/1000, 0+math.pi/2],
                                   [math.pi/2, 0, 57/1000, 0],
                                   [-math.pi/2, 0, 235/1000, 0-math.pi/2]])
        self.arm_model = NLinkArm(dh_params_list)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.f3d_bias = np.zeros(3)
        self.calibration_samples = []
        self.calibrated = False
        self.current_f_total = 0.0
        
        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)
        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_pub = rospy.Publisher("/force_control/auto/estimated_f_normal", StdFloat64, queue_size=1)

    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        
        if len(thetas) < 6 or len(torques) < 6: return
            
        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = tool_pose[0]
        self.current_y = tool_pose[1]
        self.current_z = tool_pose[2]
        
        J = self.arm_model.basic_jacobian(thetas)
        tool_force = np.linalg.pinv(J.T).dot(torques)
        
        raw_f3d = tool_force[0:3]
        
        if not self.calibrated:
            self.calibration_samples.append(raw_f3d)
            if len(self.calibration_samples) >= 40:
                self.f3d_bias = np.mean(self.calibration_samples, axis=0)
                self.calibrated = True
                rospy.loginfo(f"鉁?浼犳劅鍣ㄩ浂鐐规牎鍑嗗畬鎴愶紒娑堥櫎 3D 闈欏姏鍋忕疆: [{self.f3d_bias[0]:.2f}, {self.f3d_bias[1]:.2f}, {self.f3d_bias[2]:.2f}] N")
            return
            
        f_net = raw_f3d - self.f3d_bias
        
        # 澶氭柟鍚戝悎鍔?(鍖呭惈鍨傜洿鍘嬪姏鍜屽钩闈笂鐨勬í鍚?绾靛悜鎽╂摝闃诲姏)
        self.current_f_total = np.linalg.norm(f_net)
        self.force_pub.publish(StdFloat64(self.current_f_total))

    def run_auto_touchdown(self):
        rospy.loginfo("馃殌 寮€濮嬫部 -Y 杞寸洿绾垮闈?..")
        
        self.calibrated = False
        self.calibration_samples = []
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        rate = rospy.Rate(40)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        # -Y 杞村墠绉绘帰娴?        down_cmd.twist.linear_x = 0.0 
        down_cmd.twist.linear_y = -0.015
        down_cmd.twist.linear_z = 0.0
        
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 7
        
        while not rospy.is_shutdown():
            loop_cnt += 1
            recent_forces.append(self.current_f_total)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)
                
            if loop_cnt > 60:
                if len(recent_forces) >= verify_size and all(f >= 12.0 for f in recent_forces):
                    rospy.loginfo(f"馃煝 鍒ゅ畾瑙﹀強绾哥琛ㄩ潰锛佹帴瑙﹀悎鍔? {self.current_f_total:.2f} N")
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
                    
            self.vel_pub.publish(down_cmd)
            rate.sleep()
            
        if contact_detected:
            rospy.sleep(0.5)
            rospy.loginfo(f"馃搷 瀵婚潰璧风偣閿佸畾: X={self.current_x:.4f}, Y={self.current_y:.4f}, Z={self.current_z:.4f}")
            return self.current_x, self.current_y, self.current_z
        else:
            raise RuntimeError("瀵婚潰绋嬪簭寮傚父缁堟")

    def execute_and_draw(self, csv_file):
        rospy.loginfo(f"璇诲彇 2D 杞ㄨ抗鏂囦欢: {csv_file}")
        raw_waypoints = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append({
                    'x': float(row['x_m']),
                    'y': float(row['y_m']),
                    'phase': row['phase'],
                    'stroke_id': int(row['stroke_id'])
                })
        
        if not raw_waypoints: return
            
        contact_x, contact_y, contact_z = self.run_auto_touchdown()
        
        first_draw_idx = 0
        for idx, wp in enumerate(raw_waypoints):
            if wp['phase'] in ['draw', 'touch_down']:
                first_draw_idx = idx
                break
                
        # 鎭㈠榛樿鏄犲皠锛氫綘鎵庝笅鍘荤殑鈥滄帴瑙︾偣鈥濓紝灏辨槸杞ㄨ抗鐨勭涓€绗旇捣绗旂偣锛?        u_ref = raw_waypoints[first_draw_idx]['x']
        v_ref = raw_waypoints[first_draw_idx]['y']
        
        aligned_waypoints = []
        for wp in raw_waypoints:
            du = wp['x'] - u_ref
            dv = wp['y'] - v_ref
            
            # Y 鎺у埗娣卞害 (寰€ -Y 鍘嬪叆)銆?            # X 鎺у埗宸﹀彸锛氶潰鍚?-Y 鏃讹紝鍙宠竟鏄?-X 杞淬€傝建杩?x 澧炲ぇ鏃讹紝X鍑忓皬銆?            # Z 鎺у埗涓婁笅锛氬師杞ㄨ抗鏄熸槦鏄粠涓婂線涓嬬敾锛坹鍑忓皬, dv涓鸿礋锛夈€?            # 涓轰簡瀹炵幇鈥滄墡涓嬪幓灏卞線涓婄敾鈥濓紝鎴戜滑鍙嶈浆 Z 杞存槧灏勶紙鍑忓幓 dv锛夈€?            # 杩欎細鎶婂浘褰笂涓嬮鍊掞紝浣嗗畬缇庢弧瓒充簡浠庝笅寰€涓婄敾鐨勭墿鐞嗛渶姹傦紒
            aligned_waypoints.append({
                'x': contact_x - du,
                'y': contact_y, 
                'z': contact_z - dv, 
                'phase': wp['phase']
            })
            
        rate = rospy.Rate(40)
        dt = 0.025
        y_offset_relief = 0.0
        draw_force_window = []
        actual_log = []
        
        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown(): break
                
            k_pos = 1.2
            stuck_cnt = 0
            prev_pos = np.array([self.current_x, self.current_y, self.current_z])
            
            while not rospy.is_shutdown():
                curr_pos = np.array([self.current_x, self.current_y, self.current_z])
                
                # 持续记录高频实际执行轨迹 (仅在落笔阶段)
                if wp['phase'] in ['draw', 'touch_down']:
                    actual_log.append({'x': curr_pos[0], 'y': curr_pos[1], 'z': curr_pos[2]})
                
                # XZ 骞抽潰鐨勪綅绉昏宸?                err_x = wp['x'] - curr_pos[0]
                err_z = wp['z'] - curr_pos[2]
                dist_to_target = math.hypot(err_x, err_z)
                
                if dist_to_target < 0.005:
                    break
                    
                draw_force_window.append(self.current_f_total)
                if len(draw_force_window) > 4: draw_force_window.pop(0)
                f_filtered = np.mean(draw_force_window)
                
                if wp['phase'] in ['draw', 'touch_down'] and dist_to_target > 0.01:
                    if np.linalg.norm(curr_pos - prev_pos) < 0.0001:
                        stuck_cnt += 1
                    else:
                        stuck_cnt = max(0, stuck_cnt - 1)
                else:
                    stuck_cnt = 0
                prev_pos = curr_pos
                
                # 瀹夊叏娉勫帇锛氬鏋?3D 鍚堝姏閬囧埌澶ч樆鍔涜€屾崯锛?                if wp['phase'] in ['draw', 'touch_down']:
                    if f_filtered > 10.0 or stuck_cnt > 8:
                        y_offset_relief += 0.015 * dt 
                    elif f_filtered < 5.0:
                        y_offset_relief -= 0.005 * dt 
                    y_offset_relief = np.clip(y_offset_relief, 0.0, 0.015)
                else:
                    y_offset_relief = 0.0
                    
                fixed_depth = 0.003
                if wp['phase'] in ['draw', 'touch_down']:
                    depth_offset = fixed_depth - y_offset_relief
                else:
                    depth_offset = -0.015 
                
                target_y = wp['y'] - depth_offset
                err_y = target_y - curr_pos[1]
                
                cmd = TwistCommand()
                cmd.reference_frame = 3
                cmd.duration = 0
                cmd.twist.linear_x = np.clip(k_pos * err_x, -0.06, 0.06)
                cmd.twist.linear_y = np.clip(k_pos * err_y, -0.06, 0.06)
                cmd.twist.linear_z = np.clip(k_pos * err_z, -0.06, 0.06)
                
                self.vel_pub.publish(cmd)
                rate.sleep()
                
            rospy.loginfo(f"杩涘害: {i+1}/{len(aligned_waypoints)} | 3D鍚堝姏: {self.current_f_total:.2f}N | Y閫€缂? {y_offset_relief:.4f}m")
            
        rospy.loginfo("馃洃 缁樺埗鍒拌揪缁堢偣锛屾部 +Y 杞村悜澶栨嫈鍑?..")
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        lift_cmd.twist.linear_x = 0.0
        lift_cmd.twist.linear_y = 0.03
        lift_cmd.twist.linear_z = 0.0
        
        for _ in range(40):
            if rospy.is_shutdown(): break
            self.vel_pub.publish(lift_cmd)
            rospy.sleep(0.025)
            
        stop_cmd = TwistCommand()
        stop_cmd.reference_frame = 3
        for _ in range(15):
            self.vel_pub.publish(stop_cmd)
            rospy.sleep(0.01)
            
        rospy.loginfo("🎉 -Y 轴侧面绘制任务圆满完成！")
        
        # ========== 新增功能：自动保存轨迹并绘图对比 ==========
        theo_csv = "theo_mapped_trajectory.csv"
        with open(theo_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z'])
            writer.writeheader()
            for w in aligned_waypoints:
                if w['phase'] in ['draw', 'touch_down']:
                    writer.writerow({'x': w['x'], 'y': w['y'], 'z': w['z']})
                    
        actual_csv = "actual_executed_trajectory.csv"
        with open(actual_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z'])
            writer.writeheader()
            for p in actual_log:
                writer.writerow(p)
                
        rospy.loginfo(f"📊 轨迹数据已自动保存至 {theo_csv} 和 {actual_csv}")
        rospy.loginfo("📈 正在弹窗显示 3D 轨迹误差对比图...")
        try:
            from analyze_error import analyze_and_plot
            analyze_and_plot(actual_csv, theo_csv)
        except Exception as e:
            rospy.logerr(f"❌ 自动绘图异常: {e}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("鐢ㄦ硶: python3 side_contact_draw.py <path_to_2d_csv>")
        sys.exit(1)
        
    try:
        drawer = SideContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
