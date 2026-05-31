#!/usr/bin/env python3
import csv
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion
from kortex_driver.msg import TwistCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, ".."))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from jacobian import NLinkArm
from scipy.spatial.transform import Rotation as R


def get_orientation_for_normal(nx, ny, nz, default_rpy_deg=(0.0, 180.0, 0.0)):
    r_default = R.from_euler("xyz", default_rpy_deg, degrees=True)
    v_from = np.array([0.0, 0.0, 1.0])
    v_to = np.array([nx, ny, nz])

    if np.allclose(v_from, v_to):
        q = r_default.as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    axis = np.cross(v_from, v_to)
    axis_len = np.linalg.norm(axis)
    if axis_len < 1e-6:
        r_align = R.from_euler("x", 180, degrees=True)
    else:
        axis = axis / axis_len
        angle = np.arccos(np.clip(np.dot(v_from, v_to), -1.0, 1.0))
        r_align = R.from_rotvec(axis * angle)

    q = (r_align * r_default).as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


FREE_SPACE = 0
SOFT_CONTACT = 1
HARD_CONTACT = 2


class AutoContactDrawer:
    def __init__(self):
        rospy.init_node("auto_contact_draw_compensated", anonymous=True)

        dh_params_list = np.array(
            [
                [0, 0, 243.3 / 1000, 0],
                [math.pi / 2, 0, 10 / 1000, math.pi / 2],
                [math.pi, 280 / 1000, 0, math.pi / 2],
                [math.pi / 2, 0, 245 / 1000, math.pi / 2],
                [math.pi / 2, 0, 57 / 1000, 0],
                [-math.pi / 2, 0, 235 / 1000, -math.pi / 2],
            ]
        )
        self.arm_model = NLinkArm(dh_params_list)

        self.state = FREE_SPACE
        self.state_counter = 0

        self.base_target_force = 4.0
        self.target_force = 4.0
        self.contact_threshold = 2.5
        self.wrist_torque_threshold = 0.06

        self.kp_up = 0.005
        self.kd_up = 0.001
        self.kp_down = 0.0008
        self.kd_down = 0.0001

        self.max_step = 0.01
        self.z_offset = 0.0
        self.max_z_offset = 0.015
        self.min_z_offset = -0.03

        self.static_fz_bias = 0.0
        self.static_wrist_bias = 0.0
        self.gravity_coeffs = None
        self.gravity_fit_r = []
        self.gravity_fit_fz = []
        self.calibration_samples = []
        self.torque_calibration_samples = []
        self.calibrated = False
        self.allow_dynamic_calibration = True

        self.current_fz = 0.0
        self.current_fz_raw = 0.0
        self.current_wrist_torque = 0.0
        self.raw_wrist_torque = 0.0
        self.motion_penalty = 0.0
        self.linear_acc_norm = 0.0
        self.joint_acc_norm = 0.0
        self.current_speed_norm = 0.0
        self.is_static = True
        self.prev_force_error = 0.0

        self.last_joint_velocity = None
        self.last_callback_time = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        rospy.Subscriber("/my_gen3_lite/joint_states", JointState, self.joint_states_callback)

        self.vel_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.force_fz_pub = rospy.Publisher("/force_control/auto/estimated_fz", Float64, queue_size=1)
        self.z_offset_pub = rospy.Publisher("/force_control/auto/z_offset", Float64, queue_size=1)

        import moveit_commander

        self.robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description")
        self.move_group = moveit_commander.MoveGroupCommander(
            "arm", robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite"
        )
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

    def reset_calibration(self):
        self.calibrated = False
        self.calibration_samples = []
        self.torque_calibration_samples = []
        self.gravity_fit_r = []
        self.gravity_fit_fz = []
        self.gravity_coeffs = None

    def _update_gravity_model(self):
        if len(self.gravity_fit_r) < 8:
            return

        r_span = max(self.gravity_fit_r) - min(self.gravity_fit_r)
        if r_span < 0.03:
            return

        try:
            coeffs = np.polyfit(self.gravity_fit_r, self.gravity_fit_fz, 1)
        except np.linalg.LinAlgError:
            return
        self.gravity_coeffs = [float(coeffs[0]), float(coeffs[1])]

    def _estimate_dynamic_penalty(self):
        penalty = 0.65 * self.linear_acc_norm + 0.10 * self.joint_acc_norm
        if not self.is_static:
            penalty += 0.20 * self.current_speed_norm
        penalty = min(penalty, 6.0)
        self.motion_penalty = 0.85 * self.motion_penalty + 0.15 * penalty
        return self.motion_penalty

    def _gravity_bias(self):
        if self.gravity_coeffs is not None:
            current_r = math.hypot(self.current_x, self.current_y)
            return self.gravity_coeffs[0] * current_r + self.gravity_coeffs[1]
        return self.static_fz_bias

    def _stop_twist(self):
        cmd = TwistCommand()
        cmd.reference_frame = 3
        cmd.duration = 0
        return cmd

    def _publish_stop(self, repeat=10, sleep_s=0.01):
        cmd = self._stop_twist()
        for _ in range(repeat):
            if rospy.is_shutdown():
                break
            self.vel_pub.publish(cmd)
            rospy.sleep(sleep_s)

    def joint_states_callback(self, msg):
        thetas = msg.position[0:6]
        torques = msg.effort[0:6]
        velocities = msg.velocity[0:6] if msg.velocity else []

        if len(thetas) < 6 or len(torques) < 6 or len(velocities) < 6:
            return

        now = rospy.get_time()
        if self.last_callback_time is None:
            dt = 0.025
        else:
            dt = max(now - self.last_callback_time, 1e-3)
        self.last_callback_time = now

        qdot = np.array(velocities[:6], dtype=float)
        if self.last_joint_velocity is None:
            qddot = np.zeros(6)
        else:
            qddot = (qdot - self.last_joint_velocity) / dt
        self.last_joint_velocity = qdot.copy()

        tool_pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = float(tool_pose[0])
        self.current_y = float(tool_pose[1])
        self.current_z = float(tool_pose[2])

        J = self.arm_model.basic_jacobian(thetas)
        tool_wrench = np.linalg.pinv(J.T).dot(torques)
        raw_f3d = np.array(tool_wrench[0:3], dtype=float)
        self.current_fz_raw = float(raw_f3d[2])
        self.raw_wrist_torque = float(torques[5])

        self.current_speed_norm = float(np.linalg.norm(qdot))
        self.joint_acc_norm = float(np.linalg.norm(qddot))
        self.linear_acc_norm = float(np.linalg.norm(J[0:3, :].dot(qddot)))
        self.is_static = (
            self.current_speed_norm < 0.005
            and self.joint_acc_norm < 0.02
            and self.linear_acc_norm < 0.05
        )

        if not self.calibrated:
            if self.is_static:
                self.calibration_samples.append(self.current_fz_raw)
                self.torque_calibration_samples.append(self.raw_wrist_torque)
                if self.current_speed_norm < 0.003 and self.joint_acc_norm < 0.01:
                    current_r = math.hypot(self.current_x, self.current_y)
                    self.gravity_fit_r.append(current_r)
                    self.gravity_fit_fz.append(self.current_fz_raw)

            if len(self.calibration_samples) >= 40:
                self.static_fz_bias = float(np.mean(self.calibration_samples))
                self.static_wrist_bias = float(np.mean(self.torque_calibration_samples))
                self.calibrated = True
                self._update_gravity_model()
                rospy.loginfo(
                    "✅ Calibration done. Static bias: %.2f N, wrist bias: %.3f Nm"
                    % (self.static_fz_bias, self.static_wrist_bias)
                )
            return

        if self.allow_dynamic_calibration and self.is_static and abs(self.current_fz_raw - self.static_fz_bias) < 2.0:
            self.static_fz_bias = 0.9985 * self.static_fz_bias + 0.0015 * self.current_fz_raw
            self.static_wrist_bias = 0.9985 * self.static_wrist_bias + 0.0015 * self.raw_wrist_torque
            current_r = math.hypot(self.current_x, self.current_y)
            self.gravity_fit_r.append(current_r)
            self.gravity_fit_fz.append(self.current_fz_raw)
            if len(self.gravity_fit_r) > 80:
                self.gravity_fit_r.pop(0)
                self.gravity_fit_fz.pop(0)
            self._update_gravity_model()

        gravity_bias = self._gravity_bias()
        contact_fz = abs(self.current_fz_raw - gravity_bias)
        penalty = self._estimate_dynamic_penalty()
        compensated_fz = max(0.0, contact_fz - penalty)

        if self.state == FREE_SPACE and self.is_static and compensated_fz < 1.5:
            self.static_fz_bias = 0.9995 * self.static_fz_bias + 0.0005 * self.current_fz_raw
            self.static_wrist_bias = 0.9995 * self.static_wrist_bias + 0.0005 * self.raw_wrist_torque
            if self.gravity_coeffs is None:
                self.current_fz = abs(self.current_fz_raw - self.static_fz_bias)
            else:
                self.current_fz = compensated_fz
        else:
            self.current_fz = compensated_fz

        self.current_wrist_torque = abs(self.raw_wrist_torque - self.static_wrist_bias)
        self.force_fz_pub.publish(Float64(self.current_fz))

    def run_auto_touchdown(self):
        rospy.loginfo("🚀 Start automatic touchdown with gravity/inertia compensation...")
        self.state = FREE_SPACE
        self.allow_dynamic_calibration = True

        rospy.loginfo("⏸️ Keep the arm still for calibration...")
        self.reset_calibration()
        rospy.sleep(1.5)
        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rate = rospy.Rate(40)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        down_cmd.twist.linear_z = -0.010

        stop_cmd = self._stop_twist()
        contact_detected = False
        loop_cnt = 0
        recent_forces = []
        verify_size = 6

        while not rospy.is_shutdown():
            loop_cnt += 1
            effective_force = max(0.0, self.current_fz - 0.5 * self.motion_penalty)
            recent_forces.append(effective_force)
            if len(recent_forces) > verify_size:
                recent_forces.pop(0)

            if loop_cnt % 15 == 0:
                rospy.loginfo(
                    "⏳ touchdown: F=%.2f N | penalty=%.2f | window=%s"
                    % (effective_force, self.motion_penalty, [round(f, 2) for f in recent_forces])
                )

            if loop_cnt > 60:
                stable_window = len(recent_forces) >= verify_size and all(
                    f >= max(2.5, self.contact_threshold - 0.5) for f in recent_forces
                )
                hard_touch = effective_force >= 18.0 and self.motion_penalty < 2.5
                if stable_window or hard_touch:
                    rospy.loginfo("🟢 Contact detected, stop touchdown.")
                    for _ in range(10):
                        self.vel_pub.publish(stop_cmd)
                        rospy.sleep(0.005)
                    contact_detected = True
                    break
            else:
                if loop_cnt % 15 == 0:
                    rospy.loginfo("⏳ Skip contact check during start-up acceleration.")

            self.vel_pub.publish(down_cmd)
            rate.sleep()

        if not contact_detected:
            raise RuntimeError("Touchdown terminated unexpectedly")

        rospy.sleep(0.5)
        current_pose = Pose()
        current_pose.position.x = self.current_x
        current_pose.position.y = self.current_y
        current_pose.position.z = self.current_z
        rospy.loginfo(
            "📍 Contact pose locked: X=%.4f, Y=%.4f, Z=%.4f"
            % (current_pose.position.x, current_pose.position.y, current_pose.position.z)
        )
        return current_pose

    def update_force_control(self, fz_val, dt=0.025):
        if not self.calibrated:
            self.z_offset = 0.0
            self.prev_force_error = 0.0
            return 0.0, 0.0

        effective_force = max(0.0, fz_val - 0.35 * self.motion_penalty)
        v_z_comp = 0.0

        if self.motion_penalty > 3.0:
            self.z_offset = min(self.max_z_offset, self.z_offset + 0.0012)
            self.prev_force_error = 0.0
            self.z_offset_pub.publish(Float64(self.z_offset))
            return self.z_offset, 0.0

        if effective_force < self.contact_threshold:
            if self.state == FREE_SPACE:
                self.z_offset = max(self.min_z_offset, 0.97 * self.z_offset - 0.0004)
            else:
                self.z_offset = 0.98 * self.z_offset
            self.prev_force_error = 0.0
        elif self.state == SOFT_CONTACT:
            v_z_comp = -0.0015
            self.z_offset = 0.98 * self.z_offset + v_z_comp * dt
            self.prev_force_error = 0.0
        else:
            force_error = self.target_force - effective_force
            d_error = (force_error - self.prev_force_error) / dt if dt > 0 else 0.0
            self.prev_force_error = force_error

            if force_error < -1.0:
                v_z_comp = -(self.kp_up * (force_error + 1.0) + self.kd_up * d_error)
            elif force_error < 0:
                v_z_comp = 0.0
            else:
                v_z_comp = -(self.kp_down * force_error + self.kd_down * d_error)

            v_z_comp = np.clip(v_z_comp, -0.006, 0.006)
            v_z_comp *= max(0.3, 1.0 - 0.15 * self.motion_penalty)
            self.z_offset = 0.995 * self.z_offset + v_z_comp * dt

        self.z_offset = np.clip(self.z_offset, self.min_z_offset, self.max_z_offset)
        self.z_offset_pub.publish(Float64(self.z_offset))
        return self.z_offset, v_z_comp

    def _finish_drawing(self):
        self.state = FREE_SPACE
        self.state_counter = 0
        self.prev_force_error = 0.0
        self.z_offset = 0.0
        self.motion_penalty = 0.0
        self.z_offset_pub.publish(Float64(self.z_offset))
        self._publish_stop(repeat=20, sleep_s=0.01)

        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        lift_cmd.duration = 0
        lift_cmd.twist.linear_z = 0.025
        lift_cmd.twist.linear_x = 0.0
        lift_cmd.twist.linear_y = 0.0
        lift_cmd.twist.angular_x = 0.0
        lift_cmd.twist.angular_y = 0.0
        lift_cmd.twist.angular_z = 0.0

        for _ in range(80):
            if rospy.is_shutdown():
                break
            if self.current_fz < 0.8 and abs(self.current_wrist_torque) < max(0.5, self.wrist_torque_threshold * 2.0):
                break
            self.vel_pub.publish(lift_cmd)
            rospy.sleep(0.025)

        self._publish_stop(repeat=15, sleep_s=0.01)
        self.move_group.stop()

    def execute_and_draw(self, csv_file):
        raw_waypoints = []
        rospy.loginfo("Loading trajectory file: %s" % csv_file)
        with open(csv_file, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append(
                    {
                        "x": float(row["x_m"]),
                        "y": float(row["y_m"]),
                        "z_nominal": float(row["z_m"]),
                        "nx": float(row["nx"]),
                        "ny": float(row["ny"]),
                        "nz": float(row["nz"]),
                        "stroke_id": int(row["stroke_id"]),
                        "phase": row["phase"],
                    }
                )

        contact_pose = self.run_auto_touchdown()

        first_draw_idx = 0
        for idx, wp in enumerate(raw_waypoints):
            if wp["phase"] in ["draw", "touch_down"]:
                first_draw_idx = idx
                break

        u_ref_x = raw_waypoints[first_draw_idx]["x"]
        u_ref_y = raw_waypoints[first_draw_idx]["y"]
        u_ref_z = raw_waypoints[first_draw_idx]["z_nominal"]

        rospy.loginfo("🔄 Rebuilding aligned trajectory from the measured contact point...")
        aligned_waypoints = []
        for wp in raw_waypoints:
            aligned_waypoints.append(
                {
                    "x": contact_pose.position.x + (wp["x"] - u_ref_x),
                    "y": contact_pose.position.y + (wp["y"] - u_ref_y),
                    "z_nominal": contact_pose.position.z + (wp["z_nominal"] - u_ref_z),
                    "nx": wp["nx"],
                    "ny": wp["ny"],
                    "nz": wp["nz"],
                    "phase": wp["phase"],
                    "stroke_id": wp["stroke_id"],
                }
            )

        rospy.loginfo("✅ Trajectory aligned. Start compensated force drawing.")
        self.allow_dynamic_calibration = False

        rate = rospy.Rate(40)
        dt = 0.025
        draw_force_window = []
        draw_window_size = 6
        stuck_cnt = 0
        z_offset_relief = 0.0

        for i, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break

            _quat = get_orientation_for_normal(wp["nx"], wp["ny"], wp["nz"])
            target_x = wp["x"]
            target_y = wp["y"]
            target_z = wp["z_nominal"]
            k_pos = 1.0

            prev_servo_x = self.current_x
            prev_servo_y = self.current_y

            while not rospy.is_shutdown():
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                dz = target_z - self.current_z
                dist_to_target = math.hypot(dx, dy)
                if dist_to_target < 0.0008:
                    break

                f_est = max(0.0, self.current_fz - 0.35 * self.motion_penalty)
                draw_force_window.append(f_est)
                if len(draw_force_window) > draw_window_size:
                    draw_force_window.pop(0)
                fz_filtered = (
                    float(np.mean(draw_force_window))
                    if len(draw_force_window) >= draw_window_size
                    else f_est
                )

                movement = math.hypot(self.current_x - prev_servo_x, self.current_y - prev_servo_y)
                is_drawing_phase = wp["phase"] in ["draw", "touch_down"]
                if is_drawing_phase and dist_to_target > 0.01:
                    if movement < 0.0001 and fz_filtered > self.contact_threshold:
                        stuck_cnt += 1
                    else:
                        stuck_cnt = max(0, stuck_cnt - 1)
                else:
                    stuck_cnt = 0

                prev_servo_x = self.current_x
                prev_servo_y = self.current_y

                if is_drawing_phase:
                    if self.state == FREE_SPACE:
                        if fz_filtered > max(3.0, self.contact_threshold + 0.5 * self.motion_penalty):
                            self.state_counter += 1
                            if self.state_counter >= 5:
                                self.state = SOFT_CONTACT
                                self.state_counter = 0
                                rospy.loginfo("🟠 FREE_SPACE -> SOFT_CONTACT")
                        else:
                            self.state_counter = 0
                    elif self.state == SOFT_CONTACT:
                        if fz_filtered > 5.0 or self.current_wrist_torque > self.wrist_torque_threshold:
                            self.state_counter += 1
                            if self.state_counter >= 5:
                                self.state = HARD_CONTACT
                                self.state_counter = 0
                                rospy.loginfo("🔴 SOFT_CONTACT -> HARD_CONTACT")
                        elif fz_filtered < 1.2:
                            self.state = FREE_SPACE
                            self.state_counter = 0
                            draw_force_window = []
                        else:
                            self.state_counter = 0
                    else:
                        if fz_filtered < 3.0 and self.motion_penalty < 1.5:
                            self.state_counter += 1
                            if self.state_counter >= 8:
                                self.state = SOFT_CONTACT
                                self.state_counter = 0
                                rospy.loginfo("🟠 HARD_CONTACT -> SOFT_CONTACT")
                        elif fz_filtered < 1.0:
                            self.state = FREE_SPACE
                            self.state_counter = 0
                        else:
                            self.state_counter = 0

                    if stuck_cnt > 8:
                        z_offset_relief = min(z_offset_relief + 0.0025, 0.015)
                        rospy.logwarn(
                            "⚠️ Stuck detected. Relief raised to %.4f m (F=%.2f, penalty=%.2f)"
                            % (z_offset_relief, fz_filtered, self.motion_penalty)
                        )
                        stuck_cnt = 0

                    self.update_force_control(fz_filtered, dt)
                    target_z = wp["z_nominal"] + self.z_offset + z_offset_relief
                else:
                    self.state = FREE_SPACE
                    self.state_counter = 0
                    self.z_offset = 0.0
                    self.prev_force_error = 0.0
                    z_offset_relief = 0.0
                    target_z = wp["z_nominal"]

                dz = target_z - self.current_z
                cmd = TwistCommand()
                cmd.reference_frame = 3
                cmd.duration = 0

                xy_scale = 0.35 if self.motion_penalty > 2.5 else 1.0
                cmd.twist.linear_x = float(np.clip(k_pos * dx * xy_scale, -0.05, 0.05))
                cmd.twist.linear_y = float(np.clip(k_pos * dy * xy_scale, -0.05, 0.05))
                cmd.twist.linear_z = float(np.clip(k_pos * dz, -0.03, 0.03))
                cmd.twist.angular_x = 0.0
                cmd.twist.angular_y = 0.0
                cmd.twist.angular_z = 0.0

                self.vel_pub.publish(cmd)
                rate.sleep()

            rospy.loginfo(
                "Progress: %d/%d | raw=%.2fN | net=%.2fN | penalty=%.2f | z_offset=%.4f"
                % (
                    i + 1,
                    len(aligned_waypoints),
                    self.current_fz_raw,
                    self.current_fz,
                    self.motion_penalty,
                    self.z_offset,
                )
            )

        rospy.loginfo("🛑 Drawing finished, settling and lifting pen...")
        self._finish_drawing()
        rospy.loginfo("🎉 Compensated force drawing completed.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 auto_contact_draw_3d_force_compensated.py <path_to_csv>")
        sys.exit(1)

    try:
        drawer = AutoContactDrawer()
        drawer.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
