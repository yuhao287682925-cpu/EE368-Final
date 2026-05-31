#!/usr/bin/env python3
import csv
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

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


FREE_SPACE = 0
SOFT_CONTACT = 1
HARD_CONTACT = 2


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    nx: float
    ny: float
    nz: float
    phase: str
    stroke_id: int
    pen_down: bool
    raw: Dict[str, str]


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


class ForcePaintingController:
    def __init__(self):
        rospy.init_node("painting_control", anonymous=True)

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

        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 40.0))
        self.target_force = float(rospy.get_param("~target_force", 13.0))
        self.activation_force_threshold = float(rospy.get_param("~activation_force_threshold", 1.0))
        self.force_deadband = float(rospy.get_param("~force_deadband", 0.8))
        self.contact_confirm_samples = int(rospy.get_param("~contact_confirm_samples", 5))
        self.safe_travel_height = float(rospy.get_param("~safe_travel_height", 0.15))
        self.draw_height = float(rospy.get_param("~draw_height", 0.028))
        self.xy_speed = float(rospy.get_param("~xy_speed", 0.03))
        self.xy_speed_low = float(rospy.get_param("~xy_speed_low", 0.015))
        self.z_speed_limit = float(rospy.get_param("~z_speed_limit", 0.01))
        self.z_relief_limit = float(rospy.get_param("~z_relief_limit", 0.015))
        self.z_relief_step = float(rospy.get_param("~z_relief_step", 0.0025))
        self.wrist_torque_threshold = float(rospy.get_param("~wrist_torque_threshold", 0.06))
        self.normal_yaw_deg = float(rospy.get_param("~normal_yaw_deg", -50.0))
        self.xy_offset_x = float(rospy.get_param("~xy_offset_x", 0.0))
        self.xy_offset_y = float(rospy.get_param("~xy_offset_y", 0.0))
        self.xy_offset_z = float(rospy.get_param("~xy_offset_z", 0.0))
        self.normalized_scale = float(rospy.get_param("~normalized_scale", 2.5))
        self.normalized_shift = float(rospy.get_param("~normalized_shift", 0.1))
        self.lift_recovery_height = float(rospy.get_param("~lift_recovery_height", 0.15))
        self.motion_guard_samples = int(rospy.get_param("~motion_guard_samples", 60))

        self.kp_up = float(rospy.get_param("~kp_up", 0.005))
        self.kd_up = float(rospy.get_param("~kd_up", 0.001))
        self.kp_down = float(rospy.get_param("~kp_down", 0.0008))
        self.kd_down = float(rospy.get_param("~kd_down", 0.0001))

        self.state = FREE_SPACE
        self.state_counter = 0
        self.target_active = False
        self.force_lock = False
        self.contact_confirm_count = 0
        self.motion_penalty = 0.0
        self.z_offset = 0.0
        self.z_relief = 0.0
        self.prev_force_error = 0.0
        self.prev_joint_velocity = None
        self.last_callback_time = None
        self.allow_dynamic_calibration = True

        self.static_fz_bias = 0.0
        self.static_wrist_bias = 0.0
        self.gravity_coeffs = None
        self.gravity_fit_r = []
        self.gravity_fit_fz = []
        self.calibration_samples = []
        self.wrist_calibration_samples = []
        self.calibrated = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_fz_raw = 0.0
        self.current_fz = 0.0
        self.current_wrist_torque = 0.0
        self.raw_wrist_torque = 0.0
        self.current_speed_norm = 0.0
        self.current_acc_norm = 0.0
        self.current_linear_acc_norm = 0.0
        self.is_static = True

        self.contact_pose = None

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
        self.static_fz_bias = 0.0
        self.static_wrist_bias = 0.0
        self.gravity_coeffs = None
        self.gravity_fit_r = []
        self.gravity_fit_fz = []
        self.calibration_samples = []
        self.wrist_calibration_samples = []
        self.calibrated = False
        self.allow_dynamic_calibration = True

    def _gravity_bias(self):
        if self.gravity_coeffs is not None:
            current_r = math.hypot(self.current_x, self.current_y)
            return float(self.gravity_coeffs[0] * current_r + self.gravity_coeffs[1])
        return float(self.static_fz_bias)

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

    def _estimate_motion_penalty(self):
        penalty = 0.7 * self.current_linear_acc_norm + 0.12 * self.current_acc_norm
        if not self.is_static:
            penalty += 0.22 * self.current_speed_norm
        penalty += 0.18 * abs(self.current_wrist_torque)
        penalty = min(penalty, 8.0)
        self.motion_penalty = 0.88 * self.motion_penalty + 0.12 * penalty
        return self.motion_penalty

    def _stop_command(self):
        cmd = TwistCommand()
        cmd.reference_frame = 3
        cmd.duration = 0
        return cmd

    def _publish_velocity(self, vx, vy, vz):
        cmd = TwistCommand()
        cmd.reference_frame = 3
        cmd.duration = 0
        cmd.twist.linear_x = float(vx)
        cmd.twist.linear_y = float(vy)
        cmd.twist.linear_z = float(vz)
        cmd.twist.angular_x = 0.0
        cmd.twist.angular_y = 0.0
        cmd.twist.angular_z = 0.0
        self.vel_pub.publish(cmd)

    def _publish_stop(self, repeat=8, sleep_s=0.01):
        cmd = self._stop_command()
        for _ in range(repeat):
            if rospy.is_shutdown():
                break
            self.vel_pub.publish(cmd)
            rospy.sleep(sleep_s)

    def _extract_numeric(self, row, keys, default=None):
        for key in keys:
            value = row.get(key)
            if value not in (None, ""):
                try:
                    return float(value)
                except ValueError:
                    continue
        return default

    def _map_normalized_xy(self, x_norm, y_norm):
        x = (x_norm + self.normalized_shift) / self.normalized_scale
        y = (y_norm + self.normalized_shift) / self.normalized_scale
        theta = math.radians(self.normal_yaw_deg)
        x_rot = math.cos(theta) * x - math.sin(theta) * y
        y_rot = math.sin(theta) * x + math.cos(theta) * y
        return x_rot + self.xy_offset_x, y_rot + self.xy_offset_y

    def _parse_waypoint_row(self, row):
        phase = str(row.get("phase", "draw")).strip()
        phase_lower = phase.lower()
        stroke_id = int(float(row.get("stroke_id", 0) or 0))
        pen_down_raw = str(row.get("pen_down", "1")).strip()
        pen_down = pen_down_raw not in ("0", "false", "False")

        nx = self._extract_numeric(row, ["nx"], 0.0)
        ny = self._extract_numeric(row, ["ny"], 0.0)
        nz = self._extract_numeric(row, ["nz"], 1.0)

        if any(key in row and row.get(key) not in (None, "") for key in ("x_norm", "y_norm")):
            x_norm = self._extract_numeric(row, ["x_norm"], 0.0)
            y_norm = self._extract_numeric(row, ["y_norm"], 0.0)
            x, y = self._map_normalized_xy(x_norm, y_norm)
            z = self._extract_numeric(row, ["z_m", "z_norm"], self.draw_height)
        else:
            x = self._extract_numeric(row, ["x_m", "x"], 0.0)
            y = self._extract_numeric(row, ["y_m", "y"], 0.0)
            z = self._extract_numeric(row, ["z_m", "z"], self.draw_height)

        if phase_lower == "break":
            pen_down = False
            z = self.lift_recovery_height

        if phase_lower in ("lift", "hover", "approach") and z < self.lift_recovery_height:
            z = self.lift_recovery_height

        return Waypoint(
            x=float(x),
            y=float(y),
            z=float(z),
            nx=float(nx),
            ny=float(ny),
            nz=float(nz),
            phase=phase,
            stroke_id=stroke_id,
            pen_down=bool(pen_down),
            raw=row,
        )

    def load_trajectory(self, csv_file):
        raw_waypoints = []
        with open(csv_file, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                raw_waypoints.append(self._parse_waypoint_row(row))

        if not raw_waypoints:
            raise RuntimeError("trajectory file is empty")

        first_draw_idx = None
        for idx, wp in enumerate(raw_waypoints):
            if wp.pen_down and wp.phase.lower() not in ("lift", "hover"):
                first_draw_idx = idx
                break
        if first_draw_idx is None:
            first_draw_idx = 0

        anchor = raw_waypoints[first_draw_idx]
        return raw_waypoints, anchor

    def _estimate_contact_from_joint_state(self, msg):
        thetas = np.array(msg.position[0:6], dtype=float)
        torques = np.array(msg.effort[0:6], dtype=float)
        velocities = np.array(msg.velocity[0:6], dtype=float) if msg.velocity else np.zeros(6)

        if len(thetas) < 6 or len(torques) < 6 or len(velocities) < 6:
            return None

        now = rospy.get_time()
        if self.last_callback_time is None:
            dt = 0.025
        else:
            dt = max(now - self.last_callback_time, 1e-3)
        self.last_callback_time = now

        qdot = velocities[:6]
        if self.prev_joint_velocity is None:
            qddot = np.zeros(6)
        else:
            qddot = (qdot - self.prev_joint_velocity) / dt
        self.prev_joint_velocity = qdot.copy()

        pose = self.arm_model.forward_kinematics(thetas)
        self.current_x = float(pose[0])
        self.current_y = float(pose[1])
        self.current_z = float(pose[2])

        J = self.arm_model.basic_jacobian(thetas)
        wrench = np.linalg.pinv(J.T).dot(torques)
        raw_fz = float(wrench[2])

        self.raw_wrist_torque = float(torques[5])
        self.current_speed_norm = float(np.linalg.norm(qdot))
        self.current_acc_norm = float(np.linalg.norm(qddot))
        self.current_linear_acc_norm = float(np.linalg.norm(J[0:3, :].dot(qddot)))
        self.is_static = self.current_speed_norm < 0.005 and self.current_acc_norm < 0.02 and self.current_linear_acc_norm < 0.05
        self.current_fz_raw = raw_fz
        self.current_wrist_torque = abs(self.raw_wrist_torque - self.static_wrist_bias)

        if not self.calibrated:
            if self.is_static:
                self.calibration_samples.append(raw_fz)
                self.wrist_calibration_samples.append(self.raw_wrist_torque)
                current_r = math.hypot(self.current_x, self.current_y)
                self.gravity_fit_r.append(current_r)
                self.gravity_fit_fz.append(raw_fz)
            if len(self.calibration_samples) >= 40:
                self.static_fz_bias = float(np.mean(self.calibration_samples))
                self.static_wrist_bias = float(np.mean(self.wrist_calibration_samples)) if self.wrist_calibration_samples else 0.0
                self.calibrated = True
                self._update_gravity_model()
                rospy.loginfo(
                    "✅ Zero calibration done. fz_bias=%.3f, wrist_bias=%.4f"
                    % (self.static_fz_bias, self.static_wrist_bias)
                )
            return None

        if self.allow_dynamic_calibration and self.state == FREE_SPACE and self.is_static and abs(raw_fz - self.static_fz_bias) < 1.5:
            self.static_fz_bias = 0.999 * self.static_fz_bias + 0.001 * raw_fz
            self.static_wrist_bias = 0.999 * self.static_wrist_bias + 0.001 * self.raw_wrist_torque
            current_r = math.hypot(self.current_x, self.current_y)
            self.gravity_fit_r.append(current_r)
            self.gravity_fit_fz.append(raw_fz)
            if len(self.gravity_fit_r) > 120:
                self.gravity_fit_r.pop(0)
                self.gravity_fit_fz.pop(0)
            self._update_gravity_model()

        gravity_bias = self._gravity_bias()
        contact_mag = abs(raw_fz - gravity_bias)
        motion_penalty = self._estimate_motion_penalty()
        compensated_fz = max(0.0, contact_mag - motion_penalty)

        if self.state == FREE_SPACE and self.is_static and compensated_fz < 1.0:
            self.current_fz = max(0.0, abs(raw_fz - self.static_fz_bias) - motion_penalty)
        else:
            self.current_fz = compensated_fz

        self.force_fz_pub.publish(Float64(self.current_fz))
        return self.current_fz

    def _touchdown(self):
        rospy.loginfo("🚀 Starting touchdown sequence...")
        self.state = FREE_SPACE
        self.reset_calibration()
        rospy.sleep(1.5)

        while not self.calibrated and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rate = rospy.Rate(self.command_rate_hz)
        down_cmd = TwistCommand()
        down_cmd.reference_frame = 3
        down_cmd.duration = 0
        down_cmd.twist.linear_z = -0.010
        down_cmd.twist.linear_x = 0.0
        down_cmd.twist.linear_y = 0.0
        down_cmd.twist.angular_x = 0.0
        down_cmd.twist.angular_y = 0.0
        down_cmd.twist.angular_z = 0.0

        self.contact_confirm_count = 0
        startup_guard = int(self.command_rate_hz * 1.5)

        while not rospy.is_shutdown():
            if self.current_fz >= self.activation_force_threshold:
                self.contact_confirm_count += 1
            else:
                self.contact_confirm_count = 0

            if startup_guard > 0:
                startup_guard -= 1
            else:
                if self.contact_confirm_count >= self.contact_confirm_samples:
                    rospy.loginfo("🟢 Contact confirmed during touchdown.")
                    self._publish_stop(repeat=12, sleep_s=0.005)
                    pose = Pose()
                    pose.position.x = self.current_x
                    pose.position.y = self.current_y
                    pose.position.z = self.current_z
                    return pose

            self.vel_pub.publish(down_cmd)
            rate.sleep()

        raise RuntimeError("Touchdown aborted")

    def _compute_target_control(self, waypoint, target_z_base, target_force_active):
        force_error = self.target_force - self.current_fz
        d_error = (force_error - self.prev_force_error) * self.command_rate_hz
        self.prev_force_error = force_error

        if not target_force_active:
            self.z_offset = 0.98 * self.z_offset
            if self.z_offset > 0.0:
                self.z_offset = max(0.0, self.z_offset - 0.0005)
            return 0.0

        if self.current_fz < self.activation_force_threshold:
            self.z_offset = 0.98 * self.z_offset
            return -0.0015

        if force_error < -self.force_deadband:
            delta_z = -(self.kp_up * force_error + self.kd_up * d_error)
        elif force_error > self.force_deadband:
            delta_z = -(self.kp_down * force_error + self.kd_down * d_error)
        else:
            delta_z = 0.0

        delta_z = float(np.clip(delta_z, -self.z_speed_limit, self.z_speed_limit))
        delta_z *= max(0.25, 1.0 - 0.12 * self.motion_penalty)
        self.z_offset = 0.995 * self.z_offset + delta_z / self.command_rate_hz
        self.z_offset = float(np.clip(self.z_offset, -0.03, 0.015))
        self.z_offset_pub.publish(Float64(self.z_offset))
        return delta_z

    def _update_state_machine(self, is_draw_phase):
        if not is_draw_phase:
            self.state = FREE_SPACE
            self.state_counter = 0
            return

        if self.state == FREE_SPACE:
            if self.current_fz >= self.activation_force_threshold:
                self.state_counter += 1
                if self.state_counter >= self.contact_confirm_samples:
                    self.state = SOFT_CONTACT
                    self.state_counter = 0
                    rospy.loginfo("🟠 FREE_SPACE -> SOFT_CONTACT")
            else:
                self.state_counter = 0
        elif self.state == SOFT_CONTACT:
            if self.current_fz >= self.target_force - 0.5 or self.current_wrist_torque > self.wrist_torque_threshold:
                self.state_counter += 1
                if self.state_counter >= self.contact_confirm_samples:
                    self.state = HARD_CONTACT
                    self.state_counter = 0
                    rospy.loginfo("🔴 SOFT_CONTACT -> HARD_CONTACT")
            elif self.current_fz < 0.8:
                self.state = FREE_SPACE
                self.state_counter = 0
            else:
                self.state_counter = 0
        else:
            if self.current_fz < self.activation_force_threshold * 0.8:
                self.state_counter += 1
                if self.state_counter >= self.contact_confirm_samples:
                    self.state = FREE_SPACE
                    self.state_counter = 0
                    rospy.loginfo("🔵 HARD_CONTACT -> FREE_SPACE")
            elif self.current_fz < self.target_force - 1.5:
                self.state_counter += 1
                if self.state_counter >= self.contact_confirm_samples + 2:
                    self.state = SOFT_CONTACT
                    self.state_counter = 0
                    rospy.loginfo("🟠 HARD_CONTACT -> SOFT_CONTACT")
            else:
                self.state_counter = 0

    def _move_toward(self, target_x, target_y, target_z, max_xy=0.03, max_z=0.02, position_tolerance=0.001):
        rate = rospy.Rate(self.command_rate_hz)
        while not rospy.is_shutdown():
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dz = target_z - self.current_z
            if math.hypot(dx, dy) <= position_tolerance and abs(dz) <= position_tolerance:
                break

            xy_scale = 0.35 if self.motion_penalty > 2.5 else 1.0
            self._publish_velocity(
                np.clip(dx * 1.1 * xy_scale, -max_xy, max_xy),
                np.clip(dy * 1.1 * xy_scale, -max_xy, max_xy),
                np.clip(dz * 1.2, -max_z, max_z),
            )
            rate.sleep()

        self._publish_stop(repeat=6, sleep_s=0.01)

    def _lift_until_free(self, min_free_force=0.8, lift_speed=0.025, max_cycles=80):
        lift_cmd = TwistCommand()
        lift_cmd.reference_frame = 3
        lift_cmd.duration = 0
        lift_cmd.twist.linear_x = 0.0
        lift_cmd.twist.linear_y = 0.0
        lift_cmd.twist.linear_z = lift_speed
        lift_cmd.twist.angular_x = 0.0
        lift_cmd.twist.angular_y = 0.0
        lift_cmd.twist.angular_z = 0.0

        for _ in range(max_cycles):
            if rospy.is_shutdown():
                break
            if self.current_fz <= min_free_force:
                break
            self.vel_pub.publish(lift_cmd)
            rospy.sleep(0.025)
        self._publish_stop(repeat=10, sleep_s=0.01)

    def execute_and_draw(self, csv_file):
        raw_waypoints, anchor = self.load_trajectory(csv_file)
        rospy.loginfo("Loading trajectory file: %s" % csv_file)

        self.contact_pose = self._touchdown()

        if anchor.pen_down:
            anchor_x = anchor.x
            anchor_y = anchor.y
            anchor_z = anchor.z
        else:
            anchor_x = raw_waypoints[0].x
            anchor_y = raw_waypoints[0].y
            anchor_z = raw_waypoints[0].z

        aligned_waypoints = []
        for wp in raw_waypoints:
            aligned_waypoints.append(
                Waypoint(
                    x=self.contact_pose.position.x + (wp.x - anchor_x),
                    y=self.contact_pose.position.y + (wp.y - anchor_y),
                    z=self.contact_pose.position.z + (wp.z - anchor_z),
                    nx=wp.nx,
                    ny=wp.ny,
                    nz=wp.nz,
                    phase=wp.phase,
                    stroke_id=wp.stroke_id,
                    pen_down=wp.pen_down,
                    raw=wp.raw,
                )
            )

        rate = rospy.Rate(self.command_rate_hz)
        draw_force_window: List[float] = []
        draw_force_window_size = 6
        self.z_relief = 0.0

        for index, wp in enumerate(aligned_waypoints):
            if rospy.is_shutdown():
                break

            rospy.loginfo(
                "▶ Waypoint %d/%d | stroke=%d | phase=%s | pen_down=%s"
                % (index + 1, len(aligned_waypoints), wp.stroke_id, wp.phase, wp.pen_down)
            )

            target_quat = get_orientation_for_normal(wp.nx, wp.ny, wp.nz)
            _ = target_quat

            if wp.phase.lower() == "break" or not wp.pen_down:
                self._publish_stop(repeat=4, sleep_s=0.01)
                self._move_toward(wp.x, wp.y, self.safe_travel_height, max_xy=0.05, max_z=0.03, position_tolerance=0.002)
                self._lift_until_free(min_free_force=0.8, lift_speed=0.03, max_cycles=40)
                self.state = FREE_SPACE
                self.state_counter = 0
                self.z_offset = 0.0
                self.prev_force_error = 0.0
                self.z_relief = 0.0
                continue

            target_force_active = True
            self.target_active = True
            self._move_toward(wp.x, wp.y, wp.z, max_xy=0.03, max_z=0.02, position_tolerance=0.001)

            prev_servo_x = self.current_x
            prev_servo_y = self.current_y
            inner_loops = 0

            while not rospy.is_shutdown():
                inner_loops += 1
                dx = wp.x - self.current_x
                dy = wp.y - self.current_y
                dz_base = wp.z - self.current_z
                dist_to_target = math.hypot(dx, dy)

                if dist_to_target < 0.0008 and abs(dz_base) < 0.001:
                    break

                if len(draw_force_window) >= draw_force_window_size:
                    draw_force_window.pop(0)
                draw_force_window.append(self.current_fz)
                filtered_force = float(np.mean(draw_force_window)) if draw_force_window else self.current_fz

                movement = math.hypot(self.current_x - prev_servo_x, self.current_y - prev_servo_y)
                prev_servo_x = self.current_x
                prev_servo_y = self.current_y

                if dist_to_target > 0.008 and movement < 0.00008 and self.current_fz > self.activation_force_threshold:
                    self.z_relief = min(self.z_relief + self.z_relief_step, self.z_relief_limit)
                    rospy.logwarn(
                        "⚠️ Horizontal jam suspected, increasing Z relief to %.4f m (F=%.2f, penalty=%.2f)"
                        % (self.z_relief, filtered_force, self.motion_penalty)
                    )
                elif filtered_force < self.activation_force_threshold:
                    self.z_relief = max(0.0, self.z_relief - 0.0008)

                self._update_state_machine(is_draw_phase=True)
                delta_z = self._compute_target_control(wp, wp.z, target_force_active=True)

                if self.current_fz < self.activation_force_threshold:
                    contact_seek = -0.0015
                else:
                    contact_seek = 0.0

                target_z = wp.z + self.z_offset + self.z_relief + contact_seek

                if self.current_fz > self.target_force + 6.0 or self.current_wrist_torque > max(0.08, self.wrist_torque_threshold):
                    self.z_relief = min(self.z_relief + 0.002, self.z_relief_limit)
                    target_z = max(target_z, self.current_z + 0.002)

                k_xy = self.xy_speed_low if self.motion_penalty > 2.5 else self.xy_speed
                self._publish_velocity(
                    float(np.clip(k_xy * dx, -0.05, 0.05)),
                    float(np.clip(k_xy * dy, -0.05, 0.05)),
                    float(np.clip(delta_z + (target_z - self.current_z), -self.z_speed_limit, self.z_speed_limit)),
                )
                rate.sleep()

            if wp.pen_down:
                self.z_relief = max(0.0, self.z_relief - 0.001)

            rospy.loginfo(
                "Progress %d/%d | raw=%.2fN | net=%.2fN | penalty=%.2f | z_offset=%.4f | relief=%.4f"
                % (
                    index + 1,
                    len(aligned_waypoints),
                    self.current_fz_raw,
                    self.current_fz,
                    self.motion_penalty,
                    self.z_offset,
                    self.z_relief,
                )
            )

        rospy.loginfo("🛑 Drawing finished, settling and lifting pen...")
        self._publish_stop(repeat=20, sleep_s=0.01)
        self._lift_until_free(min_free_force=0.8, lift_speed=0.03, max_cycles=80)
        self._publish_stop(repeat=15, sleep_s=0.01)
        self.move_group.stop()
        rospy.loginfo("🎉 Painting control completed.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 auto_contact_draw_3d_force_compensated.py <path_to_csv>")
        sys.exit(1)

    try:
        controller = ForcePaintingController()
        controller.execute_and_draw(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
