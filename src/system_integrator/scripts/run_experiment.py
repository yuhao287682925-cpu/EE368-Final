#!/usr/bin/env python3
"""
run_experiment.py - Main experiment script for the block drawing system.

Complete experiment workflow:
  1. Set block pose and dimensions
  2. Call trajectory generator to compute 3D waypoints
  3. Calibrate force sensor baseline
  4. Activate force control
  5. Execute the drawing plan via action server
  6. Record rosbag and save results

Usage:
  rosrun system_integrator run_experiment.py --svg pattern.svg
  rosrun system_integrator run_experiment.py --test-square --width 80 --height 80
"""
import sys
import os
import argparse
import time
import math

import rospy
import rosbag
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
from block_drawing_msgs.srv import (
    SetBlockPose, SetBlockPoseRequest,
    GenerateTrajectory, GenerateTrajectoryRequest,
    ExecuteDrawing, ExecuteDrawingRequest,
)
from block_drawing_msgs.msg import SurfaceTrajectory
from block_drawing_msgs.msg import DrawingExecutionAction, DrawingExecutionGoal


class ExperimentRunner:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        rospy.init_node("experiment_runner")

        # Service proxies
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service("/set_block_pose", timeout=10.0)
        rospy.wait_for_service("/generate_trajectory", timeout=10.0)
        rospy.wait_for_service("/force_controller/calibrate", timeout=10.0)

        self.set_block = rospy.ServiceProxy("/set_block_pose", SetBlockPose)
        self.gen_traj = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)
        self.calibrate = rospy.ServiceProxy("/force_controller/calibrate", Trigger)

        # Action client
        self.action_client = actionlib.SimpleActionClient(
            "/execute_drawing", DrawingExecutionAction)
        rospy.loginfo("Waiting for action server...")
        self.action_client.wait_for_server(timeout=rospy.Duration(10.0))

        # Force control activation
        self.pub_force_active = rospy.Publisher(
            "/force_control/active", Bool, queue_size=1)

        rospy.loginfo("Experiment runner ready.")

    def set_block_pose(self, x, y, z, roll, pitch, yaw, L, W, H,
                        face_offsets_u=None, face_offsets_v=None,
                        center_face=-1, center_u=0.0, center_v=0.0):
        """Set block pose, size, offsets, and continuous mode center."""
        req = SetBlockPoseRequest()
        req.block_pose.position = Point(x, y, z)
        cy = math.cos(yaw / 2); sy = math.sin(yaw / 2)
        cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
        cr = math.cos(roll / 2); sr = math.sin(roll / 2)
        req.block_pose.orientation = Quaternion(
            w=cr*cp*cy + sr*sp*sy, x=sr*cp*cy - cr*sp*sy,
            y=cr*sp*cy + sr*cp*sy, z=cr*cp*sy - sr*sp*cy,
        )
        req.L = L; req.W = W; req.H = H

        if face_offsets_u is None: face_offsets_u = [0.0]*5
        if face_offsets_v is None: face_offsets_v = [0.0]*5
        req.face_offset_u = list(face_offsets_u) + [0.0]*(5-len(face_offsets_u))
        req.face_offset_v = list(face_offsets_v) + [0.0]*(5-len(face_offsets_v))

        req.center_face = center_face
        req.center_u_mm = center_u
        req.center_v_mm = center_v

        resp = self.set_block(req)
        if resp.success:
            rospy.loginfo("Block pose set: %s", resp.message)
        else:
            rospy.logerr("Block pose failed: %s", resp.message)
        return resp.success

    def generate_trajectory(self, svg_file, target_w, target_h, faces, test_pattern=""):
        """Generate 3D surface trajectories."""
        req = GenerateTrajectoryRequest()
        req.svg_file = svg_file
        req.test_pattern = test_pattern
        req.target_width_mm = target_w
        req.target_height_mm = target_h
        req.faces = faces

        resp = self.gen_traj(req)
        if resp.success:
            rospy.loginfo("Trajectory generated: %d segments, %s",
                          len(resp.trajectories), resp.message)
        else:
            rospy.logerr("Trajectory generation failed: %s", resp.message)
        return resp.success, resp.trajectories

    def calibrate_force(self):
        """Calibrate the force sensor baseline."""
        rospy.sleep(1.0)  # Wait for steady state
        resp = self.calibrate()
        if resp.success:
            rospy.loginfo("Force calibration: %s", resp.message)
        else:
            rospy.logwarn("Force calibration: %s", resp.message)
        return resp.success

    def execute_drawing(self, trajectories):
        """Execute the drawing plan with feedback."""
        goal = DrawingExecutionGoal()
        goal.trajectories = trajectories

        self.action_client.send_goal(
            goal,
            feedback_cb=self._feedback_cb,
            done_cb=self._done_cb,
        )

        # Activate force control
        self.pub_force_active.publish(Bool(data=True))

        # Wait for completion
        finished = self.action_client.wait_for_result(
            timeout=rospy.Duration(300.0))  # 5 min timeout

        # Deactivate force control
        self.pub_force_active.publish(Bool(data=False))

        result = self.action_client.get_result()
        if result:
            rospy.loginfo("Drawing %s. Faces: %d, Message: %s",
                          "completed" if result.completed else "incomplete",
                          result.faces_drawn, result.message)
        return finished and result and result.completed

    def _feedback_cb(self, feedback):
        rospy.loginfo_throttle(1.0,
            "Progress: %.1f%% | Face %d, WP %d | Force: %.2f N | %s",
            feedback.progress_fraction * 100,
            feedback.current_face,
            feedback.current_waypoint,
            feedback.estimated_force,
            feedback.state,
        )

    def _done_cb(self, state, result):
        rospy.loginfo("Drawing action finished. State: %d", state)

    def run_experiment(self, svg_file, block_pose, block_size,
                       target_w_mm, target_h_mm, faces,
                       face_offsets_u=None, face_offsets_v=None,
                       test_pattern="",
                       center_face=-1, center_u=0.0, center_v=0.0):
        """Run the full experiment pipeline."""
        rospy.loginfo("="*60)
        rospy.loginfo("EXPERIMENT START")
        rospy.loginfo("="*60)

        # Start rosbag recording
        bag_path = os.path.join(self.output_dir, "experiment.bag")
        rospy.loginfo("Recording rosbag to: %s", bag_path)
        bag = rosbag.Bag(bag_path, 'w')

        try:
            # Step 1: Set block pose
            rospy.loginfo("Step 1: Setting block pose...")
            if not self.set_block_pose(*block_pose, *block_size,
                                        face_offsets_u, face_offsets_v,
                                        center_face, center_u, center_v):
                rospy.logerr("Failed to set block pose. Aborting.")
                return False

            # Step 2: Generate trajectories
            rospy.loginfo("Step 2: Generating trajectories...")
            success, trajectories = self.generate_trajectory(
                svg_file, target_w_mm, target_h_mm, faces, test_pattern)
            if not success:
                rospy.logerr("Failed to generate trajectories. Aborting.")
                return False

            # Step 3: Calibrate force sensor
            rospy.loginfo("Step 3: Calibrating force sensor...")
            self.calibrate_force()

            # Step 4: Execute drawing
            rospy.loginfo("Step 4: Executing drawing...")
            success = self.execute_drawing(trajectories)

            if success:
                rospy.loginfo("EXPERIMENT COMPLETED SUCCESSFULLY")
            else:
                rospy.logwarn("EXPERIMENT ENDED WITH ISSUES")

            return success

        except rospy.ROSException as e:
            rospy.logerr("ROS error: %s", str(e))
            return False
        finally:
            bag.close()
            rospy.loginfo("Rosbag saved: %s", bag_path)


def main():
    parser = argparse.ArgumentParser(
        description="Run block drawing experiment on Kinova Gen3 Lite.")
    parser.add_argument('--svg', default='',
                        help='SVG file path (leave empty for test square)')
    parser.add_argument('--test-square', action='store_true',
                        help='Use built-in test square pattern')
    parser.add_argument('--test-circle', action='store_true',
                        help='Use built-in test circle pattern')
    parser.add_argument('--width', type=float, default=80.0,
                        help='Pattern width [mm]')
    parser.add_argument('--height', type=float, default=80.0,
                        help='Pattern height [mm]')
    parser.add_argument('--faces', type=int, nargs='+', default=[0],
                        help='Face IDs to draw on (0=top, 1=front, 2=right, 3=back, 4=left)')
    parser.add_argument('--output-dir', default='/tmp/block_drawing_exp',
                        help='Output directory for experiment data')
    parser.add_argument('--block-x', type=float, default=0.4,
                        help='Block X position [m]')
    parser.add_argument('--block-y', type=float, default=0.0,
                        help='Block Y position [m]')
    parser.add_argument('--block-z', type=float, default=0.075,
                        help='Block Z position [m] (half-height for floor block)')
    parser.add_argument('--block-roll', type=float, default=0.0)
    parser.add_argument('--block-pitch', type=float, default=0.0)
    parser.add_argument('--block-yaw', type=float, default=0.0)
    parser.add_argument('--block-L', type=float, default=0.20,
                        help='Block length [m]')
    parser.add_argument('--block-W', type=float, default=0.20,
                        help='Block width [m]')
    parser.add_argument('--block-H', type=float, default=0.15,
                        help='Block height [m]')
    parser.add_argument('--face-offsets-u', type=str, default="0,0,0,0,0",
                        help='Per-face u-axis offsets [mm], comma-separated 5 values '
                             '(0=top,1=front,2=right,3=back,4=left). Default: all 0')
    parser.add_argument('--face-offsets-v', type=str, default="0,0,0,0,0",
                        help='Per-face v-axis offsets [mm], comma-separated 5 values. ')
    parser.add_argument('--center-face', type=int, default=-1,
                        help='Pattern center face for continuous mode (-1=per-face mode, 0~4=continuous)')
    parser.add_argument('--center-u', type=float, default=0.0,
                        help='Pattern center u-coordinate on center_face [mm]')
    parser.add_argument('--center-v', type=float, default=0.0,
                        help='Pattern center v-coordinate on center_face [mm]')

    args = parser.parse_args()

    runner = ExperimentRunner(args.output_dir)

    block_pose = (args.block_x, args.block_y, args.block_z,
                  args.block_roll, args.block_pitch, args.block_yaw)
    block_size = (args.block_L, args.block_W, args.block_H)

    # Parse face offsets
    face_offsets_u = [float(x) for x in args.face_offsets_u.split(",")]
    face_offsets_v = [float(x) for x in args.face_offsets_v.split(",")]

    svg_file = args.svg
    test_pattern = ""
    if not svg_file:
        if args.test_circle:
            test_pattern = "circle"
        elif args.test_square:
            svg_file = ""  # default square

    success = runner.run_experiment(
        svg_file, block_pose, block_size,
        args.width, args.height, args.faces,
        face_offsets_u, face_offsets_v,
        test_pattern,
        args.center_face, args.center_u, args.center_v)

    if success:
        rospy.loginfo("Experiment completed successfully!")
    else:
        rospy.logerr("Experiment failed!")
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
