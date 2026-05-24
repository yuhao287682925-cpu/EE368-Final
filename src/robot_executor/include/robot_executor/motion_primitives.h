#ifndef ROBOT_EXECUTOR_MOTION_PRIMITIVES_H
#define ROBOT_EXECUTOR_MOTION_PRIMITIVES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <string>

// Forward declare MoveIt types if available
#ifdef HAS_MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#endif

namespace robot_executor {

/**
 * @brief Basic motion primitives for the drawing robot.
 *
 * Provides go-home, pen lift/lower, free-space movement, and stop/clear-faults.
 * Supports both MoveIt-based motion and direct Cartesian velocity control.
 */
class MotionPrimitives {
public:
    /**
     * @param robot_name  Robot name as configured in kortex_driver ("my_gen3_lite")
     */
    MotionPrimitives(const std::string& robot_name = "my_gen3_lite");

    /**
     * @brief Move the end-effector to the predefined home joint position.
     * @return true on success
     */
    bool goHome();

    /**
     * @brief Lift the pen along the surface normal.
     * @param surface_normal  Unit normal of the current face
     * @param height          Lift distance in meters
     * @return true on success
     */
    bool liftPen(const Eigen::Vector3d& surface_normal, double height);

    /**
     * @brief Lower the pen to a target pose (contact point).
     * @param target_pose  Desired end-effector pose
     * @return true on success
     */
    bool lowerPen(const Eigen::Isometry3d& target_pose);

    /**
     * @brief Move the TCP through free space to a target pose.
     *        Uses MoveIt for collision-free planning.
     * @param target  Target end-effector pose
     * @return true on success
     */
    bool moveFreeSpace(const Eigen::Isometry3d& target);

    /**
     * @brief Publish a smooth stop command.
     */
    void stop();

    /**
     * @brief Clear robot faults.
     */
    void clearFaults();

    /**
     * @brief Set MoveIt planning parameters.
     */
    void setPlanningTime(double seconds);
    void setPlannerId(const std::string& planner_id);

    /**
     * @brief Check if MoveIt interface is available.
     */
    bool hasMoveIt() const { return false; }

    /**
     * @brief Get current end-effector pose from forward kinematics.
     */
    Eigen::Isometry3d getCurrentPose() const;

    /**
     * @brief Execute a Cartesian path via MoveIt computeCartesianPath + execute.
     * @param waypoints    List of end-effector poses
     * @param eef_step     Resolution of Cartesian interpolation [m]
     * @param jump_thresh  Jump threshold for path continuity [m]
     * @param speed        Scaling factor for trajectory velocity
     * @return Fraction of path achieved [0.0, 1.0]
     */
    double executeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints,
                                double eef_step, double jump_thresh, double speed);

    /**
     * @brief Publish Cartesian velocity command directly.
     * @param linear   Linear velocity in base frame [m/s]
     * @param angular  Angular velocity in base frame [rad/s]
     */
    void publishCartesianVelocity(const Eigen::Vector3d& linear,
                                   const Eigen::Vector3d& angular);

private:
    ros::NodeHandle nh_;
    ros::Publisher  pub_cart_vel_;
    ros::Publisher  pub_stop_;
    ros::Publisher  pub_clear_faults_;
    ros::Subscriber sub_joint_state_;

    std::string robot_name_;

    // MoveIt interface (optional, via #ifdef)
    bool moveit_available_;
#ifdef HAS_MOVEIT
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
#endif

    // Current joint positions
    std::vector<double> current_joint_positions_;
    bool joint_state_received_;

    // Parameters
    std::vector<double> home_joints_;
    double free_speed_;
    double planning_time_;
    double min_x_, max_x_;  // workspace X limits [m]
    double min_y_, max_y_;  // workspace Y limits [m]
    double min_z_;          // workspace Z floor [m]
    std::string planner_id_;

    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
};

}  // namespace robot_executor

#endif  // ROBOT_EXECUTOR_MOTION_PRIMITIVES_H
