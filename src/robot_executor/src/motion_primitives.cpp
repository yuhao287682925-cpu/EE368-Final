#include <robot_executor/motion_primitives.h>
#include <kortex_driver/TwistCommand.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <cmath>

namespace robot_executor {

MotionPrimitives::MotionPrimitives(const std::string& robot_name)
    : nh_()
    , robot_name_(robot_name)
    , moveit_available_(false)
    , joint_state_received_(false)
    , free_speed_(0.15), min_x_(-0.5), max_x_(0.8), min_y_(-0.5), max_y_(0.5), min_z_(0.2)
    , planning_time_(2.0)
    , planner_id_("RRTConnectkConfigDefault")
{
    // Advertise direct control publishers
    pub_cart_vel_ = nh_.advertise<kortex_driver::TwistCommand>(
        "/" + robot_name_ + "/in/cartesian_velocity", 10);
    pub_stop_ = nh_.advertise<std_msgs::Empty>(
        "/" + robot_name_ + "/in/stop", 10);
    pub_clear_faults_ = nh_.advertise<std_msgs::Empty>(
        "/" + robot_name_ + "/in/clear_faults", 10);

    // Subscribe to joint state (ros_kortex publishes joint_states from base_feedback)
    sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>(
        "/" + robot_name_ + "/joint_states", 10,
        &MotionPrimitives::jointStateCallback, this);

    // Home joint position (Gen3 Lite 6-DOF)
    // Example: arm pointing forward with elbow tucked in
    home_joints_ = {0.0, 0.26, 3.14, -1.96, 0.0, 1.57};

    // Read workspace limits from param server (overridable in drawing_params.yaml)
    ros::NodeHandle pnh("~");
    pnh.param("min_x", min_x_, -0.5);
    pnh.param("max_x", max_x_,  0.8);
    pnh.param("min_y", min_y_, -0.5);
    pnh.param("max_y", max_y_,  0.5);
    pnh.param("min_z", min_z_,  0.2);
    ROS_INFO("Workspace limits: X[%.2f,%.2f] Y[%.2f,%.2f] Z>=%.2f",
             min_x_, max_x_, min_y_, max_y_, min_z_);

#ifdef HAS_MOVEIT
    try {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            "arm");
        move_group_->setPlanningTime(planning_time_);
        move_group_->setPlannerId(planner_id_);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        moveit_available_ = true;
        ROS_INFO("MoveIt interface initialized.");
    } catch (const std::exception& e) {
        ROS_WARN("MoveIt not available: %s. Using direct velocity control.", e.what());
    }
#endif
}

bool MotionPrimitives::goHome() {
#ifdef HAS_MOVEIT
    if (moveit_available_) {
        move_group_->setJointValueTarget(home_joints_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(plan);
            ROS_INFO("Moved to home position.");
            return true;
        } else {
            ROS_WARN("Failed to plan to home position.");
            return false;
        }
    }
#endif
    ROS_WARN("goHome: MoveIt not available.");
    return false;
}

bool MotionPrimitives::liftPen(const Eigen::Vector3d& surface_normal, double height) {
    if (!joint_state_received_) {
        ROS_WARN("liftPen: No joint state received.");
        return false;
    }

#ifdef HAS_MOVEIT
    if (moveit_available_) {
        Eigen::Isometry3d current = getCurrentPose();
        Eigen::Vector3d target_pos = current.translation() + surface_normal * height;
        Eigen::Isometry3d target = current;
        target.translation() = target_pos;

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose = tf2::toMsg(target);

        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(plan);
            ROS_INFO("Pen lifted by %.3f m.", height);
            return true;
        }
    }
#endif

    // Fallback: use velocity control to lift
    Eigen::Vector3d lift_velocity = surface_normal * free_speed_;
    publishCartesianVelocity(lift_velocity, Eigen::Vector3d::Zero());
    ros::Duration(height / free_speed_).sleep();
    publishCartesianVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return true;
}

bool MotionPrimitives::lowerPen(const Eigen::Isometry3d& target_pose) {
#ifdef HAS_MOVEIT
    if (moveit_available_) {
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose = tf2::toMsg(target_pose);

        move_group_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(plan);
            ROS_INFO("Pen lowered to target.");
            return true;
        }
    }
#endif
    ROS_WARN("lowerPen: MoveIt not available.");
    return false;
}

bool MotionPrimitives::moveFreeSpace(const Eigen::Isometry3d& target) {
#ifdef HAS_MOVEIT
    if (moveit_available_) {
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose = tf2::toMsg(target);

        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(plan);
            return true;
        }
    }
#endif
    ROS_WARN("moveFreeSpace: MoveIt not available.");
    return false;
}

void MotionPrimitives::stop() {
    std_msgs::Empty msg;
    pub_stop_.publish(msg);
    ROS_INFO("Stop command sent.");
}

void MotionPrimitives::clearFaults() {
    std_msgs::Empty msg;
    pub_clear_faults_.publish(msg);
    ROS_INFO("Clear faults command sent.");
}

void MotionPrimitives::setPlanningTime(double seconds) {
    planning_time_ = seconds;
#ifdef HAS_MOVEIT
    if (moveit_available_) {
        move_group_->setPlanningTime(seconds);
    }
#endif
}

void MotionPrimitives::setPlannerId(const std::string& planner_id) {
    planner_id_ = planner_id;
#ifdef HAS_MOVEIT
    if (moveit_available_) {
        move_group_->setPlannerId(planner_id);
    }
#endif
}

Eigen::Isometry3d MotionPrimitives::getCurrentPose() const {
    if (!joint_state_received_ || current_joint_positions_.size() < 6) {
        return Eigen::Isometry3d::Identity();
    }

    // Gen3 Lite Standard DH parameters [alpha, a, d, theta_offset]
    // Source: jacobian.py (verified with real robot)
    struct DH { double alpha, a, d, offset; };
    static const DH dh[6] = {
        {0.0,         0.0,   0.2433,  0.0},
        {M_PI/2.0,    0.0,   0.010,   M_PI/2.0},
        {M_PI,        0.280, 0.0,     M_PI/2.0},
        {M_PI/2.0,    0.0,   0.245,   M_PI/2.0},
        {M_PI/2.0,    0.0,   0.057,   0.0},
        {-M_PI/2.0,   0.0,   0.235,  -M_PI/2.0},
    };

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) {
        double th = current_joint_positions_[i] + dh[i].offset;
        double ct = std::cos(th), st = std::sin(th);
        double ca = std::cos(dh[i].alpha), sa = std::sin(dh[i].alpha);

        Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
        Ti(0,0) = ct;  Ti(0,1) = -st*ca;  Ti(0,2) =  st*sa;  Ti(0,3) = dh[i].a*ct;
        Ti(1,0) = st;  Ti(1,1) =  ct*ca;  Ti(1,2) = -ct*sa;  Ti(1,3) = dh[i].a*st;
        Ti(2,0) = 0;   Ti(2,1) =  sa;     Ti(2,2) =  ca;     Ti(2,3) = dh[i].d;

        T = T * Ti;
    }

    Eigen::Isometry3d pose;
    pose.matrix() = T;
    return pose;
}

double MotionPrimitives::executeCartesianPath(
    const std::vector<geometry_msgs::Pose>& waypoints,
    double eef_step, double jump_thresh, double speed) {

    if (waypoints.empty()) return 0.0;

#ifdef HAS_MOVEIT
    if (moveit_available_) {
        move_group_->setMaxVelocityScalingFactor(speed / 0.15);  // scale relative to max
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_thresh, trajectory);
        if (fraction > 0.0) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group_->execute(plan);
        }
        return fraction;
    }
#endif
    // Velocity control fallback
    for (size_t i = 0; i < waypoints.size(); ++i) {
        Eigen::Vector3d target(waypoints[i].position.x,
                               waypoints[i].position.y,
                               waypoints[i].position.z);
        Eigen::Vector3d current = getCurrentPose().translation();
        Eigen::Vector3d dir = target - current;
        double dist = dir.norm();
        if (dist < 0.001) continue;
        dir /= dist;
        publishCartesianVelocity(dir * speed, Eigen::Vector3d::Zero());
        ros::Duration(dist / speed).sleep();
    }
    publishCartesianVelocity(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return 1.0;
}

void MotionPrimitives::publishCartesianVelocity(
    const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {

    Eigen::Vector3d vel = linear;

    // Workspace safety clamps on all axes
    if (joint_state_received_) {
        Eigen::Vector3d pos = getCurrentPose().translation();
        double dt = 0.1;  // predict 100ms ahead

        auto clampAxis = [&](double& v, double p, double v_min, double v_max) {
            double next = p + v * dt;
            if (v < 0 && next < v_min)  v = std::max(v, (v_min - p) / dt);
            if (v > 0 && next > v_max)  v = std::min(v, (v_max - p) / dt);
        };
        clampAxis(vel.x(), pos.x(), min_x_, max_x_);
        clampAxis(vel.y(), pos.y(), min_y_, max_y_);
        clampAxis(vel.z(), pos.z(), min_z_, 999.0);  // no upper Z limit
    }

    kortex_driver::TwistCommand cmd;
    cmd.reference_frame = 0;
    cmd.twist.linear_x = vel.x();
    cmd.twist.linear_y = vel.y();
    cmd.twist.linear_z = vel.z();
    cmd.twist.angular_x = angular.x();
    cmd.twist.angular_y = angular.y();
    cmd.twist.angular_z = angular.z();
    cmd.duration = 0;
    pub_cart_vel_.publish(cmd);
    cmd.twist.angular_x = angular.x();
    cmd.twist.angular_y = angular.y();
    cmd.twist.angular_z = angular.z();
    cmd.duration = 0;
    pub_cart_vel_.publish(cmd);
}

void MotionPrimitives::jointStateCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
    current_joint_positions_ = msg->position;
    joint_state_received_ = true;
}

}  // namespace robot_executor
