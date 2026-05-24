#include <robot_executor/drawing_executor.h>
#include <kortex_driver/TwistCommand.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

namespace robot_executor {

DrawingExecutor::DrawingExecutor(const std::string& robot_name)
    : nh_()
    , action_server_(nh_, "/execute_drawing", false)
    , current_state_(IDLE)
    , current_face_idx_(0)
    , current_waypoint_idx_(0)
    , force_correction_(Eigen::Vector3d::Zero())
    , force_correction_received_(false)
    , paused_(false)
    , aborted_(false)
    , joint_state_received_(false)
{
    // Parameters
    ros::NodeHandle pnh("~");
    pnh.param("speed", params_.speed, 0.03);
    pnh.param("free_speed", params_.free_speed, 0.15);
    pnh.param("lift_height", params_.lift_height, 0.05);
    pnh.param("pen_tip_offset", params_.pen_tip_offset, 0.12);
    pnh.param("eef_step", params_.eef_step, 0.001);

    // Subscribers
    sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>(
        "/" + robot_name + "/joint_states", 10,
        &DrawingExecutor::jointStateCallback, this);
    sub_force_correction_ = nh_.subscribe<geometry_msgs::Vector3>(
        "/force_correction", 10,
        &DrawingExecutor::forceCorrectionCallback, this);

    // Motion primitives
    motion_ = std::make_unique<MotionPrimitives>(robot_name);

    // Action server (registerGoalCallback expects void())
    action_server_.registerGoalCallback([this]() {
        auto goal = action_server_.acceptNewGoal();
        if (goal) {
            this->executeDrawingAction(goal);
        }
    });
    action_server_.registerPreemptCallback(
        [this]() { this->preemptDrawingAction(); });
    action_server_.start();

    // State machine timer (100 Hz)
    state_timer_ = nh_.createTimer(ros::Duration(0.01),
        &DrawingExecutor::stateMachineStep, this);

    ROS_INFO("Drawing executor ready. State: %s", stateToString(current_state_).c_str());
}

void DrawingExecutor::executeDrawingPlan(
    const std::vector<block_drawing_msgs::SurfaceTrajectory>& trajectory_sequence,
    const DrawingParams& params) {

    trajectory_sequence_ = trajectory_sequence;
    params_ = params;
    current_face_idx_ = 0;
    current_waypoint_idx_ = 0;
    paused_ = false;
    aborted_ = false;
    force_correction_ = Eigen::Vector3d::Zero();

    if (trajectory_sequence_.empty()) {
        ROS_WARN("Execute drawing: empty trajectory sequence.");
        return;
    }

    enterMovingToStart();
}

void DrawingExecutor::pause() {
    paused_ = true;
    motion_->stop();
    ROS_INFO("Drawing paused.");
}

void DrawingExecutor::resume() {
    paused_ = false;
    ROS_INFO("Drawing resumed.");
}

void DrawingExecutor::abort() {
    aborted_ = true;
    motion_->stop();
    current_state_ = HOMING;
    ROS_INFO("Drawing aborted, going home.");
}

// ---- Action server callbacks ----

void DrawingExecutor::executeDrawingAction(
    const block_drawing_msgs::DrawingExecutionGoalConstPtr& goal) {

    ROS_INFO("Received drawing execution goal: %zu trajectories",
             goal->trajectories.size());

    executeDrawingPlan(goal->trajectories, params_);

    // Run the state machine until completion or abort
    ros::Rate rate(100);
    while (ros::ok()) {
        if (current_state_ == IDLE || current_state_ == ERROR) {
            break;
        }
        if (aborted_) {
            break;
        }
        if (action_server_.isPreemptRequested()) {
            abort();
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    block_drawing_msgs::DrawingExecutionResult result;
    if (aborted_) {
        result.completed = false;
        result.message = "Aborted by user.";
        action_server_.setPreempted(result);
    } else {
        result.completed = (current_state_ == IDLE);
        result.faces_drawn = static_cast<int>(current_face_idx_);
        result.message = result.completed ? "Completed successfully." : "Ended with error.";
        action_server_.setSucceeded(result);
    }
}

void DrawingExecutor::preemptDrawingAction() {
    abort();
    ROS_INFO("Action preempted.");
}

// ---- State machine ----

void DrawingExecutor::stateMachineStep(const ros::TimerEvent&) {
    if (paused_ || aborted_) return;
    if (current_state_ == IDLE || current_state_ == ERROR) return;

    // The actual per-waypoint tracking is done inside the drawing action callback.
    // This timer-based step could be used for velocity-mode tracking.
}

void DrawingExecutor::enterMovingToStart() {
    current_state_ = MOVING_TO_START;
    ROS_INFO("State: MOVING_TO_START (face %zu)", current_face_idx_);

    if (current_face_idx_ >= trajectory_sequence_.size()) {
        enterHoming();
        return;
    }

    const auto& traj = trajectory_sequence_[current_face_idx_];
    if (traj.waypoints.empty()) {
        current_face_idx_++;
        enterMovingToStart();
        return;
    }

    // Move to the first waypoint at lifted height
    Eigen::Isometry3d first_pose = poseToIsometry(traj.waypoints[0]);

    // Compute surface normal from the first waypoint orientation
    Eigen::Quaterniond q(
        traj.waypoints[0].orientation.w,
        traj.waypoints[0].orientation.x,
        traj.waypoints[0].orientation.y,
        traj.waypoints[0].orientation.z);
    Eigen::Vector3d z_axis = q * Eigen::Vector3d::UnitZ();
    // Surface normal is -z_axis (pen Z points into surface)
    Eigen::Vector3d surface_normal = -z_axis;

    // Lift above start position
    Eigen::Isometry3d lifted_pose = first_pose;
    lifted_pose.translation() += surface_normal * params_.lift_height;

    bool moved = false;
    if (motion_->hasMoveIt()) {
        moved = motion_->moveFreeSpace(lifted_pose);
    }

    if (!moved) {
        // Velocity control fallback: move from current to target
        Eigen::Isometry3d current = motion_->getCurrentPose();
        if (joint_state_received_) {
            Eigen::Vector3d dir = lifted_pose.translation() - current.translation();
            double dist = dir.norm();
            if (dist > 0.001) {
                dir /= dist;
                double move_time = dist / params_.free_speed;
                motion_->publishCartesianVelocity(dir * params_.free_speed,
                                                  Eigen::Vector3d::Zero());
                ros::Duration(move_time).sleep();
                motion_->publishCartesianVelocity(Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero());
            }
        }
    }

    current_waypoint_idx_ = 0;
    enterDrawing();
}

void DrawingExecutor::enterDrawing() {
    current_state_ = DRAWING;
    ROS_INFO("State: DRAWING (face %zu)", current_face_idx_);

    const auto& traj = trajectory_sequence_[current_face_idx_];

    if (motion_->hasMoveIt()) {
        // Mode 1: MoveIt computeCartesianPath
        // Lower pen to first point first
        Eigen::Isometry3d first_pose = poseToIsometry(traj.waypoints[0]);

        // Compute the approach: lower from lifted to contact
        Eigen::Quaterniond q(
            traj.waypoints[0].orientation.w,
            traj.waypoints[0].orientation.x,
            traj.waypoints[0].orientation.y,
            traj.waypoints[0].orientation.z);
        Eigen::Vector3d z_axis = q * Eigen::Vector3d::UnitZ();
        Eigen::Vector3d surface_normal = -z_axis;

        // Approach pose: first waypoint lifted by a small amount, then lower
        Eigen::Isometry3d approach_pose = first_pose;
        approach_pose.translation() += surface_normal * 0.01;  // 1cm above

        motion_->moveFreeSpace(approach_pose);
        ros::Duration(0.1).sleep();

        // Lower pen to surface
        motion_->lowerPen(first_pose);
        ros::Duration(0.2).sleep();

        // Execute remaining waypoints via Cartesian path
        if (traj.waypoints.size() > 1) {
            std::vector<geometry_msgs::Pose> remaining;
            remaining.insert(remaining.end(),
                             traj.waypoints.begin() + 1, traj.waypoints.end());

            // For MoveIt, we use computeCartesianPath (requires actual MoveIt API)
            double fraction = motion_->executeCartesianPath(remaining,
                params_.eef_step, 0.0, params_.speed);
            ROS_INFO("Cartesian path: fraction=%.2f, %zu waypoints",
                     fraction, remaining.size());

            if (fraction < 0.9) {
                ROS_WARN("Cartesian path only %.0f%% reachable.", fraction * 100);
            }
        }
    } else {
        // Mode 2: Direct Cartesian velocity control
        // Iterate through waypoints with velocity control
        for (size_t i = 0; i < traj.waypoints.size(); ++i) {
            if (aborted_) break;

            geometry_msgs::Pose target_wp = traj.waypoints[i];

            // Apply force correction along surface normal
            target_wp.position.x += force_correction_.x();
            target_wp.position.y += force_correction_.y();
            target_wp.position.z += force_correction_.z();

            // Compute direction to this waypoint
            Eigen::Isometry3d current = motion_->getCurrentPose();
            Eigen::Vector3d direction(
                target_wp.position.x - current.translation().x(),
                target_wp.position.y - current.translation().y(),
                target_wp.position.z - current.translation().z());
            double distance = direction.norm();

            if (distance < 0.001) continue;  // Already there

            direction /= distance;  // Normalize

            double move_time = distance / params_.speed;
            Eigen::Vector3d velocity = direction * params_.speed;

            motion_->publishCartesianVelocity(velocity, Eigen::Vector3d::Zero());
            ros::Duration(move_time).sleep();

            // Publish feedback
            block_drawing_msgs::DrawingExecutionFeedback feedback;
            feedback.current_face = traj.face_id;
            feedback.current_waypoint = static_cast<int>(i);
            feedback.progress_fraction = static_cast<double>(i) / traj.waypoints.size();
            feedback.state = stateToString(current_state_);
            action_server_.publishFeedback(feedback);

            ros::spinOnce();
        }
    }

    // Drawing complete for this face
    enterLifting();
}

void DrawingExecutor::enterLifting() {
    current_state_ = LIFTING;
    ROS_INFO("State: LIFTING (face %zu)", current_face_idx_);

    const auto& traj = trajectory_sequence_[current_face_idx_];
    if (!traj.waypoints.empty()) {
        Eigen::Quaterniond q(
            traj.waypoints.back().orientation.w,
            traj.waypoints.back().orientation.x,
            traj.waypoints.back().orientation.y,
            traj.waypoints.back().orientation.z);
        Eigen::Vector3d surface_normal = -(q * Eigen::Vector3d::UnitZ());

        motion_->liftPen(surface_normal, params_.lift_height);
    }

    current_face_idx_++;

    // Check if there are more faces
    if (current_face_idx_ < trajectory_sequence_.size()) {
        enterSwitchingFace();
    } else {
        enterHoming();
    }
}

void DrawingExecutor::enterSwitchingFace() {
    current_state_ = SWITCHING_FACE;
    ROS_INFO("State: SWITCHING_FACE (%zu → %zu)",
             current_face_idx_ - 1, current_face_idx_);

    // Move to the first waypoint of next face (already lifted)
    enterMovingToStart();
}

void DrawingExecutor::enterHoming() {
    current_state_ = HOMING;
    ROS_INFO("State: HOMING");

    motion_->goHome();
    current_state_ = IDLE;
    current_face_idx_ = 0;
    current_waypoint_idx_ = 0;

    ROS_INFO("Drawing execution complete. State: IDLE");
}

// ---- Callbacks ----

void DrawingExecutor::jointStateCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
    current_joint_positions_ = msg->position;
    joint_state_received_ = true;
}

void DrawingExecutor::forceCorrectionCallback(
    const geometry_msgs::Vector3ConstPtr& msg) {
    force_correction_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
    force_correction_received_ = true;
}

// ---- Utility ----

Eigen::Isometry3d DrawingExecutor::poseToIsometry(
    const geometry_msgs::Pose& pose) const {

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    T.linear() = Eigen::Quaterniond(
        pose.orientation.w, pose.orientation.x,
        pose.orientation.y, pose.orientation.z).toRotationMatrix();
    return T;
}

std::string DrawingExecutor::stateToString(State s) const {
    switch (s) {
        case IDLE:            return "IDLE";
        case MOVING_TO_START: return "MOVING_TO_START";
        case DRAWING:         return "DRAWING";
        case LIFTING:         return "LIFTING";
        case SWITCHING_FACE:  return "SWITCHING_FACE";
        case HOMING:          return "HOMING";
        case ERROR:           return "ERROR";
        default:              return "UNKNOWN";
    }
}

}  // namespace robot_executor
