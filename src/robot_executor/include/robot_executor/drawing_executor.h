#ifndef ROBOT_EXECUTOR_DRAWING_EXECUTOR_H
#define ROBOT_EXECUTOR_DRAWING_EXECUTOR_H

#include <robot_executor/motion_primitives.h>
#include <robot_executor/pen_orientation.h>
#include <block_drawing_msgs/SurfaceTrajectory.h>
#include <block_drawing_msgs/DrawingExecutionAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace robot_executor {

/**
 * @brief Main drawing state machine.
 *
 * States:
 *   IDLE → MOVING_TO_START → DRAWING → LIFTING → SWITCHING_FACE → ... → HOMING → IDLE
 *
 * Supports both MoveIt Cartesian path execution and direct velocity control.
 */
class DrawingExecutor {
public:
    enum State {
        IDLE = 0,
        MOVING_TO_START,
        DRAWING,
        LIFTING,
        SWITCHING_FACE,
        HOMING,
        ERROR
    };

    struct DrawingParams {
        double speed;           // Drawing speed [m/s]
        double free_speed;      // Free-space motion speed [m/s]
        double lift_height;     // Pen lift height [m]
        double pen_tip_offset;  // Flange-to-pen-tip [m]
        double eef_step;        // Cartesian path discretization step [m]
    };

    DrawingExecutor(const std::string& robot_name = "my_gen3_lite");

    /**
     * @brief Execute a sequence of surface trajectories.
     * @param trajectory_sequence  Ordered list of per-face trajectories
     * @param params               Drawing parameters
     */
    void executeDrawingPlan(
        const std::vector<block_drawing_msgs::SurfaceTrajectory>& trajectory_sequence,
        const DrawingParams& params);

    void pause();
    void resume();
    void abort();

    State getState() const { return current_state_; }

private:
    // Action server callbacks
    void executeDrawingAction(const block_drawing_msgs::DrawingExecutionGoalConstPtr& goal);
    void preemptDrawingAction();

    // State machine steps
    void stateMachineStep(const ros::TimerEvent&);

    void enterMovingToStart();
    void enterDrawing();
    void enterLifting();
    void enterSwitchingFace();
    void enterHoming();

    // Subscriber callbacks
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    void forceCorrectionCallback(const geometry_msgs::Vector3ConstPtr& msg);

    // Utility
    Eigen::Isometry3d poseToIsometry(const geometry_msgs::Pose& pose) const;
    std::string stateToString(State s) const;

    // Communication
    ros::NodeHandle nh_;
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_force_correction_;

    // Action server
    actionlib::SimpleActionServer<block_drawing_msgs::DrawingExecutionAction> action_server_;

    // Motion interface
    std::unique_ptr<MotionPrimitives> motion_;

    // Orientation helper
    PenOrientation pen_orientation_;

    // State
    State current_state_;
    DrawingParams params_;
    std::vector<block_drawing_msgs::SurfaceTrajectory> trajectory_sequence_;
    size_t current_face_idx_;
    size_t current_waypoint_idx_;
    Eigen::Vector3d force_correction_;  // Latest Z-correction from force controller
    bool force_correction_received_;
    bool paused_;
    bool aborted_;

    // Current joint state for FK
    std::vector<double> current_joint_positions_;
    bool joint_state_received_;

    // Timer for state machine loop
    ros::Timer state_timer_;
};

}  // namespace robot_executor

#endif  // ROBOT_EXECUTOR_DRAWING_EXECUTOR_H
