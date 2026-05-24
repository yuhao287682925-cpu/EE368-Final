#include <robot_executor/drawing_executor.h>
#include <ros/ros.h>

/**
 * @brief Main node for the robot executor module (Module B).
 *
 * Initializes the DrawingExecutor which handles the full state machine
 * and exposes the /execute_drawing action server.
 *
 * Launch:
 *   rosrun robot_executor executor_node _robot_name:=my_gen3_lite
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "executor_node");

    std::string robot_name;
    ros::param::param<std::string>("~robot_name", robot_name, "my_gen3_lite");

    ros::NodeHandle nh;

    robot_executor::DrawingExecutor executor(robot_name);

    ROS_INFO("Executor node started. Robot: %s", robot_name.c_str());
    ROS_INFO("Waiting for drawing execution goals on /execute_drawing...");

    ros::spin();

    return 0;
}
