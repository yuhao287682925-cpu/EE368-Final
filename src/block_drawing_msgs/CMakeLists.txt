cmake_minimum_required(VERSION 3.0.2)
project(block_drawing_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  SurfaceTrajectory.msg
  ContactState.msg
)

add_service_files(
  FILES
  SetBlockPose.srv
  GenerateTrajectory.srv
  ExecuteDrawing.srv
)

add_action_files(
  FILES
  DrawingExecution.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  std_msgs
)
