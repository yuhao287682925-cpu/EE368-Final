cmake_minimum_required(VERSION 3.0.2)
project(robot_executor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  block_drawing_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  kortex_driver
  tf2
  tf2_eigen
  eigen_conversions
)

find_package(catkin QUIET COMPONENTS moveit_ros_planning_interface)
if(moveit_ros_planning_interface_FOUND)
  # add_definitions(-DHAS_MOVEIT)
  message(STATUS "MoveIt found — enabling MoveIt-based motion planning")
else()
  message(WARNING "MoveIt NOT found — velocity control fallback only")
endif()

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_executor
  CATKIN_DEPENDS
    roscpp
    actionlib
    block_drawing_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    kortex_driver
    tf2
    tf2_eigen
    eigen_conversions
  DEPENDS
    Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
)

# Library
add_library(robot_executor
  src/pen_orientation.cpp
  src/motion_primitives.cpp
  src/drawing_executor.cpp
)

add_dependencies(robot_executor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  kortex_driver_generate_messages_cpp
)

target_link_libraries(robot_executor
  ${catkin_LIBRARIES}
)

# Executable
add_executable(executor_node
  src/executor_node.cpp
)

add_dependencies(executor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  kortex_driver_generate_messages_cpp
)

target_link_libraries(executor_node
  robot_executor
  ${catkin_LIBRARIES}
)

install(TARGETS robot_executor executor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
