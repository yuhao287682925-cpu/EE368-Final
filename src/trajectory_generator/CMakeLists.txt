cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  block_drawing_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_eigen
  eigen_conversions
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS
    roscpp
    block_drawing_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_eigen
    eigen_conversions
    visualization_msgs
  DEPENDS
    Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(trajectory_generator
  src/svg_reader.cpp
  src/surface_projector.cpp
)

add_dependencies(trajectory_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(trajectory_generator
  ${catkin_LIBRARIES}
)

add_executable(trajectory_server
  src/trajectory_server.cpp
)

add_dependencies(trajectory_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(trajectory_server
  trajectory_generator
  ${catkin_LIBRARIES}
)

install(TARGETS trajectory_generator trajectory_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/pattern_designer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
