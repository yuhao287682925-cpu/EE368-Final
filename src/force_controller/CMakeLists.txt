cmake_minimum_required(VERSION 3.0.2)
project(force_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  kortex_driver
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES force_controller
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    kortex_driver
  DEPENDS
    Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
)

add_library(force_controller
  src/current_estimator.cpp
  src/impedance_controller.cpp
)

add_dependencies(force_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  kortex_driver_generate_messages_cpp
)

target_link_libraries(force_controller
  ${catkin_LIBRARIES}
)

add_executable(controller_node
  src/controller_node.cpp
)

add_dependencies(controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(controller_node
  force_controller
  ${catkin_LIBRARIES}
)

install(TARGETS force_controller controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
