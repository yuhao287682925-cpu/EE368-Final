cmake_minimum_required(VERSION 3.0.2)
project(system_integrator)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
