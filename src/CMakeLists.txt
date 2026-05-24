# Top-level CMakeLists.txt for catkin workspace
# This file is symlinked by catkin_make to the build directory
cmake_minimum_required(VERSION 3.0.2)
project(catkin_workspace)

# This tells catkin to find packages in the src directory
find_package(catkin REQUIRED)
catkin_workspace()
