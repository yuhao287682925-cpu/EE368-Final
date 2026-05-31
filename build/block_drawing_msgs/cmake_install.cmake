# Install script for directory: /home/huang/catkin_ws/src/block_drawing_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/huang/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/msg" TYPE FILE FILES
    "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg"
    "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/srv" TYPE FILE FILES
    "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv"
    "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv"
    "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/action" TYPE FILE FILES "/home/huang/catkin_ws/src/block_drawing_msgs/action/DrawingExecution.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/msg" TYPE FILE FILES
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg"
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg"
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg"
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg"
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg"
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg"
    "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/cmake" TYPE FILE FILES "/home/huang/catkin_ws/build/block_drawing_msgs/catkin_generated/installspace/block_drawing_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/huang/catkin_ws/devel/include/block_drawing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/huang/catkin_ws/devel/share/roseus/ros/block_drawing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/huang/catkin_ws/devel/share/common-lisp/ros/block_drawing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/huang/catkin_ws/devel/share/gennodejs/ros/block_drawing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/huang/catkin_ws/devel/lib/python3/dist-packages/block_drawing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/huang/catkin_ws/devel/lib/python3/dist-packages/block_drawing_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/huang/catkin_ws/build/block_drawing_msgs/catkin_generated/installspace/block_drawing_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/cmake" TYPE FILE FILES "/home/huang/catkin_ws/build/block_drawing_msgs/catkin_generated/installspace/block_drawing_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs/cmake" TYPE FILE FILES
    "/home/huang/catkin_ws/build/block_drawing_msgs/catkin_generated/installspace/block_drawing_msgsConfig.cmake"
    "/home/huang/catkin_ws/build/block_drawing_msgs/catkin_generated/installspace/block_drawing_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/block_drawing_msgs" TYPE FILE FILES "/home/huang/catkin_ws/src/block_drawing_msgs/package.xml")
endif()

