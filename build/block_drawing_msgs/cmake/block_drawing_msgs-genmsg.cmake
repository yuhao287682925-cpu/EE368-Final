# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "block_drawing_msgs: 9 messages, 3 services")

set(MSG_I_FLAGS "-Iblock_drawing_msgs:/home/huang/catkin_ws/src/block_drawing_msgs/msg;-Iblock_drawing_msgs:/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(block_drawing_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" ""
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" "block_drawing_msgs/DrawingExecutionResult:block_drawing_msgs/DrawingExecutionActionGoal:block_drawing_msgs/DrawingExecutionActionFeedback:std_msgs/Header:geometry_msgs/Quaternion:block_drawing_msgs/DrawingExecutionFeedback:block_drawing_msgs/DrawingExecutionGoal:actionlib_msgs/GoalID:geometry_msgs/Point:block_drawing_msgs/DrawingExecutionActionResult:actionlib_msgs/GoalStatus:block_drawing_msgs/SurfaceTrajectory:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" "std_msgs/Header:geometry_msgs/Quaternion:block_drawing_msgs/DrawingExecutionGoal:actionlib_msgs/GoalID:geometry_msgs/Point:block_drawing_msgs/SurfaceTrajectory:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" "actionlib_msgs/GoalStatus:block_drawing_msgs/DrawingExecutionResult:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" "actionlib_msgs/GoalStatus:block_drawing_msgs/DrawingExecutionFeedback:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" "block_drawing_msgs/SurfaceTrajectory:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" ""
)

get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" ""
)

get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" "block_drawing_msgs/SurfaceTrajectory:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" NAME_WE)
add_custom_target(_block_drawing_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "block_drawing_msgs" "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)

### Generating Services
_generate_srv_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_cpp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
)

### Generating Module File
_generate_module_cpp(block_drawing_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(block_drawing_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(block_drawing_msgs_generate_messages block_drawing_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_cpp _block_drawing_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_drawing_msgs_gencpp)
add_dependencies(block_drawing_msgs_gencpp block_drawing_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_drawing_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_eus(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)

### Generating Services
_generate_srv_eus(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_eus(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_eus(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
)

### Generating Module File
_generate_module_eus(block_drawing_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(block_drawing_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(block_drawing_msgs_generate_messages block_drawing_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_eus _block_drawing_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_drawing_msgs_geneus)
add_dependencies(block_drawing_msgs_geneus block_drawing_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_drawing_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)

### Generating Services
_generate_srv_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_lisp(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
)

### Generating Module File
_generate_module_lisp(block_drawing_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(block_drawing_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(block_drawing_msgs_generate_messages block_drawing_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_lisp _block_drawing_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_drawing_msgs_genlisp)
add_dependencies(block_drawing_msgs_genlisp block_drawing_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_drawing_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)

### Generating Services
_generate_srv_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_nodejs(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
)

### Generating Module File
_generate_module_nodejs(block_drawing_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(block_drawing_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(block_drawing_msgs_generate_messages block_drawing_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_nodejs _block_drawing_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_drawing_msgs_gennodejs)
add_dependencies(block_drawing_msgs_gennodejs block_drawing_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_drawing_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_msg_py(block_drawing_msgs
  "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)

### Generating Services
_generate_srv_py(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_py(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)
_generate_srv_py(block_drawing_msgs
  "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
)

### Generating Module File
_generate_module_py(block_drawing_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(block_drawing_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(block_drawing_msgs_generate_messages block_drawing_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/SurfaceTrajectory.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/msg/ContactState.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionAction.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionActionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionGoal.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionResult.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/devel/share/block_drawing_msgs/msg/DrawingExecutionFeedback.msg" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/SetBlockPose.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/GenerateTrajectory.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huang/catkin_ws/src/block_drawing_msgs/srv/ExecuteDrawing.srv" NAME_WE)
add_dependencies(block_drawing_msgs_generate_messages_py _block_drawing_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(block_drawing_msgs_genpy)
add_dependencies(block_drawing_msgs_genpy block_drawing_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS block_drawing_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/block_drawing_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(block_drawing_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(block_drawing_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(block_drawing_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/block_drawing_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(block_drawing_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(block_drawing_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(block_drawing_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/block_drawing_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(block_drawing_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(block_drawing_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(block_drawing_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/block_drawing_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(block_drawing_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(block_drawing_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(block_drawing_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/block_drawing_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(block_drawing_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(block_drawing_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(block_drawing_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
