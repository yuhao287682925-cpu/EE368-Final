# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;actionlib;block_drawing_msgs;geometry_msgs;sensor_msgs;std_msgs;kortex_driver;tf2;tf2_eigen;eigen_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrobot_executor".split(';') if "-lrobot_executor" != "" else []
PROJECT_NAME = "robot_executor"
PROJECT_SPACE_DIR = "/home/huang/catkin_ws/install"
PROJECT_VERSION = "0.1.0"
