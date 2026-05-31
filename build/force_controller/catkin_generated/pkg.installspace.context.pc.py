# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sensor_msgs;geometry_msgs;std_msgs;std_srvs;kortex_driver".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lforce_controller".split(';') if "-lforce_controller" != "" else []
PROJECT_NAME = "force_controller"
PROJECT_SPACE_DIR = "/home/huang/catkin_ws/install"
PROJECT_VERSION = "0.1.0"
