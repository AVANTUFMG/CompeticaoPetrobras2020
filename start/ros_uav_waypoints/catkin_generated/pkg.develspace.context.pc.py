# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = (
    "/home/luiz/workspace/src/Petrobras_Challenge/start/ros_uav_waypoints/devel/include;/home/luiz/workspace/src/Petrobras_Challenge/start/ros_uav_waypoints/include;/usr/include/eigen3".split(
        ";"
    )
    if "/home/luiz/workspace/src/Petrobras_Challenge/start/ros_uav_waypoints/devel/include;/home/luiz/workspace/src/Petrobras_Challenge/start/ros_uav_waypoints/include;/usr/include/eigen3"
    != ""
    else []
)
PROJECT_CATKIN_DEPENDS = "roscpp;mrs_lib;mrs_msgs;geometry_msgs;nav_msgs".replace(
    ";", " "
)
PKG_CONFIG_LIBRARIES_WITH_PREFIX = (
    "-lWaypointFlier".split(";") if "-lWaypointFlier" != "" else []
)
PROJECT_NAME = "waypoint_flier"
PROJECT_SPACE_DIR = (
    "/home/luiz/workspace/src/Petrobras_Challenge/start/ros_uav_waypoints/devel"
)
PROJECT_VERSION = "0.0.1"
