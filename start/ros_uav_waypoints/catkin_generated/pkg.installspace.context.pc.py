# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = (
    "${prefix}/include;/usr/include/eigen3".split(";")
    if "${prefix}/include;/usr/include/eigen3" != ""
    else []
)
PROJECT_CATKIN_DEPENDS = "roscpp;mrs_lib;mrs_msgs;geometry_msgs;nav_msgs".replace(
    ";", " "
)
PKG_CONFIG_LIBRARIES_WITH_PREFIX = (
    "-lWaypointFlier".split(";") if "-lWaypointFlier" != "" else []
)
PROJECT_NAME = "waypoint_flier"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
