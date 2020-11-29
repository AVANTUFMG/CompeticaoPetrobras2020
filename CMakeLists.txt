cmake_minimum_required(VERSION 3.2.0)
project(ufmg_desafio_petrobras)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
)

##Generate services in 'srv' folder
#add_service_files()
