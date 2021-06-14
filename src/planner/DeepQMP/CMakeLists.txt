cmake_minimum_required(VERSION 3.0.2)
project(deepQMP)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS
  rospy
  roscpp
  towr_ros
  tf2_msgs
  std_msgs
  geometry_msgs
  xpp_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
# include
${catkin_INCLUDE_DIRS}
)
