cmake_minimum_required(VERSION 3.0.2)
project(control_pid_stanley)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  av_messages
  carla_msgs
  )

set(PROJECT_DEPS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  av_messages
  carla_msgs
  )

catkin_package(
  CATKIN_DEPENDS std_msgs carla_msgs geometry_msgs nav_msgs sensor_msgs av_messages # Needed to generate custom messages
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)