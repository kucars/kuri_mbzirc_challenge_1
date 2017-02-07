cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1_system_coordinator)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cmake_modules
  tf message_filters
  nav_msgs
  kuri_mbzirc_challenge_1_msgs
  kuri_msgs
  message_generation
  actionlib_msgs
  visp_bridge
#  cftld_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES kuri_mbzirc_challenge_1_trajectory_following
  CATKIN_DEPENDS roscpp rospy actionlib_msgs sensor_msgs nav_msgs kuri_mbzirc_challenge_1_msgs kuri_msgs cftld_ros
#  DEPENDS system_lib
)
find_package(kuri_mbzirc_challenge_1_msgs REQUIRED)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############



#############
## Testing ##
#############


