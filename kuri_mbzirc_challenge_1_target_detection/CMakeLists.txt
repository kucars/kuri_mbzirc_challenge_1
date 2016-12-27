cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1_target_detection)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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
#  INCLUDE_DIRS include
#  LIBRARIES kuri_mbzirc_challenge_1_target_detection
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

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

