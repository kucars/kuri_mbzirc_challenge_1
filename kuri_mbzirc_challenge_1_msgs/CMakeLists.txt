cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    actionlib_msgs
    std_msgs
    geometry_msgs
    genmsg
    actionlib
    message_generation
    kuri_msgs
    sensor_msgs

)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder

add_action_files(
   DIRECTORY action
   FILES
   exploration.action
   poseEstimation.action

 )
 
add_message_files(
   DIRECTORY msg
   FILES
   PoseEsti.msg
   pidData.msg
)


add_service_files(
  DIRECTORY srv
  FILES
  PES.srv
  navigation.srv
  Mission.srv


)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  kuri_msgs
  sensor_msgs
  # Or other packages containing msgs
)

find_package(kuri_msgs REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs std_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})


#############
## Install ##
#############


install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
endif()
