cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1_marker_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visp_bridge
  cv_bridge
  message_generation
)
unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)

set(CUSTOM_OPENCV_DIR "/usr/local")

# Few differences when dealing with ros-indigo and ros-kinetic
#set( ROS_DISTRO $ENV{ROS_DISTRO} )
#if( ${ROS_DISTRO} STREQUAL "indigo" )
#    find_package(OpenCV 2.4.8 REQUIRED PATHS ${CUSTOM_OPENCV_DIR} NO_DEFAULT_PATH)
#elseif(${ROS_DISTRO} STREQUAL "kinetic")
#    find_package(OpenCV 2 REQUIRED)
#else()
#    message("Unsupported ROS Distro")
#endif()

find_package(OpenCV 2.4.8 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

#add_message_files(
  #DIRECTORY msg
  #FILES TrackerData.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuri_mbzirc_challenge_1_marker_tracking
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
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

add_executable(visp-tracker src/visptracker.cpp src/mark_tracker.cpp src/detector/landing_mark_detection.cpp )
add_dependencies(visp-tracker kuri_mbzirc_challenge_1_marker_tracking_generate_messages_cpp)
target_link_libraries(visp-tracker ${catkin_LIBRARIES})
target_link_libraries(visp-tracker ${OpenCV_LIBRARIES})
  

#############
## Testing ##
#############


