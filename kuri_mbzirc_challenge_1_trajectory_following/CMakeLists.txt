cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1_trajectory_following)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  image_transport
  cmake_modules
  tf message_filters
  nav_msgs
  kuri_mbzirc_challenge_1_msgs
  message_generation
  actionlib_msgs
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
  CATKIN_DEPENDS roscpp rospy actionlib_msgs sensor_msgs nav_msgs
#  DEPENDS system_lib
)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kuri_mbzirc_challenge_1_msgs REQUIRED)


###########
## Build ##
###########


include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})



#############
## Install ##
#############

add_executable(attitude_control src/landingOnstaticMarker.cpp)
target_link_libraries(attitude_control ${catkin_LIBRARIES})



add_executable(poseEstimatation src/PositionEstimationServer.cpp src/PositionEstimation.cpp  include/PositionEstimation.h)
target_link_libraries(poseEstimatation ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(poseEstimatation ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

#############
## Testing ##
#############


