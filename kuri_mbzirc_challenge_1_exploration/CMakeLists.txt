cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1_exploration)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  kuri_mbzirc_challenge_1_msgs

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
#  LIBRARIES kuri_mbzirc_challenge_1_exploration
  CATKIN_DEPENDS roscpp rospy  actionlib_msgs

#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############



#############
## Testing ##
#############

add_executable(takeoffA src/offboardService.cpp)
target_link_libraries(takeoffA ${catkin_LIBRARIES})
add_dependencies(takeoffA ${catkin_EXPORTED_TARGETS})

#add_executable(takeoffA src/autotakeoff.cpp)
#target_link_libraries(takeoffA ${catkin_LIBRARIES})
#add_dependencies(takeoffA ${catkin_EXPORTED_TARGETS})
