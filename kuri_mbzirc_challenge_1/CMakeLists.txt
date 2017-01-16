cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_1)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    genmsg
    message_generation
    kuri_msgs
    cv_bridge
    sensor_msgs
    image_transport
    cmake_modules
    tf message_filters
    nav_msgs
    kuri_mbzirc_challenge_1_msgs
    )

find_package(kuri_msgs   REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kuri_mbzirc_challenge_1_msgs REQUIRED)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
#############
## Install ##
#############
install(DIRECTORY launch urdf config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

add_executable(off_board src/off_board.cpp)
target_link_libraries(off_board ${catkin_LIBRARIES})

#add_executable(offb_follow_target src/offb_follow_target.cpp)
#target_link_libraries(offb_follow_target ${catkin_LIBRARIES})


add_executable(server src/server_1.cpp)
target_link_libraries(server ${catkin_LIBRARIES})
add_dependencies(server ${catkin_LIBRARIES})

add_executable(client src/client_1.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
#  roslaunch_add_file_check(launch/task.launch)
endif()
