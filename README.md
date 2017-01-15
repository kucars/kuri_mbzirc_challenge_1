# KURI MBZIRC Challenge 1
Challenge 1 related tasks implementation

[![Build Status](https://travis-ci.org/kuri-kustar/kuri_mbzirc_challenge_1.svg?branch=master)](https://travis-ci.org/kuri-kustar/kuri_mbzirc_challenge_1)

In order run the challenge, the following Packages are needed: 

- ROS   
- mavros     
- Firmware     
- mbzirc simulation environment   
- kuri msgs 
- Challenge 1 specific tasks    


Assuming that ROS and mavros are already installed, use the following commands to install the other packages. 
# Installing using rosinstall
```
cd <catkin_ws>
$ wstool init src
$ wstool set -t src kuri_mbzirc_challenge_1 https://github.com/kuri-kustar/kuri_mbzirc_challenge_1.git --git
$ wstool merge -t src https://raw.githubusercontent.com/kuri-kustar/kuri_mbzirc_challenge_1/master/kuri_mbzirc_challenge_1.rosinstall
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### Basic Usage 
  
```
$ cd catkin_was 
$ catkin_make --cmake-args -DCONFIG=posix_sitl_default
$ cd src/Firmware
$ source /Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
$ roslaunch kuri_mbzirc_challenge_1 task.launch
$ roslaunch kuri_mbzirc_challenge_1 truck.launch
```
