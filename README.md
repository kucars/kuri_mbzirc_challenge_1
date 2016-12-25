# kuri_mbzirc_challenge_1
Challenge 1 related tasks implementation

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
$ wstool merge -t src https://raw.githubusercontent.com/kuri-kustar/kuri_mbzirc_challenge_1/master/mbzirc_challenge1.rosinstall
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ catkin_make --cmake-args -DCONFIG=ros_sitl_simple
```

### Basic Usage 
  
```
$ cd catkin_was 
$ catkin_make --cmake-args -DCONFIG=posix_sitl_default
$ roslaunch kuri_mbzirc_challenge_1 task.launch
```
