# kuri_mbzirc_challenge_1
Challenge 1 related tasks implementation

In order run the challenge, the following Packages are needed: 

- ROS
- RotorS rotor_simulator    
- mavros     
- Firmware     
- mbzirc simulation environment    
- Challenge 1 specific tasks    

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
