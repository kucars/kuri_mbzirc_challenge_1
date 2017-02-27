#!/usr/bin/env bash
# Install ROS Indigo as per the wiki instructions : http://wiki.ros.org/indigo/Installation/Ubuntu
# 
# Setup Locale
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
# Installation
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
# Initialize rosdep
sudo apt-get install -qq -y python-rosdep python-wstool python-catkin-tools python-catkin-pkg xvfb ros-indigo-xacro
sudo apt-get install python-rosdep -y
sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev clang-3.5 lldb-3.5 -y
# Initialize rosdep
sudo rosdep init
# Find available packages
rosdep update

# Environment Setup
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install rosinstall
sudo apt-get install python-rosinstall -y

# Update udev rules
sudo echo "SUBSYSTEMS==\"usb\", KERNEL==\"ttyUSB[0-9]*\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", SYMLINK+=\"piksi\", MODE=\"0666\", GROUP=\"dialout\"" > /etc/udev/rules.d/70-mbzirc_drone.rules
sudo echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", SYMLINK+=\"px4\", MODE=\"666\""  >> /etc/udev/rules.d/70-mbzirc_drone.rules 
echo "System Configurations Complete"

# Create workspace
mkdir -p ~/kuri_catkin_ws/src
cd ~/kuri_catkin_ws/
wstool init src
echo "source ~/workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Workspace Configurations Complete"

