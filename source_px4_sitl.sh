#!/bin/bash
root=/home/alvin/drone_ws/src/PX4-Autopilot
# root=/home/alvin/PX4/Firmware
source $root/Tools/setup_gazebo.bash 
source $root 
source $root/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$root/Tools/sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$root
echo $ROS_PACKAGE_PATH
