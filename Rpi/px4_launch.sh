#!/bin/bash
exec 1>/home/pi/hawkeye/Rpi/px4_log.txt 2>&1
set -x
source /home/pi/ros_catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="serial:///dev/serial0:921600"

