#!/bin/bash
# Set the ROS_MASTER of the robot
#export ROS_MASTER_URI=http://192.168.1.100:11311/

# Set the IP address of the machine running the referee box
#export ROS_IP=192.168.1.101

source /opt/ros/noetic/setup.sh
source /home/root/catkin_ws/devel/setup.bash
roslaunch metrics_refbox metrics_refbox.launch
