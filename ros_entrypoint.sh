#!/bin/bash

#script to run with a host network
# export ROS_MASTER_URI=http://$(hostname --ip-address):11311
# export ROS_HOSTNAME=$(hostname --ip-address)
# setup ros environment

echo "Starting ROS Node to publish CPU Utilization..."
source /opt/ros/foxy/setup.bash
colcon build --packages-select cpp_pubsub
source install/setup.bash 
ros2 run cpp_pubsub talker