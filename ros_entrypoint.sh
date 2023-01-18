#!/bin/bash
#script to run with a host network
# export ROS_MASTER_URI=http://$(hostname --ip-address):11311
# export ROS_HOSTNAME=$(hostname --ip-address)
# setup ros environment
echo "Hello from container"
source install/setup.bash 
ros2 run cpp_pubsub talker