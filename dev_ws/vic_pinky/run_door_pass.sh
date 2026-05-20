#!/bin/bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dev_ws/vic_pinky/fastdds_vicpinky.xml

cd ~/dev_ws/vic_pinky
source /opt/ros/jazzy/setup.bash
echo "DoorPass filter 시작 — cmd_vel_collision_out → cmd_vel"
python3 door_pass_node.py
