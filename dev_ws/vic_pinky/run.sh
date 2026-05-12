#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hong/dev_ws/vic_pinky/fastdds_vicpinky.xml
cd /home/hong/dev_ws/vic_pinky
python3 follower_udp.py
