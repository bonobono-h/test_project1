#!/bin/bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dev_ws/vic_pinky/fastdds_vicpinky.xml

pkill -9 -f "component_container" 2>/dev/null
pkill -9 -f "nav2_bringup" 2>/dev/null
sleep 2

cd ~/dev_ws/vic_pinky
bash nav2.sh
