#!/bin/bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dev_ws/vic_pinky/fastdds_vicpinky.xml

rviz2 -d ~/nav2.rviz
