#!/bin/bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dev_ws/vic_pinky/fastdds_vicpinky.xml

cd ~/dev_ws/vic_pinky
echo "Pillar filter 시작 — /scan_filtered → /scan_filtered_safe"
python3 pillar_filter_node.py
