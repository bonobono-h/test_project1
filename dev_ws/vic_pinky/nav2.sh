#!/bin/bash
# VicPinky Nav2 실행 스크립트
# 빅핑키에서 ~/bringup.sh 가 먼저 실행 중이어야 합니다!

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hong/dev_ws/vic_pinky/fastdds_vicpinky.xml

MAP_FILE="${1:-$HOME/my_map.yaml}"
PARAMS_FILE="$(dirname "$0")/nav2_params.yaml"

echo "맵 파일: $MAP_FILE"
echo "파라미터: $PARAMS_FILE"

# velocity_smoother 우회: cmd_vel_nav → cmd_vel 직접 연결
ros2 run topic_tools relay /cmd_vel_nav /cmd_vel &
RELAY_PID=$!

ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP_FILE" \
  params_file:="$PARAMS_FILE" \
  use_robot_state_publisher:=False \
  use_sim_time:=False

kill $RELAY_PID 2>/dev/null
