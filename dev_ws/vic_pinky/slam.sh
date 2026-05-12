#!/bin/bash
# [1단계] SLAM — 지도 제작 스크립트
# 사용법: ./slam.sh
# 실행 후 RViz2 에서 지도가 만들어지는 걸 보면서 로봇을 직접 조종하면 됩니다.
# 지도가 완성되면 다른 터미널에서 ./save_map.sh 실행

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hong/dev_ws/vic_pinky/fastdds_vicpinky.xml

PARAMS_FILE="$(dirname "$(realpath "$0")")/slam_params.yaml"

echo "==================================="
echo " VicPinky SLAM 지도 제작 시작"
echo " 파라미터: $PARAMS_FILE"
echo "==================================="
echo ""
echo "로봇 조종: 다른 터미널에서 아래 명령 실행"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  export ROS_DOMAIN_ID=28"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "지도 저장: 또 다른 터미널에서"
echo "  ./save_map.sh"
echo ""

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PARAMS_FILE" \
  use_sim_time:=false
