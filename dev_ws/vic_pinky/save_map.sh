#!/bin/bash
# SLAM 중에 실행해서 현재 지도를 저장합니다.
# 사용법: ./save_map.sh [지도이름]  (기본값: my_map)

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hong/dev_ws/vic_pinky/fastdds_vicpinky.xml

MAP_NAME="${1:-my_map}"
SAVE_DIR="$HOME"

echo "지도 저장 중: $SAVE_DIR/$MAP_NAME"
ros2 run nav2_map_server map_saver_cli -f "$SAVE_DIR/$MAP_NAME" --ros-args -p save_map_timeout:=5.0

echo ""
echo "저장 완료!"
echo "  $SAVE_DIR/${MAP_NAME}.yaml"
echo "  $SAVE_DIR/${MAP_NAME}.pgm"
echo ""
echo "Nav2 실행하려면:"
echo "  ./nav2.sh $SAVE_DIR/${MAP_NAME}.yaml"
