#!/bin/bash
# nav2.sh 실행 + RViz에서 2D Pose Estimate 설정 후 실행!

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hong/dev_ws/vic_pinky/fastdds_vicpinky.xml

echo "map TF 대기 중... (RViz에서 2D Pose Estimate 먼저 찍어야 해요!)"

# map → base_footprint TF 나올 때까지 대기
until ros2 run tf2_ros tf2_echo map base_footprint --timeout 2.0 2>/dev/null | grep -q "Translation"; do
  echo -n "."
  sleep 1
done
echo ""
echo "TF OK! 노드 활성화 시작..."

sleep 1

NODES=(planner_server route_server behavior_server velocity_smoother collision_monitor bt_navigator waypoint_follower docking_server)

for node in "${NODES[@]}"; do
  state=$(ros2 lifecycle get /$node 2>/dev/null)
  if [[ "$state" == *"inactive"* ]]; then
    echo -n "  $node 활성화... "
    ros2 lifecycle set /$node activate 2>/dev/null && echo "OK" || echo "실패"
  else
    echo "  $node: $state"
  fi
done

echo ""
echo "완료! 이제 RViz에서 2D Goal Pose 찍으세요!"
for node in bt_navigator controller_server planner_server velocity_smoother; do
  echo "  $node: $(ros2 lifecycle get /$node 2>/dev/null)"
done
