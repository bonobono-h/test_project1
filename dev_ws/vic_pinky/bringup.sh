#!/bin/bash
# 좀비 프로세스 정리
echo "[1/2] 기존 프로세스 정리 중..."
pkill -f sllidar_node 2>/dev/null
pkill -f vicpinky_bringup 2>/dev/null
pkill -f scan_to_scan_filter 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
sleep 1

# 포트 해제 확인
if lsof /dev/ttyUSB0 /dev/ttyUSB2 2>/dev/null | grep -q .; then
    echo "[경고] 포트 아직 점유 중, 강제 해제..."
    fuser -k /dev/ttyUSB0 /dev/ttyUSB2 2>/dev/null
    sleep 1
fi

echo "[2/2] bringup 시작!"
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_ws/install/setup.bash
exec ros2 launch vicpinky_bringup bringup.launch.xml
