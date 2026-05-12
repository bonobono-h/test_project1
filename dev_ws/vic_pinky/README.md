# VicPinky AI 추종 시스템

YOLOv8 + LiDAR 센서 퓨전 기반 사람 추종 로봇 시스템입니다.  
카메라 영상은 UDP로 노트북에 전송하고, 노트북에서 AI 처리 후 ROS2로 모터 제어 명령을 전송합니다.

---

## 시스템 구성

```
[빅핑키 로봇]                        [노트북]
  카메라 → udp_sender.py  →(UDP)→  follower_udp.py
  RPLIDAR C1 → /scan      →(ROS2)→  lidar_callback()
  /cmd_vel   ←(ROS2)←              publisher.publish()
```

| 구분 | 역할 |
|------|------|
| **빅핑키** | 카메라 영상 UDP 송출, LiDAR 데이터 ROS2 퍼블리시, 모터 수신 |
| **노트북** | YOLOv8 사람 감지/추적, LiDAR 장애물 판단, /cmd_vel 제어 |

---

## 하드웨어

- 빅핑키 로봇 (Raspberry Pi 5 + ZLAC 모터 드라이버)
- RPLIDAR C1 (`/dev/ttyUSB2`)
- USB 카메라 HCAM01Q (`/dev/camera` → udev 심볼릭 링크)
- 노트북 (YOLOv8 추론 실행)

---

## 네트워크 설정

| 기기 | IP |
|------|-----|
| 빅핑키 | `192.168.5.1` |
| 노트북 | `192.168.5.3` |

ROS2 멀티머신 통신에 FastDDS 설정 필요 (Docker 네트워크 간섭 방지).

---

## 파일 구조

```
vic_pinky/
├── follower_udp.py        # [노트북] 메인 AI 추종 노드
├── run.sh                 # [노트북] 실행 스크립트
├── fastdds_vicpinky.xml   # [노트북] FastDDS 네트워크 설정
├── udp_sender.py          # [빅핑키] 카메라 UDP 송출
├── bringup.sh             # [빅핑키] 로봇 부팅 스크립트
├── find_pillar_angles.py  # 프로파일러 각도 탐지 진단 도구
└── udp_receiver.py        # UDP 영상 수신 테스트 도구
```

---

## 설치

### 노트북

```bash
pip install ultralytics opencv-python numpy
```

`~/.bashrc`에 추가:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hong/dev_ws/vic_pinky/fastdds_vicpinky.xml
```

### 빅핑키

udev 카메라 심볼릭 링크 설정 (최초 1회):
```bash
echo 'SUBSYSTEM=="video4linux", ATTRS{idVendor}=="0c45", ATTRS{idProduct}=="631c", ATTR{index}=="0", SYMLINK+="camera"' | sudo tee /etc/udev/rules.d/99-camera.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`~/.bashrc`에 추가:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/vic/fastdds_vicpinky.xml
```

FastDDS 설정 파일 복사:
```bash
cp fastdds_vicpinky.xml ~/fastdds_vicpinky.xml
```

---

## 실행

### 빅핑키에서
```bash
~/bringup.sh          # 좀비 프로세스 정리 + bringup 실행
python3 udp_sender.py # 카메라 송출 (별도 터미널)
```

### 노트북에서
```bash
~/dev_ws/vic_pinky/run.sh
```

---

## 주요 파라미터 (`follower_udp.py`)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `target_distance` | 0.50m | 추종 목표 거리 (이 거리에서 정지) |
| `max_speed` | 0.20 m/s | 최대 전진 속도 |
| `kp` | 0.004 | 방향 제어 P 게인 |
| `deadzone` | 15px | 중앙 ±15px 회전 무시 |
| `lost_threshold` | 50 frames | 타겟 소실 후 탐색 유지 시간 |
| `PILLAR_ANGLES_DEG` | [162.0, 200.0, 245.3, 338.7, 23.3] | 프로파일러 제외 각도 |
| `PILLAR_HALF_WIDTH` | ±10° | 프로파일러 제외 범위 |

### 상태 설명

| 상태 | 조건 |
|------|------|
| `WAITING` | 타겟 없음 |
| `FOLLOWING` | 전방 > 0.80m, 전진 |
| `SLOW` | 전방 0.50~0.80m, 감속 |
| `ARRIVED` | 전방 ≤ 0.50m, 정지 |
| `SEARCHING` | 타겟 소실, 마지막 방향으로 탐색 회전 |
| `EMERGENCY` | 전방 ±40° 0.30m 이내 감지, 긴급 정지 |

---

## LiDAR 참고

- RPLIDAR C1 시리얼 포트: `/dev/ttyUSB2`
- `laser_link` 마운트: `rpy="0 0 π"` (180° 회전) → 스캔 0° = 로봇 후방, 180° = 로봇 전방
- 프로파일러(지지대) 4개: 23.3°, 162.0°, 200.0°, 338.7° (실측값)

---

# Nav2 자율 주행 시스템

SLAM으로 지도를 만들고 Nav2로 목표 좌표까지 자율 주행합니다.

---

## 파일 구조 (Nav2)

```
vic_pinky/
├── slam.sh              # [노트북] SLAM 지도 제작 실행
├── slam_params.yaml     # SLAM 파라미터
├── save_map.sh          # [노트북] 지도 저장
├── nav2.sh              # [노트북] Nav2 자율 주행 실행
├── nav2_params.yaml     # Nav2 파라미터
└── cyclone_vicpinky.xml # CycloneDDS 설정 (FastDDS 대신 사용 시)
```

---

## 1단계: SLAM 지도 제작

### 빅핑키에서
```bash
~/bringup.sh
```

### 노트북에서 (터미널 3개)

**터미널 1 — SLAM 실행:**
```bash
cd ~/dev_ws/vic_pinky && ./slam.sh
```

**터미널 2 — 로봇 조종 (키보드 또는 추종):**
```bash
# 키보드 조종
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=28
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 또는 AI 추종으로 지도 제작
./run.sh
```

**터미널 3 — RViz2 시각화:**
```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=28 && rviz2
```

지도가 완성되면 **터미널 4** 에서 저장:
```bash
cd ~/dev_ws/vic_pinky && ./save_map.sh
# → ~/my_map.yaml, ~/my_map.pgm 저장됨
```

> SLAM 종료 후 Nav2 실행 (동시 실행 불가)

---

## 2단계: Nav2 자율 주행

### 빅핑키에서
```bash
~/bringup.sh
```

### 노트북에서 (터미널 2개)

**터미널 1 — Nav2 실행:**
```bash
cd ~/dev_ws/vic_pinky && ./nav2.sh /home/hong/my_map.yaml
```
> `Managed nodes are active` 메시지 나올 때까지 대기 (약 15~20초)

**터미널 2 — RViz2:**
```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=28 && rviz2
```

### RViz2 설정
1. Displays 패널 → **Add** → `Map` (Topic: `/map`)
2. Displays 패널 → **Add** → `LaserScan` (Topic: `/scan`)
3. 맵이 안 보이면 터미널에서 맵 재발행:
```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=28
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/hong/my_map.yaml'}"
```

### 자율 주행 시작
1. RViz2 상단 **"2D Pose Estimate"** 클릭 → 로봇 현재 위치+방향 클릭+드래그
2. RViz2 상단 **"2D Goal Pose"** 클릭 → 목적지 클릭+드래그
3. 로봇 출발!

---

## Nav2 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 맵이 RViz2에 안 보임 | QoS 불일치 | `load_map` 서비스 재호출 |
| 로봇이 안 움직임 | bt_navigator inactive | `ros2 lifecycle set /bt_navigator activate` |
| 로봇이 삥글삥글 돔 | Pose Estimate 방향 틀림 | Pose Estimate 다시 설정 (LaserScan 보면서 맞추기) |
| cmd_vel 안 나옴 | velocity_smoother 문제 | nav2.sh 내 relay 자동 실행됨 |

---

## 주요 파라미터 (`nav2_params.yaml`)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `robot_radius` | 0.30m | 로봇 반경 |
| `inflation_radius` | 0.30m | 장애물 팽창 반경 (좁은 문 통과용) |
| `desired_linear_vel` | 0.25 m/s | 목표 직진 속도 |
| `max_velocity` | 0.30 m/s | 최대 속도 |
| `use_rotate_to_heading` | false | 주행 중 회전 비활성화 |
