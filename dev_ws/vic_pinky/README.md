# VicPinky — 자율주행 + 객체추종 v1

ROS2 Nav2 기반 자율주행 + YOLOv8 기반 사람 추종 통합 시스템.  
웹 UI 버튼 하나로 두 모드를 실시간 전환할 수 있습니다.

---

## 시스템 구성

```
[빅핑키 로봇 (Raspberry Pi 5)]
  RPLIDAR C1  → /scan_filtered  ─┐
  USB 카메라  → /image_raw       ├─(ROS2 FastDDS)─→ [노트북]
  ZLAC 모터   ← /cmd_vel        ─┘

[노트북]
  Nav2 (자율주행)  ─┐
  YOLO v8 (감지)  ─┤→ /cmd_vel → 로봇
  웹 UI (모드전환) ─┘
```

| 기기 | IP | 역할 |
|------|----|------|
| 빅핑키 | `192.168.5.1` | 센서 + 모터 |
| 노트북 | `192.168.5.3` | Nav2 + YOLO 연산 |

---

## 하드웨어

- Raspberry Pi 5
- ZLAC 모터 드라이버 (RS485/ttyUSB1)
- RPLIDAR C1 (/dev/ttyUSB2)
- USB 웹캠 (/dev/video0)

---

## 설치

### 노트북

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip3 install "ultralytics" "numpy<2" fastapi uvicorn
```

### 빅핑키

```bash
# usb_cam 실행 스크립트 생성
cat > ~/run_usb_cam.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_vicpinky.xml
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p pixel_format:=mjpeg2rgb
EOF
chmod +x ~/run_usb_cam.sh
```

---

## 실행 순서

### 1단계 — 빅핑키 (터미널 2개)

```bash
# 터미널 1: 로봇 브링업 (모터 + 라이다 + TF)
~/bringup.sh

# 터미널 2: 웹캠 퍼블리시
bash ~/run_usb_cam.sh
```

### 2단계 — 노트북 (터미널 6개)

```bash
# 터미널 1: Pillar filter (Nav2 실행 전 먼저!)
bash ~/dev_ws/vic_pinky/run_pillar_filter.sh

# 터미널 2: Nav2 자율주행 실행
bash ~/dev_ws/vic_pinky/run_nav2.sh

# 터미널 3: Door pass filter (통로 통과 시 angular 억제)
bash ~/dev_ws/vic_pinky/run_door_pass.sh

# 터미널 4: YOLO 장애물 감지 + 추종 노드
bash ~/dev_ws/vic_pinky/run_yolo.sh

# 터미널 5: 웹 UI 서버 → http://localhost:8080
bash ~/dev_ws/vic_pinky/run_web.sh

# 터미널 6: RViz 시각화
bash ~/dev_ws/vic_pinky/run_rviz.sh
```

> Pillar filter → Nav2 → Door pass 순서로 실행  
> Nav2가 `Managed nodes are active` 출력 후 YOLO 노드 실행

---

## 모드 전환

브라우저에서 `http://localhost:8080` 접속

| 버튼 | 동작 |
|------|------|
| 🗺️ 자율주행 | Nav2가 목표 지점까지 경로 계획 + 장애물 회피 |
| 👤 사람 추종 | YOLO가 가장 큰 사람 감지 → 0.8m 거리 유지하며 추종 |
| 🛑 정지 | 즉시 정지 + Nav2 목표 취소 |

---

## 자율주행 사용법

1. RViz 상단 **2D Pose Estimate** → 로봇 현재 위치+방향 클릭+드래그
2. RViz 상단 **2D Goal Pose** → 목적지 클릭+드래그
3. 로봇 자동 출발

---

## SLAM 지도 제작 (최초 1회)

```bash
# 빅핑키: bringup.sh 실행 후

# 노트북 터미널 1
cd ~/dev_ws/vic_pinky && bash slam.sh

# 노트북 터미널 2 (키보드 조종)
export ROS_DOMAIN_ID=28 && ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 지도 저장
cd ~/dev_ws/vic_pinky && bash save_map.sh
```

---

## 파일 구조

```
vic_pinky/
├── run_nav2.sh              # Nav2 실행 (kill + restart)
├── run_yolo.sh              # YOLO 노드 실행
├── run_web.sh               # 웹 UI 서버 실행
├── run_rviz.sh              # RViz 실행
├── run_pillar_filter.sh     # Pillar filter 실행
├── run_door_pass.sh         # Door pass filter 실행
│
├── nav2.sh                  # Nav2 내부 실행 스크립트
├── nav2_params.yaml         # Nav2 파라미터
├── nav2_bt_obstacle_wait.xml # 행동 트리 (장애물 대기 + 후진 복구)
├── slam.sh                  # SLAM 실행
├── slam_params.yaml         # SLAM 파라미터
├── save_map.sh              # 지도 저장
│
├── yolo_nav_node.py         # YOLO 통합 노드 (장애물감지 + 사람추종)
├── web_control.py           # FastAPI 웹 UI + 카메라 라이브 피드
├── pillar_filter_node.py    # 알루미늄 기둥 마스킹 필터
├── door_pass_node.py        # 문 통과 시 angular 억제 필터
├── follower_udp.py          # 구버전 UDP 기반 추종 (참고용)
│
├── fastdds_vicpinky.xml     # FastDDS 네트워크 설정
└── yolov8n.pt               # YOLO 모델 가중치
```

---

## 주요 파라미터

### Nav2 (`nav2_params.yaml`)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `robot_radius` | 0.30m | costmap 기준 로봇 반경 |
| `max_vel_x` | 0.30 m/s | 최대 직진 속도 |
| `max_vel_theta` | 0.9 rad/s | 최대 회전 속도 |
| `local inflation_radius` | 0.10m | 로컬 costmap 장애물 팽창 |
| `global inflation_radius` | 0.25m | 글로벌 경로 계획 장애물 팽창 |
| `PathDist.scale` | 16.0 | 경로 추종 강도 |
| `RearCritical polygon` | -0.25~-0.48m | 뒤쪽 긴급 감속 구역 |

### YOLO (`yolo_nav_node.py`)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `follow_target_dist` | 0.8m | 사람 추종 목표 거리 |
| `follow_max_speed` | 0.40 m/s | 추종 최대 속도 |
| `follow_max_angular` | 0.7 rad/s | 추종 최대 회전 속도 |
| `CAM_HFOV` | 60° | 웹캠 수평 화각 |
| `person safety_radius` | 0.8m | 사람 costmap 마진 |

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 로봇이 안 움직임 | TF 시간 오차 | 양쪽에서 `chronyc makestep` |
| 문 앞에서 진동 | inflation 너무 큼 | local inflation_radius 확인 |
| 뒤에 박음 | robot_radius 0.30이 실제 후면 0.45 못 커버 | RearCritical polygon으로 감속 |
| 라이다 토픽 없음 | `/dev/lidar` 심링크 없음 | USB 케이블 재연결 후 bringup 재시작 |
| cmd_vel 무반응 | collision_monitor가 pillar를 장애물로 인식 | collision_monitor scan 토픽을 `scan_filtered_safe`로 설정 |
| goal 전송해도 이동 없음 | nav2.sh 재시작마다 relay 프로세스 누적 → cmd_vel 충돌 | `pkill -f "topic_tools relay"` 후 재시작 |
| /odom 수신 불가 | FastDDS 프로파일 미설정 | `FASTRTPS_DEFAULT_PROFILES_FILE` 환경변수 확인 |
| 웹 UI 8080 포트 충돌 | 이전 web_control.py 프로세스 잔존 | `pkill -f web_control.py` 후 재시작 |

---

## 오늘의 삽질 기록 (2026-05-20)

### 1. relay 프로세스 누적 문제
- `nav2.sh`에서 매번 `ros2 run topic_tools relay` 를 background로 띄우고 kill을 안 함
- Nav2 재시작할 때마다 relay가 쌓여서 `/cmd_vel`에 13개 퍼블리셔가 생김
- **해결**: nav2.sh에서 relay 제거, `door_pass_node.py`가 `cmd_vel_collision_out → cmd_vel` 담당

### 2. collision_monitor가 자기 기둥을 장애물로 인식
- collision_monitor scan 토픽이 `/scan` (raw) → 알루미늄 프레임 기둥이 그대로 들어옴
- FootprintApproach가 기둥을 전방 장애물로 인식 → cmd_vel_collision_out 완전 차단
- **해결**: collision_monitor scan 토픽을 `/scan_filtered_safe`로 변경 (pillar_filter 거친 것)

### 3. 문 통과 시 바퀴 걸림
- DWB가 문 통과 중 회전 명령을 내려서 로봇이 비껴 통과
- door_pass_node 추가: 라이다 좌우 감지로 좁은 통로 판단 → angular.z 0.25배 감쇠
- **결과**: 통로 중심 직진 유지, 통과 후 정상 회전 복원

### 4. 웹 UI 카메라 라이브 피드 추가
- `/image_raw/compressed` → MJPEG 스트리밍 → `<img src="/video">` 방식으로 web_control.py에 통합
- 기존 모드 전환 UI 위에 카메라 피드 표시
