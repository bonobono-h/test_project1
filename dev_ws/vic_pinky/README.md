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
