import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ultralytics import YOLO
import cv2
import socket
import numpy as np
import math
import threading
import time

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

# 빅핑키 프로파일러(지지대) 4개 방위각 — 360도 스캔 시 제외
PILLAR_ANGLES_DEG = [162.0, 200.0, 245.3, 338.7, 23.3]
PILLAR_HALF_WIDTH = 10  # ±10도 범위 제외


class VicPinkyUdpFollower(Node):
    def __init__(self):
        super().__init__('vic_pinky_udp_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # AI 모델 로드
        self.model = YOLO('yolov8n.pt')

        # ── 제어 변수들 (유저 최적화 값 적용) ──
        self.center_x = 160  # 비전 루프에서 해상도에 맞게 자동 갱신됨
        self.kp = 0.005
        self.kd = 0.0
        self.max_speed = 0.30
        self.target_distance = 0.65
        self.ignore_dist = 0.22
        self.deadzone = 8
        self.hist_threshold = 0.45
        
        # 타겟 소실 후 대기 시간 (기존 lost_threshold 30프레임 = 약 1.0초)
        self.reset_timeout = 1.0 

        # ── 제어 루프 내부 상태 ──
        self.smooth_error = 0.0
        self.prev_error = 0.0
        self.last_angular_dir = 0.0
        self.current_state = "WAITING"

        # ── 라이다 상태 ──
        self.front_distance = 999.0
        self.closest_distance = 999.0
        self.closest_angle_deg = 0.0
        self.emergency_stop = False
        self._lidar_log_counter = 0

        # ── 비전 스레드 ↔ 제어 타이머 공유 상태 (Lock으로 보호) ──
        self._det_lock = threading.Lock()
        self._det_found = False       # 이번 프레임에서 타겟 찾음
        self._det_has_target = False  # 락온된 타겟이 존재함 (히스토그램 있음)
        self._det_cx = 0.0
        self._det_frame = None
        self._det_last_found_ts = 0.0 # 마지막으로 타겟을 본 시간

        # 제어 타이머가 비전 스레드에게 타겟 리셋을 지시할 때 사용
        self._reset_lock = threading.Lock()
        self._reset_requested = False

        # ── UDP 소켓 수신기 세팅 (포트 충돌 방지 SO_REUSEADDR 추가) ──
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((LISTEN_IP, LISTEN_PORT))
        self._sock.settimeout(1.0)

        # ── 스레드 및 30Hz 제어 루프 실행 ──
        threading.Thread(target=self._vision_loop, daemon=True).start()
        self.create_timer(1.0 / 30.0, self._control_loop)

        self.get_logger().info("🚀 멀티스레드 하이브리드(UDP+ROS2) AI 추종 시스템 가동 완료!")

    # ──────────────────────── 라이다 ────────────────────────

    def _is_pillar(self, angle_deg):
        for pa in PILLAR_ANGLES_DEG:
            if abs((angle_deg - pa + 180) % 360 - 180) <= PILLAR_HALF_WIDTH:
                return True
        return False

    def lidar_callback(self, data):
        angle_min = data.angle_min
        angle_inc = data.angle_increment
        closest_d, closest_a = 999.0, 0.0
        front_vals, emergency_vals = [], []

        for i, r in enumerate(data.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.05 or r > 10.0:
                continue
            angle_deg = math.degrees(angle_min + i * angle_inc) % 360

            # 1. 비상정지: (유저 설정) 140~220도, 프로파일러 제외. (0.3m 이내면 비상정지)
            if 140 <= angle_deg <= 220 and not self._is_pillar(angle_deg):
                emergency_vals.append(r)

            # 2. ignore_dist 및 프로파일러 무시
            if r <= self.ignore_dist or self._is_pillar(angle_deg):
                continue

            if r < closest_d:
                closest_d, closest_a = r, angle_deg
            
            # 3. 전방 거리 (추종 제어용)
            if 165 <= angle_deg <= 195:
                front_vals.append(r)

        self.closest_distance = closest_d
        self.closest_angle_deg = closest_a
        self.emergency_stop = bool(emergency_vals and min(emergency_vals) < 0.30)
        self.front_distance = min(front_vals) if front_vals else 999.0

        self._lidar_log_counter += 1
        if self._lidar_log_counter >= 10:
            self._lidar_log_counter = 0
            self.get_logger().info(f"[LIDAR] 최근접 물체 — 거리: {closest_d:.2f}m | 방위각: {closest_a:.1f}°")

    # ──────────────────────── 히스토그램 유틸 ────────────────────────

    def _calc_hist(self, frame, box_xyxy):
        """bbox 영역의 상체 HSV 히스토그램 계산 (안전한 슬라이싱 적용)"""
        x1 = max(0, int(box_xyxy[0]))
        y1 = max(0, int(box_xyxy[1]))
        x2 = min(frame.shape[1], int(box_xyxy[2]))
        y2 = min(frame.shape[0], int(box_xyxy[3]))
        
        h = y2 - y1
        crop = frame[y1:y1 + int(h * 0.6), x1:x2]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist)
        return hist

    # ──────────────────────── 1. 비전 스레드 (눈) ────────────────────────

    def _vision_loop(self):
        """UDP 영상 수신 및 YOLO 분석을 백그라운드에서 전담"""
        target_id = None
        target_hist = None

        while True:
            # 제어 루프에서 "타겟 리셋" 명령이 왔는지 확인
            with self._reset_lock:
                if self._reset_requested:
                    target_id = None
                    target_hist = None
                    self._reset_requested = False

            try:
                data, _ = self._sock.recvfrom(65536)
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                self.center_x = frame.shape[1] // 2  # 동적 해상도 대응

                results = self.model.track(
                    source=frame, persist=True, tracker="botsort.yaml",
                    classes=0, verbose=False, imgsz=160
                )
                r = results[0]
                found = False
                cx_out = 0.0

                if r.boxes and r.boxes.id is not None:
                    for box in r.boxes:
                        idx = int(box.id[0])
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cx = (x1 + x2) / 2

                        # 최초 락온
                        margin = frame.shape[1] * 0.35
                        if target_id is None and (self.center_x - margin) < cx < (self.center_x + margin):
                            hist = self._calc_hist(frame, (x1, y1, x2, y2))
                            if hist is not None:
                                target_id = idx
                                target_hist = hist
                                self.get_logger().info(f"🎯 타겟 최초 락온: ID {idx} + 히스토그램 저장")

                        # 추적 중
                        if idx == target_id:
                            found = True
                            cx_out = cx
                            new_h = self._calc_hist(frame, (x1, y1, x2, y2))
                            if new_h is not None and target_hist is not None:
                                target_hist = 0.7 * target_hist + 0.3 * new_h
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            break

                    # 히스토그램 기반 재락온
                    if not found and target_id is not None and target_hist is not None:
                        for box in r.boxes:
                            idx = int(box.id[0])
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx = (x1 + x2) / 2
                            h = self._calc_hist(frame, (x1, y1, x2, y2))
                            if h is not None:
                                score = cv2.compareHist(target_hist, h, cv2.HISTCMP_BHATTACHARYYA)
                                if score < self.hist_threshold:
                                    target_id = idx
                                    found = True
                                    cx_out = cx
                                    self.get_logger().info(f"🔄 히스토그램 매칭 재락온: ID {idx} (Score: {score:.3f})")
                                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 255), 2)
                                    break

                # 최신 분석 결과를 제어 루프가 가져갈 수 있게 공유 변수에 업데이트
                with self._det_lock:
                    self._det_found = found
                    self._det_has_target = target_id is not None
                    self._det_cx = cx_out
                    self._det_frame = frame
                    if found:
                        self._det_last_found_ts = time.time()

            except socket.timeout:
                pass
            except OSError:
                continue # 버퍼 초과 에러 시 스레드 종료 방지

    # ──────────────────────── 2. 제어 타이머 (다리) ────────────────────────

    def _control_loop(self):
        """초당 30번(30Hz) 실행되며 모터를 제어함"""
        # 비전 스레드에서 최신 데이터 가져오기
        with self._det_lock:
            found = self._det_found
            has_target = self._det_has_target
            cx = self._det_cx
            frame = self._det_frame
            last_found_ts = self._det_last_found_ts

        msg = Twist()
        elapsed = time.time() - last_found_ts if last_found_ts > 0 else 999.0

        # 잠깐(0.3초 이내) 놓친 건 아직 추적 중인 것으로 간주 (YOLO 깜빡임 대응)
        actively_tracking = found or (has_target and elapsed < 0.3)

        if actively_tracking:
            # (유저 코드 적용) 오차가 클수록 빠른 반응, 작을수록 부드럽게
            error = self.center_x - cx
            if abs(error) < self.deadzone:
                error = 0.0
            
            alpha = min(0.85, 0.25 + abs(error) / 200.0)
            self.smooth_error = alpha * error + (1.0 - alpha) * self.smooth_error
            
            # PD 제어 (kd=0 이므로 P제어)
            angular = self.smooth_error * self.kp
            angular_clipped = float(np.clip(angular, -0.9, 0.9))
            
            if abs(angular_clipped) > 0.05:
                self.last_angular_dir = 1.0 if angular_clipped > 0 else -1.0
            msg.angular.z = angular_clipped

            # 직진 제어 — 거리 오차에 비례한 속도 (멀수록 빠르게)
            dist_err = self.front_distance - self.target_distance
            if dist_err > 0.08:
                msg.linear.x = float(min(self.max_speed, dist_err * 1.0))
                self.current_state = "FOLLOWING" if dist_err > 0.25 else "SLOW"
            else:
                msg.linear.x = 0.0
                self.current_state = "ARRIVED"

        elif has_target:
            # 타겟은 놓쳤으나 아직 히스토그램은 기억함 (SEARCHING 상태)
            msg.linear.x = 0.0
            if elapsed < self.reset_timeout: # 설정한 시간(1.0초) 동안만 탐색
                msg.angular.z = self.last_angular_dir * 0.35 # (유저 코드 감속 적용)
                remain = int((self.reset_timeout - elapsed) * 30) # 프레임처럼 계산
                self.current_state = f"SEARCHING ({remain})"
            else:
                # 완전 소실 시 WAITING 전환 & 비전 스레드에 리셋 신호 보냄
                with self._reset_lock:
                    self._reset_requested = True
                msg.angular.z = 0.0
                self.current_state = "WAITING"
                self.get_logger().info("❌ 타겟 완전 소실 → WAITING")

        # ── 최우선 순위: 라이다 비상 정지 ──
        if self.emergency_stop:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.current_state = "EMERGENCY"

        self.publisher.publish(msg)

        # 화면 출력 (GUI 모니터가 있는 환경에서만 실행)
        if frame is not None:
            try:
                display = cv2.resize(frame, (640, 480))
                cv2.putText(display, f"State: {self.current_state}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display, f"Near: {self.closest_distance:.2f}m @ {self.closest_angle_deg:.1f}deg",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                hist_status = "HIST: Ready" if has_target else "HIST: None"
                cv2.putText(display, hist_status, (10, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2)
                cv2.imshow("VicPinky AI Brain (Multithread)", display)
                cv2.waitKey(1)
            except Exception:
                pass


def main():
    rclpy.init()
    node = VicPinkyUdpFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()