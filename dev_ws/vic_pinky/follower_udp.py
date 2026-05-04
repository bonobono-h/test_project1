import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ultralytics import YOLO
import cv2
import socket
import numpy as np
import math

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

        # 제어 변수들
        self.target_id = None
        self.center_x = 160  # 320 해상도의 절반
        self.kp, self.kd = 0.005, 0.0
        self.prev_error = 0.0
        self.smooth_error = 0.0  # EMA 필터링된 에러
        self.max_speed = 0.30
        self.emergency_stop = False
        self.target_distance = 0.75
        self.front_distance = 999.0
        self.ignore_dist = 0.22
        self.deadzone = 8  # 중앙 ±8px 이내면 회전 무시

        self.lost_count = 0
        self.lost_threshold = 30  # 50 → 30으로 축소 (헛도는 시간 줄임)
        self.current_state = "WAITING"
        self.last_angular_dir = 0.0  # 마지막으로 회전하던 방향 (부호)

        # 히스토그램 기반 ReID
        self.target_hist = None
        self.hist_threshold = 0.45  # Bhattacharyya: 낮을수록 유사 (0.5 이하 = 동일인)

        # 360도 스캔 결과
        self.closest_distance = 999.0
        self.closest_angle_deg = 0.0
        self._lidar_log_counter = 0

        # UDP 소켓 수신기 세팅
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.settimeout(1.0)

        self.get_logger().info("🚀 하이브리드(UDP+ROS2) AI 추종 시스템 가동 완료!")

    # ──────────────────────── 히스토그램 유틸 ────────────────────────

    def _calc_hist(self, frame, box_xyxy):
        """bbox 영역의 상체 HSV 히스토그램 계산"""
        x1, y1, x2, y2 = map(int, box_xyxy)
        h = y2 - y1
        crop = frame[y1:y1 + int(h * 0.6), x1:x2]  # 상체만 (하반신 노이즈 제거)
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist)
        return hist

    def _match_hist(self, hist):
        """저장된 타겟 히스토그램과 Bhattacharyya 비교"""
        if self.target_hist is None or hist is None:
            return False
        score = cv2.compareHist(self.target_hist, hist, cv2.HISTCMP_BHATTACHARYYA)
        self.get_logger().info(f"[HIST] 유사도: {score:.3f} (임계: {self.hist_threshold})")
        return score < self.hist_threshold

    # ──────────────────────── 라이다 ────────────────────────

    def _is_pillar(self, angle_deg):
        for pa in PILLAR_ANGLES_DEG:
            diff = abs((angle_deg - pa + 180) % 360 - 180)
            if diff <= PILLAR_HALF_WIDTH:
                return True
        return False

    def lidar_callback(self, data):
        angle_min = data.angle_min
        angle_inc = data.angle_increment

        closest_d = 999.0
        closest_a = 0.0
        front_vals = []
        emergency_vals = []

        for i, r in enumerate(data.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.05 or r > 10.0:
                continue
            angle_deg = math.degrees(angle_min + i * angle_inc) % 360

            # 비상정지: 로봇 전방(180°) ±40° — 프로파일러 제외, ignore_dist 미적용
            if 140 <= angle_deg <= 220 and not self._is_pillar(angle_deg):
                emergency_vals.append(r)

            # 이하는 ignore_dist 적용 (전방 기둥 노이즈 제거)
            if r <= self.ignore_dist:
                continue

            if self._is_pillar(angle_deg):
                continue

            # 360도 최근접 물체
            if r < closest_d:
                closest_d = r
                closest_a = angle_deg

            # 전방 ±15° (라이다 180° 기준) — 추종 거리 판단
            if 165 <= angle_deg <= 195:
                front_vals.append(r)

        self.closest_distance = closest_d
        self.closest_angle_deg = closest_a
        self.emergency_stop = bool(emergency_vals and min(emergency_vals) < 0.30)
        self.front_distance = min(front_vals) if front_vals else 999.0

        # 10프레임마다 터미널에 출력
        self._lidar_log_counter += 1
        if self._lidar_log_counter >= 10:
            self._lidar_log_counter = 0
            self.get_logger().info(
                f"[LIDAR] 최근접 물체 — 거리: {closest_d:.2f}m | 방위각: {closest_a:.1f}°"
            )

    # ──────────────────────── 메인 루프 ────────────────────────

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            try:
                # 1. UDP로 영상 수신
                data, addr = self.sock.recvfrom(65536)
                buffer = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

                if frame is None:
                    continue

                # 2. BoT-SORT로 사람 추적
                results = self.model.track(
                    source=frame, persist=True, tracker="botsort.yaml",
                    classes=0, verbose=False, imgsz=160
                )
                r = results[0]
                msg = Twist()
                found_target_this_frame = False

                if r.boxes and r.boxes.id is not None:
                    for box in r.boxes:
                        idx = int(box.id[0])

                        # ── 최초 락온: 히스토그램도 함께 저장 ──
                        if self.target_id is None:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx = (x1 + x2) / 2
                            if 40 < cx < 280:
                                hist = self._calc_hist(frame, (x1, y1, x2, y2))
                                if hist is not None:
                                    self.target_id = idx
                                    self.target_hist = hist
                                    self.get_logger().info(
                                        f"🎯 타겟 락온: ID {idx} + 히스토그램 저장"
                                    )

                        # ── 락온된 타겟 따라가기 ──
                        if idx == self.target_id:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx = (x1 + x2) / 2
                            found_target_this_frame = True
                            self.lost_count = 0

                            # 회전(방향) 제어 — 데드존 + 적응형 EMA
                            error = self.center_x - cx
                            if abs(error) < self.deadzone:
                                error = 0.0
                            # 오차가 클수록 alpha↑(빠른 반응), 작을수록 alpha↓(부드럽게)
                            alpha = min(0.85, 0.25 + abs(error) / 200.0)
                            self.smooth_error = alpha * error + (1.0 - alpha) * self.smooth_error
                            angular = self.smooth_error * self.kp
                            msg.angular.z = float(max(-0.9, min(0.9, angular)))
                            self.prev_error = error
                            if abs(angular) > 0.05:
                                self.last_angular_dir = 1.0 if angular > 0 else -1.0

                            # 직진 제어 (라이다 거리 기준)
                            if self.front_distance > self.target_distance + 0.3:
                                msg.linear.x = self.max_speed
                                self.current_state = "FOLLOWING"
                            elif self.front_distance > self.target_distance:
                                msg.linear.x = self.max_speed * 0.5
                                self.current_state = "SLOW"
                            else:
                                msg.linear.x = 0.0
                                self.current_state = "ARRIVED"

                            # 히스토그램 EMA 갱신 (조명 변화 대응)
                            new_hist = self._calc_hist(frame, (x1, y1, x2, y2))
                            if new_hist is not None and self.target_hist is not None:
                                self.target_hist = 0.7 * self.target_hist + 0.3 * new_hist

                            # 타겟에 초록색 박스 그리기
                            cv2.rectangle(
                                frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                (0, 255, 0), 2
                            )
                            break

                # ── 3. SEARCHING: 타겟 놓쳤을 때 히스토그램 기반 재락온 ──
                if not found_target_this_frame and self.target_id is not None:
                    self.lost_count += 1
                    msg.linear.x = 0.0

                    # 감지된 사람들 중 히스토그램 매칭되는 사람 찾기
                    if r.boxes and r.boxes.id is not None:
                        for box in r.boxes:
                            idx = int(box.id[0])
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx = (x1 + x2) / 2
                            if 40 < cx < 280:
                                cand_hist = self._calc_hist(frame, (x1, y1, x2, y2))
                                if self._match_hist(cand_hist):
                                    # 히스토그램 일치 → 재락온
                                    self.target_id = idx
                                    self.lost_count = 0
                                    self.get_logger().info(
                                        f"🔄 히스토그램 매칭 재락온: ID {idx}"
                                    )
                                    error = self.center_x - cx
                                    if abs(error) < self.deadzone:
                                        error = 0.0
                                    alpha = min(0.85, 0.25 + abs(error) / 200.0)
                                    self.smooth_error = alpha * error + (1.0 - alpha) * self.smooth_error
                                    angular = self.smooth_error * self.kp
                                    msg.angular.z = float(max(-0.9, min(0.9, angular)))
                                    found_target_this_frame = True

                                    cv2.rectangle(
                                        frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                        (0, 200, 255), 2  # 주황색 = 재락온
                                    )
                                    break

                    # 재락온 실패 → 탐색 회전 (감속)
                    if not found_target_this_frame:
                        msg.angular.z = self.last_angular_dir * 0.35  # 0.7 → 0.35 감속
                        self.current_state = f"SEARCHING ({self.lost_threshold - self.lost_count})"
                        if self.lost_count > self.lost_threshold:
                            self.target_id = None
                            self.target_hist = None  # 히스토그램도 리셋
                            msg.angular.z = 0.0
                            self.current_state = "WAITING"
                            self.get_logger().info("❌ 타겟 완전 소실 → WAITING")

                # 라이다에서 앞이 막혔다고 판단하면 긴급 정지
                if self.emergency_stop:
                    msg.linear.x, msg.angular.z = 0.0, 0.0
                    self.current_state = "EMERGENCY"

                # 4. 빅핑키에게 모터 명령 전송
                self.publisher.publish(msg)

                # 5. 노트북 화면에 상태 띄우기 (2배 확대)
                display_frame = cv2.resize(frame, (640, 480))
                cv2.putText(
                    display_frame, f"State: {self.current_state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                )
                cv2.putText(
                    display_frame,
                    f"Near: {self.closest_distance:.2f}m @ {self.closest_angle_deg:.1f}deg",
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2
                )
                # 히스토그램 매칭 임계값 표시
                hist_status = "HIST: Ready" if self.target_hist is not None else "HIST: None"
                cv2.putText(
                    display_frame, hist_status, (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 2
                )
                cv2.imshow("VicPinky AI Brain (UDP+ROS2)", display_frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except socket.timeout:
                msg = Twist()
                self.publisher.publish(msg)
                continue
            except Exception as e:
                self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = VicPinkyUdpFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()