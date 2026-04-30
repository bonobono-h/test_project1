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
        self.kp, self.kd = 0.004, 0.0
        self.prev_error = 0.0
        self.smooth_error = 0.0  # EMA 필터링된 에러
        self.max_speed = 0.20
        self.emergency_stop = False
        self.target_distance = 0.50
        self.front_distance = 999.0
        self.ignore_dist = 0.22
        self.deadzone = 15  # 중앙 ±15px 이내면 회전 무시

        self.lost_count = 0
        self.lost_threshold = 30  # 타겟 놓치고 버티는 프레임 수
        self.current_state = "WAITING"
        self.last_angular_dir = 0.0  # 마지막으로 회전하던 방향 (부호)

        # 360도 스캔 결과
        self.closest_distance = 999.0
        self.closest_angle_deg = 0.0
        self._lidar_log_counter = 0

        # UDP 소켓 수신기 세팅
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.settimeout(1.0) 

        self.get_logger().info("🚀 하이브리드(UDP+ROS2) AI 추종 시스템 가동 완료!")

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
                results = self.model.track(source=frame, persist=True, tracker="botsort.yaml", 
                                           classes=0, verbose=False, imgsz=160)
                r = results[0]
                msg = Twist()
                found_target_this_frame = False

                if r.boxes and r.boxes.id is not None:
                    for box in r.boxes:
                        idx = int(box.id[0])
                        # 새로운 타겟 락온
                        if self.target_id is None:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            if 40 < (x1 + x2) / 2 < 280:
                                self.target_id = idx
                                self.get_logger().info(f"🎯 타겟 락온: ID {idx}")

                        # 락온된 타겟 따라가기
                        if idx == self.target_id:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            cx = (x1 + x2) / 2
                            found_target_this_frame = True
                            self.lost_count = 0
                            
                            # 회전(방향) 제어 — 데드존 적용
                            error = self.center_x - cx
                            if abs(error) < self.deadzone:
                                error = 0.0
                            # EMA 저역통과 필터 (alpha=0.4: 빠른 반응 + 노이즈 제거)
                            self.smooth_error = 0.4 * error + 0.6 * self.smooth_error
                            angular = self.smooth_error * self.kp
                            msg.angular.z = float(max(-1.2, min(1.2, angular)))
                            self.prev_error = error
                            if abs(angular) > 0.05:
                                self.last_angular_dir = 1.0 if angular > 0 else -1.0

                            # 직진 제어 (라이다 거리 기준)
                            if self.front_distance > self.target_distance + 0.3:
                                msg.linear.x, self.current_state = self.max_speed, "FOLLOWING"
                            elif self.front_distance > self.target_distance:
                                msg.linear.x, self.current_state = self.max_speed * 0.5, "SLOW"
                            else:
                                msg.linear.x, self.current_state = 0.0, "ARRIVED"
                            
                            # 타겟에 초록색 박스 그리기
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            break

                # 3. 안전 로직 (타겟 놓치면 마지막 방향으로 탐색 회전)
                if not found_target_this_frame and self.target_id is not None:
                    self.lost_count += 1
                    msg.linear.x = 0.0
                    msg.angular.z = self.last_angular_dir * 0.4  # 탐색 회전
                    self.current_state = f"SEARCHING ({self.lost_threshold - self.lost_count})"
                    if self.lost_count > self.lost_threshold:
                        self.target_id = None
                        msg.angular.z = 0.0

                # 라이다에서 앞이 막혔다고 판단하면 긴급 정지
                if self.emergency_stop:
                    msg.linear.x, msg.angular.z = 0.0, 0.0
                    self.current_state = "EMERGENCY"

                # 4. 빅핑키에게 모터 명령 전송
                self.publisher.publish(msg)

                # 5. 노트북 화면에 상태 띄우기 (2배 확대)
                display_frame = cv2.resize(frame, (640, 480))
                cv2.putText(display_frame, f"State: {self.current_state}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_frame,
                            f"Near: {self.closest_distance:.2f}m @ {self.closest_angle_deg:.1f}deg",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
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