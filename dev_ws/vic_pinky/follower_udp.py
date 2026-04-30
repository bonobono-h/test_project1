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
        self.kp, self.kd = 0.005, 0.002
        self.prev_error = 0.0
        self.max_speed = 0.20
        self.emergency_stop = False
        self.target_distance = 0.50
        self.front_distance = 999.0
        self.ignore_dist = 0.20
        self.deadzone = 15  # 중앙 ±15px 이내면 회전 무시

        self.lost_count = 0
        self.lost_threshold = 15  # 타겟 놓치고 버티는 프레임 수
        self.current_state = "WAITING"

        # UDP 소켓 수신기 세팅
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_IP, LISTEN_PORT))
        self.sock.settimeout(1.0) 

        self.get_logger().info("🚀 하이브리드(UDP+ROS2) AI 추종 시스템 가동 완료!")

    def lidar_callback(self, data):
        # 전방 장애물 체크 (충돌 방지)
        valid_ranges = [r for r in (data.ranges[:40] + data.ranges[-40:]) 
                        if self.ignore_dist < r < 10.0 and not math.isinf(r)]
        self.emergency_stop = bool(valid_ranges and min(valid_ranges) < 0.15)
        
        front_ranges = data.ranges[:15] + data.ranges[-15:]
        valid_front = [r for r in front_ranges if self.ignore_dist < r < 10.0 and not math.isinf(r)]
        self.front_distance = min(valid_front) if valid_front else 999.0

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
                            angular = (error * self.kp) + (error - self.prev_error) * self.kd
                            msg.angular.z = float(max(-1.2, min(1.2, angular)))
                            self.prev_error = error

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

                # 3. 안전 로직 (타겟 놓치면 정지)
                if not found_target_this_frame and self.target_id is not None:
                    self.lost_count += 1
                    msg.linear.x, msg.angular.z = 0.0, 0.0
                    self.current_state = f"LOST ({self.lost_threshold - self.lost_count})"
                    if self.lost_count > self.lost_threshold:
                        self.target_id = None

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