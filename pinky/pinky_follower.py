import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ultralytics import YOLO
import cv2
import threading
import time
import math
import numpy as np
from PIL import Image

# 하드웨어 라이브러리 로드
sys.path.append(os.path.expanduser('~') + '/catkin_ws/src/pinky_follower/src')
try:
    from pinkylib import LED
    from pinky_lcd import LCD
except ImportError:
    print("⚠️ 하드웨어 라이브러리 로드 실패")

latest_raw_frame = None   

def camera_thread():
    global latest_raw_frame
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
    while True:
        cap.grab()
        ret, frame = cap.retrieve()
        if ret:
            latest_raw_frame = cv2.resize(cv2.flip(frame, -1), (320, 240))
        time.sleep(0.001)

class PinkyFullSystem(Node):
    def __init__(self):
        super().__init__('pinky_full_system')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        self.model = YOLO('yolov8n.pt') 
        
        try:
            self.leds = LED()
            self.lcd = LCD()
        except:
            self.leds, self.lcd = None, None
            print("❌ 하드웨어 연결 실패")

        # --- 제어 파라미터 ---
        self.target_id = None
        self.center_x = 160 
        self.kp = 0.007      
        self.kd = 0.006      
        self.prev_error = 0.0     
        self.max_speed = 0.25   
        self.emergency_stop = False

        # 🔥 [LiDAR 센서 퓨전용 파라미터 추가]
        self.target_distance = 0.45  # 로봇이 멈출 정확한 목표 거리 (0.015m = 1.5cm)
        self.front_distance = 999.0 # 라이다 정면 실제 거리 저장용

        # --- 자동 재인식 로직용 변수 ---
        self.lost_count = 0
        self.lost_threshold = 100  
        
        # 상태 관리
        self.current_state = "WAITING"
        self.rainbow_offset = 0
        self.last_lcd_update = 0
        self.COLORS = {
            'RED': (255, 0, 0), 'GREEN': (0, 255, 0), 'BLUE': (0, 0, 255),
            'ORANGE': (255, 127, 0), 'YELLOW': (255, 255, 0), 
            'VIOLET': (148, 0, 211), 'BLACK': (0, 0, 0), 'WHITE': (255, 255, 255)
        }

    def lidar_callback(self, data):
        # 1. 비상 정지용 (전방 80도 범위 내 15cm 이내 장애물)
        ranges = [r for r in (data.ranges[:40] + data.ranges[-40:]) if 0.05 < r < 10.0 and not math.isinf(r)]
        self.emergency_stop = True if ranges and min(ranges) < 0.15 else False

        # 🔥 2. 정면 거리 측정용 (전방 30도 범위의 물체 거리 정밀 측정)
        front_ranges = data.ranges[:15] + data.ranges[-15:]
        valid_front = [r for r in front_ranges if 0.05 < r < 10.0 and not math.isinf(r)]
        if valid_front:
            self.front_distance = min(valid_front)
        else:
            self.front_distance = 999.0

    def update_leds(self):
        if self.emergency_stop:
            self.leds.fill(self.COLORS['RED'])
        elif self.current_state == "FOLLOWING":
            rainbow = [self.COLORS['RED'], self.COLORS['ORANGE'], self.COLORS['YELLOW'], 
                       self.COLORS['GREEN'], self.COLORS['BLUE'], self.COLORS['VIOLET']]
            for i in range(8):
                self.leds.set_pixel(i, rainbow[(i + self.rainbow_offset) % len(rainbow)])
            self.rainbow_offset += 1
        elif self.current_state == "ARRIVED":
            self.leds.fill(self.COLORS['GREEN'])
        else:
            self.leds.fill(self.COLORS['BLUE'])
        self.leds.show()

    def draw_overlay(self, frame, text, color):
        cv2.rectangle(frame, (0, 200), (320, 240), (0, 0, 0), -1)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, text, (10, 230), font, 0.7, color, 2, cv2.LINE_AA)
        return frame

    def run(self):
        global latest_raw_frame
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if latest_raw_frame is None: continue
            
            frame = latest_raw_frame.copy()
            results = self.model.track(source=frame, persist=True, tracker="bytetrack.yaml", classes=0, verbose=False, imgsz=160)
            r = results[0]
            msg = Twist()
            
            self.current_state = "WAITING"
            status_text = "WAITING TARGET"
            text_color = self.COLORS['BLUE']
            
            found_target_this_frame = False

            if r.boxes and r.boxes.id is not None:
                best_box = None
                for box in r.boxes:
                    idx = int(box.id[0])
                    
                    if self.target_id is None:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cx = (x1 + x2) / 2
                        if 40 < cx < 280:
                            self.target_id = idx
                            print(f"🎯 주인님 감지! ID: {idx}")

                    if idx == self.target_id:
                        best_box = box
                        found_target_this_frame = True
                        self.lost_count = 0 
                        break

                if best_box is not None:
                    x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
                    cx = (x1 + x2) / 2
                    
                    # 🔥 회전 제어 (YOLO 시각 정보 사용 - PD 제어)
                    error = self.center_x - cx
                    angular = (error * self.kp) + (error - self.prev_error) * self.kd
                    msg.angular.z = float(max(-3.0, min(3.0, angular)))
                    self.prev_error = error
                    
                    # 🔥 전진 제어 (LiDAR 거리 정보 사용)
                    # 박스 크기(h) 대신 front_distance로 판단합니다!
                    if self.front_distance > self.target_distance + 0.3:
                        msg.linear.x = self.max_speed
                        self.current_state = "FOLLOWING"
                        status_text = f"FOLLOW ({self.front_distance:.2f}m)"
                        text_color = self.COLORS['WHITE']
                        
                    elif self.front_distance > self.target_distance:
                        msg.linear.x = self.max_speed * 0.4 # 감속
                        self.current_state = "FOLLOWING"
                        status_text = f"SLOW ({self.front_distance:.2f}m)"
                        text_color = self.COLORS['YELLOW']
                        
                    else:
                        msg.linear.x = 0.0 # 정지
                        self.current_state = "ARRIVED"
                        status_text = f"ARRIVED ({self.front_distance:.2f}m)"
                        text_color = self.COLORS['GREEN']
                    
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            if not found_target_this_frame and self.target_id is not None:
                self.lost_count += 1
                status_text = f"LOST.. {self.lost_threshold - self.lost_count}"
                if self.lost_count > self.lost_threshold:
                    print("⚠️ 주인님을 놓쳤습니다. 다시 찾습니다.")
                    self.target_id = None
                    self.lost_count = 0
                    self.prev_error = 0.0

            if self.emergency_stop:
                msg.linear.x, msg.angular.z = 0.0, 0.0
                self.current_state = "EMERGENCY"
                status_text = "!!! EMERGENCY !!!"
                text_color = self.COLORS['RED']

            self.publisher.publish(msg)

            if self.leds: self.update_leds()

            if self.lcd and (time.time() - self.last_lcd_update > 0.05):
                frame = self.draw_overlay(frame, status_text, (text_color[2], text_color[1], text_color[0]))
                img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                self.lcd.img_show(img_pil)
                self.last_lcd_update = time.time()

def main():
    rclpy.init()
    threading.Thread(target=camera_thread, daemon=True).start()
    node = PinkyFullSystem()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        if node.leds: node.leds.fill((0,0,0)); node.leds.show(); node.leds.close()
        if node.lcd: node.lcd.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()