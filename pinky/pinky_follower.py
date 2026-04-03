import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ultralytics import YOLO
import cv2

class PinkyFollower(Node):
    def __init__(self):
        super().__init__('pinky_follower')
        # 1. 퍼블리셔 및 AI 세팅
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.model = YOLO('yolov8n.pt')
        
        self.center_x = 320      
        self.kp = 0.002          
        self.linear_speed = 0.08 

    def run(self):
        # 1. OpenCV로 카메라의 통제권을 우리가 직접 가져옴!
        cap = cv2.VideoCapture(0)
        
        while rclpy.ok() and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("카메라 영상을 가져올 수 없습니다!")
                break

            # 🌟 [마법의 1줄] 소프트웨어 치료: 상하좌우(-1) 180도 뒤집기!
            flipped_frame = cv2.flip(frame, -1)

            # 2. 뒤집힌 정상 화면을 YOLO에게 먹이기 (persist=True로 ID 유지)
            results = self.model.track(source=flipped_frame, persist=True, tracker="bytetrack.yaml", classes=0, verbose=False)
            
            # 리스트로 나오는 결과에서 첫 번째(현재) 프레임만 빼옴
            r = results[0]
            msg = Twist()
            boxes = r.boxes
            
            if len(boxes) > 0:
                best_box = None
                min_dist = 9999
                
                for box in boxes:
                    if box.id is not None:
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = int((x1 + x2) / 2)
                        dist = abs(self.center_x - cx)
                        
                        if dist < min_dist:
                            min_dist = dist
                            best_box = box
                
                if best_box is not None:
                    x1, y1, x2, y2 = best_box.xyxy[0]
                    cx = int((x1 + x2) / 2)
                    error = self.center_x - cx
    
                    # 조향 (방향 제어)
                    msg.angular.z = -float(error * self.kp) 
                    
                    # 거리 제어
                    box_height = y2 - y1 
                    screen_height = 480  
                    
                    if box_height > screen_height * 0.7:
                        msg.linear.x = 0.0
                        print(f"🛑 대기 모드 (Box: {box_height:.1f})")
                    elif box_height < screen_height * 0.3:
                        msg.linear.x = self.linear_speed * 1.5 
                        print(f"🏃‍♂️ 추격 모드 (Box: {box_height:.1f})")
                    else:
                        msg.linear.x = self.linear_speed
                        print(f"🐕 쫄쫄 따라가는 중 (Box: {box_height:.1f})")

                else:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                print("👀 타겟 찾는 중...")

            self.publisher.publish(msg)

            # 3. 네 노트북(-X 포워딩)으로 YOLO 박스 그려진 정상 화면 띄우기!
            annotated_frame = r.plot() 
            cv2.imshow("Pinky Vision (Flipped)", annotated_frame)
            
            # 터미널에서 실행 중일 때 영상 창 클릭하고 'q' 누르면 우아하게 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        # 종료 시 카메라 자원 해제
        cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = PinkyFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        print("\n🛑 제어 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()