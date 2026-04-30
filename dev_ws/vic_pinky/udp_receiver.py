import cv2
import socket
import numpy as np

# 모든 IP에서 날아오는 데이터를 포트 5005번으로 받음
LISTEN_IP = "0.0.0.0" 
LISTEN_PORT = 5005

def start_udp_receiver():
    # 수신용 UDP 소켓 생성
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    
    print(f"📺 수신기 켜짐! {LISTEN_PORT}번 포트에서 영상을 기다리는 중...")

    while True:
        try:
            # 최대 65536 바이트 크기로 날아오는 데이터를 받음
            data, addr = sock.recvfrom(65536)
            
            # 받은 바이트 데이터를 OpenCV가 읽을 수 있는 배열로 변환
            buffer = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)

            if frame is not None:
                # 노트북 화면에 띄우기 (보기 편하게 2배로 확대)
                display_frame = cv2.resize(frame, (640, 480))
                cv2.imshow("UDP Super Fast Stream", display_frame)

                # 'q' 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            print(f"🚨 수신 에러: {e}")

    cv2.destroyAllWindows()
    sock.close()

if __name__ == '__main__':
    start_udp_receiver()