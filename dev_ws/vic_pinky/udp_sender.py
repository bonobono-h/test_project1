import cv2
import os
import socket
import time

TARGET_IP = "192.168.5.3"
TARGET_PORT = 5005
FPS = 20

def start_udp_sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

    cap = cv2.VideoCapture(int(os.path.realpath('/dev/camera').replace('/dev/video','')), cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 최소화 (최신 프레임만 유지)

    if not cap.isOpened():
        print('카메라를 열 수 없습니다!')
        return

    print(f'📡 {TARGET_IP}:{TARGET_PORT} 로 비디오 송출 시작 ({FPS}fps)...')

    interval = 1.0 / FPS
    next_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        now = time.time()
        if now < next_time:
            continue
        next_time = now + interval

        result, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if result and len(encoded) < 65500:
            sock.sendto(encoded.tobytes(), (TARGET_IP, TARGET_PORT))

if __name__ == '__main__':
    start_udp_sender()
