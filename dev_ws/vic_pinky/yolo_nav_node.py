#!/usr/bin/env python3
"""
VicPinky YOLO 통합 노드
- NAV 모드: 사람/물체 감지 → Nav2 costmap obstacle 업데이트
- FOLLOW 모드: 사람 추종 → cmd_vel 직접 제어
- /robot_mode 토픽으로 모드 전환
"""
import math
import struct
import threading

import cv2
import numpy as np
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from ultralytics import YOLO

# ── 카메라 설정 ──────────────────────────────────────────────
CAM_WIDTH   = 640
CAM_HEIGHT  = 480
CAM_HFOV    = 60.0   # 웹캠 수평 화각 (도)

# ── 장애물로 처리할 YOLO 클래스 ─────────────────────────────
OBSTACLE_CLASSES = {
    'person':     0.8,   # 사람: 80cm 안전 반경
    'chair':      0.4,
    'bottle':     0.25,
    'cup':        0.25,
    'backpack':   0.35,
    'suitcase':   0.45,
    'bicycle':    0.5,
    'dog':        0.5,
    'cat':        0.4,
}

PILLAR_ANGLES_DEG = [162.0, 200.0, 245.3, 338.7, 23.3]
PILLAR_HALF_WIDTH = 10


class YoloNavNode(Node):
    def __init__(self):
        super().__init__('yolo_nav_node')

        # YOLO 모델
        self.model = YOLO('/home/hong/dev_ws/vic_pinky/yolov8n.pt')
        self.mode = 'nav'
        self.scan = None
        self.lock = threading.Lock()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # 구독
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_cb, sensor_qos)
        self.create_subscription(LaserScan, '/scan',        self.lidar_cb,  sensor_qos)
        self.create_subscription(String,    '/robot_mode',  self.mode_cb,   10)

        # 퍼블리셔
        self.cmd_pub      = self.create_publisher(Twist,       '/cmd_vel',           10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/camera_obstacles',  10)
        self.debug_pub    = self.create_publisher(Image,       '/yolo_debug',        10)

        # Nav2 goal 전체 취소 서비스 클라이언트
        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.get_logger().info('YoloNavNode 시작 — 모드: nav')

    # ── 콜백 ────────────────────────────────────────────────

    def lidar_cb(self, msg):
        with self.lock:
            self.scan = msg

    def mode_cb(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode == self.mode:
            return
        self.mode = new_mode
        self.get_logger().info(f'모드 전환: {self.mode}')

        if self.mode == 'follow':
            self._cancel_nav2_goals()
        elif self.mode == 'stop':
            self._cancel_nav2_goals()
            self.cmd_pub.publish(Twist())

    def _cancel_nav2_goals(self):
        if self._cancel_client.service_is_ready():
            req = CancelGoal.Request()  # 빈 요청 = 모든 goal 취소
            self._cancel_client.call_async(req)
            self.get_logger().info('Nav2 목표 전체 취소')
        self.cmd_pub.publish(Twist())

    def image_cb(self, msg):
        buf = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        results = self.model(frame, verbose=False, conf=0.45)[0]

        if self.mode == 'follow':
            self._follow(frame, results)
        else:
            self._nav_obstacles(frame, results)

        # 디버그 이미지 퍼블리시
        annotated = results.plot()
        mode_color = (0, 255, 0) if self.mode == 'nav' else (0, 100, 255)
        cv2.putText(annotated, f'MODE: {self.mode.upper()}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2)
        h, w, c = annotated.shape
        debug_msg = Image()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.height = h
        debug_msg.width = w
        debug_msg.encoding = 'bgr8'
        debug_msg.step = w * c
        debug_msg.data = annotated.tobytes()
        self.debug_pub.publish(debug_msg)

    # ── NAV 모드: 장애물 → PointCloud2 → Nav2 ────────────────

    def _nav_obstacles(self, frame, results):
        points = []

        for box in results.boxes:
            cls_name = self.model.names[int(box.cls[0])]
            if cls_name not in OBSTACLE_CLASSES:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) / 2.0
            width = x2 - x1

            dist = self._lidar_distance(cx, width)
            if dist is None or dist > 3.5:
                continue

            angle_rad = math.radians(
                (cx - CAM_WIDTH / 2.0) / (CAM_WIDTH / 2.0) * (CAM_HFOV / 2.0)
            )
            ox = dist * math.cos(angle_rad)
            oy = -dist * math.sin(angle_rad)

            radius = OBSTACLE_CLASSES[cls_name]
            for deg in range(0, 360, 20):
                px = ox + radius * math.cos(math.radians(deg))
                py = oy + radius * math.sin(math.radians(deg))
                points.append((px, py, 0.3))

        if points:
            self.obstacle_pub.publish(self._make_cloud(points))

    # ── FOLLOW 모드: 사람 추종 ────────────────────────────────

    def _follow(self, frame, results):
        twist = Twist()
        best_box, best_area = None, 0

        for box in results.boxes:
            if self.model.names[int(box.cls[0])] == 'person':
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area = area
                    best_box = (x1, y1, x2, y2)

        if best_box:
            x1, y1, x2, y2 = best_box
            cx = (x1 + x2) / 2.0
            dist = self._lidar_distance(cx, x2 - x1)

            # 회전 제어
            error = (cx - CAM_WIDTH / 2.0) / (CAM_WIDTH / 2.0)
            twist.angular.z = float(np.clip(-error * 1.0, -0.7, 0.7))

            # 직진 제어
            target = 0.8  # 목표 거리 (m)
            if dist is not None:
                gap = dist - target
                twist.linear.x = float(np.clip(gap * 0.6, -0.20, 0.40))
            elif best_area > 20000:
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.15
        else:
            # 타겟 없음 → 정지
            pass

        self.cmd_pub.publish(twist)

    # ── 라이다 거리 추정 ─────────────────────────────────────

    def _lidar_distance(self, bbox_cx_px, bbox_w_px):
        with self.lock:
            scan = self.scan
        if scan is None:
            return None

        angle_center = (bbox_cx_px - CAM_WIDTH / 2.0) / (CAM_WIDTH / 2.0) * (CAM_HFOV / 2.0)
        half_w = (bbox_w_px / CAM_WIDTH) * CAM_HFOV / 2.0 + 5.0

        a_min = math.degrees(scan.angle_min)
        a_inc = math.degrees(scan.angle_increment)

        distances = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue
            lidar_deg = a_min + i * a_inc
            if abs(lidar_deg - angle_center) > half_w:
                continue
            # 로봇 지지대(pillar) 각도 제외
            skip = False
            for pa in PILLAR_ANGLES_DEG:
                if abs(lidar_deg - pa) < PILLAR_HALF_WIDTH:
                    skip = True
                    break
            if not skip:
                distances.append(r)

        return min(distances) if distances else None

    # ── PointCloud2 생성 ─────────────────────────────────────

    def _make_cloud(self, points):
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'base_footprint'
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(points)
        cloud.is_dense = True
        data = bytearray()
        for p in points:
            data += struct.pack('fff', float(p[0]), float(p[1]), float(p[2]))
        cloud.data = bytes(data)
        return cloud


def main():
    rclpy.init()
    node = YoloNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
