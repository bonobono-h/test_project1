#!/bin/bash
export ROS_DOMAIN_ID=28
export FASTRTPS_DEFAULT_PROFILES_FILE=~/dev_ws/vic_pinky/fastdds_vicpinky.xml

python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/yolo_debug', self.cb, 10)
        self.get_logger().info('카메라 뷰어 시작 — 종료: q키')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('VicPinky YOLO', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            raise SystemExit

rclpy.init()
node = Viewer()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, SystemExit):
    pass
cv2.destroyAllWindows()
EOF
