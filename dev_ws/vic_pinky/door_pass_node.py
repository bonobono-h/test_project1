#!/usr/bin/env python3
"""
좁은 통로(문) 통과 시 angular.z 감쇠 필터
cmd_vel_collision_out → (필터) → cmd_vel

라이다 좌표계: 0°=뒤, 90°=좌, 180°=앞, -90°=우
"""
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LEFT_MIN_DEG  =  60.0
LEFT_MAX_DEG  = 120.0
RIGHT_MIN_DEG = -120.0
RIGHT_MAX_DEG =  -60.0

PASSAGE_THRESH  = 0.65  # 좌우 모두 이 거리 이내 → 통로 진입
CLEAR_THRESH    = 0.85  # 좌우 모두 이 거리 이상 → 통로 탈출
ANGULAR_DAMPING = 0.25  # 통로 안에서 angular.z 감쇠 비율 (0=완전차단, 1=그대로)


class DoorPassNode(Node):
    def __init__(self):
        super().__init__('door_pass_node')
        self._lock = threading.Lock()
        self._in_passage = False
        self._mode = 'nav'

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb, sensor_qos)
        self.create_subscription(Twist,  '/cmd_vel_collision_out', self._cmd_cb, 10)
        self.create_subscription(String, '/robot_mode', self._mode_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('DoorPassNode 시작')

    def _mode_cb(self, msg: String):
        with self._lock:
            self._mode = msg.data.strip().lower()
            if self._mode != 'nav':
                self._in_passage = False  # 모드 바뀌면 상태 초기화

    def _scan_cb(self, msg: LaserScan):
        a_min = math.degrees(msg.angle_min)
        a_inc = math.degrees(msg.angle_increment)

        left_vals, right_vals = [], []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            deg = a_min + i * a_inc
            # -180 ~ +180 정규화
            deg = ((deg + 180.0) % 360.0) - 180.0
            if LEFT_MIN_DEG <= deg <= LEFT_MAX_DEG:
                left_vals.append(r)
            elif RIGHT_MIN_DEG <= deg <= RIGHT_MAX_DEG:
                right_vals.append(r)

        left  = min(left_vals)  if left_vals  else 9.9
        right = min(right_vals) if right_vals else 9.9

        with self._lock:
            if not self._in_passage:
                # 진입: 좌우 모두 막혀야
                if left < PASSAGE_THRESH and right < PASSAGE_THRESH:
                    self._in_passage = True
                    self.get_logger().info(
                        f'통로 진입 — 좌:{left:.2f}m 우:{right:.2f}m')
            else:
                # 탈출: 좌우 모두 열려야
                if left > CLEAR_THRESH and right > CLEAR_THRESH:
                    self._in_passage = False
                    self.get_logger().info(
                        f'통로 탈출 — 좌:{left:.2f}m 우:{right:.2f}m')

    def _cmd_cb(self, msg: Twist):
        with self._lock:
            in_passage = self._in_passage
            mode = self._mode

        # NAV 모드 + 통로 안일 때만 감쇠
        if mode == 'nav' and in_passage:
            msg.angular.z *= ANGULAR_DAMPING

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DoorPassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
