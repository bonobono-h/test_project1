#!/usr/bin/env python3
"""
VicPinky 알루미늄 프로파일 기둥 마스킹 노드
- 입력: /scan_filtered (기존 필터된 라이다 데이터)
- 출력: /scan_filtered_safe (4개 코너 기둥 마스킹)
- Nav2 costmap은 /scan_filtered_safe를 사용

기둥 위치 (base_footprint 기준):
  앞 좌 (+0.10, +0.20), 앞 우 (+0.10, -0.20)
  뒤 좌 (-0.50, +0.20), 뒤 우 (-0.50, -0.20)

라이다 위치: base +0.185, yaw=π (180° 회전)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

# 라이다 frame에서 본 4개 기둥의 각도(도) — yaw=π 변환 후
# 라이다 0°가 로봇 뒤(-X), 90°가 좌(+Y), 180°가 앞(+X)
PILLAR_ANGLES_DEG = [16.0, -16.0, 67.0, -67.0]  # [뒤우, 뒤좌, 앞우, 앞좌]
PILLAR_HALF_WIDTH_DEG = 6.0   # ±6° 마스킹 폭
PILLAR_MIN_DIST = 0.10        # 마스킹 거리 하한 (m)
PILLAR_MAX_DIST = 0.80        # 마스킹 거리 상한 (m) — 가장 먼 기둥 0.71m + 마진


class PillarFilterNode(Node):
    def __init__(self):
        super().__init__('pillar_filter_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan_filtered', self.scan_cb, sensor_qos)
        self.pub = self.create_publisher(
            LaserScan, '/scan_filtered_safe', sensor_qos)

        self.get_logger().info(
            f'Pillar filter: {len(PILLAR_ANGLES_DEG)}개 기둥 마스킹 '
            f'(±{PILLAR_HALF_WIDTH_DEG}°, {PILLAR_MIN_DIST}~{PILLAR_MAX_DIST}m)'
        )

    def scan_cb(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        a_min_deg = math.degrees(msg.angle_min)
        a_inc_deg = math.degrees(msg.angle_increment)

        ranges = list(msg.ranges)
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if not (PILLAR_MIN_DIST <= r <= PILLAR_MAX_DIST):
                continue
            angle_deg = a_min_deg + i * a_inc_deg
            # 정규화: -180 ~ +180
            angle_deg = ((angle_deg + 180.0) % 360.0) - 180.0
            for pa in PILLAR_ANGLES_DEG:
                if abs(angle_deg - pa) <= PILLAR_HALF_WIDTH_DEG:
                    ranges[i] = float('inf')
                    break

        out.ranges = ranges
        out.intensities = list(msg.intensities) if msg.intensities else []
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PillarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
