import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

SCAN_SAMPLES = 30       # 몇 프레임 모을지
PILLAR_DIST  = 0.60     # 이 거리 이하면 프로파일러 후보 (m)
CLUSTER_GAP  = 8        # 같은 덩어리로 묶을 인덱스 간격

class PillarFinder(Node):
    def __init__(self):
        super().__init__('pillar_finder')
        self._samples = []
        self._angle_min = 0.0
        self._angle_inc = 0.0
        self.sub = self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self.get_logger().info(f"라이다 스캔 수집 중... ({SCAN_SAMPLES}프레임 모으면 자동 분석)")

    def _cb(self, msg):
        if len(self._samples) == 0:
            self._angle_min = msg.angle_min
            self._angle_inc = msg.angle_increment

        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isinf(ranges)] = 999.0
        ranges[np.isnan(ranges)] = 999.0
        self._samples.append(ranges)

        print(f"  수집: {len(self._samples)}/{SCAN_SAMPLES}", end='\r')

        if len(self._samples) >= SCAN_SAMPLES:
            self._analyze()
            rclpy.shutdown()

    def _analyze(self):
        data = np.stack(self._samples)          # (N, num_points)
        mean_dist = np.mean(data, axis=0)       # 각 인덱스별 평균 거리

        # 1) PILLAR_DIST 이하로 항상 잡히는 인덱스 추출
        close_idx = np.where(mean_dist < PILLAR_DIST)[0]

        if len(close_idx) == 0:
            print(f"\n[결과] {PILLAR_DIST}m 이내 항상 잡히는 각도 없음 — PILLAR_DIST 값을 올려보세요")
            return

        # 2) 연속된 인덱스끼리 클러스터링
        clusters = []
        cluster = [close_idx[0]]
        for i in close_idx[1:]:
            if i - cluster[-1] <= CLUSTER_GAP:
                cluster.append(i)
            else:
                clusters.append(cluster)
                cluster = [i]
        clusters.append(cluster)

        print(f"\n\n{'='*55}")
        print(f" 발견된 고정 물체(프로파일러 후보): {len(clusters)}개")
        print(f"{'='*55}")
        print(f" {'#':>2}  {'중심각도':>8}  {'평균거리':>8}  {'인덱스범위'}")
        print(f"{'-'*55}")

        pillar_angles = []
        for i, cl in enumerate(clusters):
            center_idx  = int(np.mean(cl))
            angle_rad   = self._angle_min + center_idx * self._angle_inc
            angle_deg   = math.degrees(angle_rad) % 360
            dist        = mean_dist[center_idx]
            idx_range   = f"{cl[0]}~{cl[-1]}"
            pillar_angles.append(angle_deg)
            print(f" {i+1:>2}  {angle_deg:>7.1f}°  {dist:>7.2f}m  {idx_range}")

        print(f"{'='*55}")
        print(f"\n follower_udp.py 에 넣을 제외 각도 리스트:")
        print(f"  PILLAR_ANGLES_DEG = {[round(a,1) for a in pillar_angles]}")
        print(f"  PILLAR_HALF_WIDTH = 8  # ±8도 범위 제외\n")


def main():
    rclpy.init()
    node = PillarFinder()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
