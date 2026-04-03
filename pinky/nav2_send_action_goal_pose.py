#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def quaternion_from_yaw(yaw_rad: float):
    z = math.sin(yaw_rad / 2.0)
    w = math.cos(yaw_rad / 2.0)
    return z, w

class PinkyActionClient(Node):
    def __init__(self):
        super().__init__('pinky_action_client')
        
        # Action Client 생성 (배달 어플 켜기)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("배달 어플(Action Client) 준비 완료!")

    def send_goal(self, x: float, y: float, yaw_deg: float):
        self.get_logger().info('Nav2 주방이 열릴 때까지 기다리는 중...')
        self._action_client.wait_for_server()

        # 목표(주문서) 만들기
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w = quaternion_from_yaw(yaw_rad)

        self.get_logger().info(f'주문 전송 중... (x={x:.2f}, y={y:.2f}, 각도={yaw_deg:.1f}deg)')

        # 비동기로 주문 넣고, 피드백(중간 알림) 받을 콜백 연결
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2가 주문을 거절했습니다. (갈 수 없는 곳이거나 에러)')
            return

        self.get_logger().info('Nav2가 주문을 수락했습니다! 배달 시작!')
        
        # 최종 도착 결과를 기다림
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # 주행하는 동안 계속해서 들어오는 피드백
        feedback = feedback_msg.feedback
        self.get_logger().info(f'>> [피드백] 남은 거리: {feedback.distance_remaining:.2f} m')

    def get_result_callback(self, future):
        # 최종 결과
        status = future.result().status
        if status == 4:
            self.get_logger().info('🎉 [결과] 목표 지점에 성공적으로 도착했습니다!')
        elif status == 6:
            self.get_logger().error('💥 [결과] 주행에 실패했습니다. (장애물 등에 막힘)')
        else:
            self.get_logger().info(f'[결과] 알 수 없는 상태 (코드: {status})')
        
        rclpy.shutdown()

def main(args=None):
    if len(sys.argv) != 4:
        print("사용법: python3 pinky_action_client.py <x> <y> <yaw_deg>")
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw_deg = float(sys.argv[3])
    except ValueError:
        print("에러: x, y, yaw_deg는 숫자여야 합니다.")
        return

    rclpy.init(args=args)
    action_client = PinkyActionClient()
    
    action_client.send_goal(x, y, yaw_deg)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()