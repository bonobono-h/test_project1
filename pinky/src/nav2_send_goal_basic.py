#!/usr/bin/env python3
import math
import time
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quaternion_from_yaw(yaw_rad: float):      #로스는 쿼터니언 값을 쓰는데 yaw 값을 포터니언으로 변환
    """yaw(rad) -> (z, w) 쿼터니언 변환"""
    z = math.sin(yaw_rad / 2.0)          #좀더 간단하게 변환하는 방법 
    w = math.cos(yaw_rad / 2.0)
    return z, w


def main():              #자이제 시작~~~ 인지는 시작할때 값 3개를 받을꺼 x,y ywa_deg =요 디그리
    # -----------------------------
    # CLI 인자 파싱
    # python3 nav2_send_goal_basic.py x y yaw_deg 
    #아래가 4인데 왜 4냐면 0번이 파일명이라서 3개를 입력받아야 하는데 4개가 되는거임
    # -----------------------------
    if len(sys.argv) != 4:
        print("Usage: python3 nav2_send_goal_basic.py <x> <y> <yaw_deg>")
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw_deg = float(sys.argv[3])
    except ValueError:
        print("Error: x, y, yaw_deg must be numbers")
        return

    # ROS2 초기화
    rclpy.init(args=None)

    # Nav2 네비게이터 생성
    navigator = BasicNavigator()  #이거하는것만으로도 액션클라이언트가 사용할준비를함 

    ## 홈위치 강제지정 

    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    print("[Initial Pose] 홈 위치로 설정")
    
    navigator.setInitialPose(initial_pose)

    print("[Nav2] 활성화 대기 중...")  #프린트는 리소스 비용이 많이듬 주기가 달라진다?? #로스에서는 로버를 씀
    navigator.waitUntilNav2Active()  
    print("[Nav2] Active 상태입니다.")

    # goal pose 생성
    goal_pose = PoseStamped()  #타임스템프랑 비슷함? pose +timestamp = poseStamped 
    #요시점의 로봇의 위치와 자세를 나타내는 메시지임  goal_pose는 목표위치와 자세를 나타냄
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg() #시간 정보넣기 os상의 시간이아님

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y

    yaw_rad = math.radians(yaw_deg)  #deg는 처리하려면 라디안으로 바꿔야함
    goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = quaternion_from_yaw(
        yaw_rad
    )

    print(f"[Goal] ({x:.2f}, {y:.2f}, {yaw_deg:.1f}deg) 로 이동 시작")
    navigator.goToPose(goal_pose)

    # 진행 모니터링
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:  #이게 방어코드  값이없는데 뭐하라하면 꺼져라고함 
            print(
                f"[Feedback] 남은 거리: {feedback.distance_remaining:.2f} m, "
                f"경과 시간: {feedback.navigation_time.sec} s"
            )
        time.sleep(0.5)
        #그위에껀 0.5초마다 피드백을 보여주는거임 0.5초마다 남은거리랑 경과시간을 보여주는거임
        #이코드가 로고?를 찍는건가 ??

    # 최종 결과
    result = navigator.getResult()    #항상 결과는 성공, 취소, 실패 ,unknown가지중 하나로 나옴
    if result == TaskResult.SUCCEEDED:
        print("[Result] 성공적으로 도착했습니다.")
    elif result == TaskResult.CANCELED:
        print("[Result] Goal이 취소되었습니다.")
    elif result == TaskResult.FAILED:
        print("[Result] Goal 수행에 실패했습니다.")
    else:
        print("[Result] 알 수 없는 상태:", result)

    rclpy.shutdown()

    #여기서 종료하지말고 다시 초기화해서 또 다른 목표로 이동할수도있음  rclpy.shutdown()은 로스2를 종료하는거임
    #그럼 바꾸려면 return으로 바꿔서 다시 초기화하는걸로 바꿔야할듯?  rclpy.init()도 다시해야할듯?  rclpy.shutdown()은 로스2를 종료하는거임

if __name__ == "__main__":
    main()
