import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from my_project_interfaces.action import Summon
from std_srvs.srv import Trigger

import math
import time
import random

class SummonServer(Node):
    def __init__(self):
        super().__init__('summon_server')
        self._action_server = ActionServer(self, Summon, 'summon_robot', self.execute_callback,cancel_callback=self.cancel_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # 서비스 상태
        self.robot_state = "대기 중"
        self.srv = self.create_service(Trigger, 'get_robot_state', self.get_state_callback)

        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.scan_ranges = []

        self.avoiding = False
        self.avoid_dir = 0.0
        self.safety_timer = 0.0

    # 서비스 함수
    def get_state_callback(self, request, response):
        response.success = True
        response.message = self.robot_state
        return response

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    def cancel_callback(self, goal_handle):
      self.robot_state = "대기 중"
      self.get_logger().info(' 작업이 취소되었습니다. ')
      return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_x, target_y = goal_handle.request.x, goal_handle.request.y
        self.robot_state = "목적지로 출발"
        self.get_logger().info(' 목적지로 출발하겠습니다. ')
        cmd = Twist()
        feedback = Summon.Feedback()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if goal_handle.is_cancel_requested:
                self.publisher.publish(Twist())
                goal_handle.canceled()
                self.robot_state = "대기 중"
                self.get_logger().info(' 작업이 취소되었습니다. ')
                return Summon.Result(success=False)

            if not self.scan_ranges:
                time.sleep(0.05)
                continue

            # 거리 계산 후 피드백을 보냄
            dist = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            # 목적지 도착 (너무 작으면 빙빙 돌음)
            if dist < 0.22:
                self.robot_state = "목적지 도착!"
                self.get_logger().info(' 목적지에 도착하였습니다. ')

                break

            angle_to_goal = math.atan2(target_y - self.current_y, target_x - self.current_x)
            diff = math.atan2(math.sin(angle_to_goal - self.current_yaw), math.cos(angle_to_goal - self.current_yaw))

            # 1. 정면 40도 (직진 결정)
            front_indices = list(range(0, 20)) + list(range(340, 360))
            # 2. 측면 약간 20~40도
            side_indices = list(range(20, 40)) + list(range(320, 340))

            valid_front = [self.scan_ranges[i] for i in front_indices if i < len(self.scan_ranges) and self.scan_ranges[i] > 0.02]
            valid_side = [self.scan_ranges[i] for i in side_indices if i < len(self.scan_ranges) and self.scan_ranges[i] > 0.02]

            front_dist = min(valid_front) if valid_front else 10.0
            side_dist = min(valid_side) if valid_side else 10.0
            now = self.get_clock().now().nanoseconds / 1e9


            # 장애물 감지 후 회피
            if front_dist < 0.40 or side_dist < 0.25:
                self.robot_state = "장애물 회피 중..."
                if not self.avoiding:
                    self.avoiding = True
                    # 회피 방향을 정하기
                    left_side = min(self.scan_ranges[0:60])
                    right_side = min(self.scan_ranges[300:360])

                    # 무한 회피 방지 꺾는 각도 랜덤
                    rand_rot = random.uniform(1.0, 2.0)
                    if left_side > right_side:
                        self.avoid_dir = rand_rot
                    else:
                        self.avoid_dir = -rand_rot


                cmd.linear.x = 0.07
                cmd.angular.z = self.avoid_dir

                self.safety_timer = now + random.uniform(0.4, 0.8)

            # 장애물 회피 후 가속 탈출
            elif now < self.safety_timer:
                if not self.avoiding:
                    self.robot_state = "장애물 통과 중..."
                    # 무한 반복을 피하기 위해 랜덤값 설정
                    cmd.linear.x = random.uniform(0.3, 0.7)

                cmd.angular.z = 0.0
                self.avoiding = False
                # self.get_logger().info(f' 랜덤 속도: {cmd.linear.x:.2f}') # check
            # 일반 고속 주행
            else:
              self.robot_state = "정상 주행 중.."
              self.avoiding = False
        
              if abs(diff) < 0.1:        # 거의 정면
                  cmd.linear.x = 0.5
                  cmd.angular.z = 0.0
              elif abs(diff) < 0.2:      # 아주 살짝 틀어짐
                  cmd.linear.x = 0.4
                  cmd.angular.z = 1.2 * diff
              elif abs(diff) < 0.4:      # 약간 틀어짐
                  cmd.linear.x = 0.3
                  cmd.angular.z = 1.3 * diff
              elif abs(diff) < 0.6:      # 꽤 틀어짐
                  cmd.linear.x = 0.2
                  cmd.angular.z = 1.4 * diff
              elif abs(diff) < 0.8:      # 많이 틀어짐
                  cmd.linear.x = 0.1
                  cmd.angular.z = 1.5 * diff
              else:                      # 목표가 옆이나 뒤에 있음
                  cmd.linear.x = 0.0
                  cmd.angular.z = 1.7 if diff > 0 else -1.7

            self.publisher.publish(cmd)
            time.sleep(0.05)

        self.publisher.publish(Twist())
        goal_handle.succeed()
        return Summon.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = SummonServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('서버가 사용자에 의해 종료되었습니다.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
