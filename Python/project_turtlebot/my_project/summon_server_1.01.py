import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from my_project_interfaces.action import Summon
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy # QoS 설정을 위해 추가

import math
import time
import random

# -------------------------------------
# 현실의 터틀봇 속도 고려
# -------------------------------------

class SummonServer(Node):
    def __init__(self):
        super().__init__('summon_server')
        self._action_server = ActionServer(self, Summon, 'summon_robot', 
                                           self.execute_callback,
                                           cancel_callback=self.cancel_callback)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 실제 로봇의 라이다 데이터를 받기 위해 BEST_EFFORT 설정 적용
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile)
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.robot_state = "대기 중"
        self.srv = self.create_service(Trigger, 'get_robot_state', self.get_state_callback)

        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.scan_ranges = []
        self.avoiding = False
        self.avoid_dir = 0.0
        self.safety_timer = 0.0

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
        self.get_logger().info('작업이 취소되었습니다.')
        return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_x, target_y = goal_handle.request.x, goal_handle.request.y
        self.robot_state = "목적지로 출발"
        self.get_logger().info(f'목적지 ({target_x:.2f}, {target_y:.2f})로 출발합니다.')
        
        cmd = Twist()
        feedback = Summon.Feedback()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if goal_handle.is_cancel_requested:
                self.publisher.publish(Twist())
                goal_handle.canceled()
                self.robot_state = "대기 중"
                return Summon.Result(success=False)

            # 라이다 데이터가 아직 안 들어왔으면 일단 진행 (기본값 설정)
            if not self.scan_ranges or len(self.scan_ranges) < 360:
                time.sleep(0.1)
                continue

            dist = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            # 목적지 도착 판단 거리를 약간 여유 있게 (버거 회전 반경 고려)
            if dist < 0.20:
                self.robot_state = "목적지 도착!"
                break

            angle_to_goal = math.atan2(target_y - self.current_y, target_x - self.current_x)
            diff = math.atan2(math.sin(angle_to_goal - self.current_yaw), math.cos(angle_to_goal - self.current_yaw))

            # 인덱스 유효성 검사 추가
            front_indices = list(range(0, 25)) + list(range(335, 360))
            valid_scan = [self.scan_ranges[i] for i in front_indices if i < len(self.scan_ranges) and self.scan_ranges[i] > 0.01]
            front_dist = min(valid_scan) if valid_scan else 10.0
            
            now = self.get_clock().now().nanoseconds / 1e9

            # 장애물 감지 수치 조정 (버거 크기에 맞춰 0.3m로 축소)
            if front_dist < 0.30:
                self.robot_state = "장애물 회피 중"
                if not self.avoiding:
                    self.avoiding = True
                    self.avoid_dir = 1.5 if random.random() > 0.5 else -1.5 # 회전 속도 최적화

                cmd.linear.x = 0.02 # 회피 시 아주 천천히
                cmd.angular.z = self.avoid_dir
                self.safety_timer = now + 0.5
            
            elif now < self.safety_timer:
                cmd.linear.x = 0.1 # 탈출 시 속도 제한 (버거 최대치 고려)
                cmd.angular.z = 0.0
                self.avoiding = False
            
            else:
                self.avoiding = False
                # 주행 속도를 버거 사양(최대 0.22)에 맞춰 조정
                if abs(diff) < 0.15: 
                    cmd.linear.x = 0.18 # 최대 속도 근접
                    cmd.angular.z = 0.0
                elif abs(diff) < 0.4:
                    cmd.linear.x = 0.12
                    cmd.angular.z = 1.0 * diff
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 1.5 if diff > 0 else -1.5

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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
