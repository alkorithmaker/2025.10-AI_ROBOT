import sys
import rclpy
import random
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # 위치 수신용 추가
from my_project_interfaces.action import Summon

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QColor, QPen
from std_srvs.srv import Trigger

try:
    from .qt_ui import Ui_MainWindow
except ImportError:
    from qt_ui import Ui_MainWindow

class SummonNode(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'summon_gui_node')
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.state_cli = self.create_client(Trigger, 'get_robot_state')

        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.target_x, self.target_y = 0.0, 0.0
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.summon_ac = ActionClient(self, Summon, 'summon_robot')
        self._goal_handle = None

        # 버튼 연결
        self.ui.GO.clicked.connect(self.move_forward)
        self.ui.STOP.clicked.connect(self.stop_robot)
        self.ui.BACK.clicked.connect(self.move_backward)
        self.ui.LEFT.clicked.connect(self.turn_left)
        self.ui.RIGHT.clicked.connect(self.turn_right)

        self.ui.AUTO.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.TO_PAGE_1.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))

        self.ui.STATE.clicked.connect(self.call_state_service)
        self.ui.START.clicked.connect(self.execute_summon)
        self.ui.CANCEL.clicked.connect(self.cancel_summon)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(10)

    # ----------- 시각화 로직 -----------
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.update() # 화면 갱신 트리거

    def get_pixel_pos(self, x, y):
      frame = self.ui.frame
      px = (x +7) * (frame.width() / 25)  # (-10m ~ 10m 범위)
      py = (12 - y) * (frame.height() / 25) # (10m ~ -10m 범위)

      return int(px + frame.x()), int(py + frame.y())

    def paintEvent(self, event):
        super().paintEvent(event)
        if self.ui.stackedWidget.currentIndex() != 1: return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        # #painter.setClipRect(self.ui.frame.geometry())
        # painter.setClipRect(-20, -20, self.ui.frame.width() + 40, self.ui.frame.height()+40)
        rect = self.ui.frame.geometry()
        painter.setClipRect(rect.x()-50, rect.y()-50, rect.width(), rect.height() + 100)
        # 목적지 그리기
        tx, ty = self.get_pixel_pos(self.target_x, self.target_y)
        painter.setBrush(QColor(255, 0, 0))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(tx - 7, ty - 7, 14, 14)

        # 터틀봇 그리기
        rx, ry = self.get_pixel_pos(self.current_x, self.current_y)
        painter.setBrush(QColor(0, 150, 255))
        painter.setPen(QPen(Qt.black, 1))
        painter.drawEllipse(rx - 15, ry - 15, 30, 30)

        # 방향선 그리기
        front_x = rx + 18 * math.cos(self.current_yaw)
        front_y = ry - 18 * math.sin(self.current_yaw)
        painter.setPen(QPen(Qt.yellow, 3))
        painter.drawLine(rx, ry, int(front_x), int(front_y))
    #-----------------------------------------------------------
  
    def call_state_service(self):
        if not self.state_cli.wait_for_service(timeout_sec=1.0):
            self.ui.LABEL.setText("서버 연결 실패")
            return

        req = Trigger.Request()
        future = self.state_cli.call_async(req)
        future.add_done_callback(self.state_response_callback)

    # 서비스 응답 처리 함수
    def state_response_callback(self, future):
        try:
            response = future.result()
            # LABEL에 상태 업데이트
            self.ui.LABEL.setText(response.message)
        except Exception as e:
            self.ui.LABEL.setText("에러 발생")
          
    # 이동 처리 함수
    def ros_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

    def move_forward(self):
        msg = Twist(); msg.linear.x = 0.2
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def move_backward(self):
        msg = Twist(); msg.linear.x = -0.2
        self.cmd_vel_pub.publish(msg)

    def turn_left(self):
        msg = Twist(); msg.angular.z = 0.6
        self.cmd_vel_pub.publish(msg)

    def turn_right(self):
        msg = Twist(); msg.angular.z = -0.6
        self.cmd_vel_pub.publish(msg)

    def execute_summon(self):
        if not self.summon_ac.wait_for_server(timeout_sec=1.0):
            self.statusBar().showMessage("서버 응답 없음")
            return

        self.ui.START.setEnabled(False)
        
        # 목표 설정 
        goal_msg = Summon.Goal()
        goal_msg.x = random.uniform(-9.5, 9.5)
        goal_msg.y = random.uniform(-9.5, 9.5)

        # 시각화용 타겟 좌표 업데이트
        self.target_x, self.target_y = goal_msg.x, goal_msg.y

        self._send_goal_future = self.summon_ac.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # 목표 전송 시에는 좌표만 먼저 표시
        self.statusBar().showMessage(f"목표 설정: ({goal_msg.x:.1f}, {goal_msg.y:.1f}) 이동 시작!")


    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.statusBar().showMessage("목표 거절됨")
            # 거절되면 다시 버튼 활성화
            self.ui.START.setEnabled(True)
            return

        # 목표 수락 시 결과 콜백 연결
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 알고리즘 종료시 버튼 다시 활성화
        self.ui.START.setEnabled(True)
        self.statusBar().showMessage("이동 완료")


    def cancel_summon(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self.stop_robot()
        self.ui.START.setEnabled(True)
        self.statusBar().showMessage("소환 중단됨")

    def feedback_callback(self, feedback_msg):
         dist = feedback_msg.feedback.distance_remaining
         self.statusBar().showMessage(f"({self.target_x:.1f}, {self.target_y:.1f}) 남은 거리: {dist:.1f}m")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = SummonNode()
    node.show()
    exit_code = app.exec()
    node.ros_timer.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
