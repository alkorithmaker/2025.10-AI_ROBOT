import sys
from PyQt6.QtWidgets import QApplication

from pyqt import WidgetSample
from robot import RobotSystem
from camera import CameraSystem


if __name__ == "__main__":
    app = QApplication(sys.argv)

    robot = RobotSystem()
    robot.start_threads()

    # ✅ 카메라(OpenCV) 스레드 시작
    # show=True면 imshow 디버그 창 표시, False면 표시 안 함(부담↓)
    camera = CameraSystem(robot=robot, show=True)

    # GUI 생성 및 로봇 연결
    gui = WidgetSample(robot)
    gui.show()

    # GUI 종료 시 전체 종료
    exit_code = app.exec()
    robot.is_running = False
    camera.stop()
    sys.exit(exit_code)
