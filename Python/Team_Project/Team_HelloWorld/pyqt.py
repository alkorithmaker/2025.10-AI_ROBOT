#pyqt.py
#---------------------------------------------
#- main.ui를 로드해서 GUI를 띄우는 코드
#- UI에 이미 있는 라벨 2개를 사용
#- 속도 라벨 objectName: "Speed" - 도형 라벨 objectName: "Shape"
#- robot.py (RobotSystem) 쪽에서 아래 값이 계속 갱신되면 속도/모드가 뜸
#- 카메라/AI 코드에서 아래 값을 갱신하면 도형이 뜸
#---------------------------------------------

import sys, os
from PyQt6 import uic
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton


BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class WidgetSample(QWidget):
    def __init__(self, robot_instance):
        super().__init__()
        self.robot = robot_instance

        # UI 먼저 로드해야 findChild/Speed/Shape가 생김
        uic.loadUi(os.path.join(BASE_DIR, "main.ui"), self)

        # ---------------- 버튼 -> 명령 매핑 ----------------
        self.mapping = {
            "left":   "LEFT",
            "right":  "RIGHT",
            "go":     "GO",
            "back":   "BACK",
            "stop":   "STOP",
            "auto1":  "AUTO",
            "manual": "MANUAL",
        }

        # ---------------- 누르고 있는 동안 반복 전송용 ----------------
        self.hz = 20
        self.send_timer = QTimer(self)                 # 이름 변경(충돌 방지)
        self.send_timer.setInterval(int(1000 / self.hz))
        self.send_timer.timeout.connect(self._tick_send)
        self.active_cmd = None

        # "누르고 있는 동안 계속" 할 버튼들
        hold_buttons = ["left", "right", "go", "back"]
        for obj_name in hold_buttons:
            btn = self.findChild(QPushButton, obj_name)
            if not btn:
                continue
            cmd = self.mapping[obj_name]
            btn.pressed.connect(lambda c=cmd: self._start_hold(c))
            btn.released.connect(self._stop_hold)

        # STOP 버튼: 즉시 STOP 1번
        stop_btn = self.findChild(QPushButton, "stop")
        if stop_btn:
            stop_btn.clicked.connect(self._stop_now)

        # AUTO / MANUAL: 클릭 1번 전송
        for obj_name in ["auto1", "manual"]:
            btn = self.findChild(QPushButton, obj_name)
            if not btn:
                continue
            cmd = self.mapping[obj_name]
            btn.clicked.connect(lambda checked=False, c=cmd: self.send_command(c))

        # ---------------- 상태 갱신 타이머(10Hz) ----------------
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(100)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start()

        self.show()

    # ---------------- 상태 표시 갱신 ----------------
    def _refresh_status(self):
        mode = getattr(self.robot, "control_mode", "-")
        lin = float(getattr(self.robot, "current_lin_vel", 0.0))
        ang = float(getattr(self.robot, "current_ang_vel", 0.0))
        shape = getattr(self.robot, "current_shape", "-")

        # UI 라벨 이름: Speed / Shape
        if hasattr(self, "Speed"):
            self.Speed.setText(f"Speed : {lin:.2f}  ang:{ang:.2f}  mode:{mode}")
        if hasattr(self, "Shape"):
            self.Shape.setText(f"Shape: {shape}")

    # ---------------- 반복 전송 로직 ----------------
    def _start_hold(self, cmd: str):
        self.active_cmd = cmd
        if not self.send_timer.isActive():
            self.send_timer.start()
        self.send_command(cmd)

    def _stop_hold(self):
        self.send_timer.stop()
        self.active_cmd = None

    def _stop_now(self):
        self.send_timer.stop()
        self.active_cmd = None
        self.send_command("STOP")

    def _tick_send(self):
        if self.active_cmd is not None:
            self.send_command(self.active_cmd)

    # ---------------- 실제 전송 함수 ----------------
    def send_command(self, cmd: str):
        print("GUI", cmd)
        if getattr(self.robot, "command_queue", None) is not None:
            self.robot.command_queue.put({'source': 'GUI', 'cmd': cmd})


# ===================== 단독 실행용 =====================
if __name__ == "__main__":
    import queue

    class DummyRobot:
        """RobotSystem 없이도 GUI만 테스트하려고 만든 더미 로봇"""
        def __init__(self):
            self.command_queue = queue.Queue()

            self.control_mode = "AUTO"
            self.current_lin_vel = 0.08
            self.current_ang_vel = 0.00
            self.current_shape = "TRIANGLE"

            # 단독 실행에서도 GO/LEFT 같은 걸 누르면 값이 바뀌게 step/limit 넣음
            self.lin_step = 0.005
            self.ang_step = 0.02
            self.MAX_LIN = 0.15
            self.MAX_ANG = 1.0

        def clamp(self):
            self.current_lin_vel = max(-self.MAX_LIN, min(self.MAX_LIN, self.current_lin_vel))
            self.current_ang_vel = max(-self.MAX_ANG, min(self.MAX_ANG, self.current_ang_vel))

        def apply_cmd(self, cmd: str):
            if cmd == "GO":
                self.current_lin_vel += self.lin_step
            elif cmd == "BACK":
                self.current_lin_vel -= self.lin_step
            elif cmd == "LEFT":
                self.current_ang_vel += self.ang_step
            elif cmd == "RIGHT":
                self.current_ang_vel -= self.ang_step
            elif cmd == "STOP":
                self.current_lin_vel = 0.0
                self.current_ang_vel = 0.0
            self.clamp()

    robot = DummyRobot()

    app = QApplication(sys.argv)
    w = WidgetSample(robot)

    # 도형 데모(테스트용)
    demo_shapes = ["TRIANGLE", "SQUARE", "CIRCLE"]
    idx = {"i": 0}

    def demo_change_shape():
        idx["i"] = (idx["i"] + 1) % len(demo_shapes)
        robot.current_shape = demo_shapes[idx["i"]]

    demo_timer = QTimer()
    demo_timer.setInterval(1000)
    demo_timer.timeout.connect(demo_change_shape)
    demo_timer.start()

    # 큐에 들어오는 명령 처리
    def poll_queue():
        while not robot.command_queue.empty():
            msg = robot.command_queue.get()
            cmd = msg.get("cmd", "").upper()
            print("[DUMMY ROBOT] got:", msg)

            # 모드 변경
            if cmd in ["AUTO", "MANUAL"]:
                robot.control_mode = cmd
            else:
                # 단독 실행에서도 버튼 누르면 속도/각속도 값이 실제로 바뀜
                robot.apply_cmd(cmd)

    t = QTimer()
    t.setInterval(20)     # 50ms도 되지만 20ms가 더 부드러움
    t.timeout.connect(poll_queue)
    t.start()

    sys.exit(app.exec())
