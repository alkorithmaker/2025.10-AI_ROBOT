# robot.py
# -----------------------------------------
# 역할:
# - 로봇 제어의 "두뇌"
# - GUI / OpenCV(Camera) 등에서 들어온 명령을 해석
# - 현재 속도 상태를 관리
# - 모드 전환 처리
# - 최종 속도 값을 link.py를 통해 로봇(/cmd_vel)로 전송
# -----------------------------------------

from __future__ import annotations

import time
import threading
from queue import Queue

from link import RosbridgePublisher


class RobotSystem:
    def __init__(self):
        # -------------------------------
        # 시스템 상태
        # -------------------------------
        self.is_running = True
        self.control_mode = "MANUAL"  # "MANUAL" / "AUTO"

        # 외부 입력(GUI, Camera 등)이 들어오는 큐
        self.command_queue = Queue()

        # 현재 로봇 속도 상태
        self.current_lin_vel = 0.0
        self.current_ang_vel = 0.0

        # (GUI용) 도형 상태
        self.current_shape = "NONE"

        # -------------------------------
        # MANUAL 제어 설정
        # -------------------------------
        self.lin_step = 0.005
        self.ang_step = 0.02
        self.manual_idle_decel_delay = 0.20  # 초: 이 시간 내에는 입력 없어도 속도 유지
        self._last_manual_input_time = 0.0

        # -------------------------------
        # 속도 제한 (하드웨어 보호)
        # TurtleBot3 Burger 기준으로 안전한 범위로 설정
        # -------------------------------
        self.MAX_LIN = 0.22
        self.MAX_ANG = 2.84

        # 입력 없을 때 감속 비율(부드럽게)
        self.lin_decel_rate = 0.010
        self.ang_decel_rate = 0.100

        # 미세한 잔속도(드리프트) 제거용 데드밴드
        self.DEADBAND_LIN = 0.003
        self.DEADBAND_ANG = 0.02

        # -------------------------------
        # AUTO 제어 설정
        # - ✅ 직진 속도는 "항상 고정" (요청: 0.08)
        # - ✅ 각속도는 error에 비례(P제어)
        # -------------------------------
        self.auto_lin_speed = 0.1    # ✅ 원하는 고정 직진 속도
        self.auto_kp = 2.0            # err_norm(약 -1~+1) -> ang_z (rad/s)
        self.auto_deadzone_ratio = 0.02  # 정중앙 근처 데드존(비율)

        # -------------------------------
        # SHAPE 행동(오버라이드) 상태
        # - SHAPE_ACTION을 받으면 일정 시간 동안 AUTO_LINE을 무시하고 지정 속도로 동작
        # -------------------------------
        self._override_until = 0.0
        self._override_lin = 0.0
        self._override_ang = 0.0

        # -------------------------------
        # rosbridge 퍼블리셔
        # -------------------------------
        self.publisher = RosbridgePublisher(
            ws_url="ws://0.0.0.0:9090"  # ← 로봇 IP
        )

        # -------------------------------
        # 스레드 자동 시작(기존 구조 호환)
        # - main_code.py가 start_threads()를 따로 호출하지 않아도 동작하게 함
        # -------------------------------
        self.start_threads()


    # ---------------------------------
    # 감속 처리 함수
    # ---------------------------------
    def smooth_stop_linear(self):
        if self.current_lin_vel > 0:
            self.current_lin_vel = max(0.0, self.current_lin_vel - self.lin_decel_rate)
        elif self.current_lin_vel < 0:
            self.current_lin_vel = min(0.0, self.current_lin_vel + self.lin_decel_rate)

    def smooth_stop_angular(self):
        if self.current_ang_vel > 0:
            self.current_ang_vel = max(0.0, self.current_ang_vel - self.ang_decel_rate)
        elif self.current_ang_vel < 0:
            self.current_ang_vel = min(0.0, self.current_ang_vel + self.ang_decel_rate)

    # ---------------------------------
    # 속도 제한
    # ---------------------------------
    def clamp(self):
        self.current_lin_vel = max(-self.MAX_LIN, min(self.MAX_LIN, self.current_lin_vel))
        self.current_ang_vel = max(-self.MAX_ANG, min(self.MAX_ANG, self.current_ang_vel))

    # ---------------------------------
    # 메인 제어 루프
    # ---------------------------------
    def main_controller(self):
        print("[RobotSystem] 제어 루프 시작")

        while self.is_running:
            if not self.command_queue.empty():
                data = self.command_queue.get()
                cmd = str(data.get("cmd", "")).upper()

                # -------------------------
                # 모드 전환
                # -------------------------
                if cmd in ["AUTO", "MANUAL"]:
                    self.control_mode = cmd
                    self.current_lin_vel = 0.0
                    self.current_ang_vel = 0.0
                    self._publish_once()
                    continue

                # -------------------------
                # STOP (요청사항)
                # - AUTO일 때 STOP 누르면:
                #   1) 즉시 정지
                #   2) MANUAL로 전환
                # -------------------------
                if cmd == "STOP":
                    self.current_lin_vel = 0.0
                    self.current_ang_vel = 0.0
                    if self.control_mode == "AUTO":
                        self.control_mode = "MANUAL"
                    self._publish_once()
                    continue

                # -------------------------
                # AUTO 전용 정지(모드 유지)
                # -------------------------
                if cmd == "AUTO_STOP":
                    if self.control_mode == "AUTO":
                        self.current_lin_vel = 0.0
                        self.current_ang_vel = 0.0
                    self._publish_once()
                    continue


                # -------------------------
                # SHAPE 행동 트리거(도형 인식 결과)
                # - AUTO에서만 동작
                # - 예: CIRCLE -> 정지 후 360도 회전, RECTANGLE -> 잠깐 가속, TRIANGLE -> 잠깐 감속, X -> 정지+MANUAL
                # -------------------------
                if cmd == "SHAPE_ACTION":
                    if self.control_mode == "AUTO":
                        name = str(data.get("name", "NONE")).upper()
                        # 기본은 유지
                        if name == "TRIANGLE":
                            # 잠깐 감속
                            self._override_lin = max(0.0, self.auto_lin_speed * 0.5)
                            self._override_ang = 0.0
                            self._override_until = time.time() + 1.0
                        elif name in ["RECT", "RECTANGLE"]:
                            # 잠깐 가속 
                            self._override_lin = min(self.MAX_LIN, self.auto_lin_speed * 1.5)
                            self._override_ang = 0.0
                            self._override_until = time.time() + 1.0
                        elif name == "CIRCLE":
                            # 정지 후 360도 회전
                            # 2.0 rad/s로 약 3.14s면 360deg 근처
                            self.current_lin_vel = 0.0
                            self.current_ang_vel = 0.0
                            self._publish_once()
                            self._override_lin = 0.0
                            self._override_ang = 2.0
                            self._override_until = time.time() + 3.2
                        elif name == "X":
                            # 안전 정지 + MANUAL 전환
                            self.current_lin_vel = 0.0
                            self.current_ang_vel = 0.0
                            self.control_mode = "MANUAL"
                            self._override_until = 0.0
                        elif name == "XOTHER":
                            pass
                        # publish immediately
                        self.clamp()
                        self._publish_once()
                    continue

                # -------------------------
                # AUTO 라인트레이싱 입력
                # -------------------------
                if cmd == "AUTO_LINE":
                    if self.control_mode == "AUTO":
                        # 오버라이드 동작 중이면 AUTO_LINE은 무시
                        if time.time() < self._override_until:
                            # 현재 override 값으로 유지
                            self.current_lin_vel = self._override_lin
                            self.current_ang_vel = self._override_ang
                            self.clamp()
                            self._publish_once()
                            continue

                        err_px = float(data.get("value", 0.0))

                        # err_px: 왼쪽(+) / 오른쪽(-)
                        # 정규화: -1~+1 근처로
                        # (픽셀 기준은 카메라 640기준 320 중심)
                        err_norm = err_px / 320.0

                        # 데드존
                        if abs(err_norm) < self.auto_deadzone_ratio:
                            ang = 0.0
                        else:
                            ang = self.auto_kp * err_norm

                        # ✅ 직진 속도는 항상 고정
                        lin = float(self.auto_lin_speed)

                        self.current_lin_vel = lin
                        self.current_ang_vel = ang

                    self.clamp()
                    self._publish_once()
                    continue

                # -------------------------
                # MANUAL 조작
                # (AUTO일 때는 무시해서 부담/오작동 방지)
                # -------------------------
                if self.control_mode == "MANUAL":
                    self.apply_manual_command(cmd)

            else:
                # 입력이 없으면 감속(단, MANUAL에서는 잠깐 유지해 줌)
                if self.control_mode == "MANUAL":
                    if (time.time() - self._last_manual_input_time) >= self.manual_idle_decel_delay:
                        self.smooth_stop_linear()
                        self.smooth_stop_angular()
                else:
                    # AUTO는 안전하게 감속
                    self.smooth_stop_linear()
                        # self.smooth_stop_angular()  # MANUAL에서는 회전은 유지(버튼 입력 1회로도 지속)

            # 오버라이드 동작 중이면 AUTO_LINE 없이도 계속 동작
            if self.control_mode == "AUTO" and time.time() < self._override_until:
                self.current_lin_vel = self._override_lin
                self.current_ang_vel = self._override_ang

            # 제한 + publish
            self.clamp()
            self._apply_deadband()
            self.publisher.publish_cmd_vel(self.current_lin_vel, self.current_ang_vel)

            time.sleep(0.05)  # 20Hz

        # 루프 종료 시 정지
        try:
            self.publisher.publish_cmd_vel(0.0, 0.0)
            self.publisher.close()
        except Exception:
            pass

    def _apply_deadband(self):
        # 드리프트 방지: 아주 작은 값은 0으로
        if abs(self.current_lin_vel) < self.DEADBAND_LIN:
            self.current_lin_vel = 0.0
        if abs(self.current_ang_vel) < self.DEADBAND_ANG:
            self.current_ang_vel = 0.0

    def _publish_once(self):
        """명령 처리 직후 즉시 1회 publish(응답성)"""
        self.clamp()
        self._apply_deadband()
        self.publisher.publish_cmd_vel(self.current_lin_vel, self.current_ang_vel)

    # ---------------------------------
    # 수동 명령 처리
    # ---------------------------------
    def apply_manual_command(self, cmd: str):
        self._last_manual_input_time = time.time()
        if cmd == "GO":
            self.current_lin_vel += self.lin_step
        elif cmd == "BACK":
            self.current_lin_vel -= self.lin_step
        elif cmd == "LEFT":
            self.current_ang_vel += self.ang_step
        elif cmd == "RIGHT":
            self.current_ang_vel -= self.ang_step

    # ---------------------------------
    # 스레드 시작
    # ---------------------------------
    def start_threads(self):
        threading.Thread(target=self.main_controller, daemon=True).start()
