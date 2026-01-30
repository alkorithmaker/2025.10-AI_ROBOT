# camera.py
# -----------------------------------------
# 역할:
# - 터틀봇에서 카메라를 OpenCV로 가져옴
# - 가져온 영상을 전처리, 영상처리하여 
# - 라인트레이싱과 도형 탐지 알고리즘을 수행함함
# -----------------------------------------
from __future__ import annotations

import base64
import json
import os
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Optional, Tuple

import cv2
import numpy as np

# TensorFlow (shape model)
import tensorflow as tf

# pip install websocket-client
try:
    import websocket  # type: ignore
except Exception:
    websocket = None


@dataclass
class LineResult:
    found: bool
    error_px: float = 0.0          # 가운데 기준: 왼쪽(+) / 오른쪽(-)
    white_ratio: float = 0.0       # 0~1
    white_pixels: int = 0


@dataclass
class ShapeResult:
    found: bool
    name: str = "NONE"
    confidence: float = 0.0        # 0~100
    white_ratio_pct: float = 0.0   # 0~100


class RosbridgeCompressedImageReceiver:
    """
    rosbridge_websocket으로 sensor_msgs/CompressedImage 토픽을 subscribe해서
    JPEG(base64)를 cv2 프레임(BGR)로 바꿔 최신 프레임만 제공.
    """
    def __init__(
        self,
        ws_url: str,
        image_topic: str,
        throttle_rate_ms: int = 80,
        queue_size: int = 1,
        connect_timeout: float = 5.0,
    ):
        if websocket is None:
            raise RuntimeError(
                "websocket-client가 설치되어 있지 않습니다. "
                "노트북에서 실행한다면: pip install websocket-client"
            )

        self.ws_url = ws_url
        self.image_topic = image_topic
        self.throttle_rate_ms = int(throttle_rate_ms)

        self._running = True
        self._q: Queue[np.ndarray] = Queue(maxsize=max(1, int(queue_size)))

        self.ws = websocket.WebSocket()
        self.ws.settimeout(connect_timeout)
        self.ws.connect(self.ws_url)

        sub_msg = {
            "op": "subscribe",
            "topic": self.image_topic,
            "type": "sensor_msgs/CompressedImage",
            "throttle_rate": self.throttle_rate_ms,  # ms
            "queue_length": 1,
        }
        self.ws.send(json.dumps(sub_msg))

        self._th = threading.Thread(target=self._recv_loop, daemon=True)
        self._th.start()

    def _recv_loop(self):
        while self._running:
            try:
                raw = self.ws.recv()
                if not raw:
                    continue
                data = json.loads(raw)
                msg = data.get("msg")
                if not msg:
                    continue
                b64 = msg.get("data")
                if not b64:
                    continue

                jpg_bytes = base64.b64decode(b64)
                arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # keep only newest
                while True:
                    try:
                        self._q.get_nowait()
                    except Empty:
                        break
                try:
                    self._q.put_nowait(frame)
                except Exception:
                    pass

            except Exception:
                time.sleep(0.1)

    def read(self, timeout: float = 0.8) -> Tuple[bool, Optional[np.ndarray]]:
        try:
            frame = self._q.get(timeout=timeout)
            return True, frame
        except Empty:
            return False, None

    def close(self):
        self._running = False
        try:
            try:
                unsub_msg = {"op": "unsubscribe", "topic": self.image_topic}
                self.ws.send(json.dumps(unsub_msg))
            except Exception:
                pass
            self.ws.close()
        except Exception:
            pass


class CameraSystem:
    """
    - AUTO에서만 라인트레이싱/도형인식 가동 (MANUAL은 idle)
    - rosbridge compressed 사용
    - 디버그 윈도우:
      * show_recognition_line  -> "Line ROI View"
      * show_recognition_shape -> "Shape ROI View"
      * show_shape_threshold   -> "Shape Threshold"
      * show_line_binary       -> "Line Threshold"
    """

    def __init__(
        self,
        robot,
        show: bool = True,

        # debug windows
        show_recognition_line: bool = True,
        show_recognition_shape: bool = True,
        show_shape_threshold: bool = False,
        show_line_binary: bool = True,

        width: int = 640,
        height: int = 480,

        # --- 로봇 카메라(ROS compressed) ---
        ws_url: str = "ws://192.168.0.93:9090",
        image_topic: str = "/camera/image_raw/compressed",
        throttle_rate_ms: int = 80,

        # --- 라인트레이싱 설정 ---
        fixed_thresh_val: int = 50,
        full_threshold: float = 0.80,
        deadzone_px: int = 20,

        # --- Shape(딥러닝) 설정 ---
        model_path: str = "shape_model_9.keras",
        img_size: int = 128,
        shape_ratio_min: float = 1.0,     # %
        shape_ratio_max: float = 17.0,    # %
        shape_conf_th: float = 90.0,      # %
        shape_action_cooldown_sec: float = 1.0,
        shape_every: int = 3,             # TF 추론 주기

        # --- ROI (640x480 기준) ---
        shape_x1: int = 0,
        shape_x2: int = 220,
        shape_y1: int = 40,
        shape_y2: int = 280,
    ):
        self.robot = robot
        self.show = bool(show)

        self.show_recognition_line = bool(show_recognition_line)
        self.show_recognition_shape = bool(show_recognition_shape)
        self.show_shape_threshold = bool(show_shape_threshold)
        self.show_line_binary = bool(show_line_binary)

        self.width = int(width)
        self.height = int(height)
        self.target_center = self.width // 2  # 전체 프레임 중앙(왼+ / 오- 기준)

        # ROS 이미지
        self.ws_url = str(ws_url)
        self.image_topic = str(image_topic)
        self.throttle_rate_ms = int(throttle_rate_ms)

        # Line config
        self.fixed_thresh_val = int(fixed_thresh_val)
        self.full_threshold = float(full_threshold)
        self.deadzone_px = int(deadzone_px)

        # Shape model
        self.model_path = str(model_path)
        self.img_size = int(img_size)
        self.shape_ratio_min = float(shape_ratio_min)
        self.shape_ratio_max = float(shape_ratio_max)
        self.shape_conf_th = float(shape_conf_th)
        self.shape_action_cooldown_sec = float(shape_action_cooldown_sec)
        self.shape_every = max(1, int(shape_every))
        self._last_shape_action_time = 0.0
        self.class_names = ["circle", "rectangle", "triangle", "x", "xother"]

        self.model = None
        if os.path.exists(self.model_path):
            try:
                self.model = tf.keras.models.load_model(self.model_path, compile=False)
            except Exception as e:
                print("[camera] cwd:", os.getcwd())
                print("[camera] model_path:", os.path.abspath(self.model_path))
                print("[camera] exists:", os.path.exists(self.model_path))

                print(f"[camera] ERROR: 모델 로드 실패: {e}")
                self.model = None
        else:
            print("[camera] cwd:", os.getcwd())
            print("[camera] model_path:", os.path.abspath(self.model_path))
            print("[camera] exists:", os.path.exists(self.model_path))
            print(f"[camera] WARN: 모델 파일 없음: {self.model_path}")

        # ROI: shape 영역(상단)
        self.shape_x1 = int(shape_x1)
        self.shape_x2 = int(shape_x2)
        self.shape_y1 = int(shape_y1)
        self.shape_y2 = int(shape_y2)

        # line ROI (현재는 고정값 사용)
        self.line_x1 = 160
        self.line_x2 = 480
        self.line_y1 = 320
        self.line_y2 = self.height

        # ---- debug storage ----
        self._last_line_thr: Optional[np.ndarray] = None
        self._last_shape_thr: Optional[np.ndarray] = None

        self._last_line_roi: Optional[Tuple[int, int, int, int]] = None   # (x1,x2,y1,y2)
        self._last_shape_roi: Optional[Tuple[int, int, int, int]] = None  # (x1,x2,y1,y2)

        self._last_line_cx_global: Optional[float] = None
        self._last_line_info = None  # (found, err_px, white_ratio, white_pixels)

        self._last_shape_info = None # (found, name, conf, ratio_pct)
        self._last_shape_res = ShapeResult(found=False, name="NONE", confidence=0.0, white_ratio_pct=0.0)

        # status text
        self.line_status = "IDLE"
        self.shape_status = "IDLE"

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    # -----------------------------
    # 라인트레이싱: line ROI에서만 계산
    # -----------------------------
    def _detect_line(self, frame_bgr: np.ndarray) -> LineResult:
        img = frame_bgr
        h, w = img.shape[:2]

        x1 = int(np.clip(self.line_x1, 0, w - 1))
        x2 = int(np.clip(self.line_x2, x1 + 1, w))
        y1 = int(np.clip(self.line_y1, 0, h - 1))
        y2 = int(np.clip(self.line_y2, y1 + 1, h))

        roi = img[y1:y2, x1:x2]

        # ✅ 라인 ROI는 line에만 저장 (shape에 절대 덮어쓰지 않음)
        self._last_line_roi = (x1, x2, y1, y2)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        _, thr = cv2.threshold(blurred, self.fixed_thresh_val, 255, cv2.THRESH_BINARY_INV)
        self._last_line_thr = thr

        white_pixels = int(cv2.countNonZero(thr))
        total_pixels = int(thr.shape[0] * thr.shape[1])
        white_ratio = (white_pixels / total_pixels) if total_pixels > 0 else 0.0

        self._last_line_info = (False, 0.0, white_ratio, white_pixels)
        self._last_line_cx_global = None

        if white_ratio > self.full_threshold or white_pixels == 0:
            return LineResult(found=False, error_px=0.0, white_ratio=white_ratio, white_pixels=white_pixels)

        M = cv2.moments(thr)
        if M["m00"] <= 0:
            return LineResult(found=False, error_px=0.0, white_ratio=white_ratio, white_pixels=white_pixels)

        cx_roi = float(M["m10"] / M["m00"])
        cx_global = cx_roi + x1
        error_px = float(self.target_center - cx_global)

        self._last_line_cx_global = cx_global
        self._last_line_info = (True, error_px, white_ratio, white_pixels)

        return LineResult(found=True, error_px=error_px, white_ratio=white_ratio, white_pixels=white_pixels)

    # -----------------------------
    # 도형인식: shape ROI에서 TF 모델로 예측
    # -----------------------------
    def _detect_shape_tf(self, frame_bgr: np.ndarray) -> ShapeResult:
        if self.model is None:
            self._last_shape_info = (False, "NONE", 0.0, 0.0)
            return ShapeResult(found=False, name="NONE", confidence=0.0, white_ratio_pct=0.0)

        img = frame_bgr
        h, w = img.shape[:2]

        x1 = int(np.clip(self.shape_x1, 0, w - 1))
        x2 = int(np.clip(self.shape_x2, x1 + 1, w))
        y1 = int(np.clip(self.shape_y1, 0, h - 1))
        y2 = int(np.clip(self.shape_y2, y1 + 1, h))

        roi = img[y1:y2, x1:x2]

        # ✅ shape ROI는 shape에만 저장
        self._last_shape_roi = (x1, x2, y1, y2)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        self._last_shape_thr = thresh.copy()

        margin = 5
        if thresh.shape[0] > margin * 2 and thresh.shape[1] > margin * 2:
            thresh2 = thresh[margin:-margin, margin:-margin]
        else:
            thresh2 = thresh

        resized = cv2.resize(thresh2, (self.img_size, self.img_size))

        total_pixels = resized.size
        white_pixels = int(np.sum(resized == 255))
        white_ratio_pct = (white_pixels / total_pixels) * 100.0 if total_pixels > 0 else 0.0

        if not (self.shape_ratio_min <= white_ratio_pct < self.shape_ratio_max):
            self._last_shape_info = (False, "NONE", 0.0, white_ratio_pct)
            return ShapeResult(found=False, name="NONE", confidence=0.0, white_ratio_pct=white_ratio_pct)

        input_data = resized.reshape(1, self.img_size, self.img_size, 1).astype("float32")
        pred = self.model.predict(input_data, verbose=0)
        idx = int(np.argmax(pred))
        conf = float(np.max(pred) * 100.0)
        name = self.class_names[idx]

        found = conf >= self.shape_conf_th
        out = ShapeResult(found=found, name=name if found else "NONE", confidence=conf, white_ratio_pct=white_ratio_pct)
        self._last_shape_info = (out.found, out.name, out.confidence, out.white_ratio_pct)
        return out

    # -----------------------------
    # 입력 소스 열기
    # -----------------------------
    def _open_source(self):
        rx = RosbridgeCompressedImageReceiver(
            ws_url=self.ws_url,
            image_topic=self.image_topic,
            throttle_rate_ms=self.throttle_rate_ms,
            queue_size=1,
        )

        def read_fn():
            return rx.read(timeout=0.8)

        def close_fn():
            rx.close()

        print(f"[camera] Using ROS image via rosbridge: {self.ws_url} topic={self.image_topic}")
        return read_fn, close_fn

    # -----------------------------
    # 메인 루프
    # -----------------------------
    def _loop(self):
        try:
            read_frame, close_source = self._open_source()
        except Exception as e:
            print(f"[camera] ERROR: {e}")
            self._running = False
            return

        # windows
        if self.show:
            cv2.namedWindow("Main View", cv2.WINDOW_NORMAL)
        if self.show_recognition_line:
            cv2.namedWindow("Line ROI View", cv2.WINDOW_NORMAL)
        if self.show_recognition_shape:
            cv2.namedWindow("Shape ROI View", cv2.WINDOW_NORMAL)
        if self.show_shape_threshold:
            cv2.namedWindow("Shape Threshold", cv2.WINDOW_NORMAL)
        if self.show_line_binary:
            cv2.namedWindow("Line Threshold", cv2.WINDOW_NORMAL)

        shape_counter = 0

        try:
            while self._running and getattr(self.robot, "is_running", True):
                ret, frame = read_frame()
                if not ret or frame is None:
                    continue

                img = cv2.resize(frame, (self.width, self.height))
                mode = getattr(self.robot, "control_mode", "MANUAL")

                line_res = LineResult(found=False)
                shape_res = self._last_shape_res  # ✅ 마지막 값 기본으로 유지

                if mode == "AUTO":
                    # line: 매 프레임
                    line_res = self._detect_line(img)

                    # shape: N프레임마다 갱신, 그 외에는 last 유지
                    if shape_counter % self.shape_every == 0:
                        self._last_shape_res = self._detect_shape_tf(img)
                        try:
                            setattr(self.robot, "current_shape", self._last_shape_res.name if self._last_shape_res.found else "NONE")
                        except Exception:
                            pass
                    shape_counter += 1
                    shape_res = self._last_shape_res

                    # AUTO에서만 명령 전송
                    if not line_res.found:
                        self.line_status = "LINE LOST -> STOP"
                        if getattr(self.robot, "command_queue", None) is not None:
                            self.robot.command_queue.put({"source": "CAMERA", "cmd": "AUTO_STOP"})
                    else:
                        if abs(line_res.error_px) <= self.deadzone_px:
                            self.line_status = f"LINE OK (err={line_res.error_px:.1f}px)"
                        else:
                            self.line_status = f"LINE TRACK (err={line_res.error_px:.1f}px)"

                        if getattr(self.robot, "command_queue", None) is not None:
                            self.robot.command_queue.put({
                                "source": "CAMERA",
                                "cmd": "AUTO_LINE",
                                "value": float(line_res.error_px),
                            })

                    # shape status
                    if shape_res.found:
                        self.shape_status = f"SHAPE: {shape_res.name} ({shape_res.confidence:.1f}%)"
                    else:
                        self.shape_status = f"SHAPE: NONE (ratio={shape_res.white_ratio_pct:.1f}%)"

                    # 도형 행동 트리거(원하면 xother는 무시)
                    if (
                        shape_res.found
                        and shape_res.name != "xother"   # xother는 “아무행동 안 함”
                        and getattr(self.robot, "command_queue", None) is not None
                    ):
                        now = time.time()
                        if (now - self._last_shape_action_time) >= self.shape_action_cooldown_sec:
                            self._last_shape_action_time = now
                            self.robot.command_queue.put({
                                "source": "CAMERA",
                                "cmd": "SHAPE_ACTION",
                                "name": str(shape_res.name).upper(),
                                "confidence": float(shape_res.confidence),
                            })

                else:
                    # MANUAL: idle + debug 초기화
                    self.line_status = "MANUAL (line idle)"
                    self.shape_status = "MANUAL (shape idle)"
                    shape_counter = 0

                    self._last_line_thr = None
                    self._last_shape_thr = None
                    self._last_line_info = None
                    self._last_shape_info = None
                    self._last_line_cx_global = None
                    self._last_shape_res = ShapeResult(found=False, name="NONE", confidence=0.0, white_ratio_pct=0.0)

                    try:
                        setattr(self.robot, "current_shape", "NONE")
                    except Exception:
                        pass

                # ----------------- 화면 표시 -----------------
                if self.show:
                    overlay = img.copy()

                    # ROI 표시 (고정값 표시)
                    cv2.rectangle(overlay, (self.shape_x1, self.shape_y1), (self.shape_x2, self.shape_y2), (255, 0, 0), 2)
                    cv2.putText(overlay, "SHAPE ROI", (self.shape_x1 + 5, self.shape_y1 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)

                    cv2.rectangle(overlay, (self.line_x1, self.line_y1), (self.line_x2, self.line_y2), (0, 255, 255), 2)
                    cv2.putText(overlay, "LINE ROI", (self.line_x1 + 5, self.line_y1 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                    cv2.line(overlay, (self.target_center, 0), (self.target_center, self.height - 1), (0, 255, 0), 2)

                    lin = float(getattr(self.robot, "current_lin_vel", 0.0))
                    ang = float(getattr(self.robot, "current_ang_vel", 0.0))
                    cv2.putText(overlay, f"MODE: {mode}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.putText(overlay, f"VEL: lin={lin:.3f}  ang={ang:.3f}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(overlay, f"LINE: {self.line_status}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                    cv2.putText(overlay, f"{self.shape_status}", (10, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                    cv2.putText(overlay, f"SHAPE_STATE: {getattr(self.robot, 'current_shape', 'NONE')}", (10, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

                    cv2.imshow("Main View", overlay)

                    # ---- Line Threshold ----
                    if self.show_line_binary and self._last_line_thr is not None:
                        thr = self._last_line_thr
                        thr_vis = cv2.cvtColor(thr, cv2.COLOR_GRAY2BGR)

                        cv2.rectangle(thr_vis, (0, 0), (thr_vis.shape[1]-1, thr_vis.shape[0]-1), (0, 255, 255), 2)

                        center_in_roi_x = int(self.target_center - self.line_x1)
                        center_in_roi_x = max(0, min(thr_vis.shape[1]-1, center_in_roi_x))
                        cv2.line(thr_vis, (center_in_roi_x, 0), (center_in_roi_x, thr_vis.shape[0]-1), (0, 255, 0), 2)

                        if self._last_line_cx_global is not None:
                            cx_in_roi = int(self._last_line_cx_global - self.line_x1)
                            cx_in_roi = max(0, min(thr_vis.shape[1]-1, cx_in_roi))
                            cy = thr_vis.shape[0] // 2
                            cv2.circle(thr_vis, (cx_in_roi, cy), 6, (0, 0, 255), -1)

                        if self._last_line_info is not None:
                            found, err_px, wr, wp = self._last_line_info
                            txt = f"found={found} err={err_px:.1f}px white={wp} ratio={wr*100.0:.1f}%"
                        else:
                            txt = "LINE THR"
                        cv2.putText(thr_vis, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

                        cv2.imshow("Line Threshold", thr_vis)

                    # ---- Shape ROI View ----
                    if self.show_recognition_shape and self._last_shape_roi is not None:
                        sx1, sx2, sy1, sy2 = self._last_shape_roi
                        sroi = img[sy1:sy2, sx1:sx2].copy()
                        if self._last_shape_info is not None:
                            fnd, nm, cf, rp = self._last_shape_info
                            cv2.putText(sroi, f"{nm}  conf={cf:.1f}%  ratio={rp:.1f}%", (10, 25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
                        cv2.imshow("Shape ROI View", sroi)

                    if self.show_shape_threshold and self._last_shape_thr is not None:
                        sth = cv2.cvtColor(self._last_shape_thr, cv2.COLOR_GRAY2BGR)
                        cv2.rectangle(sth, (0, 0), (sth.shape[1]-1, sth.shape[0]-1), (255, 0, 0), 2)
                        cv2.imshow("Shape Threshold", sth)

                    # ---- Line ROI View ----
                    if self.show_recognition_line and self._last_line_roi is not None:
                        lx1, lx2, ly1, ly2 = self._last_line_roi
                        lroi = img[ly1:ly2, lx1:lx2].copy()

                        center_in_roi_x = int(self.target_center - lx1)
                        center_in_roi_x = max(0, min(lroi.shape[1]-1, center_in_roi_x))
                        cv2.line(lroi, (center_in_roi_x, 0), (center_in_roi_x, lroi.shape[0]-1), (0, 255, 0), 2)

                        if self._last_line_cx_global is not None:
                            cx_in_roi = int(self._last_line_cx_global - lx1)
                            cx_in_roi = max(0, min(lroi.shape[1]-1, cx_in_roi))
                            cy = lroi.shape[0] // 2
                            cv2.circle(lroi, (cx_in_roi, cy), 6, (0, 0, 255), -1)

                        if self._last_line_info is not None:
                            found, err_px, wr, wp = self._last_line_info
                            cv2.putText(lroi, f"found={found} err={err_px:.1f}px", (10, 25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

                        cv2.imshow("Line ROI View", lroi)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        setattr(self.robot, "is_running", False)
                        self._running = False
                        break

        finally:
            try:
                close_source()
            except Exception:
                pass
            if self.show:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
