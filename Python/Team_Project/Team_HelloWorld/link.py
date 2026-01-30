# link.py
# -----------------------------------------
# 역할:
# - rosbridge_websocket 서버에 웹소켓으로 연결
# - /cmd_vel 토픽에 Twist 메시지를 JSON 형태로 publish
# - ROS를 직접 사용하지 않음 (윈도우 전용)
# -----------------------------------------

import json
import time
import websocket


class RosbridgePublisher:
    def __init__(self, ws_url: str):
        """
        ws_url: rosbridge 웹소켓 주소
        예) ws://0.0.0.0:9090
        """
        self.ws_url = ws_url
        self.ws = websocket.WebSocket()

        # rosbridge 연결
        self.ws.connect(self.ws_url)
        print(f"[link] rosbridge 연결됨: {self.ws_url}")

        # /cmd_vel 토픽 advertise (ROS 쪽에 '이 토픽 쓸게요' 선언)
        self.ws.send(json.dumps({
            "op": "advertise",
            "topic": "/cmd_vel",
            "type": "geometry_msgs/Twist"
        }))

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        """
        선속도(linear.x), 각속도(angular.z)를 /cmd_vel로 전송
        """
        msg = {
            "op": "publish",
            "topic": "/cmd_vel",
            "msg": {
                "linear":  {"x": linear_x, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
            }
        }

        self.ws.send(json.dumps(msg))

    def close(self):
        """웹소켓 종료"""
        self.ws.close()
