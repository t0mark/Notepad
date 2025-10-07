#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Web Interface Server Node
- HTTP 서버: index.html 제공
- GPS WebSocket: GPS 데이터를 웹으로 전송
- Waypoint WebSocket: 웹 → /kakao/goal 발행
"""

import rospy
import http.server
import socketserver
import threading
import webbrowser
import os
import json
import asyncio
import websockets
import time
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# 포트 설정
HTTP_PORT = 8000
GPS_WS_PORT = 8765
WAYPOINT_WS_PORT = 8766

# 경로 설정
WEB_DIR = os.path.join(os.path.dirname(__file__), "../web")

# GPS 데이터
latest_gps = None
gps_lock = threading.Lock()


class HTTPHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return
        return super().do_GET()


def start_http():
    os.chdir(WEB_DIR)
    with socketserver.TCPServer(("", HTTP_PORT), HTTPHandler) as httpd:
        httpd.serve_forever()


def open_browser():
    time.sleep(2)
    try:
        webbrowser.open(f"http://localhost:{HTTP_PORT}/index.html")
    except:
        pass  # 브라우저 자동 실행 실패 시 무시


def gps_callback(msg):
    global latest_gps

    if msg.status.status < 0:
        return

    # 웹소켓용 데이터
    with gps_lock:
        latest_gps = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status,
            'timestamp': time.time()
        }

async def gps_ws_handler(websocket, path):
    try:
        while True:
            with gps_lock:
                data = latest_gps
            await websocket.send(json.dumps(data if data else {'error': 'No GPS'}))
            await asyncio.sleep(1)
    except:
        pass


async def start_gps_ws():
    async with websockets.serve(gps_ws_handler, "localhost", GPS_WS_PORT):
        rospy.loginfo(f"GPS WebSocket 시작: ws://localhost:{GPS_WS_PORT}")
        await asyncio.Future()


async def waypoint_ws_handler(websocket, path):
    pub = rospy.Publisher('/kakao/goal', String, queue_size=10)

    try:
        async for message in websocket:
            data = json.loads(message)

            if 'waypoints' not in data:
                await websocket.send(json.dumps({'error': 'Invalid format'}))
                continue

            # 웨이포인트 검증
            valid = []
            for wp in data['waypoints']:
                if 'lat' in wp and 'lon' in wp:
                    lat, lon = float(wp['lat']), float(wp['lon'])
                    if -90 <= lat <= 90 and -180 <= lon <= 180:
                        valid.append({'lat': lat, 'lon': lon})

            if valid:
                pub.publish(json.dumps({'waypoints': valid}))
                rospy.loginfo(f"웨이포인트 수신: {len(valid)}개")
                await websocket.send(json.dumps({
                    'status': 'success',
                    'total_received': len(data['waypoints']),
                    'valid_waypoints': len(valid)
                }))
            else:
                await websocket.send(json.dumps({'error': 'No valid waypoints'}))
    except:
        pass


async def start_waypoint_ws():
    async with websockets.serve(waypoint_ws_handler, "localhost", WAYPOINT_WS_PORT):
        rospy.loginfo(f"Waypoint WebSocket 시작: ws://localhost:{WAYPOINT_WS_PORT}")
        await asyncio.Future()


def shutdown_hook():
    """종료 시 정리 작업"""
    rospy.loginfo("Web Server 종료 중...")
    # daemon=True로 설정했으므로 자동으로 스레드 종료됨


if __name__ == '__main__':
    rospy.init_node('web_server', anonymous=True)
    rospy.on_shutdown(shutdown_hook)

    # Subscriber
    rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)

    # HTTP 서버
    http_thread = threading.Thread(target=start_http, daemon=True)
    http_thread.start()

    threading.Thread(target=open_browser, daemon=True).start()

    # WebSocket 서버
    gps_ws_thread = threading.Thread(target=lambda: asyncio.run(start_gps_ws()), daemon=True)
    waypoint_ws_thread = threading.Thread(target=lambda: asyncio.run(start_waypoint_ws()), daemon=True)

    gps_ws_thread.start()
    waypoint_ws_thread.start()

    rospy.loginfo(f"Web Server 시작: http://localhost:{HTTP_PORT}")

    # WebSocket 시작 대기 (최대 5초)
    time.sleep(2)
    rospy.loginfo("WebSocket 서버 준비 완료")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C 감지, 종료합니다.")
