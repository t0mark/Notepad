#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Web Interface Server Node
- HTTP 서버: index.html 제공
- GPS 처리: /ublox/fix → UTM 변환 → /map_frame_gps
- GPS WebSocket: GPS 데이터를 웹으로 전송
- Waypoint WebSocket: 웹 → /waypoints_gps 발행
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
import utm
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

# 포트 설정
HTTP_PORT = 8000
GPS_WS_PORT = 8765
WAYPOINT_WS_PORT = 8766

# 경로 설정
WEB_DIR = os.path.join(os.path.dirname(__file__), "../web")

# GPS 데이터
latest_gps = None
gps_lock = threading.Lock()

# map 원점
map_origin = None
utm_zone = None
origin_set = False


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
    global latest_gps, map_origin, utm_zone, origin_set

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

    # 첫 유효한 GPS로 map 원점 설정 (INS가 map 프레임을 정의하므로 주석 처리)
    # if not origin_set:
    #     try:
    #         easting, northing, zone_num, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
    #         map_origin = {'easting': easting, 'northing': northing}
    #         utm_zone = (zone_num, zone_letter)
    #         origin_set = True
    #         rospy.loginfo(f"map 원점: ({msg.latitude:.6f}, {msg.longitude:.6f})")
    #     except Exception as e:
    #         rospy.logwarn(f"map 원점 설정 실패: {e}")
    #         return

    # GPS → map 좌표 변환 (INS가 map 프레임을 정의하므로 주석 처리)
    # try:
    #     easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = rospy.Time.now()
    #     pose_msg.header.frame_id = "map"
    #     pose_msg.pose.position.x = easting - map_origin['easting']
    #     pose_msg.pose.position.y = northing - map_origin['northing']
    #     pose_msg.pose.position.z = msg.altitude
    #     pose_msg.pose.orientation.w = 1.0
    #
    #     gps_map_pub.publish(pose_msg)
    # except Exception as e:
    #     rospy.logwarn(f"GPS 변환 실패: {e}")


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
    pub = rospy.Publisher('/waypoints_gps', String, queue_size=10)

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


if __name__ == '__main__':
    rospy.init_node('web_server', anonymous=True)

    # Publisher
    gps_map_pub = rospy.Publisher('/map_frame_gps', PoseStamped, queue_size=10)

    # Subscriber
    rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)

    # HTTP 서버
    http_thread = threading.Thread(target=start_http, daemon=False)
    http_thread.start()

    threading.Thread(target=open_browser, daemon=True).start()

    # WebSocket 서버 (daemon=False로 프로세스 유지)
    gps_ws_thread = threading.Thread(target=lambda: asyncio.run(start_gps_ws()), daemon=False)
    waypoint_ws_thread = threading.Thread(target=lambda: asyncio.run(start_waypoint_ws()), daemon=False)

    gps_ws_thread.start()
    waypoint_ws_thread.start()

    rospy.loginfo(f"Web Server 시작: http://localhost:{HTTP_PORT}")

    # WebSocket 시작 대기 (최대 5초)
    time.sleep(2)
    rospy.loginfo("WebSocket 서버 준비 완료")

    rospy.spin()
