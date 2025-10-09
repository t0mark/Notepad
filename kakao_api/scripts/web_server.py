#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Web Interface Server Node
- HTTP 서버: index.html 제공
- WebSocket: GPS 송신 + Path 수신 (양방향)
- 첫 GPS 위치를 UTM 원점으로 사용
- GPS 경로 → UTM 변환 → 원점 기준 상대 좌표로 /kakao/path 발행
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
import pyproj
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# 포트 설정
HTTP_PORT = 8000
WS_PORT = 8765

# 경로 설정
WEB_DIR = os.path.join(os.path.dirname(__file__), "../web")

# GPS 데이터
latest_gps = None
gps_lock = threading.Lock()

# Map 원점 (첫 GPS의 UTM 좌표)
map_origin_utm = None
map_origin_lock = threading.Lock()

# UTM 변환기
utm_projector = None


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
    try:
        if not os.path.exists(WEB_DIR):
            rospy.logerr(f"❌ 웹 디렉토리 없음: {WEB_DIR}")
            return

        os.chdir(WEB_DIR)
        rospy.loginfo(f"📂 웹 디렉토리: {WEB_DIR}")

        # 포트 재사용 허용
        socketserver.TCPServer.allow_reuse_address = True
        with socketserver.TCPServer(("", HTTP_PORT), HTTPHandler) as httpd:
            rospy.loginfo(f"✅ HTTP 서버 실행 중: {HTTP_PORT}")
            httpd.serve_forever()
    except OSError as e:
        rospy.logerr(f"❌ HTTP 서버 시작 실패: {e}")
    except Exception as e:
        rospy.logerr(f"❌ HTTP 서버 오류: {e}")


def open_browser():
    """브라우저 자동 실행 (Docker 환경에서는 실패)"""
    time.sleep(2)
    try:
        webbrowser.open(f"http://localhost:{HTTP_PORT}/index.html")
        rospy.loginfo(f"🌐 브라우저 자동 실행: http://localhost:{HTTP_PORT}/index.html")
    except Exception:
        # Docker 등 GUI 없는 환경에서는 정상적으로 실패
        rospy.loginfo(f"💡 브라우저에서 http://localhost:{HTTP_PORT}/index.html 를 여세요")
        pass


def initialize_utm_projector(lon):
    """UTM projector 초기화"""
    global utm_projector

    if utm_projector is None:
        # UTM zone 자동 결정
        zone = int((lon + 180) / 6) + 1
        utm_projector = pyproj.Proj(proj='utm', zone=zone, ellps='WGS84')


def gps_to_utm(lat, lon):
    """GPS → UTM 변환"""
    global utm_projector

    if utm_projector is None:
        initialize_utm_projector(lon)

    easting, northing = utm_projector(lon, lat)
    return easting, northing


def initialize_origin_from_gps(lat, lon):
    """첫 GPS를 map 원점으로 설정"""
    global map_origin_utm

    with map_origin_lock:
        if map_origin_utm is None:
            initialize_utm_projector(lon)

            # GPS → UTM 변환
            easting, northing = utm_projector(lon, lat)

            map_origin_utm = {
                'easting': easting,
                'northing': northing
            }

            rospy.loginfo("=" * 60)
            rospy.loginfo("🎯 Map 원점 설정 (첫 GPS 사용)")
            rospy.loginfo(f"   GPS: ({lat:.6f}, {lon:.6f})")
            rospy.loginfo(f"   UTM: ({easting:.2f}, {northing:.2f})")
            rospy.loginfo(f"   Map 원점: (0.0, 0.0)")
            rospy.loginfo("=" * 60)


def gps_callback(msg):
    global latest_gps

    if msg.status.status < 0:
        return

    # 첫 GPS를 map 원점으로 설정
    if map_origin_utm is None:
        initialize_origin_from_gps(msg.latitude, msg.longitude)

    # 웹소켓용 데이터
    with gps_lock:
        latest_gps = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status,
            'timestamp': time.time()
        }

async def unified_ws_handler(websocket, path):
    """통합 WebSocket: GPS 송신 + Path 수신"""
    path_pub = rospy.Publisher('/kakao/path', Path, queue_size=1, latch=True)

    async def send_gps():
        """GPS 데이터 주기적 송신"""
        try:
            while True:
                with gps_lock:
                    data = latest_gps
                await websocket.send(json.dumps({
                    'type': 'gps',
                    'data': data if data else {'error': 'No GPS'}
                }))
                await asyncio.sleep(1)
        except:
            pass

    async def receive_path():
        """Path 데이터 수신 및 처리"""
        try:
            async for message in websocket:
                data = json.loads(message)

                if data.get('type') != 'path' or 'waypoints' not in data:
                    await websocket.send(json.dumps({'type': 'error', 'message': 'Invalid format'}))
                    continue

                # 웨이포인트 검증 및 변환
                valid_gps = []
                for wp in data['waypoints']:
                    if 'lat' in wp and 'lon' in wp:
                        lat, lon = float(wp['lat']), float(wp['lon'])
                        if -90 <= lat <= 90 and -180 <= lon <= 180:
                            valid_gps.append({'lat': lat, 'lon': lon, 'alt': wp.get('alt', 0.0)})

                if not valid_gps:
                    await websocket.send(json.dumps({'type': 'error', 'message': 'No valid waypoints'}))
                    continue

                # GPS → UTM 변환 및 Path 생성
                if map_origin_utm is None:
                    await websocket.send(json.dumps({'type': 'error', 'message': 'Map origin not initialized'}))
                    continue

                path_msg = Path()
                path_msg.header.stamp = rospy.Time.now()
                path_msg.header.frame_id = "map"

                for i, wp in enumerate(valid_gps):
                    # GPS → UTM
                    easting, northing = gps_to_utm(wp['lat'], wp['lon'])

                    if easting is None or northing is None:
                        continue

                    # Map 좌표로 변환 (map_origin_utm 기준)
                    x = easting - map_origin_utm['easting']
                    y = northing - map_origin_utm['northing']

                    # PoseStamped
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0

                    path_msg.poses.append(pose)

                    # 로깅 (처음 3개 + 마지막 3개)
                    if i < 3 or i >= len(valid_gps) - 3:
                        rospy.loginfo(f"   WP{i+1}: GPS({wp['lat']:.6f}, {wp['lon']:.6f}) → Map({x:.2f}, {y:.2f})")
                    elif i == 3 and len(valid_gps) > 6:
                        rospy.loginfo(f"   ... (중간 {len(valid_gps)-6}개 생략)")

                # 발행
                path_pub.publish(path_msg)
                rospy.loginfo(f"✅ 경로 발행: {len(valid_gps)}개 (원점: 첫 GPS)")

                await websocket.send(json.dumps({
                    'type': 'success',
                    'total_received': len(data['waypoints']),
                    'valid_waypoints': len(valid_gps)
                }))

        except Exception as e:
            rospy.logerr(f"❌ Path 처리 오류: {e}")

    # GPS 송신 + Path 수신 동시 실행
    await asyncio.gather(send_gps(), receive_path())


async def start_unified_ws():
    async with websockets.serve(unified_ws_handler, "localhost", WS_PORT):
        rospy.loginfo(f"WebSocket 시작: ws://localhost:{WS_PORT}")
        await asyncio.Future()


def shutdown_hook():
    """종료 시 정리 작업"""
    rospy.loginfo("Web Server 종료 중...")
    # daemon=True로 설정했으므로 자동으로 스레드 종료됨


if __name__ == '__main__':
    rospy.init_node('web_server', anonymous=True)
    rospy.on_shutdown(shutdown_hook)

    # Subscribers
    rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)

    # HTTP 서버
    http_thread = threading.Thread(target=start_http, daemon=True)
    http_thread.start()

    threading.Thread(target=open_browser, daemon=True).start()

    # 통합 WebSocket 서버
    ws_thread = threading.Thread(target=lambda: asyncio.run(start_unified_ws()), daemon=True)
    ws_thread.start()

    rospy.loginfo(f"🌐 Web Server 시작: http://localhost:{HTTP_PORT}")
    rospy.loginfo(f"🔌 WebSocket 시작: ws://localhost:{WS_PORT}")
    rospy.loginfo(f"📡 GPS → UTM 변환 후 /kakao/path 발행")
    rospy.loginfo(f"🗺️  Map 원점: 첫 GPS 위치 (UTM 변환)")

    # WebSocket 시작 대기
    time.sleep(2)
    rospy.loginfo("✅ 서버 준비 완료")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C 감지, 종료합니다.")
