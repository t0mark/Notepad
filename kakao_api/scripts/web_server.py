#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Web Interface Server Node

HTTP 서버와 WebSocket 서버를 실행하여 웹 인터페이스 제공

기능:
  - HTTP 서버: index.html 제공
  - GPS WebSocket: 실시간 GPS 데이터를 웹으로 전송
  - Waypoint WebSocket: 웹에서 받은 GPS 웨이포인트를 ROS 토픽으로 발행
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


# HTTP & WebSocket 설정
HTTP_PORT = 8000
GPS_WEBSOCKET_PORT = 8765
WAYPOINTS_WEBSOCKET_PORT = 8766

# 웹 디렉토리 경로
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(SCRIPT_DIR, "../web")

# 전역 데이터 (쓰레드 안전)
latest_gps_data = None
data_lock = threading.Lock()


class CustomHTTPHandler(http.server.SimpleHTTPRequestHandler):
    """커스텀 HTTP 핸들러 (로그 억제)"""
    def log_message(self, format, *args):
        pass  # HTTP 로그 억제

    def do_GET(self):
        if self.path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return
        return super().do_GET()


def start_http_server():
    """HTTP 서버 시작"""
    try:
        os.chdir(WEB_DIR)
        with socketserver.TCPServer(("", HTTP_PORT), CustomHTTPHandler) as httpd:
            rospy.loginfo(f"🌍 HTTP 서버 시작: http://localhost:{HTTP_PORT}")
            httpd.serve_forever()
    except OSError as e:
        rospy.logerr(f"❌ HTTP 서버 포트({HTTP_PORT}) 사용 중: {e}")
    except Exception as e:
        rospy.logerr(f"❌ HTTP 서버 시작 실패: {e}")


def open_browser():
    """웹 브라우저 자동 실행"""
    url = f"http://localhost:{HTTP_PORT}/index.html"
    rospy.loginfo(f"🌐 브라우저 열기: {url}")
    try:
        webbrowser.open(url)
    except Exception as e:
        rospy.logwarn(f"⚠️  브라우저 자동 실행 실패: {e}")


def gps_callback(msg):
    """GPS 데이터 수신"""
    global latest_gps_data

    if msg.status.status < 0:
        return

    with data_lock:
        latest_gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status,
            'timestamp': time.time()
        }


async def gps_websocket_handler(websocket, path):
    """GPS 데이터를 웹으로 전송하는 WebSocket 핸들러"""
    client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
    rospy.loginfo(f"📡 GPS WebSocket 연결: {client_info}")

    try:
        while True:
            with data_lock:
                gps_data = latest_gps_data

            if gps_data:
                await websocket.send(json.dumps(gps_data))
            else:
                await websocket.send(json.dumps({'error': 'GPS 데이터 없음'}))

            await asyncio.sleep(1)

    except websockets.exceptions.ConnectionClosed:
        rospy.loginfo(f"📡 GPS WebSocket 연결 해제: {client_info}")
    except Exception as e:
        rospy.logwarn(f"⚠️  GPS WebSocket 오류 ({client_info}): {e}")


async def start_gps_websocket_server():
    """GPS WebSocket 서버 시작"""
    try:
        rospy.loginfo(f"🔗 GPS WebSocket 시작: ws://localhost:{GPS_WEBSOCKET_PORT}")
        async with websockets.serve(gps_websocket_handler, "localhost", GPS_WEBSOCKET_PORT):
            await asyncio.Future()
    except OSError as e:
        rospy.logerr(f"❌ GPS WebSocket 포트({GPS_WEBSOCKET_PORT}) 사용 중: {e}")
    except Exception as e:
        rospy.logerr(f"❌ GPS WebSocket 시작 실패: {e}")


async def waypoints_websocket_handler(websocket, path):
    """웹에서 받은 GPS 웨이포인트를 ROS 토픽으로 발행"""
    client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
    rospy.loginfo(f"🔗 Waypoint WebSocket 연결: {client_info}")

    pub = rospy.Publisher('/waypoints_gps', String, queue_size=10)

    try:
        async for message in websocket:
            try:
                waypoints_data = json.loads(message)

                if 'waypoints' not in waypoints_data:
                    error_msg = "잘못된 형식: 'waypoints' 키 없음"
                    rospy.logwarn(f"⚠️  {error_msg}")
                    await websocket.send(json.dumps({'error': error_msg}))
                    continue

                # 웨이포인트 검증
                valid_waypoints = []
                for i, wp in enumerate(waypoints_data['waypoints']):
                    if 'lat' not in wp or 'lon' not in wp:
                        rospy.logwarn(f"⚠️  웨이포인트 {i}: GPS 좌표 누락")
                        continue

                    try:
                        lat = float(wp['lat'])
                        lon = float(wp['lon'])

                        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                            rospy.logwarn(f"⚠️  웨이포인트 {i}: GPS 좌표 범위 오류")
                            continue

                        valid_waypoints.append({'lat': lat, 'lon': lon})

                    except (ValueError, TypeError):
                        rospy.logwarn(f"⚠️  웨이포인트 {i}: GPS 좌표 형식 오류")
                        continue

                if not valid_waypoints:
                    error_msg = "유효한 웨이포인트 없음"
                    rospy.logwarn(f"⚠️  {error_msg}")
                    await websocket.send(json.dumps({'error': error_msg}))
                    continue

                # ROS 토픽으로 발행
                output_data = {'waypoints': valid_waypoints}
                pub.publish(json.dumps(output_data))

                rospy.loginfo(f"📥 GPS 웨이포인트 수신: {len(valid_waypoints)}개")

                # 성공 응답
                response = {
                    'status': 'success',
                    'total_received': len(waypoints_data['waypoints']),
                    'valid_waypoints': len(valid_waypoints)
                }
                await websocket.send(json.dumps(response))

            except json.JSONDecodeError as e:
                error_msg = f"JSON 파싱 오류: {e}"
                rospy.logerr(f"❌ {error_msg}")
                await websocket.send(json.dumps({'error': error_msg}))

            except Exception as e:
                error_msg = f"웨이포인트 처리 오류: {e}"
                rospy.logerr(f"❌ {error_msg}")
                try:
                    await websocket.send(json.dumps({'error': error_msg}))
                except:
                    pass

    except websockets.exceptions.ConnectionClosed:
        rospy.loginfo(f"📡 Waypoint WebSocket 연결 해제: {client_info}")
    except Exception as e:
        rospy.logerr(f"❌ Waypoint WebSocket 오류 ({client_info}): {e}")


async def start_waypoints_websocket_server():
    """Waypoint WebSocket 서버 시작"""
    try:
        rospy.loginfo(f"🔗 Waypoint WebSocket 시작: ws://localhost:{WAYPOINTS_WEBSOCKET_PORT}")
        async with websockets.serve(waypoints_websocket_handler, "localhost", WAYPOINTS_WEBSOCKET_PORT):
            await asyncio.Future()
    except OSError as e:
        rospy.logerr(f"❌ Waypoint WebSocket 포트({WAYPOINTS_WEBSOCKET_PORT}) 사용 중: {e}")
    except Exception as e:
        rospy.logerr(f"❌ Waypoint WebSocket 시작 실패: {e}")


if __name__ == '__main__':
    try:
        rospy.init_node('web_server', anonymous=True)
        rospy.loginfo("🚀 Web Server 시작")

        # GPS 구독
        rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)

        # HTTP 서버 (백그라운드)
        threading.Thread(target=start_http_server, daemon=True).start()

        # 서버 안정화 대기 후 브라우저 열기
        time.sleep(2)
        open_browser()

        # GPS WebSocket 서버 (백그라운드)
        threading.Thread(
            target=lambda: asyncio.run(start_gps_websocket_server()),
            daemon=True
        ).start()

        # Waypoint WebSocket 서버 (백그라운드)
        threading.Thread(
            target=lambda: asyncio.run(start_waypoints_websocket_server()),
            daemon=True
        ).start()

        rospy.loginfo("✅ 모든 서버 시작 완료")
        rospy.loginfo(f"   🌍 웹: http://localhost:{HTTP_PORT}")
        rospy.loginfo(f"   📡 GPS WebSocket: ws://localhost:{GPS_WEBSOCKET_PORT}")
        rospy.loginfo(f"   🗺️  Waypoint WebSocket: ws://localhost:{WAYPOINTS_WEBSOCKET_PORT}")

        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("🛑 Web Server 종료")
    except Exception as e:
        rospy.logerr(f"❌ Web Server 시작 실패: {e}")
