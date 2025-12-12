#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
from threading import Thread, Lock
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path as FilePath
from pyproj import Transformer

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GPSHandler:
    def __init__(self):
        self.lock = Lock()
        self.current = None
        self.utm_converter = None
        self.origin = None

    def update(self, lat, lon, alt):
        with self.lock:
            if self.utm_converter is None:
                self._init_utm(lat, lon)
            self.current = {'lat': lat, 'lon': lon, 'alt': alt}

    def get(self):
        with self.lock:
            return self.current

    def _init_utm(self, lat, lon):
        zone = int((lon + 180) / 6) + 1
        hemisphere = 'north' if lat >= 0 else 'south'
        self.utm_converter = Transformer.from_crs(
            'EPSG:4326',
            f'+proj=utm +zone={zone} +{hemisphere} +datum=WGS84 +units=m',
            always_xy=True
        )
        x, y = self.utm_converter.transform(lon, lat)
        self.origin = (x, y)
        rospy.loginfo(f"UTM origin set: Zone {zone}{hemisphere[0].upper()}, ({x:.2f}, {y:.2f})")

    def to_map_coords(self, lat, lon):
        if self.utm_converter is None or self.origin is None:
            return None, None
        x, y = self.utm_converter.transform(lon, lat)
        return x - self.origin[0], y - self.origin[1]


class PathPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)

    def publish(self, waypoints, gps_handler):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        for wp in waypoints:
            x, y = gps_handler.to_map_coords(wp['lat'], wp['lon'])
            if x is None:
                continue

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        if msg.poses:
            self.pub.publish(msg)
            rospy.loginfo(f"Published path: {len(msg.poses)} waypoints")


class WebSocketServer:
    def __init__(self, gps_handler, path_publisher, port=5500):
        self.gps = gps_handler
        self.path_pub = path_publisher
        self.port = port

    async def handler(self, ws, path):
        send_task = asyncio.create_task(self._send_gps(ws))
        recv_task = asyncio.create_task(self._recv_path(ws))
        await asyncio.gather(send_task, recv_task, return_exceptions=True)

    async def _send_gps(self, ws):
        while True:
            try:
                data = self.gps.get()
                await ws.send(json.dumps({'type': 'gps', 'data': data}))
                await asyncio.sleep(1.0)
            except:
                break

    async def _recv_path(self, ws):
        try:
            async for msg in ws:
                data = json.loads(msg)
                if data.get('type') == 'path':
                    waypoints = data.get('waypoints', [])
                    self.path_pub.publish(waypoints, self.gps)
                    await ws.send(json.dumps({'type': 'ack', 'count': len(waypoints)}))
        except:
            pass

    async def start(self):
        async with websockets.serve(self.handler, 'localhost', self.port):
            rospy.loginfo(f"WebSocket server running on port {self.port}")
            await asyncio.Future()


class HTTPHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.path = '/index.html'
        return super().do_GET()


class WebServer:
    def __init__(self, web_dir, port=5000):
        self.port = port
        self.web_dir = FilePath(web_dir).resolve()

    def start(self):
        import os
        import socketserver
        os.chdir(self.web_dir)
        socketserver.TCPServer.allow_reuse_address = True
        server = HTTPServer(('', self.port), HTTPHandler)
        rospy.loginfo(f"HTTP server running on port {self.port}")
        server.serve_forever()


def gps_callback(msg, gps_handler):
    if msg.status.status >= 0:
        gps_handler.update(msg.latitude, msg.longitude, msg.altitude)


def main():
    rospy.init_node('global_kakao_api')

    gps_handler = GPSHandler()
    path_publisher = PathPublisher()

    rospy.Subscriber('/ublox/fix', NavSatFix, lambda msg: gps_callback(msg, gps_handler))

    web_dir = rospy.get_param('~web_dir', FilePath(__file__).parent.parent.parent / 'web')

    http_thread = Thread(target=lambda: WebServer(web_dir, 8000).start(), daemon=True)
    http_thread.start()

    ws_server = WebSocketServer(gps_handler, path_publisher, 8800)
    ws_thread = Thread(target=lambda: asyncio.run(ws_server.start()), daemon=True)
    ws_thread.start()

    rospy.loginfo("Global Kakao API node started")
    rospy.spin()


if __name__ == '__main__':
    main()
