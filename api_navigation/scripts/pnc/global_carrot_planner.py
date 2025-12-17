#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import asyncio
import websockets
from threading import Thread, Lock
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path as FilePath

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal, ExePathAction, ExePathGoal
import actionlib


class GPSHandler:
    def __init__(self):
        self.lock = Lock()
        self.current = None

    def update(self, lat, lon, alt):
        with self.lock:
            self.current = {'lat': lat, 'lon': lon, 'alt': alt}

    def get(self):
        with self.lock:
            return self.current


class CarrotGoalPublisher:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base_flex/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base_flex action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base_flex")

    def publish_goal(self, x, y, theta=0.0):
        """Send goal to move_base_flex with carrot planner"""
        import math

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # Convert theta to quaternion
        goal.target_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(theta / 2.0)

        # Specify global_planner (simple straight-line planner)
        goal.planner = 'global_planner'
        goal.controller = 'dwa_controller'

        self.client.send_goal(goal)
        rospy.loginfo(f"Sent carrot planner goal: ({x:.2f}, {y:.2f}, theta={theta:.2f})")

    def cancel_goal(self):
        """Cancel current goal"""
        self.client.cancel_all_goals()
        rospy.loginfo("Cancelled current goal")


class WebSocketServer:
    def __init__(self, gps_handler, goal_publisher, port=5500):
        self.gps = gps_handler
        self.goal_pub = goal_publisher
        self.port = port

    async def handler(self, ws, path):
        send_task = asyncio.create_task(self._send_gps(ws))
        recv_task = asyncio.create_task(self._recv_goal(ws))
        await asyncio.gather(send_task, recv_task, return_exceptions=True)

    async def _send_gps(self, ws):
        while True:
            try:
                data = self.gps.get()
                await ws.send(json.dumps({'type': 'gps', 'data': data}))
                await asyncio.sleep(1.0)
            except:
                break

    async def _recv_goal(self, ws):
        try:
            async for msg in ws:
                data = json.loads(msg)

                if data.get('type') == 'goal':
                    # Receive single goal with x, y, theta
                    goal_data = data.get('goal', {})
                    x = goal_data.get('x', 0.0)
                    y = goal_data.get('y', 0.0)
                    theta = goal_data.get('theta', 0.0)

                    self.goal_pub.publish_goal(x, y, theta)
                    await ws.send(json.dumps({'type': 'ack', 'status': 'goal_sent'}))

                elif data.get('type') == 'cancel':
                    self.goal_pub.cancel_goal()
                    await ws.send(json.dumps({'type': 'ack', 'status': 'cancelled'}))
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
            self.path = '/carrot.html'
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
    rospy.init_node('global_carrot_planner')

    gps_handler = GPSHandler()
    goal_publisher = CarrotGoalPublisher()

    rospy.Subscriber('/ublox/fix', NavSatFix, lambda msg: gps_callback(msg, gps_handler))

    web_dir = rospy.get_param('~web_dir', FilePath(__file__).parent.parent.parent / 'web')

    http_thread = Thread(target=lambda: WebServer(web_dir, 8001).start(), daemon=True)
    http_thread.start()

    ws_server = WebSocketServer(gps_handler, goal_publisher, 8801)
    ws_thread = Thread(target=lambda: asyncio.run(ws_server.start()), daemon=True)
    ws_thread.start()

    rospy.loginfo("Global Carrot Planner node started")
    rospy.loginfo("Use with move_base configured with carrot_planner/CarrotPlanner")
    rospy.spin()


if __name__ == '__main__':
    main()
