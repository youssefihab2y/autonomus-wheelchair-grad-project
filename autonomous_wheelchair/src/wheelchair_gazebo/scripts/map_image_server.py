#!/usr/bin/env python3
"""
HTTP server for web dashboard: static map, costmap, robot pose, goal.
Run: ros2 run wheelchair_gazebo map_image_server.py
Endpoints: /map.json (static) | /costmap.json (global costmap)
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

QOS_MAP = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MapImageHandler(BaseHTTPRequestHandler):
    static_json = None
    costmap_json = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        path = self.path.split('?')[0]
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        if path == '/costmap.json' or path == '/costmap.json/':
            data = MapImageHandler.costmap_json or MapImageHandler.static_json
        else:
            data = MapImageHandler.static_json
        if data is None:
            self.wfile.write(json.dumps({'status': 'waiting'}).encode())
        else:
            self.wfile.write(data.encode())

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET')
        self.end_headers()


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_map_payload(data, info, robot_pose, goal_pose):
    d = {
        'status': 'ok',
        'info': {
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin': {'x': info.origin.position.x, 'y': info.origin.position.y},
        },
        'data': data,
        'robot_pose': robot_pose,
        'goal_pose': goal_pose,
    }
    return json.dumps(d)


class MapImageServer(Node):
    def __init__(self):
        super().__init__('map_image_server')
        self.robot_pose = None
        self.goal_pose = None
        self.static_data = None
        self.static_info = None
        self.costmap_data = None
        self.costmap_info = None

        self.create_subscription(OccupancyGrid, '/map', self.cb_map, QOS_MAP)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.cb_costmap, QOS_MAP)
        # Prefer localized pose from AMCL if available
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)
        # Fallback: raw odometry when AMCL is not active yet
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        self.get_logger().info('Map server: /map.json and /costmap.json on port 5000')

    def cb_map(self, msg):
        data = list(msg.data) if hasattr(msg.data, '__iter__') and not isinstance(msg.data, (str, bytes)) else []
        if len(data) != msg.info.width * msg.info.height:
            return
        self.static_data = data
        self.static_info = msg.info
        self._update_static()

    def cb_costmap(self, msg):
        data = list(msg.data) if hasattr(msg.data, '__iter__') and not isinstance(msg.data, (str, bytes)) else []
        if len(data) != msg.info.width * msg.info.height:
            return
        self.costmap_data = data
        self.costmap_info = msg.info
        self._update_costmap()

    def cb_amcl(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.robot_pose = {'x': p.x, 'y': p.y, 'yaw': quat_to_yaw(o)}
        self._update_static()
        self._update_costmap()

    def cb_odom(self, msg: Odometry):
        # Only use odom as fallback until AMCL pose is available.
        if self.robot_pose is not None:
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.robot_pose = {'x': p.x, 'y': p.y, 'yaw': quat_to_yaw(o)}
        self._update_static()
        self._update_costmap()

    def cb_goal(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self.goal_pose = {'x': p.x, 'y': p.y, 'yaw': quat_to_yaw(o)}
        self._update_static()
        self._update_costmap()

    def _update_static(self):
        if self.static_data is not None and self.static_info is not None:
            MapImageHandler.static_json = make_map_payload(
                self.static_data, self.static_info, self.robot_pose, self.goal_pose)

    def _update_costmap(self):
        if self.costmap_data is not None and self.costmap_info is not None:
            MapImageHandler.costmap_json = make_map_payload(
                self.costmap_data, self.costmap_info, self.robot_pose, self.goal_pose)


def main():
    rclpy.init()
    node = MapImageServer()

    server = HTTPServer(('', 5000), MapImageHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
