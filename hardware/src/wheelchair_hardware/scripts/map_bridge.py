#!/usr/bin/env python3
"""
Map Bridge — ROS2 node + HTTP server for the web dashboard.

Subscribes to:
  /map                      — static occupancy grid (latched)
  /local_costmap/costmap    — live local costmap
  /amcl_pose                — robot pose in map frame (from AMCL)
  /odom                     — fallback pose before AMCL converges
  /goal_pose                — current navigation goal

Serves at http://0.0.0.0:5000:
  GET /map.json             — static map + robot + goal overlay
  GET /costmap.json         — local costmap + robot + goal overlay
"""

import json
import math
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
)
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage

_bridge: 'MapBridge | None' = None


class MapBridge(Node):
    def __init__(self):
        super().__init__('map_bridge')

        self._lock        = threading.Lock()
        self._map_info    = None
        self._map_data    = None
        self._cost_info   = None
        self._cost_data   = None
        self._robot       = None
        self._goal        = None
        self._has_amcl    = False
        self._camera_jpeg = None   # latest YOLO-annotated JPEG bytes

        # /map uses transient-local (latched) QoS
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(OccupancyGrid,             '/map',                      self._map_cb,    latched)
        self.create_subscription(OccupancyGrid,             '/local_costmap/costmap',    self._cost_cb,   10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',                self._amcl_cb,   10)
        self.create_subscription(Odometry,                  '/odom',                     self._odom_cb,   10)
        self.create_subscription(PoseStamped,               '/goal_pose',                self._goal_cb,   10)
        self.create_subscription(CompressedImage,           '/yolo/image/compressed',    self._cam_cb,    1)

        self.get_logger().info('Map bridge ready → http://0.0.0.0:5000')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._map_info = _info(msg.info)
            self._map_data = list(msg.data)

    def _cost_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._cost_info = _info(msg.info)
            self._cost_data = list(msg.data)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        pose = _pose2d(msg.pose.pose)
        with self._lock:
            self._has_amcl = True
            self._robot    = pose

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            if not self._has_amcl:
                self._robot = _pose2d(msg.pose.pose)

    def _goal_cb(self, msg: PoseStamped):
        with self._lock:
            self._goal = {'x': msg.pose.position.x, 'y': msg.pose.position.y}

    def _cam_cb(self, msg: CompressedImage):
        with self._lock:
            self._camera_jpeg = bytes(msg.data)

    # ── JSON builders ─────────────────────────────────────────────────────────

    def _build(self, info, data):
        if info is None or data is None:
            return {'status': 'waiting'}
        resp = {'status': 'ok', 'info': info, 'data': data}
        if self._robot:
            resp['robot_pose'] = self._robot
        if self._goal:
            resp['goal_pose']  = self._goal
        return resp

    def map_json(self) -> bytes:
        with self._lock:
            d = self._build(self._map_info, self._map_data)
        return json.dumps(d, separators=(',', ':')).encode()

    def costmap_json(self) -> bytes:
        with self._lock:
            d = self._build(self._cost_info, self._cost_data)
        return json.dumps(d, separators=(',', ':')).encode()


# ── helpers ───────────────────────────────────────────────────────────────────

def _info(i):
    return {
        'resolution': i.resolution,
        'width':      i.width,
        'height':     i.height,
        'origin': {'x': i.origin.position.x, 'y': i.origin.position.y},
    }

def _pose2d(pose):
    p = pose.position
    o = pose.orientation
    yaw = math.atan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z),
    )
    return {'x': p.x, 'y': p.y, 'yaw': yaw}


# ── HTTP handler ──────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # silence access log spam

    def _send(self, body: bytes):
        self.send_response(200)
        self.send_header('Content-Type',                'application/json')
        self.send_header('Content-Length',              str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin',  '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.end_headers()

    def do_GET(self):
        if _bridge is None:
            self._send(b'{"status":"waiting"}')
            return
        if self.path == '/map.json':
            self._send(_bridge.map_json())
        elif self.path == '/costmap.json':
            self._send(_bridge.costmap_json())
        elif self.path.startswith('/camera.jpg'):
            with _bridge._lock:
                jpeg = _bridge._camera_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type',                'image/jpeg')
                self.send_header('Content-Length',              str(len(jpeg)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control',               'no-store')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(204)  # No Content — camera not yet available
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    global _bridge
    rclpy.init()
    _bridge = MapBridge()

    # ROS spin in background thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(_bridge,), daemon=True, name='ros-spin'
    )
    spin_thread.start()

    server = HTTPServer(('0.0.0.0', 5000), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        _bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
