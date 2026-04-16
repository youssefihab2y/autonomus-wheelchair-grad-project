#!/usr/bin/env python3
"""Publish sensor_msgs/CameraInfo for the Gazebo depth camera (matches wheelchair.gazebo.xacro)."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class DepthCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('depth_camera_info_publisher')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('horizontal_fov', 1.047)
        self._pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self._timer = self.create_timer(0.1, self._publish)
        self._base = self._make_camera_info()

    def _make_camera_info(self):
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        hfov = float(self.get_parameter('horizontal_fov').value)
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        cx = w / 2.0
        cy = h / 2.0
        msg = CameraInfo()
        msg.width = w
        msg.height = h
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.binning_x = 0
        msg.binning_y = 0
        return msg

    def _publish(self):
        msg = self._base
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
