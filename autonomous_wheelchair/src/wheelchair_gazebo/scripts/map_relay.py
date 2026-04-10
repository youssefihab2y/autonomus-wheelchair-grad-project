#!/usr/bin/env python3
"""
Relay /map to /map_web with compatible QoS for rosbridge.

Map server publishes /map with TRANSIENT_LOCAL (one-shot). Rosbridge subscribes
with VOLATILE by default and never receives it. This node subscribes with
TRANSIENT_LOCAL, gets the map, and republishes to /map_web with VOLATILE
so the web dashboard can receive it.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid


class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        self.last_map = None

        # QoS to match map_server (TRANSIENT_LOCAL)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.callback,
            map_qos,
        )

        # Publisher uses default QoS (VOLATILE)
        self.pub = self.create_publisher(OccupancyGrid, '/map_web', 1)
        # Republish map every 2 seconds so late-joining web clients receive it
        self.timer = self.create_timer(2.0, self.republish)
        self.get_logger().info('Map relay: /map -> /map_web (republishing every 2s for late joiners)')

    def callback(self, msg):
        self.last_map = msg
        self.pub.publish(msg)
        self.get_logger().info(
            'Relayed map: %dx%d, resolution %.3f' % (msg.info.width, msg.info.height, msg.info.resolution),
            throttle_duration_sec=5.0
        )

    def republish(self):
        if self.last_map is not None:
            self.pub.publish(self.last_map)


def main():
    rclpy.init()
    node = MapRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
