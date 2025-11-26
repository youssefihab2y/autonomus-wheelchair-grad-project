#!/usr/bin/env python3
"""
Filter lidar scan to remove detections from robot's own body (back bar, seat, etc.)
This prevents the robot from seeing itself as an obstacle.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__('lidar_self_filter')
        
        # Subscribe to raw lidar scan (from Gazebo bridge)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Subscribe to bridged scan topic
            self.scan_callback,
            10
        )
        
        # Publish filtered scan (same topic, will replace the original)
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_filtered',  # Filtered scan topic (Nav2 can use this)
            10
        )
        
        # Define regions to filter (relative to lidar frame)
        # Lidar is at x=0.15, z=0.35 from chassis
        # Back bar is at x=-0.2, z=0.22 from seat, seat is at z=0.25 from chassis
        # So back bar is roughly at x=-0.04, z=0.47 from chassis
        # Relative to lidar (x=0.15, z=0.35): back bar is at x=-0.19, z=0.12
        
        # Filter region: back bar and seat area (behind the robot)
        # Lidar is at front (x=0.15), back bar is at back (x=-0.2 from seat, seat at x=-0.04)
        # Filter angles roughly 120° to 240° (behind robot) and close ranges
        self.filter_angle_start = math.radians(120)  # Start filtering at 120°
        self.filter_angle_end = math.radians(240)    # Stop filtering at 240°
        self.filter_range_max = 0.6  # Maximum range to filter within this angle (m)
        
        self.get_logger().info('Lidar self-filter started')
        self.get_logger().info(f'Filtering angles: {math.degrees(self.filter_angle_start):.1f}° to {math.degrees(self.filter_angle_end):.1f}°')
        self.get_logger().info(f'Filtering range: 0.0 to {self.filter_range_max:.2f}m')
        self.get_logger().info('Publishing filtered scan to /scan_filtered')
    
    def scan_callback(self, msg):
        # Create filtered scan
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = msg.header.frame_id
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        
        # Filter ranges
        filtered_ranges = []
        filtered_intensities = []
        
        for i, range_val in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Check if this reading is in the filter region (back bar area)
            # Filter angles roughly from 120° to 240° (behind the robot)
            # And ranges less than filter_range_max (close to robot body)
            # Normalize angle to [0, 2π] range
            normalized_angle = angle
            while normalized_angle < 0:
                normalized_angle += 2 * math.pi
            while normalized_angle >= 2 * math.pi:
                normalized_angle -= 2 * math.pi
            
            # Check if angle is in filter range (behind robot)
            in_filter_angle = (self.filter_angle_start <= normalized_angle <= self.filter_angle_end)
            
            if in_filter_angle:
                if range_val < self.filter_range_max and not math.isinf(range_val):
                    # Set to max range (ignore this reading)
                    filtered_ranges.append(float('inf'))
                    if len(msg.intensities) > i:
                        filtered_intensities.append(0.0)
                    else:
                        filtered_intensities.append(0.0)
                else:
                    filtered_ranges.append(range_val)
                    if len(msg.intensities) > i:
                        filtered_intensities.append(msg.intensities[i])
                    else:
                        filtered_intensities.append(0.0)
            else:
                # Keep original reading
                filtered_ranges.append(range_val)
                if len(msg.intensities) > i:
                    filtered_intensities.append(msg.intensities[i])
                else:
                    filtered_intensities.append(0.0)
        
        filtered_msg.ranges = filtered_ranges
        if len(msg.intensities) > 0:
            filtered_msg.intensities = filtered_intensities
        
        # Publish filtered scan
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSelfFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

