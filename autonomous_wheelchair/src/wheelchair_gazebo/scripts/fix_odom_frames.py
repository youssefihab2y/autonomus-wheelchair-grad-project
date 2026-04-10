#!/usr/bin/env python3
"""
Fix odometry frame names from Gazebo's prefixed format to standard ROS frames.

Gazebo Fortress automatically prefixes frame names with model name:
- autonomous_wheelchair/odom -> odom
- autonomous_wheelchair/chassis -> base_uplayer_link
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometryFrameFixer(Node):
    def __init__(self):
        super().__init__('odom_frame_fixer')
        
        # Subscribe to odometry with Gazebo's prefixed frames
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publish odometry with corrected frame names
        self.publisher = self.create_publisher(
            Odometry,
            '/odom_fixed',
            10
        )
        
        self.get_logger().info('Odometry frame fixer started')
        self.get_logger().info('Subscribing to: /odom')
        self.get_logger().info('Publishing to: /odom_fixed')
    
    def odom_callback(self, msg):
        # Create new message with corrected frame names
        fixed_msg = Odometry()
        fixed_msg.header = msg.header
        
        # Fix frame_id: autonomous_wheelchair/odom -> odom
        if msg.header.frame_id.startswith('autonomous_wheelchair/'):
            fixed_msg.header.frame_id = msg.header.frame_id.replace('autonomous_wheelchair/', '')
        else:
            fixed_msg.header.frame_id = msg.header.frame_id
        
        # Fix child_frame_id: autonomous_wheelchair/chassis -> base_uplayer_link
        if msg.child_frame_id == 'autonomous_wheelchair/chassis':
            fixed_msg.child_frame_id = 'base_uplayer_link'
        elif msg.child_frame_id.startswith('autonomous_wheelchair/'):
            fixed_msg.child_frame_id = msg.child_frame_id.replace('autonomous_wheelchair/', '')
        else:
            fixed_msg.child_frame_id = msg.child_frame_id
        
        # Copy all other fields
        fixed_msg.pose = msg.pose
        fixed_msg.twist = msg.twist
        
        # Publish fixed message
        self.publisher.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryFrameFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

