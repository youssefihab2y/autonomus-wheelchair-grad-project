#!/usr/bin/env python3
"""
Publish odom->base_link TF transform from odometry messages.
This prevents RViz queue overflow by ensuring TF chain exists.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry to TF converter started')
        self.get_logger().info('Subscribing to: /odom')
        self.get_logger().info('Publishing TF: odom -> chassis (robot root)')
        self.get_logger().info('Publishing TF: odom -> base_uplayer_link (for compatibility)')
        self.get_logger().info('Publishing TF: odom -> base_link (ROS 2 standard)')
        self.msg_count = 0
    
    def odom_callback(self, msg):
        # Fix frame names (remove Gazebo prefixes if present)
        frame_id = msg.header.frame_id
        child_frame_id = msg.child_frame_id
        
        # Remove autonomous_wheelchair/ prefix if present
        if frame_id.startswith('autonomous_wheelchair/'):
            frame_id = frame_id.replace('autonomous_wheelchair/', '')
        if child_frame_id.startswith('autonomous_wheelchair/'):
            child_frame_id = child_frame_id.replace('autonomous_wheelchair/', '')
        
        # Ensure frame_id is 'odom' (required for TF tree root)
        if frame_id != 'odom':
            self.get_logger().warn(f'Unexpected frame_id: {frame_id}, expected "odom"')
            frame_id = 'odom'
        
        # Map chassis to base_uplayer_link for odometry
        # But we need to publish odom -> chassis (robot root) for complete TF chain
        # The odometry child_frame is base_uplayer_link, but chassis is the URDF root
        # So we publish: odom -> chassis (using odometry pose)
        # Then robot_state_publisher handles: chassis -> base_uplayer_link (from URDF)
        
        # Publish odom -> chassis (robot root)
        t_chassis = TransformStamped()
        t_chassis.header.stamp = msg.header.stamp
        t_chassis.header.frame_id = frame_id  # Should be "odom"
        t_chassis.child_frame_id = 'chassis'  # Robot root
        
        # Set translation from pose
        t_chassis.transform.translation.x = msg.pose.pose.position.x
        t_chassis.transform.translation.y = msg.pose.pose.position.y
        t_chassis.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation from pose
        t_chassis.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t_chassis)
        
        # Also publish odom -> base_uplayer_link (for compatibility with odometry child_frame)
        # This matches what the diff drive plugin expects
        t_base_uplayer = TransformStamped()
        t_base_uplayer.header.stamp = msg.header.stamp
        t_base_uplayer.header.frame_id = frame_id
        t_base_uplayer.child_frame_id = 'base_uplayer_link'
        t_base_uplayer.transform.translation.x = msg.pose.pose.position.x
        t_base_uplayer.transform.translation.y = msg.pose.pose.position.y
        t_base_uplayer.transform.translation.z = msg.pose.pose.position.z
        t_base_uplayer.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t_base_uplayer)
        
        # Also publish odom -> base_link (for ROS 2 compatibility)
        t_base = TransformStamped()
        t_base.header.stamp = msg.header.stamp
        t_base.header.frame_id = frame_id
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = msg.pose.pose.position.x
        t_base.transform.translation.y = msg.pose.pose.position.y
        t_base.transform.translation.z = msg.pose.pose.position.z
        t_base.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t_base)
        
        self.msg_count += 1
        if self.msg_count % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f'Published TF: {frame_id} -> chassis, {frame_id} -> base_uplayer_link, {frame_id} -> base_link (count: {self.msg_count})')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

