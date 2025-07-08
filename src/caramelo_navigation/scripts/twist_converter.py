#!/usr/bin/env python3
"""
Twist to TwistStamped Converter
==============================

Converts standard Twist messages to TwistStamped messages for mecanum drive controller.
This is essential for proper mecanum wheel control as per best practices.
"""

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        
        # Publisher for TwistStamped messages
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped, 'cmd_vel_out', 10)
        
        # Subscriber for Twist messages
        self.twist_sub = self.create_subscription(
            Twist, 'cmd_vel_in', self.twist_callback, 10)
        
        self.get_logger().info('Twist to TwistStamped converter started')
    
    def twist_callback(self, msg):
        """Convert Twist to TwistStamped"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist = msg
        
        self.twist_stamped_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    
    twist_converter = TwistConverter()
    
    try:
        rclpy.spin(twist_converter)
    except KeyboardInterrupt:
        pass
    finally:
        twist_converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
