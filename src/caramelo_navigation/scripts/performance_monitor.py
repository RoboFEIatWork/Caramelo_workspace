#!/usr/bin/env python3
"""
Performance Monitor for Caramelo Navigation
==========================================

Monitors system performance during navigation operations.
Based on Automatic Addison tutorial recommendations.
"""

import time

import psutil
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Publishers
        self.performance_pub = self.create_publisher(String, 'performance_status', 10)
        
        # Subscribers for monitoring
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Performance monitoring timer
        self.performance_timer = self.create_timer(5.0, self.check_performance)
        
        # Metrics
        self.cmd_vel_count = 0
        self.scan_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('Performance Monitor started')
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel_count += 1
    
    def scan_callback(self, msg):
        self.scan_count += 1
    
    def check_performance(self):
        """Check system performance metrics"""
        try:
            # CPU and Memory usage
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            
            # Check for performance issues
            performance_status = "OK"
            warnings = []
            
            if cpu_percent > 80:
                warnings.append(f"High CPU usage: {cpu_percent:.1f}%")
                performance_status = "WARNING"
            
            if memory_percent > 80:
                warnings.append(f"High memory usage: {memory_percent:.1f}%")
                performance_status = "WARNING"
            
            # Calculate message rates
            elapsed_time = time.time() - self.start_time
            cmd_vel_rate = self.cmd_vel_count / elapsed_time if elapsed_time > 0 else 0
            scan_rate = self.scan_count / elapsed_time if elapsed_time > 0 else 0
            
            # Log performance info
            self.get_logger().info(
                f"Performance: CPU={cpu_percent:.1f}%, "
                f"Memory={memory_percent:.1f}%, "
                f"cmd_vel_rate={cmd_vel_rate:.1f}Hz, "
                f"scan_rate={scan_rate:.1f}Hz"
            )
            
            if warnings:
                self.get_logger().warn(f"Performance warnings: {'; '.join(warnings)}")
            
            # Publish performance status
            status_msg = String()
            status_msg.data = f"Status: {performance_status}, CPU: {cpu_percent:.1f}%, Memory: {memory_percent:.1f}%"
            self.performance_pub.publish(status_msg)
            
            # Performance recommendations (as per Automatic Addison tutorial)
            if cpu_percent > 50:
                self.get_logger().warn(
                    "High CPU usage detected. Consider:\n"
                    "1. Reducing RViz usage in production\n"
                    "2. Lowering sensor frequencies\n"
                    "3. Optimizing navigation parameters"
                )
            
        except Exception as e:
            self.get_logger().error(f"Error in performance monitoring: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    performance_monitor = PerformanceMonitor()
    
    try:
        rclpy.spin(performance_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        performance_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
