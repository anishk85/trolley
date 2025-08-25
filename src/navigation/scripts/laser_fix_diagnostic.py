#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import math
import numpy as np

class LaserDiagnostic(Node):
    def __init__(self):
        super().__init__('laser_diagnostic')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.timer = self.create_timer(5.0, self.detailed_analysis)
        
        self.scan_data = []
        self.frame_issues = []
        
        self.get_logger().info('=== LASER SCANNER DIAGNOSTIC TOOL ===')
    
    def scan_callback(self, msg):
        if len(self.scan_data) < 5:  # Collect 5 samples
            self.scan_data.append(msg)
        
        # Check TF transforms
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                msg.header.frame_id,  # source frame  
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Transform is good
        except Exception as e:
            self.frame_issues.append(f"TF Error: {str(e)}")
    
    def detailed_analysis(self):
        if not self.scan_data:
            self.get_logger().error('NO LASER DATA RECEIVED!')
            return
            
        self.get_logger().info('=== DETAILED LASER ANALYSIS ===')
        
        # Analyze latest scan
        latest = self.scan_data[-1]
        ranges = np.array(latest.ranges)
        
        # Basic stats
        valid_mask = np.isfinite(ranges) & (ranges > 0)
        valid_ranges = ranges[valid_mask]
        
        self.get_logger().info(f'Frame ID: {latest.header.frame_id}')
        self.get_logger().info(f'Total points: {len(ranges)}')
        self.get_logger().info(f'Valid points: {len(valid_ranges)}')
        self.get_logger().info(f'Infinite points: {np.sum(np.isinf(ranges))}')
        self.get_logger().info(f'NaN points: {np.sum(np.isnan(ranges))}')
        self.get_logger().info(f'Zero points: {np.sum(ranges == 0)}')
        
        if len(valid_ranges) > 0:
            self.get_logger().info(f'Valid range min/max: {np.min(valid_ranges):.3f} / {np.max(valid_ranges):.3f}')
            self.get_logger().info(f'Scanner limits: {latest.range_min:.3f} / {latest.range_max:.3f}')
        
        # Check for problematic patterns
        inf_ratio = np.sum(np.isinf(ranges)) / len(ranges)
        if inf_ratio > 0.7:
            self.get_logger().error(f'CRITICAL: {inf_ratio*100:.1f}% of readings are infinite!')
            self.get_logger().error('This will cause phantom obstacles everywhere!')
            
        if len(valid_ranges) < 50:
            self.get_logger().error('CRITICAL: Very few valid laser readings!')
            
        # Check angular setup
        angle_span = latest.angle_max - latest.angle_min
        expected_points = int(angle_span / latest.angle_increment)
        self.get_logger().info(f'Angular span: {math.degrees(angle_span):.1f} degrees')
        self.get_logger().info(f'Expected points: {expected_points}, Actual: {len(ranges)}')
        
        # TF issues
        if self.frame_issues:
            self.get_logger().error('TF TRANSFORM ISSUES:')
            for issue in self.frame_issues[-3:]:  # Show last 3
                self.get_logger().error(f'  {issue}')
        
        # Recommendations
        self.get_logger().info('=== RECOMMENDATIONS ===')
        if inf_ratio > 0.3:
            self.get_logger().warn('1. Your laser scanner is sending too many infinite readings')
            self.get_logger().warn('   - Check if scanner is properly configured')
            self.get_logger().warn('   - Verify scanner is not blocked or damaged')
            self.get_logger().warn('   - Try: inf_is_valid: false in costmap config')
            
        if latest.header.frame_id not in ['laser', 'base_scan', 'scan_link']:
            self.get_logger().warn(f'2. Unusual laser frame: {latest.header.frame_id}')
            self.get_logger().warn('   - Verify this frame exists in your robot URDF')
            
        if self.frame_issues:
            self.get_logger().warn('3. TF transform issues detected')
            self.get_logger().warn('   - Run: ros2 run tf2_tools view_frames.py')
            self.get_logger().warn('   - Check your robot URDF has correct laser frame')
        
        self.get_logger().info('=== END ANALYSIS ===\n')

def main(args=None):
    rclpy.init(args=args)
    node = LaserDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()