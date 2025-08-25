#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        
        # Subscribe to Twist from nav2
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav_smooth',
            self.twist_callback,
            10
        )
        
        # Publish TwistStamped to robot
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        
        # Add direct Twist publisher as backup
        self.twist_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Counter for diagnostics
        self.message_count = 0
        self.last_nonzero_cmd = None
        
        # Timer for diagnostics
        self.timer = self.create_timer(5.0, self.print_diagnostics)
        
        self.get_logger().info('Twist converter started with diagnostics')
        self.get_logger().info('Input: /cmd_vel_nav_smooth')
        self.get_logger().info('Outputs: /diff_drive_controller/cmd_vel, /cmd_vel')
    
    def twist_callback(self, msg):
        self.message_count += 1
        
        # Convert Twist to TwistStamped
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist = msg
        
        # Publish both formats
        self.publisher.publish(twist_stamped)
        self.twist_publisher.publish(msg)
        
        # Track non-zero commands
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_nonzero_cmd = msg
            self.get_logger().info(
                f'CMD: linear={msg.linear.x:.3f} m/s, angular={msg.angular.z:.3f} rad/s'
            )
    
    def print_diagnostics(self):
        self.get_logger().info(f'Messages processed: {self.message_count}')
        if self.last_nonzero_cmd:
            self.get_logger().info(
                f'Last command: linear={self.last_nonzero_cmd.linear.x:.3f}, '
                f'angular={self.last_nonzero_cmd.angular.z:.3f}'
            )
        else:
            self.get_logger().warn('No non-zero commands received yet!')

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()