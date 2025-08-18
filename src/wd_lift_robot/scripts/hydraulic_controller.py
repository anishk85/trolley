#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math

class HydraulicLiftController(Node):
    def __init__(self):
        super().__init__('hydraulic_lift_controller')
        
        # Action client for lift control
        self.lift_action_client = ActionClient(
            self, FollowJointTrajectory, '/lift_controller/follow_joint_trajectory'
        )
        
        # Subscribers
        self.lift_cmd_sub = self.create_subscription(
            Float64MultiArray, 'lift_command', self.lift_command_callback, 10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        
        # Publishers
        self.lift_status_pub = self.create_publisher(Float64MultiArray, 'lift_status', 10)
        
        # Variables
        self.current_lift_position = 0.0
        self.lift_limits = {'min': 0.0, 'max': 0.5}
        
        self.get_logger().info('Hydraulic Lift Controller initialized')
    
    def joint_state_callback(self, msg):
        """Update current lift position from joint states"""
        if 'lift_joint' in msg.name:
            idx = msg.name.index('lift_joint')
            self.current_lift_position = msg.position[idx]
            
            # Publish status
            status_msg = Float64MultiArray()
            status_msg.data = [self.current_lift_position]
            self.lift_status_pub.publish(status_msg)
    
    def lift_command_callback(self, msg):
        """Handle lift position commands"""
        if len(msg.data) > 0:
            target_position = msg.data[0]
            self.move_lift_to_position(target_position)
    
    def move_lift_to_position(self, target_position):
        """Move lift to specified position"""
        # Clamp position to limits
        target_position = max(self.lift_limits['min'], 
                            min(self.lift_limits['max'], target_position))
        
        if not self.lift_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Lift action server not available')
            return
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['lift_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [target_position]
        point.time_from_start.sec = 2  # 2 seconds to reach position
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        future = self.lift_action_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Moving lift to position: {target_position}')

def main(args=None):
    rclpy.init(args=args)
    controller = HydraulicLiftController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()