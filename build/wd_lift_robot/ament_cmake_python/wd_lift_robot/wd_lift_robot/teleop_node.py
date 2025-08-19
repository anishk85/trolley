#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import termios
import tty

class SimpleTeleopNode(Node):
    def __init__(self):
        super().__init__('simple_teleop_node')
        
        # Publishers - use TwistStamped for diff_drive_controller
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.lift_pub = self.create_publisher(JointTrajectory, '/lift_controller/joint_trajectory', 10)
        
        # Movement parameters
        self.linear_speed = 1.5
        self.angular_speed = 1.5
        self.lift_position = 0.0
        self.lift_step = 0.1
        
        # Terminal settings
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except:
            print("Error: This script must be run in a terminal")
            sys.exit(1)
        
        self.get_logger().info("Simple WASD Teleop started!")
        print("\n=== WASD ROBOT CONTROL ===")
        print("W: Forward")
        print("S: Backward") 
        print("A: Turn Left")
        print("D: Turn Right")
        print("Q: Lift Up")
        print("E: Lift Down")
        print("X: Stop")
        print("CTRL-C: Quit")
        print("Press keys to move robot...")
        print("========================\n")

    def get_key(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except:
            return ''

    def send_lift_command(self, position):
        """Send lift command using JointTrajectory"""
        traj = JointTrajectory()
        traj.joint_names = ['lift_joint']
        traj.header.stamp = self.get_clock().now().to_msg()
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        
        traj.points = [point]
        self.lift_pub.publish(traj)

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                
                # Create TwistStamped message
                twist_stamped = TwistStamped()
                twist_stamped.header.stamp = self.get_clock().now().to_msg()
                twist_stamped.header.frame_id = "base_link"
                
                if key == 'w':
                    twist_stamped.twist.linear.x = self.linear_speed
                    print("→ Forward")
                elif key == 's':
                    twist_stamped.twist.linear.x = -self.linear_speed
                    print("→ Backward")
                elif key == 'a':
                    twist_stamped.twist.angular.z = self.angular_speed
                    print("→ Turn Left")
                elif key == 'd':
                    twist_stamped.twist.angular.z = -self.angular_speed
                    print("→ Turn Right")
                elif key == 'q':
                    self.lift_position += self.lift_step
                    self.lift_position = min(0.5, self.lift_position)
                    self.send_lift_command(self.lift_position)
                    print(f"→ Lift Up: {self.lift_position:.2f}")
                elif key == 'e':
                    self.lift_position -= self.lift_step
                    self.lift_position = max(0.0, self.lift_position)
                    self.send_lift_command(self.lift_position)
                    print(f"→ Lift Down: {self.lift_position:.2f}")
                elif key == 'x':
                    twist_stamped.twist.linear.x = 0.0
                    twist_stamped.twist.angular.z = 0.0
                    print("→ Stop")
                elif key == '\x03':  # CTRL-C
                    break
                
                # Publish the movement command
                self.cmd_vel_pub.publish(twist_stamped)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Stop the robot
            stop_twist = TwistStamped()
            stop_twist.header.stamp = self.get_clock().now().to_msg()
            stop_twist.header.frame_id = "base_link"
            self.cmd_vel_pub.publish(stop_twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = SimpleTeleopNode()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()