#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import select
import termios
import tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Lift Control:
---------------------------
r/f : lift up/down

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

liftBindings = {
    'r': 0.1,  # lift up
    'f': -0.1,  # lift down
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, '/lift_controller/commands', 10)
        
        # Movement parameters
        self.speed = 0.5
        self.turn = 1.0
        self.lift_position = 0.0
        
        # Terminal settings - handle both terminal and non-terminal environments
        try:
            if sys.stdin.isatty():
                self.settings = termios.tcgetattr(sys.stdin)
                self.has_terminal = True
            else:
                self.has_terminal = False
                self.get_logger().warn("No terminal detected - teleop may not work properly")
        except:
            self.has_terminal = False
            self.get_logger().warn("Terminal setup failed - using keyboard input mode")
        
        self.get_logger().info("Teleop node started. Use keyboard to control the robot.")
        print(msg)

    def getKey(self, timeout=0.1):
        if not self.has_terminal:
            return ''
            
        try:
            tty.setcbreak(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
        except:
            return ''

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

    def run(self):
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        status = 0

        try:
            while rclpy.ok():
                key = self.getKey()
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]

                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                elif key in liftBindings.keys():
                    self.lift_position += liftBindings[key]
                    self.lift_position = max(0.0, min(0.5, self.lift_position))  # Clamp between 0 and 0.5
                    
                    # Publish lift command
                    lift_msg = Float64MultiArray()
                    lift_msg.data = [self.lift_position]
                    self.lift_pub.publish(lift_msg)
                    
                    print(f"Lift position: {self.lift_position:.2f}")
                else:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    th = 0.0
                    if (key == '\x03'):  # CTRL-C
                        break

                # Create and publish twist message
                twist = Twist()
                twist.linear.x = x * self.speed
                twist.linear.y = y * self.speed
                twist.linear.z = z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th * self.turn
                
                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            print(f"Error: {e}")

        finally:
            # Stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            if self.has_terminal:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()