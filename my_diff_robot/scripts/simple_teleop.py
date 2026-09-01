#!/usr/bin/env python3
import sys, tty, termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_raw', 10)
        print("w=fwd x=back a=left d=right s=stop q=quit")

    def run(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        while True:
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                key = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)

            if key == 'w': msg.twist.linear.x, msg.twist.angular.z = 0.5, 0.0
            elif key == 'x': msg.twist.linear.x, msg.twist.angular.z = -0.5, 0.0
            elif key == 'a': msg.twist.linear.x, msg.twist.angular.z = 0.0, 1.0
            elif key == 'd': msg.twist.linear.x, msg.twist.angular.z = 0.0, -1.0
            elif key == 's': msg.twist.linear.x, msg.twist.angular.z = 0.0, 0.0
            elif key in ['q', '\x03']: break
            else: continue
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)

def main():
    rclpy.init()
    SimpleTeleop().run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()