#!/usr/bin/env python
'''
    square_drive.py
    This will drive the turtlebot in a square and then stop.
    Jake Yax, Sep 2023
'''

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class SquareDrive(Node):
    def __init__(self, topic_name='/cmd_vel'):
        super().__init__('square_drive')
        self.publisher_ = self.create_publisher(Twist, topic_name, 1)
        time.sleep(2.0)
        self.get_logger().info('Publishing: ' + topic_name)

    def drive(self, wait, motion = Twist() ):
        info = f'Drive for {wait:.2f} sec, linear speed {motion.linear.x:.2f} m/s, angular speed {motion.angular.z:.2f} rad/s'
        self.get_logger().info(info)
        self.publisher_.publish( motion )
        time.sleep( wait )
        self.publisher_.publish( Twist() )  # All-zero motion
        self.get_logger().info('Done')

def main(args=None):
    rclpy.init(args=args)

    av = SquareDrive()
    motion_corner = Twist( linear=Vector3(x=0.2), angular=Vector3(z=0.5))
    motion_side = Twist( linear=Vector3(x=0.2), angular=Vector3(z=0.0))
    wait_corner = math.pi / (1.47 * motion_corner.angular.z)
    wait_side = 1 / motion_side.linear.x
    for i in range(0,4):
        av.drive(wait = wait_side, motion = motion_side)
        av.drive(wait = wait_corner, motion = motion_corner)
