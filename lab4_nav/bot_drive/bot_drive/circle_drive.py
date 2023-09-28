#!/usr/bin/env python
'''
    CircleDrive.py 

    This is an exmple ROS 2 node for driving a Turtlebot in a circle and then stopping.
    It is open-loop and does not use feedback.

    Daniel Morris, Sep 2022
'''
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

class CircleDrive(Node):
    def __init__(self, topic_name='/cmd_vel'):
        super().__init__('circle_drive')
        self.publisher_ = self.create_publisher(Twist, topic_name, 1)
        time.sleep(2.0)  # Wait for the node to connect
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

    av = CircleDrive()  # Initialize Node
    motion = Twist( linear=Vector3(x=0.2), angular=Vector3(z=0.4))
    wait = 2*math.pi / motion.angular.z    # Time to complete a full circle: 2*pi/angular.z
    av.drive(wait = wait, motion = motion )


