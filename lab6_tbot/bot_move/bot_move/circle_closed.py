'''
    circled_closed.py
'''

import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CircleDrive(Node):
    def __init__(self, topic_name='/cmd_vel'):
        super().__init__('circle_closed')

        # Flags for determining certain conditions
        self.init = False
        self.startPoint = Point()
        self.shouldMove = True
        self.shouldCheckStop = False
        self.startTime = time.time()
        self.hasSetTime = False
        
        # Fix for data transmission, reliable to best effort
        # May lead to some major variation in start time
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Subscriber for odometry to determine current position in comparison to start position
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.callback,
            qos_profile=qos_profile)
        self.subscription

        self.get_logger().info('subscribing to odom')
        # Publisher for publishing the twist movement to drive in a circle
        self.publisher_ = self.create_publisher(Twist, topic_name, 1)
        self.get_logger().info('creating motion publisher')
        time.sleep(2.0) # Time sleep to help deal with the QOS change 
    
    def callback(self, msg):
        motion = Twist( linear=Vector3(x=0.2), angular=Vector3(z=0.4)) # movement in circle

        self.get_logger().info('getting position')
        currentPoint = msg.pose.pose.position
        currentTime = time.time() # track current time to prevent continuous movement
        # if it bumps into something then it will possibly never reach the start point

        # moves the robot
        if self.shouldMove:
            # check if we have initializaed the start position and set a flag if we haven't
            if not self.init:
                self.init = True
                self.startPoint = currentPoint
            self.get_logger().info('moving')
            self.publisher_.publish(motion)
            time.sleep(motion.angular.z) # time sleep again to help with the change of QoS reliability

        # get distance from origin
        distFromOrigin = ((self.startPoint.x - currentPoint.x)**2 +
            (self.startPoint.y - currentPoint.y)**2)**(1/2)
        print(distFromOrigin)

        # set start time the first time robot moves
        if distFromOrigin > 0.01 and not self.hasSetTime:
            self.startTime = time.time()
            self.hasSetTime = True

        # only start to check end condition after robot
        # crosses a certain point
        if distFromOrigin > 0.5:
            self.shouldCheckStop = True

        # check timeout
        if currentTime - self.startTime > 20 and self.hasSetTime:
            print('time elapsed: ', currentTime - self.startTime)
            self.shouldMove = False;

        # check for end condition
        if self.shouldCheckStop and distFromOrigin < 0.05:
            self.get_logger().info('stop')
            self.shouldMove = False;
            self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    av = CircleDrive()
    rclpy.spin(av)
