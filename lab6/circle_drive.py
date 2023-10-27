'''
    CircleDrive.py
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

        self.isInit = False
        self.sPoint = Point()
        self.shouldMove = True
        self.shouldCheckStop = False
        self.sTime = time.time()
        self.hasSetTime = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.callback,
            qos_profile=qos_profile)
        self.subscription

        self.get_logger().info('subscribing to odom')

        self.publisher_ = self.create_publisher(Twist, topic_name, 1)
        self.get_logger().info('creating motion publisher')
        time.sleep(2.0)

    def callback(self, msg):
        motion = Twist(linear=Vector3(x=0.2), angular=Vector3(z=0.4))

        self.get_logger().info('getting position')
        cPoint = msg.pose.pose.position
        cTime = time.time()

        # moves the robot
        if self.shouldMove:
            if not self.isInit:
                self.isInit = True
                self.sPoint = cPoint
            self.get_logger().info('moving')
            self.publisher_.publish(motion)
            time.sleep(motion.angular.z)

        # get distance from origin
        distFromOrigin = ((self.sPoint.x - cPoint.x) ** 2 +
                          (self.sPoint.y - cPoint.y) ** 2) ** (1 / 2)
        print(distFromOrigin)

        # set start time the first time robot moves
        if distFromOrigin > 0.01 and not self.hasSetTime:
            self.sTime = time.time()
            self.hasSetTime = True

        # only start to check end condition after robot
        # crosses a certain point
        if distFromOrigin > 0.5:
            self.shouldCheckStop = True

        # check timeout
        if cTime - self.sTime > 20 and self.hasSetTime:
            print('time elapsed: ', cTime - self.sTime)
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