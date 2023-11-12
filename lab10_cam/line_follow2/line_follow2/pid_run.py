import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import argparse
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from .pid import pid_controller
import math
from datetime import datetime


class PurePursuit(Node):
    def __init__(self, speed):
        super().__init__('pure_pursuit')
        self.speed = float(speed)
        print('Set speed: ', speed)

        self.kp, self.ki, self.kd = 1.0, 8.0, 1.0
        self.init_time = None
        self.target = 0
        self.times = []
        self.angles = []

        self.pid = pid_controller(self.kd, self.ki, self.kd)

        self.subscription = self.create_subscription(PointStamped, 'ground_point', self.callback, 1)
        self.subscription

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)

    def callback(self, msg):
        current_time = self.get_clock().now().nanoseconds/1e9
        x_t, y_t = msg.point.x, msg.point.y
        if self.init_time is None:
            self.init_time = current_time

        if 0 < x_t:
            current_angle = math.atan2(y_t,x_t)
            self.times.append(current_time)
            self.angles.append(current_angle)

            p,q,r,s = self.pid.update_control(self.target, current_angle, self.times[-1])
            k = (2 * y_t) / ((x_t)**2 + (y_t)**2)
            w = p * k * self.speed 

            motion = Twist()
            motion.linear.x, motion.angular.z = self.speed, w
            self.publisher_.publish( motion )
            print('Published Twist: ', motion)
        else:
            motion = Twist()
            motion.linear.x, motion.angular.z = 0.0, -0.3
            self.publisher_.publish( motion )
            print('Rotating to find line: ', motion)

def main(args = None):
    rclpy.init(args = args)

    parser = argparse.ArgumentParser(description = 'Optional Speed')
    parser.add_argument('--speed', default=0.5, type=float, help="Optional linear speed")
    
    args, unknown = parser.parse_known_args()
    if unknown: print('Unknown args:', unknown)

    node = PurePursuit(args.speed)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
