import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import argparse
import numpy as np
from geometry_msgs.msg import Twist, Vector3


class PurePursuit(Node)
    def __init__(self, speed):
        super().__init__('pure_pursuit')
        self.speed = speed
        print('Set speed: ', speed)

        self.subscription = self.create_subscription(PointStamped, 'ground_point', self.callback, 1)
        self.subscription

        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',1)

    def callback(self, msg):
        x_t, y_t = msg.point.x, msg.point.y
        if 0 < x_t:
            k = (2 * y_t) / ((x_t)**2 + (y_t)**2)
            w = k * self.speed
            motion = Twist( linear=Vector3(x=self.speed), Angular=Vector3(z=w)
            self.publisher_.publish( motion )
            print('Published Twist: ', motion)
        else:
            motion = Twist( linear=Vector3( x=0 ), Angular=Vector3( z=0.3 ) )
            self.publisher_.publish( motion )

def main(args = None):
    rclpy.init(args = args)

    parser = argparse.ArgumentParser(description = 'Optional Speed')
    parser.add_argument('--speed', default=1, type=float, help="Optional linear speed")
    
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
