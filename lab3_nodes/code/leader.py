#!/usr/bin/env python
''' leader.py

    This is a ROS node that defines room-A and robot-A. Robot-A is controlled 
    by the keyboard. Additionally the motion commands are published so other 
    nodes can follow.

    Your task is to complete the missing portions that are marked with
    # <...> 
'''
from std_msgs.msg import Int32
from .robot_room import RobotRoom
import rclpy
from rclpy.node import Node

class Leader(Node):
    def __init__(self, topic_name='robot_move'):
        ''' Initialize Node -- we only want one leader to run 
            Define a publisher for a message of type Int32
            Initialize a RobotRoom
            Call listKeys() to tell user what to press
       '''
        super().__init__('lead')
        # <...>  Create your publisher
        self.get_logger().info('Publishing: ' + topic_name)
        # <...> Initialize your RobotRoom for the leader
        # <...> list the keys the user can press
        
    def run(self):
        ''' Create a loop that:
            Draws the robot and returns a key
            Based on the key pressed, moves it accordingly 
            Publishes the key to the topic 
            Quits when the user presses a 'q'
        '''
        key = 0        
        while key != ord('q'):  # While q has not been pressed:
            # <...> Draw the room
            # <...> Move according to the key
            # <...> Publish the key as a Int32()
            pass
        
def main(args=None):
    rclpy.init(args=args)
    # <...> Initialize the Leader class
    # <...> run the leader class

if __name__ == '__main__':
    main()
