import rclpy
import math
from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import DriveArc
from geometry_msgs.msg import PoseStamped

class DriveArcActionClient(Node):
    def __init__(self):
        super().__init__('drivearc_action_client')
        self._action_client = ActionClient(self, DriveArc, '/drive_arc')

    def send_goal(self, trans_dir, angle, radius):
        goal_msg = DriveArc.Goal()
        goal_msg.translate_direction = trans_dir
        goal_msg.angle = angle
        goal_msg.radius = radius

        self.get_logger().info('waiting for server')
        self._action_client.wait_for_server()

        self.get_logger().info('sending goal')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.pose))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.remaining_angle_travel))

def main(args=None):
    rclpy.init(args=args)

    action_client = DriveArcActionClient()
    
    trans_dir = 1
    angle = 2 * math.pi
    radius = 0.6

    action_client.send_goal(trans_dir, angle, radius)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
