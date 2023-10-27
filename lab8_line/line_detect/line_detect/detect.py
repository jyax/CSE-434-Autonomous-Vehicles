'''
    detect.py 

    This is an exmple ROS node for subscribing to image topics and displaying images
    Can specify the image topic name as well as whether or not it is compressed
      Usage:
    ros2 run image_fun image_display <topic_name>
    ros2 run image_fun image_display <topic_name> --compressed

    Use the optional --compressed argument if the image topic is type compressed

    Daniel Morris, Nov 2020, 2022
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
import argparse
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from scipy.special import expit
import numpy as np

class ImageDisplay(Node):
    def __init__(self):
        super().__init__('image_display')
        topic = "camera/image_raw/compressed"
        self.compressed = False
        self.publisher_ = self.create_publisher(PointStamped, 'line_point', 10)
        self.title = f'{topic}, type: compressed' if self.compressed else f'{topic}, type: raw'
        self.bridge = CvBridge()
        self.get_logger().info(f'Subscribed to: {self.title}')
        if self.compressed:
            self.subscription = self.create_subscription(CompressedImage, topic, self.image_callback, 1)
        else:
            self.subscription = self.create_subscription(Image, topic, self.image_callback, 1)
        self.subscription

    def image_callback( self, msg ):
        publish_msg = PointStamped()
        if self.compressed:
            img = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        else:
            img = self.bridge.imgmsg_to_cv2(msg,'bgr8')

        # crop last 5 rows to get the green part closest to robot
        height, width, channels = img.shape
        crop_height = 5
        cropped_img = img[height - crop_height:,:]

        cvec = np.array([[ 0.12073938, 0.0637727, -0.21671088]])
        intercept = 0.74563056

        score = (cropped_img.astype(float) * cvec).sum(axis=2) + intercept
        probt = expit(score)
        threshold = 0.5

        binary_target = (probt > threshold).astype(np.uint8)
        cc = cv.connectedComponentsWithStats(binary_target)

        inds = np.argsort(cc[2][:,4]) # sort on number of pixels in each continguous region

        centroid = np.zeros(2)
        for i in inds[::-1]:
            # If the average probability of target in region > 0.99
            if binary_target[cc[1]==i].astype(float).mean() > 0.99:
                # Then keep this region
                centroid = cc[3][i,:]
                break

        # add back offset
        if np.any(centroid):
            center = (int(centroid[0]), int(centroid[1] + height - crop_height))
            publish_msg.point.x = centroid[0]
            publish_msg.point.y = centroid[1]
            img = cv.circle(img, center, radius=10, color=(0, 0, 255), thickness=-1)

        publish_msg.header = msg.header

        self.publisher_.publish(publish_msg)

        cv.imshow(self.title, img )
        if cv.waitKey(1) & 0xFF == ord('q'):
            raise SystemExit  # If user pressed "q"

def main(args=None):
    rclpy.init(args=args)

    node = ImageDisplay()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
