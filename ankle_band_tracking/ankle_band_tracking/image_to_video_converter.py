#!/home/nvidia/env/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageToVideoConverter(Node):
    def __init__(self):
        super().__init__('image_to_video_converter')
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.video_writer = None

    def image_callback(self, msg):
        try:
            rgb_image = self.cv_bridge.imgmsg_to_cv2(msg)
            rgb_image = cv2.cvtColor(rgb_image,cv2.COLOR_BGR2RGB)
            if self.video_writer is None:
                self.init_video_writer(rgb_image)
            self.video_writer.write(rgb_image)
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_format = 'mp4'  # or any other video format supported by OpenCV
            video_filename = 'output_video.' + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 30  # Frames per second
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error('Error initializing video writer: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_to_video_converter = ImageToVideoConverter()
    rclpy.spin(image_to_video_converter)
    image_to_video_converter.destroy_node()