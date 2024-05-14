#!/home/nvidia/env/bin/python
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from ankle_band_tracking_interfaces.msg import BoundingBoxArray


class Follower(Node):

    def __init__(self):
        super().__init__('follower')

        # Declare params
        self.declare_parameter("camera_fov", '', ParameterDescriptor(description="Realsense Field of View"))
        self.declare_parameter("img_size", 640, ParameterDescriptor(description="Image size"))
        self.declare_parameter('bboxes_sub_topic', "/yolo_detection/detector/bboxes", ParameterDescriptor(description="Bbox array subscribed"))
        self.declare_parameter('cmd_vel_pub_topic', "/follower/cmd_vel", ParameterDescriptor(description="Cmd Vel to publish"))
        
        # Get params
        self.camera_fov = self.get_parameter(name="camera_fov").get_parameter_value().double_value
        self.img_size = self.get_parameter(name="img_size").get_parameter_value().integer_value

        ros_params = {
            "bboxes_sub_topic": self.get_parameter(name="bboxes_sub_topic").get_parameter_value().string_value,
            "cmd_vel_pub_topic": self.get_parameter(name="cmd_vel_pub_topic").get_parameter_value().string_value,
        }

        if not self.camera_fov and not ros_params["bboxes_sub_topic"]:
            raise Exception("Invalid or empty paths provided in YAML file.")

        self.args = ros_params

        # Subscribers
        self.bboxes_sub = self.create_subscription(BoundingBoxArray, ros_params["bboxes_sub_topic"], self.bboxes_callback, 10)

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, ros_params["cmd_vel_pub_topic"], 10)
        self.vcmd = 0.0
        self.wcmd = 0.0
        self.count = 0
        self.get_logger().info("Follower Node has been started.")


    def bboxes_callback(self, msg):
        if msg:
            target_detected = False
            min_distance = 0.3

            for bbox in msg.bounding_boxes:
                if bbox.class_name == "target" and bbox.conf >= 0.5:
                    # Calculate an angle between the target and the camera
                    angle = self.calculate_angle_to_person_from_box_position(bbox.center_x)
                    target_detected = True

                    cmd = Twist()
                    L_d = 1.0
                    self.wcmd = math.atan(2*math.sin(angle)/(L_d))
                    cmd.angular.z = self.wcmd


                    if bbox.distance < min_distance:                   
                        self.vcmd = 0.0                       
                        cmd.linear.x = self.vcmd                        
                        self.cmd_vel_pub.publish(cmd)
                    else:
                        if bbox.distance > 0.5:
                            self.vcmd = 0.52
                        else:
                            self.vcmd = bbox.distance

                        cmd.linear.x = self.vcmd
                        self.cmd_vel_pub.publish(cmd)

            if not target_detected:
                cmd = Twist()
                self.count += 1
                if self.count >= 20 :
                    self.vcmd = 0.0
                    self.wcmd = 0.0
                    cmd.linear.x = self.vcmd
                    cmd.angular.z = self.wcmd
                    self.cmd_vel_pub.publish(cmd)

     
    def calculate_angle_to_person_from_box_position(self, center_x):
        pixel_per_angle = self.img_size / self.camera_fov
        # Find the offset from the center
        offset_x = center_x - self.img_size / 2
        # Calculate angle
        angle = offset_x / pixel_per_angle
        # Convert to radian
        angle_rad = math.radians(angle)
        return angle_rad


def main(args=None):
    rclpy.init(args=args)
    following_node = Follower()
    try:
        rclpy.spin(following_node)
    except KeyboardInterrupt:
        pass
    following_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
