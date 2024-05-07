#!/home/nvidia/env/bin/python
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from ankle_band_tracking_interfaces.msg import BoundingBoxArray


class Follower(Node):

    def __init__(self):
        super().__init__('follower')

        # Declare params
        self.declare_parameter("camera_fov", '', ParameterDescriptor(description="Realsense Field of View"))
        self.declare_parameter("img_size", 640, ParameterDescriptor(description="Image size"))
        self.declare_parameter('bboxes_sub_topic', "/yolo_detection/detector/bboxes", ParameterDescriptor(description="Bbox array subscribed"))
        self.declare_parameter('cmd_vel_pub_topic', "/follower/target_angle", ParameterDescriptor(description="Cmd Vel to publish"))
        
        # Get params
        self.camera_fov = self.get_parameter(name="camera_fov").get_parameter_value().double_value
        self.img_size = self.get_parameter(name="img_size").get_parameter_value().integer_value

        ros_params = {
            "bboxes_sub_topic": self.get_parameter(name="bboxes_sub_topic").get_parameter_value().string_value,
            "cmd_vel_pub_topic": self.get_parameter(name="cmd_vel_pub_topic").get_parameter_value().string_value
        }

        if not self.camera_fov and not ros_params["bboxes_sub_topic"]:
            raise Exception("Invalid or empty paths provided in YAML file.")

        self.args = ros_params

        # Subscribers
        self.bboxes_sub = self.create_subscription(BoundingBoxArray, ros_params["bboxes_sub_topic"], self.bboxes_callback, 10)

        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, ros_params["cmd_vel_pub_topic"], 10)

        self.get_logger().info("Follower Node has been started.")


    def bboxes_callback(self, msg):
        if msg:
            target_detected = False
            min_distance = 0.5
            min_angular_velocity = -0.5  # Minimum angular speed
            max_angular_velocity = 0.5  # Maximum angular speed
            min_linear_velocity = 0.0  # Minimum linear speed
            max_linear_velocity = 1.0  # Maximum linear speed

            for bbox in msg.bounding_boxes:
                if bbox.class_name == "target" and bbox.conf >= 0.5:
                    target_detected = True

                    if bbox.distance < min_distance:
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd)
                    else:
                        cmd = Twist()
                        # Calculate an angle between the target and the camera
                        angle_rad = self.calculate_angle_to_person_from_box_position(bbox.center_x)

                        # Consider max velocity
                        target_linear_velocity = min(bbox.distance, max_linear_velocity)
                        target_angular_velocity = min(angle_rad, max_angular_velocity)

                        # Calculate linear.x and angular.z
                        linear_velocity_x = target_linear_velocity * math.cos(angle_rad)
                        angular_velocity_z = target_angular_velocity

                        # Safety bounds
                        cmd.linear.x = max(min_linear_velocity, min(max_linear_velocity, linear_velocity_x))
                        cmd.angular.z = max(min_angular_velocity, min(max_angular_velocity, angular_velocity_z))

                        self.cmd_vel_pub.publish(cmd)

            if not target_detected:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
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
