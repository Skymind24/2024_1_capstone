#!/home/nvidia/env/bin/python
import rclpy
from rclpy.node import Node
import time
import math

from tf2_ros import TransformBroadcaster

# import tf
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float32

# import sys, rospkg
# rospack = rospkg.RosPack()

from .control_module.serial_simple_ctrl import UGVControl

# import serial  
# import json  

# Open the sereial port   
# ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  

class Controller(Node):

    def __init__(self):
        super().__init__('ugv_controller')

        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Initialize
        self.motor = UGVControl()
        self.motor.connect('/dev/ttyUSB0')

        self.input = ""

        # self.odom_broadcaster = TransformBroadcaster()

        # self.process_call = time.time()

        # self.complete_set_vel = True

        # self.motor.meal_init()

        


    def cmd_vel_callback(self, msg):
        if msg:
            print("msg.linear.x:  ", msg.linear.x)  
            print("msg.angular.z:  ", msg.angular.z)

            linear_x = msg.linear.x
            angular_z = msg.angular.z

            self.input = f'{{"T": 13, "X": {linear_x}, "Z": {angular_z}}}'
            self.motor.set_input(self.input)

            print("cmd_input: ", self.input)
        else:
            self.get_logger().info("Don't subscribe /cmd_vel topic.")


def main():
    rclpy.init()
    controller_node = Controller()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
