#!/home/nvidia/env/bin/python
import cv2
import torch
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from ankle_band_tracking_interfaces.msg import BoundingBox, BoundingBoxArray

from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized, TracedModel

import pyrealsense2 as rs

class Detector(Node):

    def __init__(self):
        super().__init__('detector')

        # Declare params
        # Yolo params
        self.declare_parameter("yolo_model.weight_file", '', ParameterDescriptor(description="Weights file"))
        self.declare_parameter("yolo_model.conf_thresh", 0.5, ParameterDescriptor(description="Confidence threshold"))
        self.declare_parameter("yolo_model.iou_thresh", 0.45, ParameterDescriptor(description="IOU threshold for NMS"))
        self.declare_parameter("yolo_model.device", '', ParameterDescriptor(description="Name of the device"))
        self.declare_parameter("yolo_model.img_size", 640, ParameterDescriptor(description="Image size"))
        self.declare_parameter("yolo_model.trace", False, ParameterDescriptor(description="Is traced model"))

        # Ros params
        self.declare_parameter('rgb_img_sub_topic', "/color/image_raw", ParameterDescriptor(description="Subscribed RGB image topic"))
        self.declare_parameter('depth_img_sub_topic', "/camera/camera/depth/image_rect_raw", ParameterDescriptor(description="Subscribed Depth image topic"))
        self.declare_parameter('img_pub_topic', "/yolo_detection/detector/image", ParameterDescriptor(description="Image topic to publish"))
        self.declare_parameter('bboxes_pub_topic', "/yolo_detection/detector/bboxes", ParameterDescriptor(description="Bbox Array to publish"))

        # Get params
        self.weights = self.get_parameter(name="yolo_model.weight_file").get_parameter_value().string_value
        self.conf_thresh = self.get_parameter(name="yolo_model.conf_thresh").get_parameter_value().double_value
        self.iou_thresh = self.get_parameter(name="yolo_model.iou_thresh").get_parameter_value().double_value
        self.device = self.get_parameter(name="yolo_model.device").get_parameter_value().string_value
        self.img_size = self.get_parameter(name="yolo_model.img_size").get_parameter_value().integer_value
        self.trace = self.get_parameter(name="yolo_model.trace").get_parameter_value().bool_value

        ros_params = {
            "rgb_img_sub_topic": self.get_parameter(name="rgb_img_sub_topic").get_parameter_value().string_value,
            "depth_img_sub_topic": self.get_parameter(name="depth_img_sub_topic").get_parameter_value().string_value,
            "img_pub_topic": self.get_parameter(name="img_pub_topic").get_parameter_value().string_value,
            "bboxes_pub_topic": self.get_parameter(name="bboxes_pub_topic").get_parameter_value().string_value,
        }

        if not self.weights and not ros_params["rgb_img_sub_topic"]:
            raise Exception("Invalid or empty paths provided in YAML file.")

        self.args = ros_params

        # Subscribers
        self.rgb_img_sub = self.create_subscription(Image, ros_params["rgb_img_sub_topic"], self.rgb_img_callback, 10)
        self.depth_img_sub = self.create_subscription(Image, ros_params["depth_img_sub_topic"], self.depth_img_callback, 10)
        self.intr_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.intr_callback, 10)

        # Initialize publishers
        self.img_pub = self.create_publisher(Image, ros_params["img_pub_topic"], 10)
        self.bboxes_pub = self.create_publisher(BoundingBoxArray, ros_params["bboxes_pub_topic"], 10)

        # Camera info
        self.intr = None
        
        # Frames
        self.depth = None
        self.rgb_image = None

        # Flags
        self.camera_RGB = False
        self.camera_depth = False

        # Timer callback
        self.frequency = 20  # Hz
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
    
        # Initialize yolov7
        set_logging()
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device) # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.img_size, s=stride)  # check img_size
        if self.trace:
            self.model = TracedModel(self.model, self.device, self.img_size)
        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))
        self.old_img_w = self.old_img_h = imgsz
        self.old_img_b = 1

        # ROS attributes
        self.cv_bridge = CvBridge()

        self.get_logger().info("Detector Node has been started.")

    
    def rgb_img_callback(self, msg):
        if msg:
            self.rgb_image = self.cv_bridge.imgmsg_to_cv2(msg) # (480, 640, 3)
            self.camera_RGB = True

    def depth_img_callback(self, msg):
        """Subscription to the depth camera topic."""
        if msg:
            self.depth  = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") # (480, 848)
            self.camera_depth = True
        else:
            self.get_logger().info("Don't subscribe the depth img.")

    def intr_callback(self, cameraInfo):
        """Camera information obtained from the aligned_depth_to_color/camera_info topic"""
        if self.intr:
            return
        self.intr = rs.intrinsics()
        self.intr.width = cameraInfo.width
        self.intr.height = cameraInfo.height
        self.intr.ppx = cameraInfo.k[2]
        self.intr.ppy = cameraInfo.k[5]
        self.intr.fx = cameraInfo.k[0]
        self.intr.fy = cameraInfo.k[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            self.intr.model = rs.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intr.model = rs.distortion.kannala_brandt4
        self.intr.coeffs = [i for i in cameraInfo.d]

    def timer_callback(self):
        if self.camera_RGB == True:
            self.yolov7_detection()


    def yolov7_detection(self):
        """ Perform object detection with custom yolov7-tiny"""
        img = self.rgb_image

        im0 = img.copy()
        img = img[np.newaxis, :, :, :]
        img = np.stack(img, 0)
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if self.device.type != 'cpu' and (self.old_img_b != img.shape[0] or self.old_img_h != img.shape[2] or self.old_img_w != img.shape[3]):
            self.old_img_b = img.shape[0]
            self.old_img_h = img.shape[2]
            self.old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thresh, self.iou_thresh)
        t3 = time_synchronized()

        # Process detections
        det = pred[0].cpu().numpy()

        bboxes_array = BoundingBoxArray() 
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                bbox = BoundingBox()
                c = int(cls)
                # Fill in bounding box message
                bbox.class_idx = c
                bbox.class_name = self.names[c]
                bbox.conf = conf.item() 
                bbox.xmin = int(xyxy[0])
                bbox.ymin = int(xyxy[1])
                bbox.xmax = int(xyxy[2])
                bbox.ymax = int(xyxy[3])
                
                # Calculate center coordinates
                bbox.center_x = (bbox.xmin + bbox.xmax) // 2
                bbox.center_y = (bbox.ymin + bbox.ymax) // 2

                # Calculate distance
                if bbox.center_x < im0.shape[1] and bbox.center_y < im0.shape[0]:
                    real_coords = rs.rs2_deproject_pixel_to_point(self.intr, [bbox.center_y, bbox.center_x], self.depth[bbox.center_y][bbox.center_x]) # yxyx right
                    distance = real_coords[2] * 0.001 # mm to m
                    bbox.distance = distance
                else:
                    bbox.distance = -1.0

                bboxes_array.bounding_boxes.append(bbox)

                # draw bbox and center
                label = f'{self.names[c]} {conf:.2f}'
                plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=2)
            
        # Publish
        self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(im0, "bgr8"))
        self.bboxes_pub.publish(bboxes_array)     

        im0_cv2 = cv2.cvtColor(im0,cv2.COLOR_BGR2RGB)
        cv2.imshow("YOLOv7-tiny-detection", im0_cv2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            raise StopIteration


def main(args=None):
    rclpy.init(args=args)
    with torch.no_grad():
        detection_node = Detector()
        try:
            rclpy.spin(detection_node)
        except KeyboardInterrupt:
            pass
        detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()