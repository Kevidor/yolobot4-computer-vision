#!/home/ubuntu/img_detect_ws/.venv/bin/python3
#from irobot_create_msgs.msg import InterfaceButtons, LightringLeds

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import os
import cv2
from datetime import datetime
import numpy as np
from ultralytics import YOLO

class ImageDetectionNode(Node):
    def __init__(self):
        super().__init__('img_detection_node')

        # Declare parameters
        self.declare_parameter("image_topic", "robot7/oakd/rgb/preview/image_raw") #'oakd/rgb/preview/image_raw'
        self.declare_parameter("tracked_image_topic", "/image_tracking")
        self.declare_parameter("model_name", "yolov8n.pt")

        # Get parameters
        image_topic = self.get_parameter("image_topic").value
        tracked_image_topic = self.get_parameter("tracked_image_topic").value
        model_name = self.get_parameter("model_name").value

        self.bridge = CvBridge()

        # Load YOLO model from package share directory
        package_share_dir = get_package_share_directory("image_detection")
        full_model_path = os.path.join(package_share_dir, "models", model_name)

        self.get_logger().info(f"Loading YOLO model from: {full_model_path}")
        self.model = YOLO(full_model_path)
        self.get_logger().info("YOLO model loaded successfully")

        # Subscribe to the oakd camera topic
        self.oakd_image_subscriber = self.create_subscription(
            Image,
            image_topic,
            self.oakd_image_callback,
            10)

        self.image_detect_publisher = self.create_publisher(
            Image,
            tracked_image_topic,
            10
        )

    # Image subscription callback
    def oakd_image_callback(self, msg):
        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.model.track(frame, conf=0.1, classes=[0], persist=True, verbose=False)
        frame = results[0].plot()

        # Convert back OpenCV → ROS
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_detect_publisher.publish(out_msg)

def main(args=None):
    print('[INFO] Image Detect Node Startup')
    rclpy.init(args=args)
    node = ImageDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()