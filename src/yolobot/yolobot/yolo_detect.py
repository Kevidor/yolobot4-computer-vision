import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose
)

from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.declare_parameter('enable_viz', False)
        self.declare_parameter('enable_basic', False)

        self.enable_viz = self.get_parameter(
            'enable_viz'
        ).get_parameter_value().bool_value

        self.enable_basic = self.get_parameter(
            'enable_basic'
        ).get_parameter_value().bool_value

        # --------------------------------------------------
        # YOLO model
        # --------------------------------------------------

        self.bridge = CvBridge()

        package_path = get_package_share_directory("yolobot")
        model_path = os.path.join(
            package_path,
            "models",
            "yolov8s.pt"
        )

        self.model = YOLO(model_path)

        # --------------------------------------------------
        # Camera subscription
        # --------------------------------------------------

        self.image_sub = self.create_subscription(
            Image,
            "oakd/rgb/preview/image_raw",
            self.process_image,
            10
        )

        # --------------------------------------------------
        # Detection publisher
        # --------------------------------------------------

        self.detect_pub = self.create_publisher(
            Detection2DArray,
            "/yolobot/detections",
            10
        )

        # --------------------------------------------------
        # Visualization publisher
        # --------------------------------------------------

        self.viz_pub = None

        if self.enable_viz:
            self.viz_pub = self.create_publisher(
                Image,
                "/yolobot/debug_yolo_vizualization",
                10
            )

        # --------------------------------------------------
        # Startup information
        # --------------------------------------------------

        self.get_logger().info("YOLO Detection node started.")

        self.get_logger().info(
            f"Visualization: {'enabled' if self.enable_viz else 'disabled'}"
        )

        self.get_logger().info(
            f"Text output: {'enabled' if self.enable_basic else 'disabled'}"
        )

    def process_image(self, msg):

        # --------------------------------------------------
        # Convert ROS image to OpenCV image
        # --------------------------------------------------

        cv_image = self.bridge.imgmsg_to_cv2(
            msg,
            'bgr8'
        )

        # --------------------------------------------------
        # Run YOLO inference ONCE
        # --------------------------------------------------

        results = self.model(
            cv_image,
            conf=0.8,
            classes=[0], # class 0 is 'person' in COCO dataset
            verbose=False
        )

        result = results[0]

        # --------------------------------------------------
        # Create Detection2DArray
        # --------------------------------------------------

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Used for basic text output
        basic_detections = []

        # --------------------------------------------------
        # Process detections
        # --------------------------------------------------

        for box in result.boxes:

            # Bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # --------------------------------------------------
            # ROS Detection2D
            # --------------------------------------------------

            detection = Detection2D()

            bbox = BoundingBox2D()

            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0

            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1

            detection.bbox = bbox

            # --------------------------------------------------
            # Detection hypothesis
            # --------------------------------------------------

            cls = int(box.cls[0])
            conf = float(box.conf[0])

            hypothesis = ObjectHypothesisWithPose()

            hypothesis.hypothesis.class_id = str(cls)
            hypothesis.hypothesis.score = conf

            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

            # --------------------------------------------------
            # Basic text output
            # --------------------------------------------------

            if self.enable_basic:

                name = self.model.names[cls]

                basic_detections.append(
                    f"{name} ({conf:.2f})"
                )

        # --------------------------------------------------
        # Publish detections
        # --------------------------------------------------

        self.detect_pub.publish(detection_array)

        # --------------------------------------------------
        # Basic text output
        # --------------------------------------------------

        if self.enable_basic and basic_detections:

            self.get_logger().info(
                "Currently detecting: "
                + ", ".join(basic_detections)
            )

        # --------------------------------------------------
        # Visualization
        # --------------------------------------------------

        if self.enable_viz:

            annotated = result.plot()

            msg_out = self.bridge.cv2_to_imgmsg(
                annotated,
                'bgr8'
            )

            msg_out.header = msg.header

            self.viz_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()