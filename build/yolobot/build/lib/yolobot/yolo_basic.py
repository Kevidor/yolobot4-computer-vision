import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class YoloDetectorBasic(Node):
    def __init__(self):
        super().__init__('yolo_detector_basic')

        self.bridge = CvBridge()

        package_path = get_package_share_directory("yolobot")
        model_path = os.path.join(package_path,"models","yolov8s.pt")
        self.model = YOLO(model_path)

        self.image_sub = self.create_subscription(
            Image,
            "robot7/oakd/rgb/preview/image_raw",
            self.process_image,
            10
        )

        self.get_logger().info("YOLO basic detector started")

    def process_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(cv_image, conf=0.1, classes=[0], verbose=False)

        detections = []

        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            name = self.model.names[cls]

            if conf > 0.5:
                detections.append(f"{name} ({conf:.2f})")

        if detections:
            self.get_logger().info(
                "Currently detecting: " + ", ".join(detections)
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorBasic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()