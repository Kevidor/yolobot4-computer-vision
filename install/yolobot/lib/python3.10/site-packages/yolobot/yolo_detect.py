import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class YoloDetector(Node):
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

        self.detect_pub = self.create_publisher(Detection2DArray, "/yolo/detections", 10)

        self.get_logger().info("YOLO Detection node started.")
    
    def process_image(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(cv_image, conf=0.1, classes=[0], verbose=False)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for result in results:
            for box in result.boxes:

                detection = Detection2D()

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                bbox = BoundingBox2D()
                bbox.center.position.x = (x1 + x2) / 2.0
                bbox.center.position.y = (y1 + y2) / 2.0
                bbox.size_x = x2 - x1
                bbox.size_y = y2 - y1

                detection.bbox = bbox

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                hypothesis.hypothesis.score = float(box.conf[0])

                detection.results.append(hypothesis)

                detection_array.detections.append(detection)

        self.detect_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()