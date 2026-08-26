import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image

import cv2
from cv_bridge import CvBridge
import numpy as np


class ObjectLocator(Node):

    def __init__(self):
        super().__init__("object_locator")

        self.declare_parameter("visual", False)
        self.visual = self.get_parameter("visual").value

        self.bridge = CvBridge()

        self.detections = None
        self.camera_info = None
        self.depth_image = None
        self.rgb_image = None

        self.create_subscription(
            Detection2DArray,
            "/yolo/detections",
            self.detection_cb,
            10
        )

        self.create_subscription(
            CameraInfo,
            "/robot7/oakd/rgb/preview/camera_info",
            self.camera_info_cb,
            10
        )

        self.create_subscription(
            Image,
            "/robot7/oakd/stereo/image_raw",
            self.depth_cb,
            10
        )

        self.create_subscription(
            Image,
            "/robot7/oakd/rgb/preview/image_raw",
            self.rgb_cb,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            "/yolobot/object_position",
            10
        )

        if self.visual:
            self.debug_pub = self.create_publisher(
                Image,
                "/yolobot/debug_image",
                10
            )

        self.get_logger().info("Object Locator started")

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def rgb_cb(self, msg):
        self.rgb_msg = msg
        self.rgb_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

    def detection_cb(self, msg):

        if (
            self.camera_info is None
            or self.depth_image is None
            or self.rgb_image is None
        ):
            return

        debug = self.rgb_image.copy()

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        rgb_w = self.camera_info.width
        rgb_h = self.camera_info.height

        depth_h, depth_w = self.depth_image.shape[:2]

        scale_x = depth_w / rgb_w
        scale_y = depth_h / rgb_h

        for det in msg.detections:

            bbox = det.bbox

            # -------------------------
            # YOLO center (RGB space)
            # -------------------------
            u = int(bbox.center.position.x)
            v = int(bbox.center.position.y)

            # -------------------------
            # Map to depth space
            # -------------------------
            u_depth = int(u * scale_x)
            v_depth = int(v * scale_y)

            if (
                u_depth < 0 or v_depth < 0
                or u_depth >= depth_w
                or v_depth >= depth_h
            ):
                continue

            # -------------------------
            # Depth smoothing window
            # -------------------------
            window = self.depth_image[
                max(0, v_depth - 5):min(depth_h, v_depth + 6),
                max(0, u_depth - 5):min(depth_w, u_depth + 6)
            ]

            valid = window[window > 0]

            if len(valid) == 0:
                continue

            z = float(np.median(valid))

            # mm → meters
            if z > 50:
                z /= 1000.0

            if z <= 0:
                continue

            # -------------------------
            # Projection (RGB intrinsics!)
            # -------------------------
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            # -------------------------
            # Publish 3D point
            # -------------------------
            point = PointStamped()
            point.header = self.rgb_msg.header  # IMPORTANT FIX
            point.point.x = float(x)
            point.point.y = float(y)
            point.point.z = float(z)

            self.pub.publish(point)

            # -------------------------
            # Debug drawing
            # -------------------------
            
            if not self.visual:
                continue

            box_w = int(bbox.size_x)
            box_h = int(bbox.size_y)

            x1 = int(u - box_w / 2)
            y1 = int(v - box_h / 2)
            x2 = int(u + box_w / 2)
            y2 = int(v + box_h / 2)

            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(debug, (u, v), 5, (0, 0, 255), -1)

            label = f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m"

            cv2.putText(
                debug,
                label,
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        if not self.visual:
            return
        
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = self.rgb_msg.header

        self.debug_pub.publish(debug_msg)

def main():
    rclpy.init()
    node = ObjectLocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()