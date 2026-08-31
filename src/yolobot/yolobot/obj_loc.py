import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image

import cv2
from cv_bridge import CvBridge
import numpy as np

# Definiere ein extrem schnelles, lag-freies Profil
fast_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, # package are allowed to be discarded
    history=HistoryPolicy.KEEP_LAST,
    depth=1                                    # Keep newest image
)

class ObjectLocator(Node):

    def __init__(self):
        super().__init__("object_locator")

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter("enable_viz", False)
        self.enable_viz = self.get_parameter("enable_viz").value

        # =====================================================
        # Utilities
        # =====================================================

        self.bridge = CvBridge()

        # =====================================================
        # Latest data
        # =====================================================

        self.detections = None
        self.camera_info = None
        self.depth_image = None

        if self.enable_viz:
            self.rgb_image = None
            self.rgb_msg = None

        # =====================================================
        # Subscribers
        # =====================================================

        self.create_subscription(
            Detection2DArray,
            "/yolobot/detections",
            self.detection_cb,
            fast_qos
        )

        self.create_subscription(
            CameraInfo,
            "/robot7/oakd/rgb/preview/camera_info",
            self.camera_info_cb,
            1
        )

        self.create_subscription(
            Image,
            "/robot7/oakd/stereo/image_raw",
            self.depth_cb,
            fast_qos
        )

        # RGB is only needed for visualization
        if self.enable_viz:
            self.create_subscription(
                Image,
                "/robot7/oakd/rgb/preview/image_raw",
                self.rgb_cb,
                fast_qos
            )

        # =====================================================
        # Publishers
        # =====================================================

        self.pub = self.create_publisher(
            PointStamped,
            "/yolobot/obj_loc_position",
            1
        )

        if self.enable_viz:
            self.viz_pub = self.create_publisher(
                Image,
                "/yolobot/debug_obj_loc_vizualization",
                1
            )

        # =====================================================
        # Startup message
        # =====================================================

        self.get_logger().info("Object Locator started")
        self.get_logger().info(
            f"Visualization: "
            f"{'enabled' if self.enable_viz else 'disabled'}"
        )

    # =========================================================
    # Camera Info
    # =========================================================

    def camera_info_cb(self, msg):
        # Get camera intrinsics once
        self.camera_info.fx = msg.k[0]
        self.camera_info.fy = msg.k[4]
        self.camera_info.cx = msg.k[2]
        self.camera_info.cy = msg.k[5]
        
        self.camera_info.rgb_w = msg.width
        self.camera_info.rgb_h = msg.height

        self.get_logger().info("CameraInfo was retrieved successfully. Destroying subscriber...")

        # Then destroy subsciption
        self.destroy_subscription(self.info_sub)

    # =========================================================
    # Depth Image
    # =========================================================

    def depth_cb(self, msg):

        # Only keep the newest depth image.
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

    # =========================================================
    # RGB Image
    # =========================================================

    def rgb_cb(self, msg):

        self.rgb_msg = msg

        self.rgb_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

    # =========================================================
    # Detection
    # =========================================================

    def detection_cb(self, msg):

        # Wait until camer_info was retrieved once
        if self.camera_info is None or self.depth_image is None:
            return

        # Visualization additionally requires an RGB image.
        if self.enable_viz and self.rgb_image is None:
            return

        # -----------------------------------------------------
        # Prepare debug image
        # -----------------------------------------------------

        if self.enable_viz:
            debug = self.rgb_image.copy()

        # -----------------------------------------------------
        # Camera intrinsics
        # -----------------------------------------------------

        depth_h, depth_w = self.depth_image.shape[:2]

        # RGB → depth coordinate scaling
        scale_x = depth_w / self.camera_info.rgb_w
        scale_y = depth_h / self.camera_info.rgb_h

        # -----------------------------------------------------
        # Process detections
        # -----------------------------------------------------

        for det in msg.detections:

            bbox = det.bbox

            # =================================================
            # YOLO center in RGB image
            # =================================================

            u = int(bbox.center.position.x)
            v = int(bbox.center.position.y)

            # =================================================
            # Map RGB coordinate to depth image
            # =================================================

            u_depth = int(u * scale_x)
            v_depth = int(v * scale_y)

            if (
                u_depth < 0
                or v_depth < 0
                or u_depth >= depth_w
                or v_depth >= depth_h
            ):
                continue

            # =================================================
            # Depth smoothing window
            # =================================================

            window = self.depth_image[
                max(0, v_depth - 5):min(depth_h, v_depth + 6),
                max(0, u_depth - 5):min(depth_w, u_depth + 6)
            ]

            valid = window[window > 0]

            if len(valid) == 0:
                continue

            # Median is more robust against outliers than
            # simply using the center pixel.
            z = float(np.median(valid))

            # =================================================
            # Convert depth to meters
            # =================================================

            if z > 50:
                z /= 1000.0

            if z <= 0:
                continue

            # =================================================
            # Project pixel into 3D
            # =================================================

            x = (u - self.camera_info.cx) * z / self.camera_info.fx
            y = (v - self.camera_info.cy) * z / self.camera_info.fy

            # =================================================
            # Publish 3D point
            # =================================================

            point = PointStamped()

            if self.enable_viz:
                point.header = self.rgb_msg.header
            else:
                point.header = msg.header

            point.point.x = float(x)
            point.point.y = float(y)
            point.point.z = float(z)

            self.pub.publish(point)

            # =================================================
            # Debug visualization
            # =================================================

            if not self.enable_viz:
                continue

            box_w = int(bbox.size_x)
            box_h = int(bbox.size_y)

            x1 = int(u - box_w / 2)
            y1 = int(v - box_h / 2)

            x2 = int(u + box_w / 2)
            y2 = int(v + box_h / 2)

            cv2.rectangle(
                debug,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2
            )

            cv2.circle(
                debug,
                (u, v),
                5,
                (0, 0, 255),
                -1
            )

            # =================================================
            # Coordinates
            # =================================================

            text_x = f"X: {x:.2f} m"
            text_y = f"Y: {y:.2f} m"
            text_z = f"Z: {z:.2f} m"

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 1

            # OpenCV uses BGR
            text_color = (255, 0, 0)

            text_x_pos = x1
            text_y_pos = max(15, y1 - 30)

            cv2.putText(
                debug,
                text_x,
                (text_x_pos, text_y_pos),
                font,
                font_scale,
                text_color,
                thickness
            )

            cv2.putText(
                debug,
                text_y,
                (text_x_pos, text_y_pos + 15),
                font,
                font_scale,
                text_color,
                thickness
            )

            cv2.putText(
                debug,
                text_z,
                (text_x_pos, text_y_pos + 30),
                font,
                font_scale,
                text_color,
                thickness
            )

        # =====================================================
        # Publish debug image
        # =====================================================

        if not self.enable_viz:
            return

        debug_msg = self.bridge.cv2_to_imgmsg(
            debug,
            encoding="bgr8"
        )

        debug_msg.header = self.rgb_msg.header

        self.viz_pub.publish(debug_msg)

def main():
    rclpy.init()
    node = ObjectLocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()