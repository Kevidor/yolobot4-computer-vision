import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class TargetGenerator(Node):

    def __init__(self):
        super().__init__("target_generator")

        self.sub = self.create_subscription(
            PointStamped,
            "/yolo/object_position",
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            "/people/follow_goal",
            10
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.desired_distance = 1.2  # meters behind person

        self.get_logger().info("Target generator started")

    def cb(self, msg: PointStamped):

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        # -------------------------
        # Person in map frame
        # -------------------------
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        person_map = do_transform_pose(pose, transform)

        # -------------------------
        # Compute "follow behind" goal
        # -------------------------
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # simple: stay behind person on x-axis (improve later with heading)
        goal.pose.position.x = person_map.pose.position.x - self.desired_distance
        goal.pose.position.y = person_map.pose.position.y
        goal.pose.orientation.w = 1.0

        self.pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = TargetGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()