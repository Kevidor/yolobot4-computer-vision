import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


class PersonFollower(Node):

    def __init__(self):
        super().__init__("person_follower")

        self.navigator = BasicNavigator()

        self.current_goal = None

        self.sub = self.create_subscription(
            PoseStamped,
            "/people/follow_goal",
            self.cb,
            10
        )

        self.timer = self.create_timer(10.0, self.update_nav)

        self.get_logger().info("Nav2 follower started")

    def cb(self, msg):
        self.current_goal = msg

    def update_nav(self):

        if self.current_goal is None:
            return

        self.navigator.goToPose(self.current_goal)

        self.get_logger().info("Sending updated follow goal")

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()