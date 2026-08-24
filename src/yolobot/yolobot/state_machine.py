#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist
#from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class State(Enum):
    INIT = auto()
    SEARCHING = auto()
    FOLLOWING = auto()
    CLOSE = auto()
    LOST = auto()


class PersonFollower(Node):

    def __init__(self):
        super().__init__("person_follower")

        # ----------------------------
        # Parameters
        # ----------------------------
        self.follow_distance = 1.2
        self.close_distance = 1.0
        self.resume_distance = 1.4
        self.goal_update_period = 0.5
        self.target_timeout = 2.0

        # ----------------------------
        # State
        # ----------------------------
        self.state = State.INIT

        self.target_position = None
        self.last_detection_time = None

        # ----------------------------
        # Nav2
        # ----------------------------
        #self.navigator = BasicNavigator()
        self.navigator = TurtleBot4Navigator()

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        self.navigator.info("Nav2 is Active")

        # ----------------------------
        # Subscribers
        # ----------------------------
        self.target_sub = self.create_subscription(
            PointStamped,
            "/yolo/object_position",
            self.target_callback,
            10
        )

        # ----------------------------
        # Publishers
        # ----------------------------
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # ----------------------------
        # Main loop
        # ----------------------------
        self.timer = self.create_timer(
            0.1,
            self.update
        )

    # =====================================================
    # Callbacks
    # =====================================================

    def target_callback(self, msg):
        """
        Receives the tracked person's position.
        """

        self.target_position = msg
        self.last_detection_time = self.get_clock().now()

    # =====================================================
    # Main update loop
    # =====================================================

    def update(self):

        match self.state:

            case State.INIT:
                self.state_init()

            case State.SEARCHING:
                self.searching()

            case State.FOLLOWING:
                self.following()

            case State.CLOSE:
                self.close()

            case State.LOST:
                self.lost()

    # =====================================================
    # State: INIT
    # =====================================================

    def state_init(self):

        # Start on dock
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before intialising pose')
            self.navigator.dock()

        # Set initial pose
        if not self.navigator.initial_pose_received:
            initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
            self.navigator.clearAllCostmaps()
            self.navigator.setInitialPose(initial_pose)
            self.navigator.undock()

        if not self.navigator.getDockedStatus() and self.navigator.initial_pose_received:
            self.change_state(State.SEARCHING)

    # =====================================================
    # State: SEARCHING
    # =====================================================

    def searching(self):

        self.rotate()

        if self.target_visible():

            if self.distance_to_target() < self.close_distance:
                self.change_state(State.CLOSE)
            else:
                self.change_state(State.FOLLOWING)

    # =====================================================
    # State: FOLLOWING
    # =====================================================

    def following(self):

        if not self.target_visible():
            self.change_state(State.LOST)
            return

        if self.distance_to_target() < self.close_distance:
            self.navigator.cancelTask()
            self.change_state(State.CLOSE)
            return

        self.send_follow_goal()

    # =====================================================
    # State: CLOSE
    # =====================================================

    def close(self):

        self.stop_robot()

        if not self.target_visible():
            self.change_state(State.LOST)
            return

        if self.distance_to_target() > self.resume_distance:
            self.change_state(State.FOLLOWING)

    # =====================================================
    # State: LOST
    # =====================================================

    def lost(self):

        self.navigator.cancelTask()

        if self.target_visible():
            self.change_state(State.FOLLOWING)
            return

        if self.target_timed_out():
            self.change_state(State.SEARCHING)

    # =====================================================
    # Helper Functions
    # =====================================================

    def change_state(self, new_state):

        if self.state != new_state:
            self.get_logger().info(
                f"{self.state.name} -> {new_state.name}"
            )
            self.state = new_state

    def target_visible(self):

        return self.target_position is not None

    def target_timed_out(self):

        if self.last_detection_time is None:
            return True

        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        return elapsed > self.target_timeout

    def distance_to_target(self):

        if self.target_position is None:
            return float("inf")

        x = self.target_position.point.x
        y = self.target_position.point.y

        return (x**2 + y**2) ** 0.5

    def send_follow_goal(self):
        """
        Compute a goal behind the person and send it to Nav2.

        TODO:
            - Transform point into map frame
            - Offset by follow_distance
            - Send navigator.goToPose(goal)
        """
        # Set goal poses
        goal_pose = self.navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
        self.navigator.startToPose(goal_pose)

    def rotate(self, angle: float = 0.5):

        twist = Twist()
        twist.angular.z = angle

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):

        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()