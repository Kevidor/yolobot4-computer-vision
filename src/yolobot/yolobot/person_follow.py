#!/usr/bin/env python3

from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
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
        self.last_detection_timeout = 1.0
        self.lost_timeout = 5.0

        # ----------------------------
        # State
        # ----------------------------
        self.state = State.INIT
        self.init_dock_started = False
        self.init_pose_set = False
        self.init_undock_started = False

        # ----------------------------
        # Nav2
        # ----------------------------
        self.navigator = TurtleBot4Navigator()
        self.has_goal = False
        # targets
        self.target_point = None
        self.last_detection_time = None
        self.lost_start_time = None

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        self.navigator.info("Nav2 is Active")

        # ----------------------------
        # Subscribers
        # ----------------------------
        self.target_sub = self.create_subscription(
            PointStamped,
            "/yolobot/obj_loc_position",
            self.target_callback,
            10
        )

        # ----------------------------
        # Publishers
        # ----------------------------
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )

        # ----------------------------
        # Main loop
        # ----------------------------
        self.timer = self.create_timer(
            0.1,
            self.update
        )

        # --------------------------------------------------
        # Startup information
        # --------------------------------------------------
        self.get_logger().info("YOLO Person Follower node started.")

    # =====================================================
    # Callbacks
    # =====================================================

    def target_callback(self, msg):
        """
        Receives the tracked person's position.
        """

        self.stop_robot()
        self.target_point = msg
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

        # -----------------------------------------
        # Enter searching
        # -----------------------------------------
        if not self.navigator.getDockedStatus() \
        and self.init_dock_started \
        and self.init_pose_set \
        and self.init_undock_started: # check if everything is done else jump over it and let it init

            self.navigator.info("Initialization complete")
            self.change_state(State.SEARCHING)
            return

        # -----------------------------------------
        # Step 1: Make sure robot is docked
        # -----------------------------------------
        if not self.navigator.getDockedStatus(): # if not docked, then dock first
            self.navigator.info("Docking before initialization")

            if not self.init_dock_started:
                self.navigator.dock()
                self.init_pose_set = False
                self.init_dock_started = True
            return
        else:                                    # if docked continue
            if not self.init_dock_started:
                self.navigator.info("Already docked for initialization")
                self.init_pose_set = False
                self.init_dock_started = True

        # -----------------------------------------
        # Step 2: Set initial pose
        # -----------------------------------------
        if not self.navigator.initial_pose_received: # if no inital pose, then set pose
            self.navigator.info("Setting initial pose")

            if not self.init_pose_set:
                initial_pose = self.navigator.getPoseStamped(
                    [0.0, 0.0],
                    TurtleBot4Directions.NORTH
                )

                self.navigator.clearAllCostmaps()
                self.navigator.setInitialPose(initial_pose)

                self.init_pose_set = True

            return
        else:                                        # if inital pose set continue
            if not self.init_pose_set:
                self.navigator.info("Initial pose already set")
                self.init_pose_set = True

        # -----------------------------------------
        # Step 3: Undock
        # -----------------------------------------
        if self.navigator.getDockedStatus() and self.init_dock_started and self.init_pose_set: # if docked, then undock
            #self.navigator.info("Undocking")

            if not self.init_undock_started:
                self.navigator.undock()
                self.init_undock_started = True

            return


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
            self.lost_start_time = self.get_clock().now()
            self.change_state(State.LOST)
            return

        if self.distance_to_target() < self.close_distance:
            self.navigator.cancelTask()
            self.has_goal = False
            self.change_state(State.CLOSE)
            return

        if not self.has_goal:
            self.send_follow_goal()
#        elif self.target_moved_significantly():
#            self.navigator.cancelTask()
#            self.send_follow_goal()

    # =====================================================
    # State: CLOSE
    # =====================================================

    def close(self):

        self.stop_robot()

        if not self.target_visible():
            self.lost_start_time = self.get_clock().now()
            self.change_state(State.LOST)
            return

        if self.distance_to_target() > self.resume_distance:
            self.change_state(State.FOLLOWING)

    # =====================================================
    # State: LOST
    # =====================================================

    def lost(self):

        if self.target_visible():
            self.change_state(State.FOLLOWING)
            return

        if self.target_timed_out():
            self.navigator.cancelTask()
            self.has_goal = False
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

        if self.last_detection_time is None:
            return False

        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        return elapsed <= self.last_detection_timeout

    def target_timed_out(self):

        if self.lost_start_time is None:
            return True

        elapsed = (
            self.get_clock().now() - self.lost_start_time
        ).nanoseconds / 1e9

        return elapsed > self.lost_timeout

    def distance_to_target(self):

        if self.target_point is None:
            return float("inf")

        x = self.target_point.point.x
        y = self.target_point.point.y

        return (x**2 + y**2) ** 0.5

    def send_follow_goal(self):

        #if not self.target_visible():
        #    return

        goal_pose = PoseStamped()

        goal_pose.header.frame_id = self.target_point.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = self.target_point.point.x
        goal_pose.pose.position.y = self.target_point.point.y
        goal_pose.pose.position.z = 0.0

        # No rotation for now
        goal_pose.pose.orientation.w = 1.0

        self.navigator.startToPose(goal_pose)
        self.has_goal = True

#    def target_moved_significantly(self):
#
#        if self.goal_target_position is None:
#            return True
#
#        dx = (self.target_point.point.x - self.goal_target_position.point.x)
#        dy = (self.target_point.point.y - self.goal_target_position.point.y)
#
#        distance = (dx**2 + dy**2) ** 0.5
#
#        return distance > self.goal_update_distance

    def rotate(self, angle: float = 0.1):

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