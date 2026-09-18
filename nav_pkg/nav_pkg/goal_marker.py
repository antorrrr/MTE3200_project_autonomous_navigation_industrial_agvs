#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class GoalMarker(Node):

    def __init__(self):
        super().__init__("goal_marker")

        self.publisher = self.create_publisher(
            Marker,
            "/goal_marker",
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.publish_marker
        )

    def publish_marker(self):

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "goal"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = -4.72791
        marker.pose.position.y = 18.3567
        marker.pose.position.z = 0.2

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)
        
        start_marker = Marker()

        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()

        start_marker.ns = "start"
        start_marker.id = 1

        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD

        start_marker.pose.position.x = -0.011577
        start_marker.pose.position.y = -0.0428872
        start_marker.pose.position.z = 0.2

        start_marker.pose.orientation.w = 1.0

        start_marker.scale.x = 0.4
        start_marker.scale.y = 0.4
        start_marker.scale.z = 0.4

        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0

        self.publisher.publish(start_marker)


def main(args=None):
    rclpy.init(args=args)

    node = GoalMarker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()