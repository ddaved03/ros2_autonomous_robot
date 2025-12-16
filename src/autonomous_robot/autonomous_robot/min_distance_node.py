#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool


class MinDistanceNode(Node):
    def __init__(self):
        super().__init__("min_distance_node")

        self.declare_parameter("stop_distance", 0.5)
        self.stop_distance = float(self.get_parameter("stop_distance").value)

        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        self.pub_min = self.create_publisher(Float32, "/min_distance", 10)
        self.pub_warn = self.create_publisher(Bool, "/collision_warning", 10)

        self.get_logger().info(f"MinDistanceNode started | stop_distance={self.stop_distance}")

    def scan_cb(self, msg: LaserScan):
        valid = []
        for r in msg.ranges:
            # NaN
            if r != r:
                continue
            # inf -> "nincs találat" (megtartjuk)
            if math.isinf(r):
                valid.append(r)
                continue
            # 0 vagy negatív értékek kuka
            if r <= 0.0:
                continue
            # range_min/range_max szűrés (ahogy kérted)
            if r < msg.range_min or r > msg.range_max:
                continue
            valid.append(r)

        if not valid:
            return

        min_dist = min(valid)
        if math.isinf(min_dist):
            min_dist = msg.range_max

        self.pub_min.publish(Float32(data=float(min_dist)))
        self.pub_warn.publish(Bool(data=(min_dist < self.stop_distance)))


def main(args=None):
    rclpy.init(args=args)
    node = MinDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

