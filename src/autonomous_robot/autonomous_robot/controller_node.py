import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Paraméterek
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.8)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        # Publisher / Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.get_logger().info(
            f'Controller started | stop_distance={self.stop_distance} '
            f'forward_speed={self.forward_speed} turn_speed={self.turn_speed}'
        )

    def on_scan(self, msg: LaserScan):
        """
        TurtleBot3 / Gazebo LaserScan-ben sok érték 'inf' lehet (nincs találat).
        Ez nem hiba: ilyenkor úgy kezeljük, mintha messze lenne az akadály.
        """

        valid = []
        for r in msg.ranges:
            # NaN szűrés: (NaN != NaN) igaz
            if r != r:
                continue
            # 0 vagy negatív értékeket dobjuk
            if r < msg.range_min or r > msg.range_max:
                continue
            valid.append(r)


            # inf-et megtartjuk: "nincs találat"
            valid.append(r)

        if not valid:
            # nincs értelmezhető adat
            return

        min_dist = min(valid)

        # Ha minden inf, akkor tekintsük úgy, hogy "max hatótávig tiszta"
        if min_dist == float('inf'):
            min_dist = msg.range_max if msg.range_max > 0.0 else 10.0

        twist = Twist()

        if min_dist < self.stop_distance:
            # Akadály közel -> fordulás
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info(f'Obstacle: {min_dist:.2f} m -> TURN')
        else:
            # Tiszta -> előre
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.get_logger().info(f'Clear: {min_dist:.2f} m -> FORWARD')

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

