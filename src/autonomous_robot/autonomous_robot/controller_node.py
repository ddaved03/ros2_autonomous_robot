import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.8)

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.get_logger().info(
            f'Controller started | stop_distance={self.stop_distance} '
            f'forward_speed={self.forward_speed} turn_speed={self.turn_speed}'
        )

    def on_scan(self, msg: LaserScan):
        # védekezés: 0 / inf / nan értékek kiszűrése
        valid = [r for r in msg.ranges if r > 0.0 and r != float('inf')]
        if not valid:
            return

        min_dist = min(valid)

        twist = Twist()
        if min_dist < self.stop_distance:
            # akadály közel: fordulj balra (helyben)
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info(f'Obstacle: {min_dist:.2f} m -> TURN')
        else:
            # szabad: előre
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

