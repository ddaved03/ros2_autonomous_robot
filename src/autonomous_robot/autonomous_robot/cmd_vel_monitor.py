import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        self.get_logger().info(f'/cmd_vel -> lin.x={msg.linear.x:.2f}, ang.z={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

