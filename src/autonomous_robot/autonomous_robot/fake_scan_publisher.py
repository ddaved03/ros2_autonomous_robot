import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.t = 0.0
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz

    def tick(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.range_min = 0.05
        msg.range_max = 10.0

        n = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1

        # alapból 2.0m minden irányban
        ranges = [2.0] * n

        # “akadály” szimuláció: időnként 0.3m-re kerül elénk
        if int(self.t) % 6 < 3:
            mid = n // 2
            for i in range(mid - 5, mid + 6):
                ranges[i] = 0.3

        msg.ranges = ranges
        self.pub.publish(msg)

        self.t += 0.2

def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

