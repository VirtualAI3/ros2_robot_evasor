#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        self.declare_parameter('robot_description', '')
        desc = self.get_parameter('robot_description').value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, '/robot_description', qos)

        self._msg = String()
        self._msg.data = desc
        self._first = True

        self.create_timer(0.5, self.publish)

    def publish(self):
        self.pub.publish(self._msg)
        if self._first:
            self.get_logger().info('Robot description publicado')
            self._first = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
