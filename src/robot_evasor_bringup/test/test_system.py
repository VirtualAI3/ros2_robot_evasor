#!/usr/bin/env python3
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SystemTest(Node):
    def __init__(self):
        super().__init__('system_test')
        self.odom_received = False
        self.scan_received = False

    def test_topics(self):
        for i in range(30):
            topics = self.get_topic_names_and_types()
            topic_names = [t[0] for t in topics]
            if '/odom' in topic_names:
                self.get_logger().info('OK: /odom topic found')
                self.odom_received = True
            if '/scan' in topic_names:
                self.get_logger().info('OK: /scan topic found')
                self.scan_received = True
            if '/cmd_vel' in topic_names:
                self.get_logger().info('OK: /cmd_vel topic found')
            if self.odom_received and self.scan_received:
                break
            time.sleep(1)

        if self.odom_received and self.scan_received:
            self.get_logger().info('TEST PASSED: All topics available')
            pub = self.create_publisher(Twist, '/cmd_vel', 10)
            twist = Twist()
            twist.linear.x = 0.3
            pub.publish(twist)
            self.get_logger().info('Published cmd_vel: linear.x=0.3')
            time.sleep(0.5)
            twist.angular.z = 0.3
            pub.publish(twist)
            self.get_logger().info('Published cmd_vel: angular.z=0.3')
            return True
        else:
            self.get_logger().error(f'TEST FAILED: odom={self.odom_received}, scan={self.scan_received}')
            return False


def main():
    rclpy.init()
    test = SystemTest()
    result = test.test_topics()
    test.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == '__main__':
    main()
