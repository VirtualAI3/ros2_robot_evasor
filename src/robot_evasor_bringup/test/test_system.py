#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class SystemTest(Node):
    def __init__(self):
        super().__init__('system_test')
        self.odom_received = False
        self.scan_received = False
        self.cmd_vel_received = False
        self.cmd_vel_msg = None
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

    def cmd_vel_cb(self, msg):
        self.cmd_vel_received = True
        self.cmd_vel_msg = msg

    def test_topics(self):
        for i in range(30):
            topics = self.get_topic_names_and_types()
            names = [t[0] for t in topics]
            if '/odom' in names:
                self.get_logger().info('OK: /odom topic found')
                self.odom_received = True
            if '/scan' in names:
                self.get_logger().info('OK: /scan topic found')
                self.scan_received = True
            if self.odom_received and self.scan_received:
                break
            time.sleep(1)

        if not (self.odom_received and self.scan_received):
            self.get_logger().error(
                f'FAIL: odom={self.odom_received} scan={self.scan_received}')
            return False

        self.get_logger().info('TEST PASSED: topics odom + scan disponibles')

        # test cmd_vel is being published by controller
        for i in range(10):
            if self.cmd_vel_received:
                break
            time.sleep(0.5)
        if self.cmd_vel_received:
            self.get_logger().info(
                f'OK: /cmd_vel recibido (v={self.cmd_vel_msg.linear.x:.2f})')
        else:
            self.get_logger().warn('WARN: /cmd_vel no recibido (puede necesitar goal)')

        # test goal_pose topic
        if '/goal_pose' in [t[0] for t in self.get_topic_names_and_types()]:
            self.get_logger().info('OK: /goal_pose topic found')

        self.get_logger().info('TEST COMPLETED')
        return True


def main():
    rclpy.init()
    test = SystemTest()
    result = test.test_topics()
    test.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == '__main__':
    main()
