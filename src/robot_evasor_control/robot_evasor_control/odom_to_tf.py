#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):

    def __init__(self):
        super().__init__('odom_to_tf')
        
        self.get_logger().info('=== ODOM_TO_TF NODE STARTING ===')
        
        use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'use_sim_time: {use_sim_time}')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Creating subscription to /model/evasor_bot/odometry')
        self.subscription = self.create_subscription(
            Odometry, '/model/evasor_bot/odometry', self.odom_callback, 10)
        
        self.msg_count = 0
        self.tf_count = 0
        
        self.status_timer = self.create_timer(2.0, self.status_callback)
        
        self.get_logger().info('=== ODOM_TO_TF NODE READY ===')

    def odom_callback(self, msg):
        self.msg_count += 1
        
        if self.msg_count <= 3:
            self.get_logger().info(f'Received odom message #{self.msg_count}: stamp={msg.header.stamp}')
        
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        self.tf_count += 1
        
        if self.tf_count <= 3:
            self.get_logger().info(f'Published TF #{self.tf_count}: odom -> base_footprint')

    def status_callback(self):
        self.get_logger().info(f'[STATUS] Messages received: {self.msg_count}, TFs published: {self.tf_count}')


def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
