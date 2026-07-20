#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class DiagnosticsNode(Node):

    def __init__(self):
        super().__init__('diagnostics_node')
        self.get_logger().info('=== DIAGNOSTICS NODE STARTED ===')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_subscription = self.create_subscription(
            Odometry, '/model/evasor_bot/odometry', self.odom_callback, 10)
        self.odom_received = False
        self.odom_count = 0

        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.scan_received = False
        self.scan_count = 0

        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_received = False
        self.cmd_vel_count = 0

        action_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status_callback, action_qos)
        self.goal_status_count = 0
        self.last_goal_status = 'NONE'

        self.timer = self.create_timer(5.0, self.check_status)

    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info(f'First odom received! stamp={msg.header.stamp}')
            self.get_logger().info(f'   Position: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}')
            self.odom_received = True
        self.odom_count += 1

    def scan_callback(self, msg):
        if not self.scan_received:
            self.get_logger().info(f'First scan received! stamp={msg.header.stamp}')
            self.get_logger().info(f'   Ranges: {len(msg.ranges)} pts, angles=[{msg.angle_min:.3f}, {msg.angle_max:.3f}]')
            self.scan_received = True
        self.scan_count += 1

    def cmd_vel_callback(self, msg):
        if not self.cmd_vel_received:
            self.get_logger().info(f'First cmd_vel! linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
            self.cmd_vel_received = True
        self.cmd_vel_count += 1

    def goal_status_callback(self, msg):
        self.goal_status_count += 1
        for status in msg.status_list:
            goal_id = status.goal_id.id[:8] if status.goal_id.id else 'N/A'
            state = status.status
            state_names = {
                0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED',
                3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED',
                6: 'PREEMPTING', 7: 'RECALLING', 8: 'RECALLED',
                9: 'LOST'
            }
            state_name = state_names.get(state, f'UNKNOWN({state})')
            if self.goal_status_count <= 5:
                self.get_logger().info(f'🎯 Goal status #{self.goal_status_count}: id={goal_id} state={state_name}')
            self.last_goal_status = state_name

            if state == 3:
                self.get_logger().info(f'✅ Goal SUCCEEDED!')
            elif state == 4:
                self.get_logger().info(f'❌ Goal ABORTED!')
            elif state == 5:
                self.get_logger().info(f'⛔ Goal REJECTED!')

    def check_status(self):
        self.get_logger().info('--- DIAGNOSTICS CHECK ---')
        self.get_logger().info(f'Odom: {self.odom_count} | Scan: {self.scan_count} | cmd_vel: {self.cmd_vel_count}')

        if self.cmd_vel_count > 0:
            self.get_logger().info('   ✅ Robot receiving velocity commands - should be moving!')
        else:
            self.get_logger().warn('   ⚠️  NO velocity commands received - robot will not move')

        frames = ['odom', 'map', 'base_footprint', 'base_link', 'lidar_link']
        for frame in frames:
            try:
                self.tf_buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
                self.get_logger().info(f'   TF ok: base_footprint -> {frame}')
            except tf2_ros.LookupException:
                self.get_logger().warn(f'   TF missing: base_footprint -> {frame}')
            except Exception as e:
                self.get_logger().warn(f'   TF error for {frame}: {str(e)[:50]}')

        try:
            self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            self.get_logger().info(f'   TF ok: map -> odom (SLAM running)')
        except:
            self.get_logger().warn('   TF missing: map -> odom (SLAM inactive)')

        self.get_logger().info(f'   Last nav goal status: {self.last_goal_status}')
        self.get_logger().info(f'   Goal status updates received: {self.goal_status_count}')

        topics = self.get_topic_names_and_types()
        nav_actions = [t for t, _ in topics if 'navigate_to_pose' in t or 'navigate_through_poses' in t]
        if nav_actions:
            self.get_logger().info(f'   ✅ Nav actions: {len(nav_actions)} topics found')
        else:
            self.get_logger().warn('   ⚠️  No navigation actions detected!')


def main():
    rclpy.init()
    node = DiagnosticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
