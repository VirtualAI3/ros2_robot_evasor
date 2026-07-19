#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64, ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

GOAL_X = 4.0
GOAL_Y = 4.0
GOAL_TOLERANCE = 0.3
OBSTACLE_THRESHOLD = 0.5
SAFE_DISTANCE = 1.2
ANGLE_TOLERANCE = 0.15
LINEAR_SPEED = 0.25
SLOW_SPEED = 0.12
ANGULAR_SPEED = 0.5

FRONT_ANGLE = math.radians(30)
SIDE_ANGLE = math.radians(60)

WHEELBASE = 0.24
TRACK_WIDTH = 0.24
WHEEL_RADIUS = 0.04


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller')
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.min_front = float('inf')
        self.min_left = float('inf')
        self.min_right = float('inf')

        self.scan = None
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.joint_states_received = False

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, '/world/room_world/model/evasor_bot/joint_state', self.joint_state_callback, 10)

        self.rl_wheel_pub = self.create_publisher(Float64, '/model/evasor_bot/joint/rl_wheel_joint/cmd_vel', 10)
        self.rr_wheel_pub = self.create_publisher(Float64, '/model/evasor_bot/joint/rr_wheel_joint/cmd_vel', 10)
        self.fl_steer_pub = self.create_publisher(Float64, '/model/evasor_bot/joint/fl_steering_joint/cmd_pos', 10)
        self.fr_steer_pub = self.create_publisher(Float64, '/model/evasor_bot/joint/fr_steering_joint/cmd_pos', 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.obstacle_marker_pub = self.create_publisher(Marker, '/obstacle_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_scan_time = self.get_clock().now()
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('ReactiveController iniciado (modo: comandos directos)')
        self.get_logger().info('Suscrito a: /scan, /joint_states')
        self.get_logger().info(f'Publicando comandos en 4 topics de joints')
        self.get_logger().info(f'Meta: ({GOAL_X}, {GOAL_Y}), Tolerancia: {GOAL_TOLERANCE}')

    def joint_state_callback(self, msg: JointState):
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info(f'/joint_states recibido con {len(msg.name)} joints')

        try:
            rl_idx = msg.name.index('rl_wheel_joint')
            rr_idx = msg.name.index('rr_wheel_joint')
        except ValueError:
            return

        left_pos = msg.position[rl_idx]
        right_pos = msg.position[rr_idx]

        if self.prev_left_pos is None or self.prev_right_pos is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            return

        delta_left = (left_pos - self.prev_left_pos) * WHEEL_RADIUS
        delta_right = (right_pos - self.prev_right_pos) * WHEEL_RADIUS

        ds = (delta_left + delta_right) / 2.0
        dtheta = (delta_right - delta_left) / TRACK_WIDTH

        self.yaw += dtheta
        self.pose_x += ds * math.cos(self.yaw)
        self.pose_y += ds * math.sin(self.yaw)

        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos

        self.publish_odom_tf()

    def publish_odom_tf(self):
        now = self.get_clock().now()
        qx = math.sin(self.yaw / 2.0) * 0.0
        qy = math.sin(self.yaw / 2.0) * 0.0
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

    def scan_callback(self, msg: LaserScan):
        self.scan = msg
        self.last_scan_time = self.get_clock().now()
        n = len(msg.ranges)
        ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                ranges.append(msg.range_max)
            else:
                ranges.append(max(r, msg.range_min))

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        def indices_for_range(start_angle, end_angle):
            start = max(0, int((start_angle - angle_min) / angle_inc))
            end = min(n, int((end_angle - angle_min) / angle_inc) + 1)
            return list(range(start, end))

        front_indices = indices_for_range(-FRONT_ANGLE, FRONT_ANGLE)
        left_indices = indices_for_range(FRONT_ANGLE, FRONT_ANGLE + SIDE_ANGLE)
        right_indices = indices_for_range(-FRONT_ANGLE - SIDE_ANGLE, -FRONT_ANGLE)

        self.min_front = min((ranges[i] for i in front_indices), default=float('inf'))
        self.min_left = min((ranges[i] for i in left_indices), default=float('inf'))
        self.min_right = min((ranges[i] for i in right_indices), default=float('inf'))

        if self.debug:
            n_valid = sum(1 for r in msg.ranges if not (math.isinf(r) or math.isnan(r)))
            n_close = sum(1 for r in msg.ranges
                          if not (math.isinf(r) or math.isnan(r)) and r < SAFE_DISTANCE)
            self.get_logger().info(
                f'LIDAR: {n_valid}/{len(msg.ranges)} valid rays, '
                f'{n_close} within safe_distance={SAFE_DISTANCE:.1f}m '
                f'f={self.min_front:.2f} l={self.min_left:.2f} r={self.min_right:.2f}',
                throttle_duration_sec=1.0)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_obstacle_markers(self):
        if self.scan is None:
            return
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacles'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08

        msg = self.scan
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            r = max(r, msg.range_min)
            if r > SAFE_DISTANCE * 1.2:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            world_angle = self.yaw + angle

            ox = self.pose_x + r * math.cos(world_angle)
            oy = self.pose_y + r * math.sin(world_angle)

            p = Point()
            p.x = ox
            p.y = oy
            p.z = 0.05
            marker.points.append(p)

            c = ColorRGBA()
            c.a = 1.0
            if r < 0.3:
                c.r = 1.0; c.g = 0.0; c.b = 0.0
            elif r < OBSTACLE_THRESHOLD:
                c.r = 1.0; c.g = 0.5; c.b = 0.0
            elif r < SAFE_DISTANCE:
                c.r = 1.0; c.g = 1.0; c.b = 0.0
            else:
                c.r = 0.0; c.g = 1.0; c.b = 0.0
            marker.colors.append(c)

        self.obstacle_marker_pub.publish(marker)

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = GOAL_X
        marker.pose.position.y = GOAL_Y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime.sec = 1

        text = Marker()
        text.header.frame_id = 'odom'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'goal_text'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = GOAL_X
        text.pose.position.y = GOAL_Y
        text.pose.position.z = 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = 'META'
        text.lifetime.sec = 1

        arrow = Marker()
        arrow.header.frame_id = 'odom'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'goal_arrow'
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = self.pose_x
        arrow.pose.position.y = self.pose_y
        arrow.pose.position.z = 0.1
        dx = GOAL_X - self.pose_x
        dy = GOAL_Y - self.pose_y
        yaw_to_goal = math.atan2(dy, dx)
        arrow.pose.orientation.z = math.sin(yaw_to_goal * 0.5)
        arrow.pose.orientation.w = math.cos(yaw_to_goal * 0.5)
        arrow.scale.x = math.hypot(dx, dy)
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color.a = 0.6
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.lifetime.sec = 1

        self.goal_marker_pub.publish(marker)
        self.goal_marker_pub.publish(text)
        self.goal_marker_pub.publish(arrow)

    def publish_joint_commands(self, v, w):
        left_vel = (v - w * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        right_vel = (v + w * TRACK_WIDTH / 2.0) / WHEEL_RADIUS

        if abs(v) > 0.01:
            steering_angle = math.atan2(w * WHEELBASE, v)
        else:
            steering_angle = 0.0

        steering_angle = max(-0.6, min(0.6, steering_angle))

        self.rl_wheel_pub.publish(Float64(data=left_vel))
        self.rr_wheel_pub.publish(Float64(data=right_vel))
        self.fl_steer_pub.publish(Float64(data=steering_angle))
        self.fr_steer_pub.publish(Float64(data=steering_angle))

        self.get_logger().info(
            f'CMD: rl={left_vel:.1f} rr={right_vel:.1f} steer={steering_angle:.2f}',
            throttle_duration_sec=2.0
        )

    def control_loop(self):
        if not self.joint_states_received:
            self.get_logger().warn('Esperando /joint_states...', throttle_duration_sec=5.0)
            return

        if self.scan is None:
            self.get_logger().warn('Esperando /scan...', throttle_duration_sec=5.0)
            return

        self.publish_goal_marker()
        self.publish_obstacle_markers()

        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed > 3.0:
            min_front = float('inf')
            min_left = float('inf')
            min_right = float('inf')
        else:
            min_front = self.min_front
            min_left = self.min_left
            min_right = self.min_right

        v = 0.0
        w = 0.0

        dx = GOAL_X - self.pose_x
        dy = GOAL_Y - self.pose_y
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < GOAL_TOLERANCE:
            self.get_logger().info(f'Meta alcanzada en ({self.pose_x:.2f}, {self.pose_y:.2f})!')
            self.publish_joint_commands(0.0, 0.0)
            return

        if min_front < OBSTACLE_THRESHOLD:
            v = SLOW_SPEED
            w = ANGULAR_SPEED if min_left > min_right else -ANGULAR_SPEED
            if self.debug:
                self.get_logger().info(
                    f'REACTIVE: OBSTACLE_THRESHOLD obs_front={min_front:.2f} '
                    f'turn={"L" if min_left > min_right else "R"}',
                    throttle_duration_sec=1.0)
        elif min_front < SAFE_DISTANCE:
            v = SLOW_SPEED
            w = ANGULAR_SPEED * 0.6 if min_left > min_right else -ANGULAR_SPEED * 0.6
            if self.debug:
                self.get_logger().info(
                    f'REACTIVE: SAFE_DISTANCE obs_front={min_front:.2f} '
                    f'turn={"L" if min_left > min_right else "R"}',
                    throttle_duration_sec=1.0)
        else:
            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - self.yaw)

            v = LINEAR_SPEED
            if abs(angle_error) > ANGLE_TOLERANCE:
                w = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, angle_error * 1.5))
            else:
                w = angle_error * 0.5
            if self.debug:
                self.get_logger().info(
                    f'REACTIVE: SEEK goal_angle={math.degrees(angle_to_goal):.1f}deg '
                    f'angle_err={math.degrees(angle_error):.1f}deg',
                    throttle_duration_sec=1.0)

        self.get_logger().info(
            f'v={v:.2f}, w={w:.2f} | pos=({self.pose_x:.2f},{self.pose_y:.2f}) | '
            f'yaw={self.yaw:.2f} | obs_front={min_front:.2f} '
            f'obs_left={min_left:.2f} obs_right={min_right:.2f}',
            throttle_duration_sec=2.0
        )
        self.publish_joint_commands(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
