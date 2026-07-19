#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64, ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class State(Enum):
    SEEK_GOAL = 1
    GAP = 2
    REVERSE = 3
    STOP = 4


class VFHController(Node):
    def __init__(self):
        super().__init__('vfh_controller')
        self.setup_parameters()
        self.load_parameters()
        self.init_variables()
        self.init_ros_interfaces()
        self.get_logger().info('FTG Controller iniciado')
        self.get_logger().info(
            f'Meta: ({self.goal_x:.1f}, {self.goal_y:.1f}), '
            f'tolerancia: {self.goal_tolerance}')

    # ===================== PARAMETERS =====================

    def setup_parameters(self):
        for name, default in {
            'goal_x': 4.0, 'goal_y': 4.0, 'goal_tolerance': 0.4,
            'safe_distance': 1.5, 'linear_speed': 0.25,
            'slow_speed': 0.12, 'angular_speed': 0.35,
            'robot_width': 0.24, 'clearance': 0.1,
            'stuck_timeout': 20.0,
            'wheelbase': 0.24, 'track_width': 0.24, 'wheel_radius': 0.04,
            'scan_timeout': 3.0, 'recovery_back_distance': 0.8,
            'max_steering_angle': 0.6,
            'collision_margin': 0.35, 'reverse_speed': -0.12,
            'debug': False,
        }.items():
            self.declare_parameter(name, default)

    def load_parameters(self):
        p = {}
        for name in ('goal_x', 'goal_y', 'goal_tolerance',
                      'safe_distance',
                      'linear_speed', 'slow_speed',
                      'angular_speed',
                      'robot_width', 'clearance',
                      'stuck_timeout',
                      'wheelbase', 'track_width', 'wheel_radius',
                      'scan_timeout', 'recovery_back_distance',
                      'max_steering_angle',
                      'collision_margin', 'reverse_speed',
                      'debug'):
            p[name] = self.get_parameter(name).value
        self.goal_x = p['goal_x']
        self.goal_y = p['goal_y']
        self.goal_tolerance = p['goal_tolerance']
        self.safe_distance = p['safe_distance']
        self.linear_speed = p['linear_speed']
        self.slow_speed = p['slow_speed']
        self.angular_speed = p['angular_speed']
        self.robot_width = p['robot_width']
        self.clearance = p['clearance']
        self.stuck_timeout = p['stuck_timeout']
        self.wheelbase = p['wheelbase']
        self.track_width = p['track_width']
        self.wheel_radius = p['wheel_radius']
        self.scan_timeout = p['scan_timeout']
        self.recovery_back_distance = p['recovery_back_distance']
        self.max_steering_angle = p['max_steering_angle']
        self.collision_margin = p['collision_margin']
        self.reverse_speed = p['reverse_speed']
        self.debug = p['debug']
        self.min_gap_width = self.robot_width + self.clearance

    # ===================== INIT =====================

    def init_variables(self):
        self.state = State.SEEK_GOAL
        self.entry_time = self.get_clock().now()
        self.entry_yaw = 0.0
        self.entry_x = 0.0
        self.entry_y = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.scan = None
        self.last_scan_time = self.get_clock().now()
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.joint_states_received = False
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

    def init_ros_interfaces(self):
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/world/room_world/model/evasor_bot/joint_state',
            self.joint_state_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rl_pub = self.create_publisher(
            Float64, '/model/evasor_bot/joint/rl_wheel_joint/cmd_vel', 10)
        self.rr_pub = self.create_publisher(
            Float64, '/model/evasor_bot/joint/rr_wheel_joint/cmd_vel', 10)
        self.fl_pub = self.create_publisher(
            Float64, '/model/evasor_bot/joint/fl_steering_joint/cmd_pos', 10)
        self.fr_pub = self.create_publisher(
            Float64, '/model/evasor_bot/joint/fr_steering_joint/cmd_pos', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.obstacle_marker_pub = self.create_publisher(Marker, '/obstacle_markers', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self.control_loop)

    # ===================== CALLBACKS =====================

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(
            f'Nuevo goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def joint_state_cb(self, msg):
        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info('/joint_states recibido')
        try:
            rl = msg.position[msg.name.index('rl_wheel_joint')]
            rr = msg.position[msg.name.index('rr_wheel_joint')]
        except ValueError:
            return
        if self.prev_left_pos is None:
            self.prev_left_pos = rl
            self.prev_right_pos = rr
            return
        dl = (rl - self.prev_left_pos) * self.wheel_radius
        dr = (rr - self.prev_right_pos) * self.wheel_radius
        ds = (dl + dr) * 0.5
        dy = (dr - dl) / self.track_width if self.track_width > 0 else 0.0
        self.yaw += dy
        self.pose_x += ds * math.cos(self.yaw)
        self.pose_y += ds * math.sin(self.yaw)
        self.prev_left_pos = rl
        self.prev_right_pos = rr
        self.publish_odom()

    def scan_cb(self, msg):
        self.scan = msg
        self.last_scan_time = self.get_clock().now()
        if self.debug:
            n_valid = sum(1 for r in msg.ranges if not (math.isinf(r) or math.isnan(r)))
            n_close = sum(1 for r in msg.ranges
                          if not (math.isinf(r) or math.isnan(r)) and r < self.safe_distance)
            self.get_logger().info(
                f'LIDAR: {n_valid}/{len(msg.ranges)} valid rays, {n_close} close',
                throttle_duration_sec=1.0)

    # ===================== ODOMETRY / TF =====================

    def publish_odom(self):
        now = self.get_clock().now()
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.1
        self.odom_pub.publish(odom)

    # ===================== FTG (Follow The Gap) =====================

    def find_gaps(self):
        if self.scan is None:
            return []
        msg = self.scan
        angles = []
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            if math.isinf(r) or math.isnan(r):
                r = msg.range_max
            r = max(r, msg.range_min)
            angle = msg.angle_min + i * msg.angle_increment
            angles.append((angle, r))

        gaps = []
        i = 0
        n = len(angles)
        while i < n:
            angle, r = angles[i]
            if r > self.safe_distance:
                s = i
                while i < n and angles[i][1] > self.safe_distance:
                    i += 1
                e = i - 1
                start_angle = msg.angle_min + s * msg.angle_increment
                end_angle = msg.angle_min + e * msg.angle_increment
                width = abs(end_angle - start_angle)
                center = (start_angle + end_angle) * 0.5
                gaps.append({'center': center, 'width': width})
            else:
                i += 1

        if self.debug:
            self.get_logger().info(
                f'GAPS: {len(gaps)} found ' +
                ', '.join(f'w={math.degrees(g["width"]):.1f}deg c={math.degrees(g["center"]):.1f}deg' for g in gaps),
                throttle_duration_sec=1.0)
        return gaps

    def select_best_gap(self, gaps):
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        goal_a = math.atan2(dy, dx)

        best = None
        best_score = -float('inf')
        for g in gaps:
            if g['width'] < self.min_gap_width:
                if self.debug:
                    self.get_logger().info(
                        f'GAP_SKIP: w={math.degrees(g["width"]):.1f}deg < {math.degrees(self.min_gap_width):.1f}deg',
                        throttle_duration_sec=1.0)
                continue
            diff = abs(self.normalize_angle(g['center'] - goal_a))
            score = g['width'] * 2.0 - diff * 0.8
            if self.debug:
                self.get_logger().info(
                    f'GAP_SCORE: w={math.degrees(g["width"]):.1f}deg diff={math.degrees(diff):.1f}deg score={score:.3f}',
                    throttle_duration_sec=1.0)
            if score > best_score:
                best_score = score
                best = g
        if best and self.debug:
            self.get_logger().info(
                f'GAP_BEST: w={math.degrees(best["width"]):.1f}deg c={math.degrees(best["center"]):.1f}deg score={best_score:.3f}',
                throttle_duration_sec=1.0)
        return best

    def get_front_clearance_raw(self):
        if self.scan is None:
            return float('inf')
        msg = self.scan
        half_angle = math.radians(15)
        d = float('inf')
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > half_angle:
                continue
            r = msg.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            d = min(d, max(r, msg.range_min))
        return max(0.0, d - self.collision_margin)

    def get_side_clearance_raw(self, left=True):
        if self.scan is None:
            return float('inf')
        msg = self.scan
        if left:
            s_ang = math.radians(30)
            e_ang = math.radians(90)
        else:
            s_ang = math.radians(-90)
            e_ang = math.radians(-30)
        d = float('inf')
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < s_ang or angle > e_ang:
                continue
            r = msg.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            d = min(d, max(r, msg.range_min))
        return max(0.0, d - self.collision_margin)

    def best_escape_dir(self):
        left = self.get_side_clearance_raw(left=True)
        right = self.get_side_clearance_raw(left=False)
        return 1.0 if left > right else -1.0

    def has_frontal_gap(self, gaps):
        for g in gaps:
            if abs(g['center']) < math.radians(90) and g['width'] > self.min_gap_width:
                return True
        return False

    def turn_toward_goal(self):
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        goal_a = math.atan2(dy, dx)
        err = self.normalize_angle(goal_a - self.yaw)
        w = max(-self.angular_speed, min(self.angular_speed, err * 1.5))
        return 0.0, w

    # ===================== UTILITY =====================

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ===================== STATE MACHINE =====================

    def set_state(self, new_state, reason=''):
        if self.state == new_state:
            return
        old = self.state
        self.state = new_state
        self.entry_time = self.get_clock().now()
        self.entry_yaw = self.yaw
        self.entry_x = self.pose_x
        self.entry_y = self.pose_y
        msg = f'FSM: {old.name} -> {new_state.name}'
        if reason:
            msg += f' ({reason})'
        self.get_logger().info(msg)

    def elapsed_since_entry(self):
        return (self.get_clock().now() - self.entry_time).nanoseconds / 1e9

    def dist_to_goal(self):
        return math.hypot(self.goal_x - self.pose_x, self.goal_y - self.pose_y)

    def check_stuck(self, d):
        if self.state in (State.SEEK_GOAL, State.GAP):
            elapsed = self.elapsed_since_entry()
            moved = math.hypot(self.pose_x - self.entry_x, self.pose_y - self.entry_y)
            if self.debug and elapsed > self.stuck_timeout * 0.7:
                self.get_logger().info(
                    f'STUCK: elapsed={elapsed:.1f}s moved={moved:.2f}m state={self.state.name}',
                    throttle_duration_sec=1.0)
            return elapsed > self.stuck_timeout and moved < 0.3
        return False

    # ===================== CONTROL LOOP =====================

    def control_loop(self):
        if not self.joint_states_received:
            self.get_logger().warn('Esperando joint_states...', throttle_duration_sec=5.0)
            return
        if self.scan is None:
            self.get_logger().warn('Esperando scan...', throttle_duration_sec=5.0)
            return

        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed > self.scan_timeout:
            self.get_logger().warn('Scan obsoleto, deteniendo', throttle_duration_sec=2.0)
            self.publish_commands(0.0, 0.0)
            return

        self.publish_goal_marker()
        self.publish_obstacle_markers()

        d = self.dist_to_goal()
        if d < self.goal_tolerance:
            if self.state != State.STOP:
                self.get_logger().info(f'Meta alcanzada en ({self.pose_x:.2f}, {self.pose_y:.2f})!')
                self.set_state(State.STOP, reason='goal reached')
            self.publish_commands(0.0, 0.0)
            return

        gaps = self.find_gaps()
        best_gap = self.select_best_gap(gaps)

        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        goal_a = math.atan2(dy, dx)
        goal_err = abs(self.normalize_angle(goal_a - self.yaw))
        front_fc = self.get_front_clearance_raw()

        if self.debug:
            target_str = 'none' if best_gap is None else f'c={math.degrees(best_gap["center"]):.1f}deg w={math.degrees(best_gap["width"]):.1f}deg'
            self.get_logger().info(
                f'DEBUG: goal_a={math.degrees(goal_a):.1f}deg err={math.degrees(goal_err):.1f}deg '
                f'fc={front_fc:.2f} d={d:.2f} gaps={len(gaps)} frontal={self.has_frontal_gap(gaps)} '
                f'best={target_str}',
                throttle_duration_sec=1.0)

        has_frontal = self.has_frontal_gap(gaps)
        too_close = front_fc < self.collision_margin * 0.3

        if too_close and self.state not in (State.REVERSE, State.STOP):
            self.set_state(State.REVERSE, reason=f'fc={front_fc:.2f} < collision_margin*0.3={self.collision_margin*0.3:.2f}')
        elif not has_frontal and self.state == State.SEEK_GOAL:
            if best_gap:
                self.set_state(State.GAP, reason='no frontal gaps, best behind')
            elif too_close:
                self.set_state(State.REVERSE, reason='no gaps at all, too close')

        if self.state == State.STOP:
            self.publish_commands(0.0, 0.0)
            return

        v = 0.0
        w = 0.0

        if self.state == State.SEEK_GOAL:
            v, w = self.run_seek(d, goal_a, goal_err, front_fc)
        elif self.state == State.GAP:
            v, w = self.run_gap(goal_a, best_gap, front_fc, has_frontal, gaps)
        elif self.state == State.REVERSE:
            v, w = self.run_reverse(front_fc)

        if self.check_stuck(d):
            self.get_logger().warn(f'Atascado -> REVERSE')
            self.set_state(State.REVERSE, reason='stuck')
            v, w = self.run_reverse(front_fc)

        self.get_logger().info(
            f'{self.state.name:15s} | v={v:.2f} w={w:.2f} | '
            f'pos=({self.pose_x:.2f},{self.pose_y:.2f}) fc={front_fc:.2f} d={d:.2f}',
            throttle_duration_sec=2.0)

        self.publish_commands(v, w)

    # ===================== STATE BEHAVIORS =====================

    def run_seek(self, d, goal_a, goal_err, front_fc):
        if front_fc < self.collision_margin * 0.3:
            self.set_state(State.REVERSE, reason=f'seek: fc={front_fc:.2f}')
            return self.reverse_speed, 0.0

        if front_fc < self.safe_distance:
            left = self.get_side_clearance_raw(left=True)
            right = self.get_side_clearance_raw(left=False)
            dir_ = 1.0 if left > right else -1.0
            speed = self.slow_speed * min(1.0, front_fc / (self.safe_distance * 0.5))
            w = dir_ * self.angular_speed * min(1.0, 1.0 - (front_fc / self.safe_distance))
            if self.debug:
                self.get_logger().info(
                    f'STEER_AROUND: fc={front_fc:.2f} l={left:.2f} r={right:.2f} '
                    f'dir={"L" if dir_ > 0 else "R"} v={speed:.2f} w={w:.2f}',
                    throttle_duration_sec=0.5)
            return speed, w

        near_goal = d < 2.0
        if near_goal and goal_err < math.radians(60):
            speed = min(self.linear_speed * (d / 2.0), 0.2)
            w = self.normalize_angle(goal_a - self.yaw) * 1.5
            w = max(-self.angular_speed, min(self.angular_speed, w))
            if self.debug:
                self.get_logger().info(f'SEEK_NEAR: v={speed:.2f} w={w:.2f}', throttle_duration_sec=0.5)
            return speed, w

        if goal_err < math.radians(90):
            v = self.linear_speed * min(1.0, front_fc / self.safe_distance)
            w = self.normalize_angle(goal_a - self.yaw) * 1.5
            w = max(-self.angular_speed, min(self.angular_speed, w))
            return v, w

        return self.turn_toward_goal()

    def run_gap(self, goal_a, best_gap, front_fc, has_frontal, gaps):
        if has_frontal and front_fc > self.safe_distance:
            self.set_state(State.SEEK_GOAL, reason=f'gap clear: fc={front_fc:.2f}')
            return self.run_seek(self.dist_to_goal(), goal_a,
                                 abs(self.normalize_angle(goal_a - self.yaw)), front_fc)

        if best_gap is None:
            if front_fc < self.collision_margin * 0.3:
                self.set_state(State.REVERSE, reason='no gap, too close')
                return self.reverse_speed, 0.0
            return self.turn_toward_goal()

        err = self.normalize_angle(best_gap['center'] - self.yaw)
        w = max(-self.angular_speed, min(self.angular_speed, err * 1.2))
        base_w = best_gap['width'] / math.pi
        v = self.slow_speed * min(1.0, base_w + 0.3)

        if has_frontal and front_fc > self.collision_margin:
            goal_err = self.normalize_angle(goal_a - self.yaw)
            blend = min(1.0, front_fc / self.safe_distance)
            w_goal = max(-self.angular_speed, min(self.angular_speed, goal_err * 1.5))
            w = w * (1.0 - blend * 0.5) + w_goal * (blend * 0.5)
            v = v * (1.0 - blend * 0.3) + self.linear_speed * (blend * 0.3)

        return v, w

    def run_reverse(self, front_fc):
        backed = math.hypot(self.pose_x - self.entry_x, self.pose_y - self.entry_y)
        if self.debug:
            self.get_logger().info(
                f'REVERSE: backed={backed:.2f}/{self.recovery_back_distance} fc={front_fc:.2f}',
                throttle_duration_sec=1.0)

        if backed >= self.recovery_back_distance:
            self.set_state(State.SEEK_GOAL, reason=f'reverse done: backed={backed:.2f}')
            return self.linear_speed * 0.5, 0.0

        dir_ = self.best_escape_dir()
        w = dir_ * self.angular_speed * 0.5
        v = self.reverse_speed
        return v, w

    # ===================== OBSTACLE MARKERS =====================

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
            if r > self.safe_distance * 1.2:
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
            if r < self.collision_margin:
                c.r = 1.0; c.g = 0.0; c.b = 0.0
            elif r < self.safe_distance * 0.5:
                c.r = 1.0; c.g = 0.5; c.b = 0.0
            elif r < self.safe_distance:
                c.r = 1.0; c.g = 1.0; c.b = 0.0
            else:
                c.r = 0.0; c.g = 1.0; c.b = 0.0
            marker.colors.append(c)

        self.obstacle_marker_pub.publish(marker)

    # ===================== GOAL MARKER =====================

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
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
        text.pose.position.x = self.goal_x
        text.pose.position.y = self.goal_y
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
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
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

    # ===================== COMMAND PUBLISHING =====================

    def publish_commands(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

        half_track = self.track_width * 0.5
        rl_vel = (v - w * half_track) / self.wheel_radius if self.wheel_radius > 0 else 0.0
        rr_vel = (v + w * half_track) / self.wheel_radius if self.wheel_radius > 0 else 0.0

        if abs(v) > 0.01:
            steer = math.atan2(w * self.wheelbase, abs(v))
            if v < 0:
                steer = -steer
        else:
            steer = 0.0
        steer = max(-self.max_steering_angle,
                    min(self.max_steering_angle, steer))

        self.rl_pub.publish(Float64(data=rl_vel))
        self.rr_pub.publish(Float64(data=rr_vel))
        self.fl_pub.publish(Float64(data=steer))
        self.fr_pub.publish(Float64(data=steer))

        self.get_logger().info(
            f'CMD: v={v:.2f} w={w:.2f} | '
            f'lw={rl_vel:.1f} rw={rr_vel:.1f} steer={steer:.2f}',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = VFHController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
