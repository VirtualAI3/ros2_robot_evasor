#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class ZoneData:
    __slots__ = ('min_distance', 'density', 'avg_close', 'centroid_angle')

    def __init__(self):
        self.min_distance = float('inf')
        self.density = 0.0
        self.avg_close = 0.0
        self.centroid_angle = 0.0


class VFHController(Node):
    def __init__(self):
        super().__init__('vfh_controller')
        self.setup_parameters()
        self.load_parameters()
        self.init_variables()
        self.init_ros_interfaces()
        self.get_logger().info('FTG Controller iniciado (diff drive + 3 zonas)')
        self.get_logger().info(
            f'Meta: ({self.goal_x:.1f}, {self.goal_y:.1f}), '
            f'tolerancia: {self.goal_tolerance}')

    # ===================== PARAMETERS =====================

    def setup_parameters(self):
        for name, default in {
            'goal_x': 4.0, 'goal_y': 4.0, 'goal_tolerance': 0.4,
            'safe_distance': 1.5, 'linear_speed': 0.25,
            'slow_speed': 0.12, 'angular_speed': 0.35,
            'robot_width': 0.24, 'track_width': 0.24, 'wheel_radius': 0.04,
            'clearance': 0.1, 'collision_margin': 0.35,
            'recovery_back_distance': 0.8, 'scan_timeout': 3.0,
            'smooth_factor': 0.20, 'max_accel_linear': 0.4,
            'max_accel_angular': 0.8, 'front_angle': 30.0,
            'side_angle': 90.0, 'loop_radius': 0.3,
            'loop_count_threshold': 3, 'loop_history_size': 200,
            'debug': False,
        }.items():
            self.declare_parameter(name, default)

    def load_parameters(self):
        p = {}
        for name in (
            'goal_x', 'goal_y', 'goal_tolerance',
            'safe_distance', 'linear_speed', 'slow_speed', 'angular_speed',
            'robot_width', 'track_width', 'wheel_radius', 'clearance',
            'collision_margin', 'recovery_back_distance', 'scan_timeout',
            'smooth_factor', 'max_accel_linear', 'max_accel_angular',
            'front_angle', 'side_angle', 'loop_radius',
            'loop_count_threshold', 'loop_history_size', 'debug',
        ):
            p[name] = self.get_parameter(name).value
        self.goal_x = p['goal_x']
        self.goal_y = p['goal_y']
        self.goal_tolerance = p['goal_tolerance']
        self.safe_distance = p['safe_distance']
        self.linear_speed = p['linear_speed']
        self.slow_speed = p['slow_speed']
        self.angular_speed = p['angular_speed']
        self.robot_width = p['robot_width']
        self.track_width = p['track_width']
        self.wheel_radius = p['wheel_radius']
        self.clearance = p['clearance']
        self.collision_margin = p['collision_margin']
        self.recovery_back_distance = p['recovery_back_distance']
        self.scan_timeout = p['scan_timeout']
        self.smooth_factor = p['smooth_factor']
        self.max_accel_linear = p['max_accel_linear']
        self.max_accel_angular = p['max_accel_angular']
        self.front_angle = math.radians(p['front_angle'])
        self.side_angle = math.radians(p['side_angle'])
        self.loop_radius = p['loop_radius']
        self.loop_count_threshold = p['loop_count_threshold']
        self.loop_history_size = p['loop_history_size']
        self.debug = p['debug']
        self.min_gap_width = self.robot_width + self.clearance

        self.zones_def = {
            'front': (-self.front_angle, self.front_angle),
            'left': (self.front_angle, self.front_angle + self.side_angle),
            'right': (-self.front_angle - self.side_angle, -self.front_angle),
        }

    # ===================== INIT =====================

    def init_variables(self):
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.scan = None
        self.last_scan_time = self.get_clock().now()
        self.smooth_v = 0.0
        self.smooth_w = 0.0
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.dt = 0.1
        self.reversing = False
        self.reverse_start_pose = (0.0, 0.0)
        self.reverse_start_time = None
        self.position_history = []
        self.loop_detected = False

    def init_ros_interfaces(self):
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/world/room_world/model/evasor_bot/joint_state',
            self.joint_state_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
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
        try:
            rl = msg.position[msg.name.index('rl_wheel_joint')]
            rr = msg.position[msg.name.index('rr_wheel_joint')]
        except ValueError:
            return
        if self.prev_left_pos is None:
            self.prev_left_pos = rl
            self.prev_right_pos = rr
            if self.debug:
                self.get_logger().info(f'ODOM: inicio odometría rl={rl:.2f} rr={rr:.2f}')
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
        if self.debug:
            self.get_logger().info(
                f'ODOM: rl={rl:.2f}rr={rr:.2f} dl={dl:.3f}dr={dr:.3f} '
                f'ds={ds:.3f}dy={dy:.3f} '
                f'pos=({self.pose_x:.2f},{self.pose_y:.2f}) yaw={math.degrees(self.yaw):.1f}deg',
                throttle_duration_sec=0.5)
        self.publish_odom()

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

    # ===================== ZONE PROCESSING =====================

    def compute_zones(self):
        zones = {name: ZoneData() for name in self.zones_def}
        if self.scan is None:
            return zones

        msg = self.scan
        total_rays = len(msg.ranges)
        close_counts = {name: 0 for name in self.zones_def}
        close_sum = {name: 0.0 for name in self.zones_def}
        hit_count = {name: 0 for name in self.zones_def}

        for i in range(total_rays):
            r = msg.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            r = max(r, msg.range_min)
            angle = msg.angle_min + i * msg.angle_increment

            for name, (start, end) in self.zones_def.items():
                if start <= angle <= end:
                    hit_count[name] += 1
                    if r < zones[name].min_distance:
                        zones[name].min_distance = r
                        zones[name].centroid_angle = angle
                    if r < self.safe_distance:
                        close_counts[name] += 1
                        close_sum[name] += r

        for name in self.zones_def:
            total = max(hit_count[name], 1)
            zones[name].density = close_counts[name] / total
            zones[name].avg_close = (close_sum[name] / max(close_counts[name], 1)
                                     if close_counts[name] > 0 else self.safe_distance)
            if zones[name].min_distance == float('inf'):
                zones[name].min_distance = msg.range_max

        return zones

    # ===================== GAP DETECTION =====================

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

        return gaps

    def select_best_gap(self, gaps):
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        goal_a = math.atan2(dy, dx)

        best = None
        best_score = -float('inf')
        for g in gaps:
            if g['width'] < self.min_gap_width:
                continue
            diff = abs(self.normalize_angle(g['center'] - goal_a))
            score = -diff * 3.0 + g['width'] * 0.5
            if score > best_score:
                best_score = score
                best = g
        return best

    # ===================== NAVIGATION ENGINE =====================

    def compute_navigation(self, zones, gaps):
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        goal_angle = math.atan2(dy, dx)
        heading_error = self.normalize_angle(goal_angle - self.yaw)
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            if self.debug:
                self.get_logger().info(f'NAV: meta alcanzada d={dist:.2f}<tol={self.goal_tolerance}')
            return 0.0, 0.0

        front = zones['front']
        left = zones['left']
        right = zones['right']
        front_urgency = 1.0 - min(front.min_distance / self.safe_distance, 1.0)
        left_urgency = 1.0 - min(left.min_distance / self.safe_distance, 1.0)
        right_urgency = 1.0 - min(right.min_distance / self.safe_distance, 1.0)

        lateral_bias = right_urgency - left_urgency
        gap_steer = 0.0
        gap_info = 'none'

        if front_urgency > 0.3 and gaps:
            best = self.select_best_gap(gaps)
            if best:
                gap_steer = self.normalize_angle(best['center'])
                gap_info = f'c={math.degrees(best["center"]):.0f}deg w={math.degrees(best["width"]):.0f}deg'
                if self.debug:
                    self.get_logger().info(
                        f'GAP: {gap_info} goal_a={math.degrees(goal_angle):.0f}deg '
                        f'diff={abs(math.degrees(self.normalize_angle(best["center"] - goal_angle))):.0f}deg',
                        throttle_duration_sec=0.5)

        if self.reversing:
            backed = math.hypot(self.pose_x - self.reverse_start_pose[0],
                                self.pose_y - self.reverse_start_pose[1])
            elapsed_rev = (self.get_clock().now() - self.reverse_start_time).nanoseconds / 1e9
            if self.debug:
                self.get_logger().info(
                    f'REV: backed={backed:.2f}/0.4 elapsed={elapsed_rev:.1f}s/3.0 '
                    f'front_urg={front_urgency:.2f} heading_err={math.degrees(heading_error):.0f}deg',
                    throttle_duration_sec=0.3)
            if backed >= 0.4 or front_urgency < 0.4 or elapsed_rev >= 3.0:
                self.reversing = False
                self.get_logger().info(
                    f'REVERSE exit: backed={backed:.2f} urg={front_urgency:.2f} '
                    f'elapsed={elapsed_rev:.1f}s')
                return self.linear_speed, 0.0
            w = self.normalize_angle(goal_angle - self.yaw) * 0.8
            w = max(-self.angular_speed, min(self.angular_speed, w))
            return self.slow_speed * -0.5, w

        if front.min_distance < self.collision_margin * 0.5:
            self.reversing = True
            self.reverse_start_pose = (self.pose_x, self.pose_y)
            self.reverse_start_time = self.get_clock().now()
            self.get_logger().warn(
                f'INICIO REVERSE: front={front.min_distance:.2f}m '
                f'coll_margin*0.5={self.collision_margin*0.5:.2f}m '
                f'lateral_bias={lateral_bias:.2f}')
            w = lateral_bias * self.angular_speed * 0.5
            return self.slow_speed * -0.8, w

        decision = ''
        if front_urgency < 0.2:
            v = self.linear_speed
            w = heading_error * 1.8
            decision = 'SEEK'
        elif front_urgency < 0.6:
            v = self.linear_speed * (1.0 - front_urgency * 0.5)
            w_blend = lateral_bias * front_urgency
            if abs(gap_steer) > 0.1 and gap_steer * lateral_bias > -0.1:
                w_blend = w_blend * 0.6 + gap_steer * 0.4
            w = heading_error * (1.0 - front_urgency * 0.7) + w_blend
            decision = 'BLEND'
        else:
            v = self.slow_speed * (1.0 - front_urgency * 0.5)
            if abs(gap_steer) > 0.1:
                w = gap_steer * 0.7 + lateral_bias * 0.3
                decision = 'GAP'
            else:
                w = lateral_bias * 0.8 + heading_error * 0.2
                decision = 'ESCAPE'

        w = self.clamp(w, -self.angular_speed, self.angular_speed)

        if self.debug:
            self.get_logger().info(
                f'NAV[{decision}] v={v:.2f} w={math.degrees(w):.0f}deg/s | '
                f'urg: f={front_urgency:.2f} l={left_urgency:.2f} r={right_urgency:.2f} | '
                f'heading_err={math.degrees(heading_error):.0f}deg gap_steer={math.degrees(gap_steer):.0f}deg | '
                f'gap={gap_info}',
                throttle_duration_sec=0.5)

        return v, w

    # ===================== LOOP DETECTION =====================

    def update_loop_detection(self):
        if self.reversing:
            return

        now = self.get_clock().now()
        current = (self.pose_x, self.pose_y)

        if len(self.position_history) >= 5:
            avg_speed = 0.0
            for px, py, t in self.position_history[-5:]:
                speed = math.hypot(current[0] - px, current[1] - py) / max((now - t).nanoseconds / 1e9, 0.001)
                avg_speed += speed
            avg_speed /= 5.0
            if avg_speed > 0.03:
                self.position_history.clear()
                self.loop_detected = False
                return

        self.position_history.append((*current, now))
        if len(self.position_history) > self.loop_history_size:
            self.position_history.pop(0)

        if len(self.position_history) < 20:
            return

        visit_count = 0
        for px, py, t in self.position_history[:-10]:
            if math.hypot(px - current[0], py - current[1]) < self.loop_radius:
                if (now - t).nanoseconds / 1e9 > 5.0:
                    visit_count += 1

        self.loop_detected = visit_count >= self.loop_count_threshold
        if self.loop_detected:
            self.get_logger().warn(f'BUCLE detectado: {visit_count} visitas en ({self.pose_x:.2f}, {self.pose_y:.2f})')

    # ===================== CONTROL LOOP =====================

    def control_loop(self):
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

        zones = self.compute_zones()
        gaps = self.find_gaps()

        d = math.hypot(self.goal_x - self.pose_x, self.goal_y - self.pose_y)
        if d < self.goal_tolerance:
            self.get_logger().info(f'Meta alcanzada en ({self.pose_x:.2f}, {self.pose_y:.2f})!')
            self.publish_commands(0.0, 0.0)
            return

        target_v, target_w = self.compute_navigation(zones, gaps)

        self.update_loop_detection()
        if self.loop_detected and not self.reversing:
            self.reversing = True
            self.reverse_start_pose = (self.pose_x, self.pose_y)
            self.reverse_start_time = self.get_clock().now()
            target_v = self.slow_speed * -0.8
            target_w = 0.5

        self.smooth_v = self.smooth_v * (1.0 - self.smooth_factor) + target_v * self.smooth_factor
        self.smooth_w = self.smooth_w * (1.0 - self.smooth_factor) + target_w * self.smooth_factor

        v = self.clamp(self.smooth_v,
                       self.prev_v - self.max_accel_linear * self.dt,
                       self.prev_v + self.max_accel_linear * self.dt)
        w = self.clamp(self.smooth_w,
                       self.prev_w - self.max_accel_angular * self.dt,
                       self.prev_w + self.max_accel_angular * self.dt)

        self.prev_v = v
        self.prev_w = w

        if self.debug:
            self.get_logger().info(
                f'v={v:.2f} w={w:.2f} | pos=({self.pose_x:.2f},{self.pose_y:.2f}) '
                f'f={zones["front"].min_distance:.2f} '
                f'l={zones["left"].min_distance:.2f} '
                f'r={zones["right"].min_distance:.2f} '
                f'd={d:.2f} rev={self.reversing}',
                throttle_duration_sec=1.0)

        self.publish_commands(v, w)

    # ===================== COMMAND PUBLISHING =====================

    def publish_commands(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

        if self.debug:
            self.get_logger().info(
                f'CMD: v={v:.2f} w={w:.2f}',
                throttle_duration_sec=0.5)

    # ===================== UTILITY =====================

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def clamp(val, lo, hi):
        return max(lo, min(hi, val))

    # ===================== MARKERS =====================

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
