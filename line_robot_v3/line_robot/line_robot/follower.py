#!/usr/bin/env python3
"""
follower.py — Dijkstra Shortest-Path Line Follower with LiDAR Obstacle Bypass
==============================================================================
ROS2 Jazzy / Ubuntu 24.04 / Gazebo Harmonic

COORDINATE FRAME
----------------
All (x, y) positions use the ODOMETRY frame:
  odom origin = robot spawn point = world (-3.5, 0).
  odom x = world_x + 3.5,   odom y = world_y.
Waypoints from path_planner.py are already in odom frame.
FINISH_X_ODOM = world(6.38) + 3.5 = 9.88.

ALGORITHM
---------
1. Dijkstra (path_planner.py) pre-plans the shortest route once at startup.
   Two branches: main zig-zag (passes near both obstacles, penalised) vs
   shortcut (clears obstacles, shorter).  Dijkstra picks the shortcut.

2. NAVIGATE_TO_WP: steer toward each odom-frame waypoint with a heading
   P-controller.  When the camera sees the black line AND the robot is
   within LINE_TRUST_LATERAL of the planned path, switch to LINE_FOLLOW.

3. LINE_FOLLOW: camera P-controller on centroid error.  Waypoint index
   advances by proximity so the path stays tracked end-to-end.

4. Reactive LiDAR bypass (unchanged): right-side odometry detour if an
   unexpected obstacle blocks the path, then return to LINE_FOLLOW.

5. DONE: end-wall detection stops the robot.
"""

import cv2
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
import numpy as np

from line_robot.path_planner import plan_waypoints

# ── Tuning ────────────────────────────────────────────────────────────
SPEED            = 0.16   # m/s line-follow speed
NAV_SPEED        = 0.18   # m/s waypoint-navigation speed
OBS_DIST         = 0.35   # m   front LiDAR obstacle trigger
OBS_CLEAR_DIST   = 0.55   # m   front clearance to leave bypass
FRONT_HALF       = 20     # deg half-cone for obstacle detection

LINE_KP          = 1.25
LINE_MAX_ANG     = 0.90
LINE_MIN_AREA    = 35
LINE_THRESH      = 70
LINE_CENTER_TOL  = 0.15
LINE_STABLE_TICKS = 6
LINE_SEARCH_ANG  = 0.32
REACQUIRE_KP     = 1.35
REACQUIRE_MAX_ANG = 0.95
REACQUIRE_YAW_TOL = 0.14

BYPASS_LIN          = 0.15
BYPASS_ANG          = 0.9
BYPASS_SIDE_OFFSET  = 0.30
BYPASS_PASS_DIST    = 0.62
BYPASS_ANGLE_DEG    = 35.0
RETURN_ANGLE_DEG    = 28.0
YAW_TOL             = 0.06
LINE_SEARCH_TICKS   = 50
POST_BYPASS_IGNORE_TICKS = 45

WAYPOINT_REACH_DIST = 0.35   # m — advance waypoint when this close
WP_MAX_ANG          = 1.0    # rad/s max turn while navigating waypoints
LINE_TRUST_LATERAL  = 0.45   # m — switch to camera when line visible + near path

# FINISH in odom frame: world 6.38 + spawn_offset 3.5 = 9.88
FINISH_X_ODOM    = 9.88
FINISH_STOP_DIST = 0.22

HZ = 20
# ─────────────────────────────────────────────────────────────────────


class Follower(Node):
    NAVIGATE_TO_WP    = "NAVIGATE_TO_WP"
    LINE_FOLLOW       = "LINE_FOLLOW"
    TURN_RIGHT        = "TURN_RIGHT"
    SLANT_AWAY        = "SLANT_AWAY"
    TURN_PARALLEL     = "TURN_PARALLEL"
    DRIVE_PAST        = "DRIVE_PAST"
    TURN_BACK_TO_LINE = "TURN_BACK_TO_LINE"
    RETURN_TO_LINE    = "RETURN_TO_LINE"
    ALIGN_WITH_LINE   = "ALIGN_WITH_LINE"
    DONE              = "DONE"

    def __init__(self):
        super().__init__("follower")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan",             self._scan_cb,  10)
        self.create_subscription(Odometry,  "/odom",             self._odom_cb,  10)
        self.create_subscription(Image,     "/camera/image_raw", self._image_cb, 10)

        # LiDAR
        self._ranges    = np.array([])
        self._angle_min = -math.pi
        self._angle_max =  math.pi
        self._range_min = 0.05
        self._range_max = 5.0
        self.got_scan   = False
        self.front_d    = 9.9

        # Camera
        self.line_visible    = False
        self.line_error      = 0.0
        self.line_area       = 0.0
        self.last_line_error = 0.0

        # Odometry  (odom frame: starts at 0,0 at spawn)
        self.x        = 0.0
        self.y        = 0.0
        self.yaw      = 0.0
        self.got_odom = False

        # Bypass saved pose
        self.detect_x = 0.0
        self.detect_y = 0.0
        self.line_yaw = 0.0

        # ── Dijkstra shortest-path ────────────────────────────────────
        # All waypoints are in the same odom frame as self.x / self.y
        waypoints, cost, route_name = plan_waypoints()
        self.waypoints  = waypoints   # list of (odom_x, odom_y)
        self.wp_index   = 1           # start from index 1: wp[0]=spawn(0,0), skip it
        self.route_name = route_name
        self.plan_cost  = cost

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Dijkstra chose : {route_name}")
        self.get_logger().info(f"  Plan cost      : {cost:.3f}")
        self.get_logger().info(f"  Waypoints ({len(waypoints)}) — odom frame:")
        for i, (wx, wy) in enumerate(waypoints):
            marker = " <-- NEXT" if i == self.wp_index else ""
            self.get_logger().info(f"    [{i}] ({wx:6.2f}, {wy:6.2f}){marker}")
        self.get_logger().info("=" * 60)

        self.state = self.NAVIGATE_TO_WP
        self.tick  = 0
        self.ignore_obstacle_ticks = 0

        self.create_timer(1.0 / HZ, self._step)
        self.get_logger().info("Follower ready — waiting for sensors ...")

    # ── Sensor callbacks ──────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self._angle_min = msg.angle_min
        self._angle_max = msg.angle_max
        self._range_min = msg.range_min
        self._range_max = msg.range_max
        r = np.array(msg.ranges, dtype=np.float32)
        r = np.where(np.isfinite(r), r, self._range_max)
        r = np.clip(r, self._range_min, self._range_max)
        self._ranges = r
        self.got_scan = True
        self.front_d  = self._arc_min(0.0, FRONT_HALF)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.x    = p.x
        self.y    = p.y
        self.yaw  = math.atan2(siny_cosp, cosy_cosp)
        self.got_odom = True

    def _image_cb(self, msg: Image):
        frame = self._image_to_bgr(msg)
        if frame is None:
            self.line_visible = False
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, LINE_THRESH, 255, cv2.THRESH_BINARY_INV)
        h, w = mask.shape
        roi = mask[int(h * 0.10):h, :]
        kernel = np.ones((3, 3), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, kernel)
        moments = cv2.moments(roi)
        area = moments["m00"] / 255.0
        self.line_area = area
        if area < LINE_MIN_AREA:
            self.line_visible = False
            return
        cx = moments["m10"] / moments["m00"]
        self.line_error      = (cx - w * 0.5) / (w * 0.5)
        self.last_line_error = self.line_error
        self.line_visible    = True

    def _image_to_bgr(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            return None
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            enc  = msg.encoding.lower()
            if enc in ("rgb8", "r8g8b8"):
                img = data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if enc == "bgr8":
                return data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
            if enc in ("rgba8", "r8g8b8a8"):
                img = data.reshape((msg.height, msg.step // 4, 4))[:, :msg.width, :]
                return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            if enc == "bgra8":
                img = data.reshape((msg.height, msg.step // 4, 4))[:, :msg.width, :]
                return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            if enc in ("mono8", "8uc1"):
                img = data.reshape((msg.height, msg.step))[:, :msg.width]
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        except ValueError as exc:
            self.get_logger().warn(f"Camera parse error: {exc}",
                                   throttle_duration_sec=2.0)
            return None
        self.get_logger().warn(f"Unsupported encoding: {msg.encoding}",
                               throttle_duration_sec=2.0)
        return None

    # ── Helpers ───────────────────────────────────────────────────────
    def _to_idx(self, deg: float) -> int:
        n    = len(self._ranges)
        span = self._angle_max - self._angle_min
        return int(round((math.radians(deg) - self._angle_min) / span * n)) % n

    def _arc_min(self, center_deg: float, half_deg: float) -> float:
        if len(self._ranges) == 0:
            return self._range_max
        n  = len(self._ranges)
        c  = self._to_idx(center_deg)
        h  = max(1, round(half_deg / 360.0 * n))
        ii = np.array([(c + i) % n for i in range(-h, h + 1)])
        v  = self._ranges[ii]
        ok = v[(v > self._range_min + 0.01) & (v < self._range_max * 0.95)]
        return float(ok.min()) if len(ok) else self._range_max

    def _norm_angle(self, a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _yaw_error(self, target: float) -> float:
        return self._norm_angle(target - self.yaw)

    def _turn_cmd(self, target_yaw: float, max_ang: float = BYPASS_ANG) -> float:
        err = self._yaw_error(target_yaw)
        return max(-max_ang, min(max_ang, 2.2 * err))

    def _local_motion(self):
        dx, dy = self.x - self.detect_x, self.y - self.detect_y
        c, s   = math.cos(self.line_yaw), math.sin(self.line_yaw)
        return dx * c + dy * s, -dx * s + dy * c

    def _dist_to_segment(self, ax, ay, bx, by) -> float:
        dx, dy  = bx - ax, by - ay
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            return math.dist((self.x, self.y), (ax, ay))
        t  = max(0.0, min(1.0, ((self.x-ax)*dx + (self.y-ay)*dy) / seg_len**2))
        return math.dist((self.x, self.y), (ax + t*dx, ay + t*dy))

    def _near_planned_path(self) -> bool:
        """True if robot is within LINE_TRUST_LATERAL of any planned segment."""
        for i in range(len(self.waypoints) - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            if self._dist_to_segment(ax, ay, bx, by) < LINE_TRUST_LATERAL:
                return True
        return False

    def _current_waypoint(self):
        idx = min(self.wp_index, len(self.waypoints) - 1)
        return self.waypoints[idx]

    def _advance_waypoint(self):
        if self.wp_index < len(self.waypoints) - 1:
            self.wp_index += 1
            wx, wy = self._current_waypoint()
            self.get_logger().info(
                f"-> WP {self.wp_index}/{len(self.waypoints)-1} "
                f"odom({wx:.2f}, {wy:.2f})")

    def _line_follow_cmd(self, cmd: Twist):
        cmd.linear.x = SPEED
        if not self.line_visible:
            cmd.linear.x  = SPEED * 0.25
            direction      = -1.0 if self.last_line_error >= 0.0 else 1.0
            cmd.angular.z  = direction * LINE_SEARCH_ANG
            self.get_logger().warn("Line not visible — searching",
                                   throttle_duration_sec=1.0)
            return
        steer = -LINE_KP * self.line_error
        cmd.angular.z = max(-LINE_MAX_ANG, min(LINE_MAX_ANG, steer))

    def _reacquire_line_cmd(self, cmd: Twist):
        cmd.linear.x  = SPEED * 0.55
        steer          = -REACQUIRE_KP * self.line_error
        cmd.angular.z = max(-REACQUIRE_MAX_ANG, min(REACQUIRE_MAX_ANG, steer))

    # ── Main control loop ─────────────────────────────────────────────
    def _step(self):
        if not self.got_scan or not self.got_odom:
            return
        if self.ignore_obstacle_ticks > 0:
            self.ignore_obstacle_ticks -= 1

        cmd = Twist()

        # ── Finish check (odom frame) ─────────────────────────────────
        if self.x >= FINISH_X_ODOM and self.front_d < FINISH_STOP_DIST:
            self.state = self.DONE
            self.get_logger().info("End wall reached — STOP")
            self.pub.publish(cmd)
            return

        # ══════════════════════════════════════════════════════════════
        # NAVIGATE_TO_WP
        #   Pure heading-controller navigation toward planned waypoints.
        #   Transitions to LINE_FOLLOW when the camera sees the line on
        #   the planned corridor.
        # ══════════════════════════════════════════════════════════════
        if self.state == self.NAVIGATE_TO_WP:
            wx, wy = self._current_waypoint()

            # Advance waypoint index if close enough
            dist_to_wp = math.dist((self.x, self.y), (wx, wy))
            if dist_to_wp < WAYPOINT_REACH_DIST:
                self._advance_waypoint()
                wx, wy = self._current_waypoint()

            # Heading toward current waypoint (all in odom frame — correct)
            target_yaw    = math.atan2(wy - self.y, wx - self.x)
            cmd.linear.x  = NAV_SPEED
            cmd.angular.z = self._turn_cmd(target_yaw, WP_MAX_ANG)

            # Switch to camera line follow when line is seen on planned path
            if self.line_visible and self._near_planned_path():
                self.state = self.LINE_FOLLOW
                self.get_logger().info(
                    f"Line detected — switching to camera follow ({self.route_name})")

        # ══════════════════════════════════════════════════════════════
        # LINE_FOLLOW
        # ══════════════════════════════════════════════════════════════
        elif self.state == self.LINE_FOLLOW:
            # Keep waypoint index in sync
            wx, wy = self._current_waypoint()
            if math.dist((self.x, self.y), (wx, wy)) < WAYPOINT_REACH_DIST:
                self._advance_waypoint()

            # Reactive obstacle bypass
            if self.ignore_obstacle_ticks == 0 and self.front_d < OBS_DIST:
                self.detect_x = self.x
                self.detect_y = self.y
                self.line_yaw = self.yaw
                self.state    = self.TURN_RIGHT
                self.tick     = 0
                self.get_logger().warn(
                    f"Unexpected obstacle at {self.front_d:.2f} m — bypass")
                self.pub.publish(cmd)
                return

            self._line_follow_cmd(cmd)

        # ══════════════════════════════════════════════════════════════
        # BYPASS STATES (reactive, right-side odom detour)
        # ══════════════════════════════════════════════════════════════
        elif self.state == self.TURN_RIGHT:
            target = self.line_yaw - math.radians(BYPASS_ANGLE_DEG)
            cmd.angular.z = self._turn_cmd(target)
            if abs(self._yaw_error(target)) < YAW_TOL:
                self.state = self.SLANT_AWAY
                self.get_logger().info("BYPASS: turned right")

        elif self.state == self.SLANT_AWAY:
            cmd.linear.x = BYPASS_LIN
            _, lateral   = self._local_motion()
            if lateral <= -BYPASS_SIDE_OFFSET:
                self.state = self.TURN_PARALLEL
                self.get_logger().info("BYPASS: side offset reached")

        elif self.state == self.TURN_PARALLEL:
            cmd.angular.z = self._turn_cmd(self.line_yaw)
            if abs(self._yaw_error(self.line_yaw)) < YAW_TOL:
                self.state = self.DRIVE_PAST
                self.get_logger().info("BYPASS: driving past obstacle")

        elif self.state == self.DRIVE_PAST:
            cmd.linear.x = SPEED
            forward, _   = self._local_motion()
            if forward >= BYPASS_PASS_DIST and self.front_d > OBS_CLEAR_DIST:
                self.state = self.TURN_BACK_TO_LINE
                self.get_logger().info("BYPASS: obstacle cleared")

        elif self.state == self.TURN_BACK_TO_LINE:
            target = self.line_yaw + math.radians(RETURN_ANGLE_DEG)
            cmd.angular.z = self._turn_cmd(target)
            if abs(self._yaw_error(target)) < YAW_TOL:
                self.state = self.RETURN_TO_LINE
                self.tick  = 0

        elif self.state == self.RETURN_TO_LINE:
            if self.line_visible:
                self.state = self.ALIGN_WITH_LINE
                self.tick  = 0
                self.get_logger().info("BYPASS: line visible, centering")
            else:
                cmd.linear.x  = BYPASS_LIN * 0.75
                cmd.angular.z = LINE_SEARCH_ANG * 0.45
                self.tick += 1
                if self.tick >= LINE_SEARCH_TICKS:
                    self.state = self.ALIGN_WITH_LINE
                    self.tick  = 0

        elif self.state == self.ALIGN_WITH_LINE:
            if self.line_visible:
                centered        = abs(self.line_error) <= LINE_CENTER_TOL
                heading_aligned = abs(self._yaw_error(self.line_yaw)) < REACQUIRE_YAW_TOL
                if not centered:
                    self._reacquire_line_cmd(cmd)
                    self.tick = 0
                elif not heading_aligned:
                    cmd.linear.x  = SPEED * 0.15
                    cmd.angular.z = self._turn_cmd(self.line_yaw, LINE_MAX_ANG)
                    self.tick = 0
                else:
                    cmd.linear.x = SPEED * 0.35
                    self.tick += 1
                if self.tick >= LINE_STABLE_TICKS:
                    self.state = self.LINE_FOLLOW
                    self.tick  = 0
                    self.ignore_obstacle_ticks = POST_BYPASS_IGNORE_TICKS
                    self.get_logger().info("BYPASS done — resuming line follow")
            else:
                self.tick     = 0
                cmd.linear.x  = BYPASS_LIN * 0.35
                cmd.angular.z = LINE_SEARCH_ANG

        elif self.state == self.DONE:
            pass  # zero cmd = stopped

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
