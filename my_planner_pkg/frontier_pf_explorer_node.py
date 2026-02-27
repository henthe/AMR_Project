#!/usr/bin/env python3
import math
import time
from collections import deque
from typing import List, Tuple, Optional, Set

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FrontierPotentialFieldExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_pf_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("min_frontier_cluster_size", 10)

        self.declare_parameter("goal_blacklist_radius_m", 0.25)
        self.declare_parameter("blacklist_max_size", 80)

        self.declare_parameter("min_map_size_x_m", 2.0)
        self.declare_parameter("min_map_size_y_m", 2.0)

        self.declare_parameter("warmup_enable", True)
        self.declare_parameter("warmup_max_duration_s", 45.0)
        self.declare_parameter("warmup_linear_x", 0.08)
        self.declare_parameter("warmup_angular_z", 0.55)

        self.declare_parameter("goal_reached_dist_m", 0.35)
        self.declare_parameter("min_goal_separation_m", 0.60)
        self.declare_parameter("goal_timeout_s", 180.0)

        self.declare_parameter("k_att", 0.85)
        self.declare_parameter("k_rep", 1.15)
        self.declare_parameter("repulsion_range_m", 0.90)
        self.declare_parameter("stop_range_m", 0.22)

        self.declare_parameter("k_heading", 2.3)
        self.declare_parameter("max_lin", 0.60)
        self.declare_parameter("max_ang", 1.8)
        self.declare_parameter("lin_scale_on_heading", 1.4)

        self.declare_parameter("stuck_check_enable", True)
        self.declare_parameter("stuck_grace_s", 5.0)
        self.declare_parameter("stuck_window_s", 10.0)
        self.declare_parameter("stuck_min_path_m", 0.18)

        self.declare_parameter("reselect_goal_every_s", 3.0)

        self.declare_parameter("recovery_enable", True)
        self.declare_parameter("recovery_backup_s", 1.2)
        self.declare_parameter("recovery_turn_s", 1.6)
        self.declare_parameter("recovery_backup_speed", -0.10)
        self.declare_parameter("recovery_turn_speed", 1.3)
        self.declare_parameter("max_recoveries_per_goal", 2)

        self.declare_parameter("heading_lpf_alpha", 0.25)
        self.declare_parameter("ang_lpf_alpha", 0.35)
        self.declare_parameter("min_lin_when_turning", 0.02)

        self.heading_alpha = float(self.get_parameter("heading_lpf_alpha").value)
        self.ang_alpha = float(self.get_parameter("ang_lpf_alpha").value)
        self.min_lin_when_turning = float(self.get_parameter("min_lin_when_turning").value)

        self.filtered_heading = 0.0
        self.filtered_ang = 0.0
        self.have_filtered = False

        self.map_topic = self.get_parameter("map_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.min_cluster_size = int(self.get_parameter("min_frontier_cluster_size").value)

        self.blacklist_radius = float(self.get_parameter("goal_blacklist_radius_m").value)
        self.blacklist_max_size = int(self.get_parameter("blacklist_max_size").value)

        self.min_map_x_m = float(self.get_parameter("min_map_size_x_m").value)
        self.min_map_y_m = float(self.get_parameter("min_map_size_y_m").value)

        self.warmup_enable = bool(self.get_parameter("warmup_enable").value)
        self.warmup_max_duration_s = float(self.get_parameter("warmup_max_duration_s").value)
        self.warmup_linear_x = float(self.get_parameter("warmup_linear_x").value)
        self.warmup_angular_z = float(self.get_parameter("warmup_angular_z").value)

        self.goal_reached_dist = float(self.get_parameter("goal_reached_dist_m").value)
        self.min_goal_sep = float(self.get_parameter("min_goal_separation_m").value)
        self.goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)

        self.k_att = float(self.get_parameter("k_att").value)
        self.k_rep = float(self.get_parameter("k_rep").value)
        self.rep_range = float(self.get_parameter("repulsion_range_m").value)
        self.stop_range = float(self.get_parameter("stop_range_m").value)

        self.k_heading = float(self.get_parameter("k_heading").value)
        self.max_lin = float(self.get_parameter("max_lin").value)
        self.max_ang = float(self.get_parameter("max_ang").value)
        self.lin_scale = float(self.get_parameter("lin_scale_on_heading").value)

        self.stuck_enable = bool(self.get_parameter("stuck_check_enable").value)
        self.stuck_grace_s = float(self.get_parameter("stuck_grace_s").value)
        self.stuck_window_s = float(self.get_parameter("stuck_window_s").value)
        self.stuck_min_path_m = float(self.get_parameter("stuck_min_path_m").value)

        self.reselect_every_s = float(self.get_parameter("reselect_goal_every_s").value)

        self.recovery_enable = bool(self.get_parameter("recovery_enable").value)
        self.recovery_backup_s = float(self.get_parameter("recovery_backup_s").value)
        self.recovery_turn_s = float(self.get_parameter("recovery_turn_s").value)
        self.recovery_backup_speed = float(self.get_parameter("recovery_backup_speed").value)
        self.recovery_turn_speed = float(self.get_parameter("recovery_turn_speed").value)
        self.max_recoveries_per_goal = int(self.get_parameter("max_recoveries_per_goal").value)

        self.declare_parameter("progress_window_s", 8.0)
        self.declare_parameter("progress_min_delta_m", 0.12)
        self.progress_window_s = float(self.get_parameter("progress_window_s").value)
        self.progress_min_delta_m = float(self.get_parameter("progress_min_delta_m").value)
        self.goal_dist_hist = deque(maxlen=400)
        self.ang_cmd_hist: deque = deque(maxlen=200)

        qos_map = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos_map)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_map: Optional[OccupancyGrid] = None
        self.scan: Optional[LaserScan] = None

        self.in_warmup = self.warmup_enable
        self.warmup_start_wall_time = time.time()

        self.current_goal_world: Optional[Tuple[float, float]] = None
        self.goal_start_wall_time = 0.0
        self.blacklisted_goals: List[Tuple[float, float]] = []

        self.pose_hist: deque = deque(maxlen=400)

        self.last_reselect_wall_time = time.time()

        self.mode = "NORMAL"
        self.recovery_start_wall_time = 0.0
        self.recovery_turn_sign = 1.0
        self.recoveries_this_goal = 0

        self.loop_count = 0
        self.status_every_n = 40  # every ~2s at 20Hz

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("Frontier PF Explorer started")

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        if msg.header.frame_id:
            self.global_frame = msg.header.frame_id

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def loop(self) -> None:
        if self.latest_map is None or self.scan is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, yaw = pose

        self.loop_count += 1
        if self.loop_count % self.status_every_n == 0:
            self.log_status(rx, ry)

        if self.in_warmup:
            if self.warmup_done():
                self.in_warmup = False
                self.stop_robot()
                self.get_logger().info("Warmup complete, selecting frontiers")
            else:
                self.publish_warmup_cmd()
            return

        if not self.map_big_enough(self.latest_map):
            self.stop_robot()
            return

        if self.mode != "NORMAL":
            self.run_recovery()
            return

        if (time.time() - self.last_reselect_wall_time) >= self.reselect_every_s:
            self.last_reselect_wall_time = time.time()
            self.current_goal_world = None

        if self.current_goal_world is None:
            self.pick_new_goal(rx, ry)
            if self.current_goal_world is None:
                self.stop_robot()
            return

        gx, gy = self.current_goal_world

        dist_goal = math.hypot(gx - rx, gy - ry)
        self.goal_dist_hist.append((time.time(), dist_goal))
        if dist_goal <= self.goal_reached_dist:
            self.add_blacklist(gx, gy)
            self.current_goal_world = None
            self.recoveries_this_goal = 0
            self.stop_robot()
            return

        if time.time() - self.goal_start_wall_time > self.goal_timeout_s:
            self.add_blacklist(gx, gy)
            self.current_goal_world = None
            self.recoveries_this_goal = 0
            self.stop_robot()
            return

        if self.stuck_enable:
            self.update_pose_hist(rx, ry)
            if (time.time() - self.goal_start_wall_time) >= self.stuck_grace_s and self.is_stuck():
                if self.recovery_enable and self.recoveries_this_goal < self.max_recoveries_per_goal:
                    self.start_recovery()
                    return
                self.add_blacklist(gx, gy)
                self.current_goal_world = None
                self.recoveries_this_goal = 0
                self.stop_robot()
                return

        cmd = self.potential_field_cmd(rx, ry, yaw, gx, gy, self.scan)

        # Oscillation detection: rapidly alternating angular commands = corner wiggling
        self.ang_cmd_hist.append((time.time(), cmd.angular.z))
        if self.is_oscillating():
            self.get_logger().warn("Oscillation detected, triggering recovery")
            self.ang_cmd_hist.clear()
            self.have_filtered = False
            if self.recovery_enable and self.recoveries_this_goal < self.max_recoveries_per_goal:
                self.start_recovery()
                return
            self.add_blacklist(gx, gy)
            self.current_goal_world = None
            self.recoveries_this_goal = 0
            self.stop_robot()
            return

        self.cmd_pub.publish(cmd)

    def warmup_done(self) -> bool:
        elapsed = time.time() - self.warmup_start_wall_time
        if elapsed >= self.warmup_max_duration_s:
            return True
        if self.latest_map is None:
            return False
        return self.map_big_enough(self.latest_map)

    def publish_warmup_cmd(self) -> None:
        elapsed = time.time() - self.warmup_start_wall_time
        cmd = Twist()
        if elapsed < 2.0:
            cmd.linear.x = -0.06
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = float(self.warmup_linear_x)
            cmd.angular.z = float(self.warmup_angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def log_status(self, rx: float, ry: float) -> None:
        if self.in_warmup:
            elapsed = time.time() - self.warmup_start_wall_time
            self.get_logger().info(f"[WARMUP] elapsed={elapsed:.1f}s pos=({rx:.2f},{ry:.2f})")
            return
        if self.mode != "NORMAL":
            self.get_logger().info(
                f"[{self.mode}] attempt={self.recoveries_this_goal} pos=({rx:.2f},{ry:.2f})")
            return
        if self.current_goal_world is None:
            self.get_logger().info(f"[SEARCHING] pos=({rx:.2f},{ry:.2f}) blacklisted={len(self.blacklisted_goals)}")
            return
        gx, gy = self.current_goal_world
        dist = math.hypot(gx - rx, gy - ry)
        self.get_logger().info(
            f"[DRIVING] goal=({gx:.2f},{gy:.2f}) dist={dist:.2f}m pos=({rx:.2f},{ry:.2f})"
            f" recoveries={self.recoveries_this_goal}")

    def map_big_enough(self, grid: OccupancyGrid) -> bool:
        sx = float(grid.info.width) * float(grid.info.resolution)
        sy = float(grid.info.height) * float(grid.info.resolution)
        return (sx >= self.min_map_x_m) and (sy >= self.min_map_y_m)

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tfm = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        x = tfm.transform.translation.x
        y = tfm.transform.translation.y
        q = tfm.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, y, yaw)

    def add_blacklist(self, gx: float, gy: float) -> None:
        self.blacklisted_goals.append((gx, gy))
        if len(self.blacklisted_goals) > self.blacklist_max_size:
            self.blacklisted_goals.pop(0)

    def blacklist_effective_radius(self) -> float:
        grid = self.latest_map
        r = self.blacklist_radius
        if grid is not None:
            sx = float(grid.info.width) * float(grid.info.resolution)
            sy = float(grid.info.height) * float(grid.info.resolution)
            map_scale = max(1.0, min(sx, sy))
            r = min(r, 0.15 * map_scale)
            r = max(0.10, r)
        return r

    def is_blacklisted_world(self, wx: float, wy: float) -> bool:
        r = self.blacklist_effective_radius()
        for bx, by in self.blacklisted_goals:
            if math.hypot(wx - bx, wy - by) <= r:
                return True
        return False

    def pick_new_goal(self, rx: float, ry: float) -> None:
        grid = self.latest_map
        if grid is None:
            return

        frontiers = self.find_frontier_cells(grid)
        if not frontiers:
            return

        clusters = self.cluster_frontiers(frontiers, grid.info.width, grid.info.height)
        clusters = [c for c in clusters if len(c) >= self.min_cluster_size]
        if not clusters:
            return

        best = None
        best_score = -1e18

        for cluster in clusters:
            wx, wy = self.pick_cluster_point_farthest_from_robot(cluster, rx, ry, grid)

            if math.hypot(wx - rx, wy - ry) < self.min_goal_sep:
                continue
            if self.is_blacklisted_world(wx, wy):
                continue

            dist = math.hypot(wx - rx, wy - ry)
            size = float(len(cluster))
            score = 2.0 * size + 0.6 * dist

            if score > best_score:
                best_score = score
                best = (wx, wy)

        if best is None:
            self.blacklisted_goals.clear()
            for cluster in clusters:
                wx, wy = self.pick_cluster_point_farthest_from_robot(cluster, rx, ry, grid)
                if math.hypot(wx - rx, wy - ry) < self.min_goal_sep:
                    continue
                dist = math.hypot(wx - rx, wy - ry)
                size = float(len(cluster))
                score = 2.0 * size + 0.6 * dist
                if score > best_score:
                    best_score = score
                    best = (wx, wy)
            if best is None:
                return

        self.current_goal_world = best
        self.goal_start_wall_time = time.time()
        self.pose_hist.clear()
        self.pose_hist.append((time.time(), rx, ry))
        self.recoveries_this_goal = 0

    def pick_cluster_point_farthest_from_robot(
        self,
        cluster: List[Tuple[int, int]],
        rx: float,
        ry: float,
        grid: OccupancyGrid
    ) -> Tuple[float, float]:
        best = None
        best_d = -1.0
        step = max(1, len(cluster) // 40)
        for i in range(0, len(cluster), step):
            cx, cy = cluster[i]
            wx, wy = self.cell_to_world(cx, cy, grid)
            d = math.hypot(wx - rx, wy - ry)
            if d > best_d:
                best_d = d
                best = (wx, wy)
        if best is None:
            cx, cy = cluster[len(cluster) // 2]
            best = self.cell_to_world(cx, cy, grid)
        return best

    def find_frontier_cells(self, grid: OccupancyGrid) -> List[Tuple[int, int]]:
        w = grid.info.width
        h = grid.info.height
        data = grid.data

        def idx(x: int, y: int) -> int:
            return y * w + x

        def is_free(v: int) -> bool:
            if v < 0:
                return False
            return v <= self.free_threshold

        def is_unknown(v: int) -> bool:
            return v < 0

        frontiers: List[Tuple[int, int]] = []
        for y in range(h):
            for x in range(w):
                v = data[idx(x, y)]
                if not is_free(v):
                    continue
                if self.has_unknown_neighbor(x, y, w, h, data, idx, is_unknown):
                    frontiers.append((x, y))
        return frontiers

    def has_unknown_neighbor(self, x: int, y: int, w: int, h: int, data: List[int], idx_fn, is_unknown_fn) -> bool:
        for ny in (y - 1, y, y + 1):
            for nx in (x - 1, x, x + 1):
                if nx == x and ny == y:
                    continue
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if is_unknown_fn(data[idx_fn(nx, ny)]):
                    return True
        return False

    def cluster_frontiers(self, frontier_cells: List[Tuple[int, int]], w: int, h: int) -> List[List[Tuple[int, int]]]:
        frontier_set: Set[Tuple[int, int]] = set(frontier_cells)
        visited: Set[Tuple[int, int]] = set()
        clusters: List[List[Tuple[int, int]]] = []

        for cell in frontier_cells:
            if cell in visited:
                continue
            q = deque([cell])
            visited.add(cell)
            cluster: List[Tuple[int, int]] = []

            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for ny in (cy - 1, cy, cy + 1):
                    for nx in (cx - 1, cx, cx + 1):
                        if nx == cx and ny == cy:
                            continue
                        if nx < 0 or nx >= w or ny < 0 or ny >= h:
                            continue
                        nb = (nx, ny)
                        if nb in visited:
                            continue
                        if nb in frontier_set:
                            visited.add(nb)
                            q.append(nb)

            clusters.append(cluster)

        return clusters

    def cell_to_world(self, cx: int, cy: int, grid: OccupancyGrid) -> Tuple[float, float]:
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        return ox + (cx + 0.5) * res, oy + (cy + 0.5) * res

    def potential_field_cmd(self, rx: float, ry: float, yaw: float, gx: float, gy: float, scan: LaserScan) -> Twist:
        dx_w = gx - rx
        dy_w = gy - ry

        c = math.cos(-yaw)
        s = math.sin(-yaw)
        dx_r = c * dx_w - s * dy_w
        dy_r = s * dx_w + c * dy_w

        F_att = np.array([self.k_att * dx_r, self.k_att * dy_r], dtype=np.float32)

        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        front_cone = np.abs(angles) < math.radians(35.0)
        front_blocked = bool(np.any(ranges[front_cone] < self.stop_range))

        F_rep = np.zeros(2, dtype=np.float32)
        in_range = ranges < self.rep_range
        rr = ranges[in_range]
        aa = angles[in_range]

        if rr.size > 0:
            ox = rr * np.cos(aa)
            oy = rr * np.sin(aa)

            inv_r = 1.0 / np.maximum(rr, 1e-3)
            mag = self.k_rep * (inv_r - 1.0 / self.rep_range) * (inv_r ** 2)

            dirx = -ox / np.maximum(rr, 1e-3)
            diry = -oy / np.maximum(rr, 1e-3)

            fx = mag * dirx
            fy = mag * diry

            F_rep[0] = float(np.clip(np.sum(fx), -6.0, 6.0))
            F_rep[1] = float(np.clip(np.sum(fy), -6.0, 6.0))

        # Tangential force: when repulsion opposes attraction (corner/dead-end),
        # add a perpendicular component so the robot slides along the wall.
        F_rep_mag = float(np.linalg.norm(F_rep))
        F_att_mag = float(np.linalg.norm(F_att))
        if F_rep_mag > 0.3 and F_att_mag > 0.01:
            cos_angle = float(np.dot(F_att, F_rep)) / (F_att_mag * F_rep_mag)
            if cos_angle < -0.2:  # forces opposing (angle > ~102°)
                tangent = np.array([-F_rep[1], F_rep[0]], dtype=np.float32)
                if np.dot(tangent, F_att) < 0:
                    tangent = -tangent
                t_norm = float(np.linalg.norm(tangent))
                if t_norm > 1e-6:
                    tangent = tangent / t_norm
                    blend = min(1.0, (-cos_angle - 0.2) / 0.6)
                    F_rep = F_rep + blend * 0.5 * F_rep_mag * tangent

        F = F_att + F_rep

        desired_heading = math.atan2(F[1], F[0])

        if not self.have_filtered:
            self.filtered_heading = desired_heading
            self.filtered_ang = 0.0
            self.have_filtered = True
        else:
            err = wrap_angle(desired_heading - self.filtered_heading)
            self.filtered_heading = wrap_angle(self.filtered_heading + self.heading_alpha * err)

        heading_err = wrap_angle(self.filtered_heading)

        raw_ang = clamp(self.k_heading * heading_err, -self.max_ang, self.max_ang)
        self.filtered_ang = (1.0 - self.ang_alpha) * self.filtered_ang + self.ang_alpha * raw_ang
        ang = float(self.filtered_ang)

        heading_factor = max(0.0, math.cos(self.lin_scale * heading_err))
        lin = clamp(0.7 * heading_factor * self.max_lin, 0.0, self.max_lin)

        # Wenn er stark drehen muss, gib eine minimale Vorwärtsfahrt, sonst zittert er auf der Stelle
        if abs(heading_err) > math.radians(50.0):
            lin = max(lin, self.min_lin_when_turning)

        # Gradually slow down when obstacles are close in the front hemisphere
        front_hemi = np.abs(angles) < math.radians(90.0)
        front_ranges = ranges[front_hemi]
        if front_ranges.size > 0:
            min_front = float(np.min(front_ranges))
            if min_front < self.rep_range:
                prox = max(0.0, (min_front - self.stop_range) / (self.rep_range - self.stop_range))
                lin *= prox

        # Front blocked: stop forward motion but keep turning to escape
        if front_blocked:
            lin = 0.0

        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        return cmd

    def update_pose_hist(self, rx: float, ry: float) -> None:
        self.pose_hist.append((time.time(), rx, ry))

    def is_stuck(self) -> bool:
        if len(self.pose_hist) < 6:
            return False
        now = time.time()
        pts = [p for p in self.pose_hist if (now - p[0]) <= self.stuck_window_s]
        if len(pts) < 6:
            return False
        path = 0.0
        for i in range(1, len(pts)):
            path += math.hypot(pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2])
        return path < self.stuck_min_path_m

    def no_progress_toward_goal(self) -> bool:
        now = time.time()
        pts = [p for p in self.goal_dist_hist if (now - p[0]) <= self.progress_window_s]
        if len(pts) < 6:
            return False
        d0 = pts[0][1]
        d1 = pts[-1][1]
        return (d0 - d1) < self.progress_min_delta_m

    def is_oscillating(self) -> bool:
        now = time.time()
        pts = [(t, v) for t, v in self.ang_cmd_hist if (now - t) <= 2.5]
        if len(pts) < 15:
            return False
        sign_changes = 0
        for i in range(1, len(pts)):
            if (pts[i][1] * pts[i - 1][1] < 0
                    and abs(pts[i][1]) > 0.05
                    and abs(pts[i - 1][1]) > 0.05):
                sign_changes += 1
        return sign_changes >= 5

    def start_recovery(self) -> None:
        self.recoveries_this_goal += 1
        self.recovery_start_wall_time = time.time()
        self.mode = "RECOVERY_BACKUP"
        self.recovery_turn_sign = self.choose_turn_direction_from_scan()
        self.ang_cmd_hist.clear()
        self.have_filtered = False
        self.get_logger().warn(f"Recovery started, attempt {self.recoveries_this_goal}, turn_sign {self.recovery_turn_sign}")

    def choose_turn_direction_from_scan(self) -> float:
        scan = self.scan
        if scan is None:
            return 1.0
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]
        left = ranges[(angles > 0.6) & (angles < 1.2)]
        right = ranges[(angles < -0.6) & (angles > -1.2)]
        left_score = float(np.nanmean(left)) if left.size > 0 else 0.0
        right_score = float(np.nanmean(right)) if right.size > 0 else 0.0
        return 1.0 if left_score >= right_score else -1.0

    def run_recovery(self) -> None:
        elapsed = time.time() - self.recovery_start_wall_time

        if self.mode == "RECOVERY_BACKUP":
            cmd = Twist()
            cmd.linear.x = float(self.recovery_backup_speed)
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            if elapsed >= self.recovery_backup_s:
                self.mode = "RECOVERY_TURN"
                self.recovery_start_wall_time = time.time()
            return

        if self.mode == "RECOVERY_TURN":
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = float(self.recovery_turn_sign * self.recovery_turn_speed)
            self.cmd_pub.publish(cmd)
            if elapsed >= self.recovery_turn_s:
                self.mode = "NORMAL"
                self.current_goal_world = None
                self.last_reselect_wall_time = time.time()
            return


def main() -> None:
    rclpy.init()
    node = FrontierPotentialFieldExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()