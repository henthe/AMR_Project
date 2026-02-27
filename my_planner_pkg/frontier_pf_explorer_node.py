#!/usr/bin/env python3
"""
Frontier-based exploration with potential field navigation.

Subscribes to the SLAM map (/map), detects frontier cells (free cells adjacent
to unknown space), clusters them, selects the best frontier goal, plans an A*
path, and navigates using an attractive/repulsive potential field controller.

Builds on Assignment 1 (planner_pf_node.py) — reuses A*, waypoint extraction,
and the potential field concept.
"""

import math
import random
from collections import deque
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion

# Reuse planning utilities from Assignment 1
from my_planner_pkg.planner_pf_node import (
    astar,
    extract_waypoints,
    clamp,
    wrap_angle,
)


# ------------------------------------------------------------------ #
# Coordinate transforms for OccupancyGrid (no Y-flip, unlike PGM)
# ------------------------------------------------------------------ #
def occ_world_to_grid(
    x: float, y: float, origin_x: float, origin_y: float, resolution: float
) -> Tuple[int, int]:
    col = int(math.floor((x - origin_x) / resolution))
    row = int(math.floor((y - origin_y) / resolution))
    return (row, col)


def occ_grid_to_world(
    row: int, col: int, origin_x: float, origin_y: float, resolution: float
) -> Tuple[float, float]:
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return (x, y)


# ------------------------------------------------------------------ #
# State constants
# ------------------------------------------------------------------ #
WARMUP = 0
FIND_FRONTIER = 1
NAVIGATE = 2
RECOVERY = 3
DONE = 4

_STATE_NAMES = {
    WARMUP: "WARMUP",
    FIND_FRONTIER: "FIND_FRONTIER",
    NAVIGATE: "NAVIGATE",
    RECOVERY: "RECOVERY",
    DONE: "DONE",
}


class FrontierPotentialFieldExplorer(Node):
    def __init__(self):
        super().__init__("frontier_pf_explorer")

        # ----- Parameters -----
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("inflation_radius_m", 0.50)
        self.declare_parameter("k_att", 1.0)
        self.declare_parameter("k_rep", 1.5)
        self.declare_parameter("repulsion_range_m", 0.7)
        self.declare_parameter("stop_range_m", 0.30)

        self.declare_parameter("k_heading", 2.5)
        self.declare_parameter("max_lin", 0.5)
        self.declare_parameter("max_ang", 1.5)

        self.declare_parameter("goal_reached_dist_m", 0.35)
        self.declare_parameter("min_frontier_cluster_size", 5)

        self.declare_parameter("reselect_goal_every_s", 5.0)
        self.declare_parameter("warmup_duration_s", 5.0)
        self.declare_parameter("warmup_angular_speed", 0.8)

        self.declare_parameter("waypoint_every_n_cells", 20)
        self.declare_parameter("stuck_window_s", 5.0)
        self.declare_parameter("stuck_threshold_m", 0.20)
        self.declare_parameter("goal_blacklist_radius_m", 0.5)

        self.declare_parameter("recovery_back_duration_s", 1.0)
        self.declare_parameter("recovery_back_speed", -0.15)
        self.declare_parameter("random_walk_duration_s", 5.0)
        self.declare_parameter("random_walk_lin", 0.25)
        self.declare_parameter("random_walk_ang", 1.5)

        # ----- State -----
        self.state = WARMUP
        self.warmup_start: Optional[float] = None

        # Map from SLAM
        self.map_grid: Optional[np.ndarray] = None  # (H, W), int16
        self.map_resolution: float = 0.05
        self.map_origin_x: float = 0.0
        self.map_origin_y: float = 0.0
        self.map_width: int = 0
        self.map_height: int = 0

        # Scan
        self.scan: Optional[LaserScan] = None

        # Navigation
        self.current_goal: Optional[Tuple[float, float]] = None
        self.waypoints_world: List[Tuple[float, float]] = []
        self.last_path_cells: list = []
        self.wp_index: int = 0
        self.last_goal_select_time: float = 0.0

        # Blacklist
        self.blacklisted_goals: List[Tuple[float, float]] = []

        # Stuck detection
        self.pose_history: List[Tuple[float, float, float]] = []  # (x, y, stamp)
        self.navigate_start_time: float = 0.0

        # Front-blocked tracking (corner detection)
        self.front_blocked_since: Optional[float] = None
        self.front_clear_since: Optional[float] = None  # hysteresis
        self.last_status_log: float = 0.0

        # Recovery / random walk
        self.recovery_start: Optional[float] = None
        self.recovery_phase: int = 0  # 0=back, 1=turn, 2=drive
        self.random_walk_start: Optional[float] = None
        self.random_walk_turn_dir: float = 1.0
        self.random_walk_phase_dur: float = 0.0

        # ----- TF -----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----- Subscriptions -----
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, map_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

        # ----- Publishers -----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/explorer_markers", 10)

        # ----- Timer: 20 Hz control loop -----
        self.timer = self.create_timer(0.05, self.control_loop)

        print("FrontierPotentialFieldExplorer started.")

    # ================================================================ #
    #  Callbacks
    # ================================================================ #
    def on_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = w
        self.map_height = h
        # OccupancyGrid data: -1 unknown, 0 free, 1-100 occupied
        self.map_grid = np.array(msg.data, dtype=np.int16).reshape((h, w))

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    # ================================================================ #
    #  Robot pose
    # ================================================================ #
    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tfm = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        x = tfm.transform.translation.x
        y = tfm.transform.translation.y
        q = tfm.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, y, yaw)

    # ================================================================ #
    #  Frontier detection
    # ================================================================ #
    def find_frontiers(self) -> List[List[Tuple[int, int]]]:
        """Return list of frontier clusters (each cluster = list of (row,col))."""
        grid = self.map_grid
        if grid is None:
            return []
        H, W = grid.shape

        free_mask = grid == 0
        unknown_mask = grid == -1

        # Check 8-connected neighbors for unknown cells
        adjacent_to_unknown = np.zeros((H, W), dtype=bool)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                # Shift unknown_mask by (-dr, -dc) so that
                # adjacent_to_unknown[r,c] is True if (r+dr, c+dc) is unknown
                r_start = max(0, dr)
                r_end = H + min(0, dr)
                c_start = max(0, dc)
                c_end = W + min(0, dc)

                src_r_start = max(0, -dr)
                src_r_end = H + min(0, -dr)  # = H - max(0, dr)
                src_c_start = max(0, -dc)
                src_c_end = W + min(0, -dc)  # = W - max(0, dc)

                adjacent_to_unknown[r_start:r_end, c_start:c_end] |= unknown_mask[
                    src_r_start:src_r_end, src_c_start:src_c_end
                ]

        frontier_mask = free_mask & adjacent_to_unknown

        # Collect frontier coordinates
        frontier_rows, frontier_cols = np.where(frontier_mask)
        if frontier_rows.size == 0:
            return []

        frontier_set = set(zip(frontier_rows.tolist(), frontier_cols.tolist()))

        # BFS clustering
        min_size = int(self.get_parameter("min_frontier_cluster_size").value)
        visited = set()
        clusters: List[List[Tuple[int, int]]] = []

        for cell in frontier_set:
            if cell in visited:
                continue
            cluster: List[Tuple[int, int]] = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cr, cc = queue.popleft()
                cluster.append((cr, cc))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nb = (cr + dr, cc + dc)
                        if nb in frontier_set and nb not in visited:
                            visited.add(nb)
                            queue.append(nb)
            if len(cluster) >= min_size:
                clusters.append(cluster)

        return clusters

    # ================================================================ #
    #  Frontier goal selection
    # ================================================================ #
    def select_frontier_goal(
        self, clusters: List[List[Tuple[int, int]]], robot_x: float, robot_y: float
    ) -> Optional[Tuple[float, float]]:
        """Pick the closest non-blacklisted frontier cluster centroid."""
        bl_radius = float(self.get_parameter("goal_blacklist_radius_m").value)

        best_dist = float("inf")
        best_goal: Optional[Tuple[float, float]] = None

        for cluster in clusters:
            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]
            cr = int(np.mean(rows))
            cc = int(np.mean(cols))
            gx, gy = occ_grid_to_world(
                cr, cc, self.map_origin_x, self.map_origin_y, self.map_resolution
            )

            # Skip blacklisted
            blacklisted = False
            for bx, by in self.blacklisted_goals:
                if math.hypot(gx - bx, gy - by) < bl_radius:
                    blacklisted = True
                    break
            if blacklisted:
                continue

            dist = math.hypot(gx - robot_x, gy - robot_y)
            if dist < best_dist:
                best_dist = dist
                best_goal = (gx, gy)

        return best_goal

    # ================================================================ #
    #  Planning
    # ================================================================ #
    def build_planning_grid(self) -> Optional[np.ndarray]:
        """Build inflated binary grid from SLAM map. 0=free, 100=obstacle."""
        if self.map_grid is None:
            return None

        grid = np.copy(self.map_grid)
        # Unknown → FREE for exploration (robot must drive into unknown space;
        # safety is handled by the LiDAR-based potential field, not the planner)
        grid[grid == -1] = 0
        # Any positive value → obstacle
        grid[grid > 0] = 100
        # Free stays 0

        inflation_m = float(self.get_parameter("inflation_radius_m").value)
        inflation_cells = int(math.ceil(inflation_m / self.map_resolution))

        if inflation_cells <= 0:
            return grid.astype(np.uint8)

        obstacle = (grid == 100).astype(np.uint8)
        try:
            from scipy.ndimage import binary_dilation

            size = 2 * inflation_cells + 1
            structure = np.ones((size, size), dtype=bool)
            inflated = binary_dilation(obstacle.astype(bool), structure=structure)
            out = np.zeros_like(grid, dtype=np.uint8)
            out[inflated] = 100
        except ImportError:
            inflated = np.copy(obstacle)
            H, W = grid.shape
            ys, xs = np.where(obstacle == 1)
            for y_i, x_i in zip(ys, xs):
                r0 = max(0, y_i - inflation_cells)
                r1 = min(H, y_i + inflation_cells + 1)
                c0 = max(0, x_i - inflation_cells)
                c1 = min(W, x_i + inflation_cells + 1)
                inflated[r0:r1, c0:c1] = 1
            out = np.zeros_like(grid, dtype=np.uint8)
            out[inflated == 1] = 100

        return out

    def plan_to_goal(self, goal_x: float, goal_y: float) -> bool:
        """Plan A* path to goal. Returns True on success."""
        pose = self.get_robot_pose()
        if pose is None:
            return False

        planning_grid = self.build_planning_grid()
        if planning_grid is None:
            return False

        rx, ry, _ = pose
        start = occ_world_to_grid(
            rx, ry, self.map_origin_x, self.map_origin_y, self.map_resolution
        )
        goal = occ_world_to_grid(
            goal_x, goal_y, self.map_origin_x, self.map_origin_y, self.map_resolution
        )

        H, W = planning_grid.shape

        # Clamp to grid bounds
        start = (clamp(start[0], 0, H - 1), clamp(start[1], 0, W - 1))
        goal = (clamp(goal[0], 0, H - 1), clamp(goal[1], 0, W - 1))

        # Make sure start is int
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))

        # If start or goal is in obstacle, try to find nearest free cell
        start = self._nearest_free(planning_grid, start)
        goal = self._nearest_free(planning_grid, goal)
        if start is None or goal is None:
            return False

        path = astar(planning_grid, start, goal, allow_diagonal=True)
        if path is None:
            return False

        every_n = int(self.get_parameter("waypoint_every_n_cells").value)
        wp_cells = extract_waypoints(path, take_every_n=every_n)

        self.waypoints_world = [
            occ_grid_to_world(
                r, c, self.map_origin_x, self.map_origin_y, self.map_resolution
            )
            for (r, c) in wp_cells
        ]
        self.wp_index = 0
        self.last_path_cells = list(path)

        self.publish_markers()
        return True

    def publish_markers(self):
        """Publish A* path + waypoints + goal marker for RViz."""
        ma = MarkerArray()
        frame = self.get_parameter("map_frame").value
        stamp = self.get_clock().now().to_msg()

        # --- A* path (green line) ---
        path_m = Marker()
        path_m.header.frame_id = frame
        path_m.header.stamp = stamp
        path_m.ns = "explorer_path"
        path_m.id = 0
        path_m.type = Marker.LINE_STRIP
        path_m.action = Marker.ADD
        path_m.pose.orientation.w = 1.0
        path_m.scale.x = 0.03
        path_m.color.g = 1.0
        path_m.color.a = 1.0
        for (r, c) in self.last_path_cells:
            x, y = occ_grid_to_world(
                r, c, self.map_origin_x, self.map_origin_y, self.map_resolution
            )
            path_m.points.append(Point(x=x, y=y, z=0.0))
        ma.markers.append(path_m)

        # --- Waypoints (red spheres) ---
        wps_m = Marker()
        wps_m.header.frame_id = frame
        wps_m.header.stamp = stamp
        wps_m.ns = "explorer_waypoints"
        wps_m.id = 1
        wps_m.type = Marker.SPHERE_LIST
        wps_m.action = Marker.ADD
        wps_m.pose.orientation.w = 1.0
        wps_m.scale.x = 0.10
        wps_m.scale.y = 0.10
        wps_m.scale.z = 0.10
        wps_m.color.r = 1.0
        wps_m.color.g = 0.2
        wps_m.color.b = 0.2
        wps_m.color.a = 1.0
        for (x, y) in self.waypoints_world:
            wps_m.points.append(Point(x=x, y=y, z=0.0))
        ma.markers.append(wps_m)

        # --- Goal (blue sphere) ---
        if self.current_goal is not None:
            goal_m = Marker()
            goal_m.header.frame_id = frame
            goal_m.header.stamp = stamp
            goal_m.ns = "explorer_goal"
            goal_m.id = 2
            goal_m.type = Marker.SPHERE
            goal_m.action = Marker.ADD
            goal_m.scale.x = 0.20
            goal_m.scale.y = 0.20
            goal_m.scale.z = 0.20
            goal_m.color.b = 1.0
            goal_m.color.a = 1.0
            goal_m.pose.position.x = self.current_goal[0]
            goal_m.pose.position.y = self.current_goal[1]
            goal_m.pose.orientation.w = 1.0
            ma.markers.append(goal_m)

        self.marker_pub.publish(ma)

    def _nearest_free(
        self, grid: np.ndarray, cell: Tuple[int, int], max_radius: int = 20
    ) -> Optional[Tuple[int, int]]:
        """Find nearest free cell to 'cell' within max_radius. BFS spiral."""
        H, W = grid.shape
        r, c = cell
        if 0 <= r < H and 0 <= c < W and grid[r, c] == 0:
            return cell

        visited = set()
        queue = deque([cell])
        visited.add(cell)
        while queue:
            cr, cc = queue.popleft()
            if abs(cr - r) > max_radius or abs(cc - c) > max_radius:
                continue
            if 0 <= cr < H and 0 <= cc < W and grid[cr, cc] == 0:
                return (cr, cc)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nb = (cr + dr, cc + dc)
                    if nb not in visited and 0 <= nb[0] < H and 0 <= nb[1] < W:
                        visited.add(nb)
                        queue.append(nb)
        return None

    # ================================================================ #
    #  Stuck detection
    # ================================================================ #
    def check_stuck(self, x: float, y: float) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9

        # Grace period: don't check stuck for the first stuck_window seconds
        # after entering NAVIGATE (robot needs time to rotate toward waypoint)
        window = float(self.get_parameter("stuck_window_s").value)
        if now - self.navigate_start_time < window:
            return False

        self.pose_history.append((x, y, now))
        threshold = float(self.get_parameter("stuck_threshold_m").value)

        # Trim old entries
        self.pose_history = [(px, py, t) for (px, py, t) in self.pose_history if now - t <= window]

        if len(self.pose_history) < 2:
            return False
        # Need the full window of data before declaring stuck
        if now - self.pose_history[0][2] < window * 0.8:
            return False

        total_dist = 0.0
        for i in range(1, len(self.pose_history)):
            dx = self.pose_history[i][0] - self.pose_history[i - 1][0]
            dy = self.pose_history[i][1] - self.pose_history[i - 1][1]
            total_dist += math.hypot(dx, dy)

        # Net displacement: straight-line from start to end of window.
        # Catches oscillation/wobbling where total_dist accumulates
        # but the robot makes no actual progress.
        net_dist = math.hypot(
            self.pose_history[-1][0] - self.pose_history[0][0],
            self.pose_history[-1][1] - self.pose_history[0][1],
        )

        return total_dist < threshold or net_dist < threshold

    # ================================================================ #
    #  State transitions
    # ================================================================ #
    def set_state(self, new_state: int, reason: str = ""):
        if new_state != self.state:
            msg = (
                f"[{_STATE_NAMES.get(self.state, '?')} -> "
                f"{_STATE_NAMES.get(new_state, '?')}]"
            )
            if reason:
                msg += f" {reason}"
            print(msg)
            self.state = new_state

    # ================================================================ #
    #  Main control loop (20 Hz)
    # ================================================================ #
    def control_loop(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9

        # Periodic status log + marker republish (every ~5s)
        if now_s - self.last_status_log > 5.0:
            self.last_status_log = now_s
            pose = self.get_robot_pose()
            pos_str = f"({pose[0]:.2f}, {pose[1]:.2f})" if pose else "unknown"
            goal_str = (
                f"({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})"
                if self.current_goal else "none"
            )
            print(
                f"Status: {_STATE_NAMES.get(self.state, '?')} | "
                f"pos={pos_str} goal={goal_str} | "
                f"wp={self.wp_index}/{len(self.waypoints_world)} "
                f"bl={len(self.blacklisted_goals)}"
            )
            if self.last_path_cells:
                self.publish_markers()

        # ---- WARMUP ----
        if self.state == WARMUP:
            if self.warmup_start is None:
                self.warmup_start = now_s
            elapsed = now_s - self.warmup_start
            warmup_dur = float(self.get_parameter("warmup_duration_s").value)

            if elapsed < warmup_dur:
                cmd = Twist()
                cmd.angular.z = float(self.get_parameter("warmup_angular_speed").value)
                self.cmd_pub.publish(cmd)
                return
            else:
                self.cmd_pub.publish(Twist())
                if self.map_grid is not None:
                    self.set_state(FIND_FRONTIER)
                return

        # ---- DONE ----
        if self.state == DONE:
            self.cmd_pub.publish(Twist())
            return

        # ---- RECOVERY ----
        if self.state == RECOVERY:
            self._do_recovery(now_s)
            return

        # ---- FIND_FRONTIER ----
        if self.state == FIND_FRONTIER:
            self._do_find_frontier(now_s)
            return

        # ---- NAVIGATE ----
        if self.state == NAVIGATE:
            self._do_navigate(now_s)
            return

    # ================================================================ #
    #  FIND_FRONTIER logic
    # ================================================================ #
    def _do_find_frontier(self, now_s: float):
        if self.map_grid is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry, _ = pose
        clusters = self.find_frontiers()

        if not clusters:
            if self.blacklisted_goals:
                self.blacklisted_goals.clear()
                clusters = self.find_frontiers()

        if not clusters:
            self.set_state(DONE, "no frontiers remain")
            return

        goal = self.select_frontier_goal(clusters, rx, ry)
        if goal is None:
            self.blacklisted_goals.clear()
            goal = self.select_frontier_goal(clusters, rx, ry)

        if goal is None:
            self.set_state(DONE, "no reachable frontier")
            return

        # Check if this is the same goal as before (periodic reselection picked
        # the same frontier).  In that case, keep stuck-detection state so the
        # robot can still be declared stuck across reselections.
        same_goal = (
            self.current_goal is not None
            and math.hypot(goal[0] - self.current_goal[0],
                           goal[1] - self.current_goal[1]) < 0.5
        )

        self.current_goal = goal

        if self.plan_to_goal(goal[0], goal[1]):
            self.last_goal_select_time = now_s
            if not same_goal:
                # Genuinely new goal → reset stuck detection
                self.pose_history.clear()
                self.navigate_start_time = now_s
            self.set_state(
                NAVIGATE,
                f"goal=({goal[0]:.2f},{goal[1]:.2f}) "
                f"{len(self.waypoints_world)}wp"
            )
        else:
            self.blacklisted_goals.append(goal)

    # ================================================================ #
    #  NAVIGATE logic
    # ================================================================ #
    def _do_navigate(self, now_s: float):
        if self.scan is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose

        # Periodic frontier reselection
        reselect_s = float(self.get_parameter("reselect_goal_every_s").value)
        if now_s - self.last_goal_select_time > reselect_s:
            self.set_state(FIND_FRONTIER, "periodic reselection")
            return

        # Check if goal reached (or close enough)
        goal_dist = float(self.get_parameter("goal_reached_dist_m").value)
        if self.current_goal is not None:
            dist_to_goal = math.hypot(
                self.current_goal[0] - x, self.current_goal[1] - y
            )
            if dist_to_goal < goal_dist:
                self.set_state(FIND_FRONTIER, "goal reached")
                return
            if dist_to_goal < goal_dist * 1.5 and now_s - self.navigate_start_time > 5.0:
                self.set_state(FIND_FRONTIER, f"close enough ({dist_to_goal:.2f}m)")
                return

        # Check if all waypoints exhausted
        if not self.waypoints_world or self.wp_index >= len(self.waypoints_world):
            self.set_state(FIND_FRONTIER, "waypoints exhausted")
            return

        # Stuck detection
        if self.check_stuck(x, y):
            print(f"[STUCK] at ({x:.2f}, {y:.2f}) — starting random walk")
            self.recovery_start = now_s
            self.recovery_phase = 0
            self.random_walk_start = None
            self.front_blocked_since = None
            self.front_clear_since = None
            if self.current_goal is not None:
                self.blacklisted_goals.append(self.current_goal)
            self.set_state(RECOVERY, "stuck detected → random walk")
            return

        # Corner detection: front blocked for too long → recovery
        if self.front_blocked_since is not None:
            blocked_dur = now_s - self.front_blocked_since
            if blocked_dur > 2.0:
                print(
                    f"[STUCK] front blocked {blocked_dur:.1f}s at "
                    f"({x:.2f}, {y:.2f}) — starting random walk"
                )
                self.recovery_start = now_s
                self.recovery_phase = 0
                self.random_walk_start = None
                self.front_blocked_since = None
                self.front_clear_since = None
                if self.current_goal is not None:
                    self.blacklisted_goals.append(self.current_goal)
                self.set_state(RECOVERY, f"front blocked {blocked_dur:.1f}s → random walk")
                return

        # Waypoint advancement (skip passed waypoints)
        wp_reached = float(self.get_parameter("goal_reached_dist_m").value)
        while self.wp_index < len(self.waypoints_world) - 1:
            wx0, wy0 = self.waypoints_world[self.wp_index]
            wx1, wy1 = self.waypoints_world[self.wp_index + 1]

            segx = wx1 - wx0
            segy = wy1 - wy0
            seg_len = math.hypot(segx, segy)
            if seg_len < 1e-6:
                self.wp_index += 1
                continue

            dirx = segx / seg_len
            diry = segy / seg_len
            relx = x - wx0
            rely = y - wy0
            progress = relx * dirx + rely * diry
            dist_to_current = math.hypot(wx0 - x, wy0 - y)

            if progress > wp_reached and dist_to_current > wp_reached:
                self.wp_index += 1
                continue
            break

        # Current waypoint check
        wx, wy = self.waypoints_world[self.wp_index]
        dist_wp = math.hypot(wx - x, wy - y)
        if dist_wp <= wp_reached:
            if self.wp_index >= len(self.waypoints_world) - 1:
                self.set_state(FIND_FRONTIER)
                return
            self.wp_index += 1
            wx, wy = self.waypoints_world[self.wp_index]

        # ---- Potential field controller ----
        self._drive_potential_field(x, y, yaw, wx, wy)

    # ================================================================ #
    #  Potential field navigation
    # ================================================================ #
    def _drive_potential_field(
        self, x: float, y: float, yaw: float, wx: float, wy: float
    ):
        k_att = float(self.get_parameter("k_att").value)
        k_rep = float(self.get_parameter("k_rep").value)
        rep_range = float(self.get_parameter("repulsion_range_m").value)
        stop_range = float(self.get_parameter("stop_range_m").value)

        # Attractive force in robot frame
        dx_w = wx - x
        dy_w = wy - y
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        dx_r = c * dx_w - s * dy_w
        dy_r = s * dx_w + c * dy_w

        F_att = np.array([k_att * dx_r, k_att * dy_r], dtype=np.float64)
        # Cap attractive force so it can never overwhelm repulsion
        att_mag = np.linalg.norm(F_att)
        if att_mag > 1.0:
            F_att = F_att / att_mag * 1.0

        # Repulsive forces from LaserScan
        scan = self.scan
        ranges = np.array(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float64) * scan.angle_increment

        valid = np.isfinite(ranges) & (ranges > 0.0)
        ranges = ranges[valid]
        angles = angles[valid]

        # Front-blocked rotation: suppress linear but keep rotating
        front_cone = np.abs(angles) < math.radians(50.0)
        front_blocked = bool(np.any(ranges[front_cone] < stop_range))

        F_rep = np.zeros(2, dtype=np.float64)
        in_range = ranges < rep_range
        rr = ranges[in_range]
        aa = angles[in_range]

        if rr.size > 0:
            ox = rr * np.cos(aa)
            oy = rr * np.sin(aa)

            inv_r = 1.0 / np.maximum(rr, 1e-3)
            mag = k_rep * (inv_r - 1.0 / rep_range) * (inv_r ** 2)

            dir_x = -ox / np.maximum(rr, 1e-3)
            dir_y = -oy / np.maximum(rr, 1e-3)

            fx = mag * dir_x
            fy = mag * dir_y

            F_rep[0] = float(np.clip(np.sum(fx), -5.0, 5.0))
            F_rep[1] = float(np.clip(np.sum(fy), -5.0, 5.0))

        # Tangential wall-sliding: when forces oppose, add perpendicular component
        # Choose the tangent direction that aligns with the attractive force
        # to avoid oscillation in corners
        att_norm = np.linalg.norm(F_att)
        rep_norm = np.linalg.norm(F_rep)
        if att_norm > 1e-4 and rep_norm > 1e-4:
            dot = float(np.dot(F_att / att_norm, F_rep / rep_norm))
            if dot < -0.3:
                tangent_a = np.array([-F_rep[1], F_rep[0]], dtype=np.float64)
                tangent_b = np.array([F_rep[1], -F_rep[0]], dtype=np.float64)
                # Pick the tangent that points more toward the goal
                if np.dot(tangent_a, F_att) >= np.dot(tangent_b, F_att):
                    tangent = tangent_a
                else:
                    tangent = tangent_b
                F_rep = F_rep + 1.0 * tangent

        F = F_att + F_rep

        desired_heading = math.atan2(float(F[1]), float(F[0]))
        heading_err = wrap_angle(desired_heading)

        max_ang = float(self.get_parameter("max_ang").value)
        max_lin = float(self.get_parameter("max_lin").value)
        k_heading = float(self.get_parameter("k_heading").value)

        ang = clamp(k_heading * heading_err, -max_ang, max_ang)

        heading_factor = max(0.0, math.cos(heading_err))
        lin = clamp(heading_factor * max_lin, 0.0, max_lin)

        # Proximity speed scaling: slow down near obstacles
        if rr.size > 0:
            min_range = float(np.min(rr))
            proximity_factor = clamp(min_range / rep_range, 0.05, 1.0)
            lin *= proximity_factor

        # Front-blocked: stop forward motion, rotation continues
        # Hysteresis: only clear front_blocked_since after front is clear for 0.5s
        now_fb = self.get_clock().now().nanoseconds * 1e-9
        if front_blocked:
            lin = 0.0
            self.front_clear_since = None
            if self.front_blocked_since is None:
                self.front_blocked_since = now_fb
        else:
            if self.front_clear_since is None:
                self.front_clear_since = now_fb
            if now_fb - self.front_clear_since > 0.5:
                self.front_blocked_since = None

        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_pub.publish(cmd)

    # ================================================================ #
    #  Recovery behavior
    # ================================================================ #
    def _do_recovery(self, now_s: float):
        """Recovery = back up briefly, then random walk to escape."""
        if self.recovery_start is None:
            self.recovery_start = now_s

        elapsed = now_s - self.recovery_start
        back_dur = float(self.get_parameter("recovery_back_duration_s").value)
        rw_dur = float(self.get_parameter("random_walk_duration_s").value)
        rw_lin = float(self.get_parameter("random_walk_lin").value)
        rw_ang = float(self.get_parameter("random_walk_ang").value)
        stop_range = float(self.get_parameter("stop_range_m").value)

        cmd = Twist()

        # Phase 0: back up
        if self.recovery_phase == 0:
            if elapsed < back_dur:
                cmd.linear.x = float(self.get_parameter("recovery_back_speed").value)
                self.cmd_pub.publish(cmd)
                return
            else:
                # Transition to random walk
                self.recovery_phase = 1
                self.random_walk_start = now_s
                self.recovery_start = now_s
                self._new_random_walk_step()
                print("[RECOVERY] Random walk started")
                return

        # Phase 1: random turn
        if self.recovery_phase == 1:
            elapsed_phase = now_s - self.recovery_start
            if elapsed_phase < self.random_walk_phase_dur:
                cmd.angular.z = self.random_walk_turn_dir * rw_ang
                self.cmd_pub.publish(cmd)
                return
            else:
                # Transition to forward drive
                self.recovery_phase = 2
                self.recovery_start = now_s
                self.random_walk_phase_dur = random.uniform(0.5, 1.5)
                return

        # Phase 2: drive forward (with front safety check)
        if self.recovery_phase == 2:
            elapsed_phase = now_s - self.recovery_start

            # Check if front is blocked — skip forward, go to next turn
            front_blocked = False
            if self.scan is not None:
                ranges = np.array(self.scan.ranges, dtype=np.float64)
                angles = (
                    self.scan.angle_min
                    + np.arange(len(ranges), dtype=np.float64)
                    * self.scan.angle_increment
                )
                valid = np.isfinite(ranges) & (ranges > 0.0)
                front_cone = np.abs(angles[valid]) < math.radians(50.0)
                if front_cone.any() and np.any(ranges[valid][front_cone] < stop_range):
                    front_blocked = True

            if elapsed_phase < self.random_walk_phase_dur and not front_blocked:
                cmd.linear.x = rw_lin
                self.cmd_pub.publish(cmd)
            else:
                self.cmd_pub.publish(Twist())

            # Check if total random walk time is up
            if now_s - self.random_walk_start >= rw_dur:
                self.cmd_pub.publish(Twist())
                self.pose_history.clear()
                self.set_state(FIND_FRONTIER, "random walk complete")
                return

            # If this drive phase is done, start a new turn
            if elapsed_phase >= self.random_walk_phase_dur or front_blocked:
                self.recovery_phase = 1
                self.recovery_start = now_s
                self._new_random_walk_step()

    def _new_random_walk_step(self):
        """Pick a new random turn direction and duration."""
        self.random_walk_turn_dir = random.choice([-1.0, 1.0])
        self.random_walk_phase_dur = random.uniform(0.5, 1.5)


def main():
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
