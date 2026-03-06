#!/usr/bin/env python3
import math
from enum import Enum, auto
from typing import List, Tuple, Optional
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion

from my_planner_pkg.planner_pf_node import (
    astar, extract_waypoints, clamp, wrap_angle,
)


# -----------------------------------------------
# OccupancyGrid coordinate helpers
# -----------------------------------------------
def og_world_to_grid(x: float, y: float, origin: Tuple[float, float],
                     resolution: float) -> Tuple[int, int]:
    c = int(math.floor((x - origin[0]) / resolution))
    r = int(math.floor((y - origin[1]) / resolution))
    return (r, c)


def og_grid_to_world(r: int, c: int, origin: Tuple[float, float],
                     resolution: float) -> Tuple[float, float]:
    x = origin[0] + (c + 0.5) * resolution
    y = origin[1] + (r + 0.5) * resolution
    return (x, y)


# -----------------------------------------------
# State machine
# -----------------------------------------------
class State(Enum):
    FIND_FRONTIER = auto()
    NAVIGATE = auto()
    RANDOM_WALK = auto()
    DONE = auto()


class FrontierPotentialFieldExplorer(Node):
    def __init__(self):
        super().__init__("frontier_pf_explorer")

        # --- Parameters ---
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_topic", "/map")

        self.declare_parameter("inflation_radius_m", 0.6)
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("waypoint_every_n_cells", 20)
        self.declare_parameter("waypoints_include_turns", True)

        self.declare_parameter("k_att", 0.8)
        self.declare_parameter("k_rep", 5.0)
        self.declare_parameter("repulsion_range_m", 0.4)
        self.declare_parameter("stop_range_m", 0.25)
        self.declare_parameter("k_heading", 2.0)
        self.declare_parameter("max_lin", 0.7)
        self.declare_parameter("max_ang", 1.5)
        self.declare_parameter("wp_reached_dist_m", 0.20)
        self.declare_parameter("goal_reached_dist_m", 0.50)

        self.declare_parameter("min_frontier_cluster", 5)
        self.declare_parameter("min_goal_wall_clearance_m", 0.5)
        self.declare_parameter("visited_goal_radius_m", 0.8)
        self.declare_parameter("stuck_window_s", 4.0)
        self.declare_parameter("stuck_threshold_m", 0.15)

        self.declare_parameter("random_walk_turn_duration_s", 6.0)
        self.declare_parameter("random_walk_move_duration_s", 6.0)
        self.declare_parameter("random_walk_angular_speed", 0.8)
        self.declare_parameter("random_walk_linear_speed", 0.7)

        # --- State ---
        self.state = State.FIND_FRONTIER

        # Map data (from /map OccupancyGrid)
        self.occ_grid: Optional[np.ndarray] = None  # raw grid: -1/0/100
        self.planning_grid: Optional[np.ndarray] = None  # inflated: 0=free, 100=blocked
        self.map_resolution = 0.0
        self.map_origin: Tuple[float, float] = (0.0, 0.0)
        self.map_H = 0
        self.map_W = 0

        # Navigation
        self.waypoints_world: List[Tuple[float, float]] = []
        self.wp_index = 0
        self.goal_world: Optional[Tuple[float, float]] = None
        self.scan: Optional[LaserScan] = None

        # Stuck detection
        self.pose_history: List[Tuple[float, float, float]] = []  # (time_s, x, y)

        # Random walk
        self.rw_phase: Optional[str] = None  # 'turn' or 'move'
        self.rw_phase_start: float = 0.0

        # Blacklist of recently visited goal locations
        self._visited_goals: List[Tuple[float, float]] = []

        # Done flag (log only once)
        self._done_logged = False

        # Track whether navigation has ever started (to distinguish
        # "not ready yet" from "truly no frontiers left")
        self._ever_navigated = False

        # Track whether map has been updated since last frontier search
        self._map_updated = False

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            map_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "~/viz_waypoints",
            10,
        )

        # --- Control loop at 20 Hz ---
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Frontier explorer node started. Waiting for map...")

    # ==========================================================
    # Callbacks
    # ==========================================================
    def on_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        raw = np.array(msg.data, dtype=np.int8).reshape((h, w))

        self.occ_grid = raw
        self.map_resolution = msg.info.resolution
        self.map_origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
        )
        self.map_H = h
        self.map_W = w

        self.planning_grid = self._build_planning_grid()
        self._map_updated = True

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    # ==========================================================
    # Planning grid (unknown = free for exploration)
    # ==========================================================
    def _build_planning_grid(self) -> np.ndarray:
        grid = self.occ_grid.astype(np.int16).copy()

        # Unknown (-1) treated as free so the robot can plan through unexplored space
        grid[grid == -1] = 0
        # Anything > 0 is occupied
        grid[grid > 0] = 100
        grid[grid < 0] = 0  # safety

        inflation_cells = int(math.ceil(
            self.get_parameter("inflation_radius_m").value / self.map_resolution
        ))

        if inflation_cells <= 0:
            return grid.astype(np.int8)

        obstacle = (grid == 100).astype(bool)
        try:
            from scipy.ndimage import binary_dilation
            structure = np.ones(
                (2 * inflation_cells + 1, 2 * inflation_cells + 1), dtype=bool
            )
            inflated = binary_dilation(obstacle, structure=structure)
        except ImportError:
            inflated = obstacle.copy()
            ys, xs = np.where(obstacle)
            for yy, xx in zip(ys, xs):
                y0 = max(0, yy - inflation_cells)
                y1 = min(self.map_H, yy + inflation_cells + 1)
                x0 = max(0, xx - inflation_cells)
                x1 = min(self.map_W, xx + inflation_cells + 1)
                inflated[y0:y1, x0:x1] = True

        out = np.zeros_like(grid, dtype=np.int8)
        out[inflated] = 100
        return out

    # ==========================================================
    # Wall clearance check (uses raw occ_grid, not planning_grid)
    # ==========================================================
    def _cell_clear_of_walls(self, r: int, c: int, clearance_cells: int) -> bool:
        """Return True if no occupied cell (>0) is within clearance_cells of (r, c)."""
        r0 = max(0, r - clearance_cells)
        r1 = min(self.map_H, r + clearance_cells + 1)
        c0 = max(0, c - clearance_cells)
        c1 = min(self.map_W, c + clearance_cells + 1)
        patch = self.occ_grid[r0:r1, c0:c1]
        return not np.any(patch > 0)

    # ==========================================================
    # Frontier detection
    # ==========================================================
    def _find_frontiers(self) -> List[Tuple[float, float]]:
        if self.occ_grid is None:
            return []

        grid = self.occ_grid  # -1=unknown, 0=free, >0=occupied
        free_mask = (grid == 0)
        unknown_mask = (grid == -1)

        # A free cell is a frontier cell if it has at least one unknown 8-neighbor
        has_unknown_neighbor = np.zeros(grid.shape, dtype=bool)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                        (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            shifted = np.roll(np.roll(unknown_mask, dr, axis=0), dc, axis=1)
            has_unknown_neighbor |= shifted

        frontier_mask = free_mask & has_unknown_neighbor

        # BFS clustering
        visited = np.zeros(grid.shape, dtype=bool)
        clusters: List[List[Tuple[int, int]]] = []
        frontier_coords = list(zip(*np.where(frontier_mask)))

        min_cluster = self.get_parameter("min_frontier_cluster").value

        for r, c in frontier_coords:
            if visited[r, c]:
                continue
            cluster: List[Tuple[int, int]] = []
            queue = deque()
            queue.append((r, c))
            visited[r, c] = True
            while queue:
                cr, cc = queue.popleft()
                cluster.append((cr, cc))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                                (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nr, nc = cr + dr, cc + dc
                    if (0 <= nr < self.map_H and 0 <= nc < self.map_W
                            and not visited[nr, nc] and frontier_mask[nr, nc]):
                        visited[nr, nc] = True
                        queue.append((nr, nc))
            if len(cluster) >= min_cluster:
                clusters.append(cluster)

        if not clusters:
            return []

        # Get robot position for distance sorting
        pose = self._get_robot_pose()
        if pose is None:
            return []
        rx, ry, _ = pose

        clearance_cells = int(math.ceil(
            self.get_parameter("min_goal_wall_clearance_m").value / self.map_resolution
        ))

        # For each cluster, pick a goal point and compute distance
        goals = []
        for cluster in clusters:
            # Compute centroid in grid coords
            rows = [p[0] for p in cluster]
            cols = [p[1] for p in cluster]
            cr = int(round(sum(rows) / len(rows)))
            cc = int(round(sum(cols) / len(cols)))

            goal_cell = None
            # If centroid is free in planning grid AND clear of walls, use it
            if (0 <= cr < self.map_H and 0 <= cc < self.map_W
                    and self.planning_grid[cr, cc] == 0
                    and self._cell_clear_of_walls(cr, cc, clearance_cells)):
                goal_cell = (cr, cc)
            else:
                # Pick closest cluster cell to robot that is free and clear of walls
                best_dist = float('inf')
                for cell_r, cell_c in cluster:
                    if self.planning_grid[cell_r, cell_c] != 0:
                        continue
                    if not self._cell_clear_of_walls(cell_r, cell_c, clearance_cells):
                        continue
                    wx, wy = og_grid_to_world(cell_r, cell_c, self.map_origin, self.map_resolution)
                    d = math.hypot(wx - rx, wy - ry)
                    if d < best_dist:
                        best_dist = d
                        goal_cell = (cell_r, cell_c)

            if goal_cell is None:
                continue  # skip cluster — no safe goal candidate

            gx, gy = og_grid_to_world(goal_cell[0], goal_cell[1],
                                       self.map_origin, self.map_resolution)
            dist = math.hypot(gx - rx, gy - ry)
            goals.append((dist, gx, gy))

        goals.sort(key=lambda t: t[0])

        # Filter out goals too close to already-visited locations
        visited_radius = self.get_parameter("visited_goal_radius_m").value
        filtered = []
        for _, gx, gy in goals:
            too_close = any(
                math.hypot(gx - vx, gy - vy) < visited_radius
                for vx, vy in self._visited_goals
            )
            if not too_close:
                filtered.append((gx, gy))
        return filtered

    # ==========================================================
    # Path planning to a goal
    # ==========================================================
    def _plan_to_goal(self, goal_x: float, goal_y: float) -> bool:
        pose = self._get_robot_pose()
        if pose is None or self.planning_grid is None:
            return False

        rx, ry, _ = pose
        start = og_world_to_grid(rx, ry, self.map_origin, self.map_resolution)
        goal = og_world_to_grid(goal_x, goal_y, self.map_origin, self.map_resolution)

        # Clamp to grid bounds
        start = (clamp(start[0], 0, self.map_H - 1), clamp(start[1], 0, self.map_W - 1))
        goal = (clamp(goal[0], 0, self.map_H - 1), clamp(goal[1], 0, self.map_W - 1))
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))

        # If start is in obstacle, find nearest free cell
        if self.planning_grid[start[0], start[1]] != 0:
            start = self._nearest_free_cell(start)
            if start is None:
                return False

        # If goal is in obstacle, find nearest free cell
        if self.planning_grid[goal[0], goal[1]] != 0:
            goal = self._nearest_free_cell(goal)
            if goal is None:
                return False

        allow_diag = self.get_parameter("allow_diagonal").value
        path = astar(self.planning_grid, start, goal, allow_diagonal=allow_diag)
        if path is None:
            return False

        every_n = int(self.get_parameter("waypoint_every_n_cells").value)
        include_turns = bool(self.get_parameter("waypoints_include_turns").value)
        wp_cells = extract_waypoints(path, take_every_n=every_n,
                                      include_turning_points=include_turns)

        self.waypoints_world = [
            og_grid_to_world(r, c, self.map_origin, self.map_resolution)
            for (r, c) in wp_cells
        ]
        self.wp_index = 0
        self.goal_world = (goal_x, goal_y)
        return True

    def _nearest_free_cell(self, cell: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """BFS outward from cell to find nearest free cell in planning grid."""
        visited = set()
        queue = deque()
        queue.append(cell)
        visited.add(cell)
        while queue:
            r, c = queue.popleft()
            if self.planning_grid[r, c] == 0:
                return (r, c)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if (0 <= nr < self.map_H and 0 <= nc < self.map_W
                        and (nr, nc) not in visited):
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        return None

    # ==========================================================
    # TF helper
    # ==========================================================
    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
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

    # ==========================================================
    # Stuck detection
    # ==========================================================
    def _is_stuck(self, x: float, y: float) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        self.pose_history.append((now, x, y))

        window = self.get_parameter("stuck_window_s").value
        threshold = self.get_parameter("stuck_threshold_m").value

        # Prune old entries
        while self.pose_history and (now - self.pose_history[0][0]) > window:
            self.pose_history.pop(0)

        if not self.pose_history:
            return False

        # Need at least the full window to declare stuck
        if (now - self.pose_history[0][0]) < window:
            return False

        # Check max displacement from current position
        for t, px, py in self.pose_history:
            if math.hypot(px - x, py - y) > threshold:
                return False
        return True

    # ==========================================================
    # Main control loop (20 Hz)
    # ==========================================================
    def control_loop(self):
        if self.state == State.FIND_FRONTIER:
            self._handle_find_frontier()
        elif self.state == State.NAVIGATE:
            self._handle_navigate()
        elif self.state == State.RANDOM_WALK:
            self._handle_random_walk()
        elif self.state == State.DONE:
            self._handle_done()

    # ==========================================================
    # RViz visualization
    # ==========================================================
    def _publish_viz_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("map_frame").value

        # --- Goal marker (red sphere) ---
        goal_marker = Marker()
        goal_marker.header.stamp = stamp
        goal_marker.header.frame_id = frame
        goal_marker.ns = "goal"
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.scale.x = 0.25
        goal_marker.scale.y = 0.25
        goal_marker.scale.z = 0.25
        goal_marker.color.a = 1.0
        if self.goal_world is not None:
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.goal_world[0]
            goal_marker.pose.position.y = self.goal_world[1]
            goal_marker.pose.position.z = 0.15
            goal_marker.pose.orientation.w = 1.0
            goal_marker.color.r = 1.0
        else:
            goal_marker.action = Marker.DELETE
        ma.markers.append(goal_marker)

        # --- Waypoint spheres (green, current = yellow) ---
        for i, (wx, wy) in enumerate(self.waypoints_world):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.12
            m.color.a = 1.0
            if i == self.wp_index:
                m.color.r = 1.0
                m.color.g = 1.0
            else:
                m.color.g = 1.0
            ma.markers.append(m)

        # Delete stale waypoint markers from previous (longer) paths
        for i in range(len(self.waypoints_world), len(self.waypoints_world) + 50):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.ns = "waypoints"
            m.id = i
            m.action = Marker.DELETE
            ma.markers.append(m)

        # --- Path line strip (blue) ---
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = frame
        line.ns = "path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.04
        line.color.a = 0.8
        line.color.b = 1.0
        if len(self.waypoints_world) >= 2:
            line.action = Marker.ADD
            for wx, wy in self.waypoints_world:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.05
                line.points.append(p)
        else:
            line.action = Marker.DELETE
        ma.markers.append(line)

        self.marker_pub.publish(ma)

    def _handle_find_frontier(self):
        if self.occ_grid is None or not self._map_updated:
            return  # still waiting for (updated) map
        self._map_updated = False

        frontiers = self._find_frontiers()
        if not frontiers:
            # If we have blacklisted goals, clear them and retry — the map may
            # have changed enough that revisiting those areas finds new frontiers.
            if self._visited_goals:
                self.get_logger().info(
                    f"No new frontiers found. Clearing {len(self._visited_goals)} "
                    f"visited goals and retrying."
                )
                self._visited_goals.clear()
                frontiers = self._find_frontiers()

        if not frontiers:
            if not self._ever_navigated:
                # Not ready yet (e.g. TF not available) — retry on next map update
                self._map_updated = True
                return
            self.get_logger().info("Exploration complete: no frontier clusters found.")
            self.state = State.DONE
            return

        # Try each frontier, not just the closest
        for gx, gy in frontiers:
            if self._plan_to_goal(gx, gy):
                pose = self._get_robot_pose()
                dist = math.hypot(gx - pose[0], gy - pose[1]) if pose else 0.0
                self.get_logger().info(
                    f"New goal at ({gx:.2f}, {gy:.2f}), distance: {dist:.2f} m, "
                    f"waypoints: {len(self.waypoints_world)}"
                )
                self.get_logger().info(
                    f"Following waypoint 1/{len(self.waypoints_world)} at "
                    f"({self.waypoints_world[0][0]:.2f}, {self.waypoints_world[0][1]:.2f})"
                )
                self.pose_history.clear()
                self._ever_navigated = True
                self.state = State.NAVIGATE
                self._publish_viz_markers()
                return

        if not self._ever_navigated:
            # Can't plan yet — retry on next map update
            self._map_updated = True
            return
        self.get_logger().info(
            "Exploration complete: cannot plan path to any frontier."
        )
        self.state = State.DONE

    def _handle_navigate(self):
        if self.scan is None:
            return

        pose = self._get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose

        # --- Revalidate goal when map updates reveal walls nearby ---
        if self._map_updated and self.goal_world is not None:
            self._map_updated = False
            gr, gc = og_world_to_grid(
                self.goal_world[0], self.goal_world[1],
                self.map_origin, self.map_resolution,
            )
            gr = clamp(gr, 0, self.map_H - 1)
            gc = clamp(gc, 0, self.map_W - 1)
            clearance_cells = int(math.ceil(
                self.get_parameter("min_goal_wall_clearance_m").value
                / self.map_resolution
            ))
            if not self._cell_clear_of_walls(int(gr), int(gc), clearance_cells):
                self.get_logger().info(
                    f"Goal ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f}) "
                    f"now too close to wall — abandoning, finding new frontier"
                )
                self._visited_goals.append(self.goal_world)
                self.cmd_pub.publish(Twist())
                self.waypoints_world.clear()
                self.wp_index = 0
                self.goal_world = None
                self.pose_history.clear()
                self._publish_viz_markers()
                self.state = State.FIND_FRONTIER
                return

        # --- Stuck detection ---
        if self._is_stuck(x, y):
            self.get_logger().warn(
                f"Stuck at ({x:.2f}, {y:.2f}), starting random walk"
            )
            self.rw_phase = 'turn'
            self.rw_phase_start = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f"Random walk: turn phase "
                f"({self.get_parameter('random_walk_angular_speed').value} rad/s "
                f"for {self.get_parameter('random_walk_turn_duration_s').value}s)"
            )
            self.state = State.RANDOM_WALK
            return

        # --- Waypoint skip logic (same as planner_pf_node) ---
        wp_reached_dist = float(self.get_parameter("wp_reached_dist_m").value)
        while self.wp_index < (len(self.waypoints_world) - 1):
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

            if progress > wp_reached_dist and dist_to_current > wp_reached_dist:
                self.get_logger().info(
                    f"Skipping waypoint {self.wp_index + 1}/{len(self.waypoints_world)} "
                    f"(missed/passed)."
                )
                self.wp_index += 1
                continue
            break

        if self.wp_index >= len(self.waypoints_world):
            self._on_goal_reached()
            return

        wx, wy = self.waypoints_world[self.wp_index]
        dist_wp = math.hypot(wx - x, wy - y)

        # --- Check waypoint reached ---
        if dist_wp <= wp_reached_dist:
            total_wps = len(self.waypoints_world)

            if self.wp_index >= total_wps - 1:
                self._on_goal_reached()
                return

            self.wp_index += 1
            wx, wy = self.waypoints_world[self.wp_index]
            dist_wp = math.hypot(wx - x, wy - y)
            self.get_logger().info(
                f"Following waypoint {self.wp_index + 1}/{total_wps} "
                f"at ({wx:.2f}, {wy:.2f})"
            )
            self._publish_viz_markers()

        # --- Also check goal distance directly ---
        goal_reached_dist = float(self.get_parameter("goal_reached_dist_m").value)
        if self.goal_world is not None:
            dist_goal = math.hypot(self.goal_world[0] - x, self.goal_world[1] - y)
            if dist_goal <= goal_reached_dist:
                self._on_goal_reached()
                return

        # --- Potential field control ---
        self._potential_field_step(x, y, yaw, wx, wy)

    def _on_goal_reached(self):
        if self.goal_world:
            self.get_logger().info(
                f"Goal reached at ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f})"
            )
            self._visited_goals.append(self.goal_world)
        self.cmd_pub.publish(Twist())
        self.waypoints_world.clear()
        self.wp_index = 0
        self.goal_world = None
        self.pose_history.clear()
        self._publish_viz_markers()
        self.state = State.FIND_FRONTIER

    def _potential_field_step(self, x: float, y: float, yaw: float,
                               wx: float, wy: float):
        # Attractive force in robot frame
        dx_w = wx - x
        dy_w = wy - y
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        dx_r = c * dx_w - s * dy_w
        dy_r = s * dx_w + c * dy_w

        k_att = self.get_parameter("k_att").value
        F_att = np.array([k_att * dx_r, k_att * dy_r], dtype=np.float32)

        # Repulsive forces from scan
        k_rep = self.get_parameter("k_rep").value
        rep_range = self.get_parameter("repulsion_range_m").value
        stop_range = self.get_parameter("stop_range_m").value

        ranges = np.array(self.scan.ranges, dtype=np.float32)
        angles = (self.scan.angle_min
                  + np.arange(len(ranges), dtype=np.float32) * self.scan.angle_increment)

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        # Front cone obstacle detection
        front_cone = np.abs(angles) < math.radians(35.0)
        if np.any(ranges[front_cone] < stop_range):
            # Stop linear, but still allow rotation toward waypoint
            desired_heading = math.atan2(dy_r, dx_r)
            heading_err = wrap_angle(desired_heading)
            cmd = Twist()
            cmd.angular.z = float(clamp(
                self.get_parameter("k_heading").value * heading_err,
                -self.get_parameter("max_ang").value,
                self.get_parameter("max_ang").value,
            ))
            self.cmd_pub.publish(cmd)
            return

        F_rep = np.zeros(2, dtype=np.float32)
        in_range = ranges < rep_range
        rr = ranges[in_range]
        aa = angles[in_range]
        if rr.size > 0:
            ox = rr * np.cos(aa)
            oy = rr * np.sin(aa)
            inv_r = 1.0 / np.maximum(rr, 1e-3)
            mag = k_rep * (inv_r - 1.0 / rep_range) * (inv_r ** 2)
            dirx = -ox / np.maximum(rr, 1e-3)
            diry = -oy / np.maximum(rr, 1e-3)
            fx = mag * dirx
            fy = mag * diry
            F_rep[0] = float(np.clip(np.sum(fx), -5.0, 5.0))
            F_rep[1] = float(np.clip(np.sum(fy), -5.0, 5.0))

        F = F_att + F_rep

        desired_heading = math.atan2(F[1], F[0])
        heading_err = wrap_angle(desired_heading)

        ang = clamp(
            self.get_parameter("k_heading").value * heading_err,
            -self.get_parameter("max_ang").value,
            self.get_parameter("max_ang").value,
        )

        heading_factor = max(0.0, math.cos(heading_err))
        lin = clamp(
            0.6 * heading_factor * self.get_parameter("max_lin").value,
            0.0,
            self.get_parameter("max_lin").value,
        )

        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_pub.publish(cmd)

    # ==========================================================
    # Random walk
    # ==========================================================
    def _handle_random_walk(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.rw_phase_start

        if self.rw_phase == 'turn':
            turn_dur = self.get_parameter("random_walk_turn_duration_s").value
            if elapsed >= turn_dur:
                # Switch to move phase
                self.rw_phase = 'move'
                self.rw_phase_start = now
                self.get_logger().info(
                    f"Random walk: move phase "
                    f"({self.get_parameter('random_walk_linear_speed').value} m/s "
                    f"for {self.get_parameter('random_walk_move_duration_s').value}s)"
                )
            else:
                cmd = Twist()
                cmd.angular.z = float(
                    self.get_parameter("random_walk_angular_speed").value
                )
                self.cmd_pub.publish(cmd)

        elif self.rw_phase == 'move':
            move_dur = self.get_parameter("random_walk_move_duration_s").value
            if elapsed >= move_dur:
                self.get_logger().info("Random walk finished")
                self.cmd_pub.publish(Twist())
                self.pose_history.clear()
                self.state = State.FIND_FRONTIER
            else:
                # Check front cone for safety during forward motion
                cmd = Twist()
                if self.scan is not None:
                    ranges = np.array(self.scan.ranges, dtype=np.float32)
                    angles = (self.scan.angle_min
                              + np.arange(len(ranges), dtype=np.float32)
                              * self.scan.angle_increment)
                    valid = np.isfinite(ranges)
                    ranges_v = ranges[valid]
                    angles_v = angles[valid]
                    front_cone = np.abs(angles_v) < math.radians(35.0)
                    stop_range = self.get_parameter("stop_range_m").value
                    if np.any(ranges_v[front_cone] < stop_range):
                        # Obstacle ahead during random walk forward — stop and finish
                        self.get_logger().info("Random walk finished (obstacle ahead)")
                        self.cmd_pub.publish(Twist())
                        self.pose_history.clear()
                        self.state = State.FIND_FRONTIER
                        return

                cmd.linear.x = float(
                    self.get_parameter("random_walk_linear_speed").value
                )
                self.cmd_pub.publish(cmd)

    def _handle_done(self):
        if not self._done_logged:
            self.get_logger().info("Exploration complete. Robot stopped.")
            self._done_logged = True
        self.cmd_pub.publish(Twist())


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
