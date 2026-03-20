#!/usr/bin/env python3
import json
import math
from collections import deque
from enum import Enum, auto
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


def og_grid_to_world(
    r: int,
    c: int,
    origin: Tuple[float, float],
    resolution: float,
) -> Tuple[float, float]:
    x = origin[0] + (c + 0.5) * resolution
    y = origin[1] + (r + 0.5) * resolution
    return (x, y)


class State(Enum):
    FIND_FRONTIER = auto()
    WAIT_FOR_NAVIGATION = auto()
    DONE = auto()


class FrontierPotentialFieldExplorer(Node):
    def __init__(self):
        super().__init__("frontier_pf_explorer")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("planner_status_topic", "/planner_pf/status")

        self.declare_parameter("inflation_radius_m", 0.6)
        self.declare_parameter("min_frontier_cluster", 5)
        self.declare_parameter("min_goal_wall_clearance_m", 0.5)
        self.declare_parameter("visited_goal_radius_m", 0.8)
        self.declare_parameter("status_goal_tolerance_m", 0.2)
        self.declare_parameter("planner_ack_timeout_s", 2.0)
        self.declare_parameter("goal_republish_period_s", 0.5)

        self.state = State.FIND_FRONTIER

        self.occ_grid: Optional[np.ndarray] = None
        self.planning_grid: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_origin: Tuple[float, float] = (0.0, 0.0)
        self.map_H = 0
        self.map_W = 0

        self.pending_frontiers: List[Tuple[float, float]] = []
        self.active_goal_world: Optional[Tuple[float, float]] = None
        self.active_goal_sent_at = 0.0
        self.active_goal_last_publish_at = 0.0
        self._visited_goals: List[Tuple[float, float]] = []
        self._map_updated = False
        self._done_logged = False

        self.planner_state = "idle"
        self.planner_goal: Optional[Tuple[float, float]] = None
        self.planner_message = ""

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        status_qos = QoSProfile(
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
            String,
            self.get_parameter("planner_status_topic").value,
            self.on_planner_status,
            status_qos,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "~/viz_frontiers",
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            "Frontier explorer started. Waiting for map and planner status."
        )

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

    def on_planner_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Ignoring malformed planner status payload.")
            return

        self.planner_state = str(payload.get("state", "idle"))
        self.planner_message = str(payload.get("message", ""))
        goal = payload.get("goal")
        if isinstance(goal, dict) and "x" in goal and "y" in goal:
            self.planner_goal = (float(goal["x"]), float(goal["y"]))
        else:
            self.planner_goal = None

    def _build_planning_grid(self) -> np.ndarray:
        grid = self.occ_grid.astype(np.int16).copy()
        grid[grid == -1] = 0
        grid[grid > 0] = 100
        grid[grid < 0] = 0

        inflation_cells = int(
            math.ceil(
                self.get_parameter("inflation_radius_m").value / self.map_resolution
            )
        )
        if inflation_cells <= 0:
            return grid.astype(np.int8)

        obstacle = (grid == 100).astype(bool)
        try:
            from scipy.ndimage import binary_dilation

            structure = np.ones(
                (2 * inflation_cells + 1, 2 * inflation_cells + 1),
                dtype=bool,
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

    def _cell_clear_of_walls(self, r: int, c: int, clearance_cells: int) -> bool:
        r0 = max(0, r - clearance_cells)
        r1 = min(self.map_H, r + clearance_cells + 1)
        c0 = max(0, c - clearance_cells)
        c1 = min(self.map_W, c + clearance_cells + 1)
        patch = self.occ_grid[r0:r1, c0:c1]
        return not np.any(patch > 0)

    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tfm = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        x = tfm.transform.translation.x
        y = tfm.transform.translation.y
        q = tfm.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, y, yaw)

    def _find_frontiers(self) -> List[Tuple[float, float]]:
        if self.occ_grid is None or self.planning_grid is None:
            return []

        grid = self.occ_grid
        h, w = grid.shape
        free_mask = grid == 0
        unknown_mask = grid == -1

        has_unknown_neighbor = np.zeros(grid.shape, dtype=bool)
        for dr, dc in [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]:
            shifted = np.zeros_like(unknown_mask, dtype=bool)
            src_r0 = max(0, -dr)
            src_r1 = h - max(0, dr)
            src_c0 = max(0, -dc)
            src_c1 = w - max(0, dc)
            dst_r0 = max(0, dr)
            dst_r1 = h - max(0, -dr)
            dst_c0 = max(0, dc)
            dst_c1 = w - max(0, -dc)
            shifted[dst_r0:dst_r1, dst_c0:dst_c1] = unknown_mask[src_r0:src_r1, src_c0:src_c1]
            has_unknown_neighbor |= shifted

        frontier_mask = free_mask & has_unknown_neighbor
        visited = np.zeros(grid.shape, dtype=bool)
        clusters: List[List[Tuple[int, int]]] = []
        frontier_coords = list(zip(*np.where(frontier_mask)))
        min_cluster = int(self.get_parameter("min_frontier_cluster").value)

        for r, c in frontier_coords:
            if visited[r, c]:
                continue

            cluster: List[Tuple[int, int]] = []
            queue = deque([(r, c)])
            visited[r, c] = True
            while queue:
                cr, cc = queue.popleft()
                cluster.append((cr, cc))
                for dr, dc in [
                    (-1, 0),
                    (1, 0),
                    (0, -1),
                    (0, 1),
                    (-1, -1),
                    (-1, 1),
                    (1, -1),
                    (1, 1),
                ]:
                    nr = cr + dr
                    nc = cc + dc
                    if (
                        0 <= nr < self.map_H
                        and 0 <= nc < self.map_W
                        and not visited[nr, nc]
                        and frontier_mask[nr, nc]
                    ):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

            if len(cluster) >= min_cluster:
                clusters.append(cluster)

        if not clusters:
            return []

        pose = self._get_robot_pose()
        if pose is None:
            return []
        rx, ry, _ = pose

        clearance_cells = int(
            math.ceil(
                self.get_parameter("min_goal_wall_clearance_m").value
                / self.map_resolution
            )
        )

        goals = []
        for cluster in clusters:
            rows = [p[0] for p in cluster]
            cols = [p[1] for p in cluster]
            cr = int(round(sum(rows) / len(rows)))
            cc = int(round(sum(cols) / len(cols)))

            goal_cell = None
            if (
                0 <= cr < self.map_H
                and 0 <= cc < self.map_W
                and self.planning_grid[cr, cc] == 0
                and self._cell_clear_of_walls(cr, cc, clearance_cells)
            ):
                goal_cell = (cr, cc)
            else:
                best_dist = float("inf")
                for cell_r, cell_c in cluster:
                    if self.planning_grid[cell_r, cell_c] != 0:
                        continue
                    if not self._cell_clear_of_walls(cell_r, cell_c, clearance_cells):
                        continue
                    wx, wy = og_grid_to_world(
                        cell_r,
                        cell_c,
                        self.map_origin,
                        self.map_resolution,
                    )
                    dist = math.hypot(wx - rx, wy - ry)
                    if dist < best_dist:
                        best_dist = dist
                        goal_cell = (cell_r, cell_c)

            if goal_cell is None:
                continue

            gx, gy = og_grid_to_world(
                goal_cell[0],
                goal_cell[1],
                self.map_origin,
                self.map_resolution,
            )
            dist = math.hypot(gx - rx, gy - ry)
            goals.append((dist, gx, gy))

        goals.sort(key=lambda item: item[0])

        visited_radius = float(self.get_parameter("visited_goal_radius_m").value)
        filtered = []
        for _, gx, gy in goals:
            too_close = any(
                math.hypot(gx - vx, gy - vy) < visited_radius
                for vx, vy in self._visited_goals
            )
            if not too_close:
                filtered.append((gx, gy))
        return filtered

    def _publish_goal(self, goal_x: float, goal_y: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.pose.position.x = float(goal_x)
        msg.pose.position.y = float(goal_y)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        now = self.get_clock().now().nanoseconds / 1e9
        if self.active_goal_sent_at <= 0.0:
            self.active_goal_sent_at = now
        self.active_goal_last_publish_at = now

    def _goal_matches_status(self, goal: Optional[Tuple[float, float]]) -> bool:
        if goal is None or self.planner_goal is None:
            return False
        tol = float(self.get_parameter("status_goal_tolerance_m").value)
        return math.hypot(goal[0] - self.planner_goal[0], goal[1] - self.planner_goal[1]) <= tol

    def _publish_viz_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("map_frame").value

        goal_marker = Marker()
        goal_marker.header.stamp = stamp
        goal_marker.header.frame_id = frame
        goal_marker.ns = "active_goal"
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.scale.x = 0.25
        goal_marker.scale.y = 0.25
        goal_marker.scale.z = 0.25
        goal_marker.color.a = 1.0
        if self.active_goal_world is not None:
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.active_goal_world[0]
            goal_marker.pose.position.y = self.active_goal_world[1]
            goal_marker.pose.position.z = 0.15
            goal_marker.pose.orientation.w = 1.0
            goal_marker.color.r = 1.0
        else:
            goal_marker.action = Marker.DELETE
        ma.markers.append(goal_marker)

        for idx, (gx, gy) in enumerate(self.pending_frontiers[:50], start=1):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "candidate_frontiers"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = gx
            marker.pose.position.y = gy
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.a = 0.85
            marker.color.b = 1.0
            marker.color.g = 0.6
            ma.markers.append(marker)

        for idx in range(len(self.pending_frontiers[:50]) + 1, 60):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame
            marker.ns = "candidate_frontiers"
            marker.id = idx
            marker.action = Marker.DELETE
            ma.markers.append(marker)

        path_marker = Marker()
        path_marker.header.stamp = stamp
        path_marker.header.frame_id = frame
        path_marker.ns = "frontier_links"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.scale.x = 0.03
        path_marker.color.a = 0.8
        path_marker.color.g = 0.8
        path_marker.color.b = 1.0
        if self.active_goal_world is not None:
            pose = self._get_robot_pose()
            if pose is not None:
                path_marker.action = Marker.ADD
                start = Point(x=float(pose[0]), y=float(pose[1]), z=0.03)
                end = Point(
                    x=float(self.active_goal_world[0]),
                    y=float(self.active_goal_world[1]),
                    z=0.03,
                )
                path_marker.points = [start, end]
            else:
                path_marker.action = Marker.DELETE
        else:
            path_marker.action = Marker.DELETE
        ma.markers.append(path_marker)

        self.marker_pub.publish(ma)

    def _dispatch_next_goal(self) -> bool:
        while self.pending_frontiers:
            goal = self.pending_frontiers.pop(0)
            self.active_goal_world = goal
            self.active_goal_sent_at = 0.0
            self.active_goal_last_publish_at = 0.0
            self._publish_goal(goal[0], goal[1])
            self.state = State.WAIT_FOR_NAVIGATION
            self.get_logger().info(
                f"Dispatching frontier goal ({goal[0]:.2f}, {goal[1]:.2f}) "
                f"to planner; {len(self.pending_frontiers)} candidate(s) remain."
            )
            self._publish_viz_markers()
            return True
        return False

    def control_loop(self):
        if self.state == State.FIND_FRONTIER:
            self._handle_find_frontier()
        elif self.state == State.WAIT_FOR_NAVIGATION:
            self._handle_wait_for_navigation()
        elif self.state == State.DONE:
            self._handle_done()

    def _handle_find_frontier(self):
        if self.occ_grid is None or self.planning_grid is None:
            return
        if not self._map_updated:
            return

        self._map_updated = False
        self.pending_frontiers = self._find_frontiers()
        if not self.pending_frontiers and self._visited_goals:
            self.get_logger().info(
                f"No new frontiers found. Clearing {len(self._visited_goals)} visited goals and retrying."
            )
            self._visited_goals.clear()
            self.pending_frontiers = self._find_frontiers()

        if not self.pending_frontiers:
            if self._get_robot_pose() is None:
                return
            self.get_logger().info("Exploration complete: no reachable frontier goals found.")
            self.active_goal_world = None
            self._publish_viz_markers()
            self.state = State.DONE
            return

        self._dispatch_next_goal()

    def _handle_wait_for_navigation(self):
        if self.active_goal_world is None:
            self.state = State.FIND_FRONTIER
            self._map_updated = True
            return

        now = self.get_clock().now().nanoseconds / 1e9
        republish_period = float(self.get_parameter("goal_republish_period_s").value)
        ack_timeout = float(self.get_parameter("planner_ack_timeout_s").value)

        if self.planner_state == "succeeded" and self._goal_matches_status(
            self.active_goal_world
        ):
            goal = self.active_goal_world
            self._visited_goals.append(goal)
            self.get_logger().info(
                f"Planner reached frontier goal ({goal[0]:.2f}, {goal[1]:.2f})."
            )
            self.active_goal_world = None
            self.active_goal_sent_at = 0.0
            self.active_goal_last_publish_at = 0.0
            self.pending_frontiers = []
            self._map_updated = True
            self._publish_viz_markers()
            self.state = State.FIND_FRONTIER
            return

        if self.planner_state == "failed" and self._goal_matches_status(
            self.active_goal_world
        ):
            goal = self.active_goal_world
            self._visited_goals.append(goal)
            self.get_logger().warn(
                f"Planner failed for frontier goal ({goal[0]:.2f}, {goal[1]:.2f}): "
                f"{self.planner_message or 'unknown_reason'}"
            )
            self.active_goal_world = None
            self.active_goal_sent_at = 0.0
            self.active_goal_last_publish_at = 0.0
            if self._map_updated:
                self.pending_frontiers = []
                self._publish_viz_markers()
                self.state = State.FIND_FRONTIER
            elif not self._dispatch_next_goal():
                self._map_updated = True
                self.state = State.FIND_FRONTIER
                self._publish_viz_markers()
            return

        if self.planner_state in {"planning", "navigating"} and self._goal_matches_status(
            self.active_goal_world
        ):
            return

        if (now - self.active_goal_last_publish_at) >= republish_period:
            self._publish_goal(self.active_goal_world[0], self.active_goal_world[1])

        if self.active_goal_sent_at > 0.0 and (now - self.active_goal_sent_at) >= ack_timeout:
            goal = self.active_goal_world
            self._visited_goals.append(goal)
            self.get_logger().warn(
                f"Planner did not acknowledge frontier goal ({goal[0]:.2f}, {goal[1]:.2f}); "
                "trying a different candidate."
            )
            self.active_goal_world = None
            self.active_goal_sent_at = 0.0
            self.active_goal_last_publish_at = 0.0
            if self._map_updated:
                self.pending_frontiers = []
                self._publish_viz_markers()
                self.state = State.FIND_FRONTIER
            elif not self._dispatch_next_goal():
                self._map_updated = True
                self.state = State.FIND_FRONTIER
                self._publish_viz_markers()

    def _handle_done(self):
        if not self._done_logged:
            self.get_logger().info("Exploration complete.")
            self._done_logged = True


def main():
    rclpy.init()
    node = FrontierPotentialFieldExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
