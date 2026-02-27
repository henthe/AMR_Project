#!/usr/bin/env python3
import math
import time
from collections import deque
from typing import List, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


class FrontierExplorer(Node):
    """
    Frontier exploration node.
    Works with slam_toolbox providing /map and TF map->odom, and Nav2 NavigateToPose.

    Additions vs basic version:
    - Auto warmup motion (rotate) to grow SLAM map without manual driving
    - Hard gating: waits for TF, waits for map big enough, waits for Nav2 server
    - Robust shutdown
    """

    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("occupied_threshold", 65)

        self.declare_parameter("min_frontier_cluster_size", 30)
        self.declare_parameter("goal_blacklist_radius_m", 0.7)

        self.declare_parameter("planning_period_s", 1.0)
        self.declare_parameter("goal_timeout_s", 240.0)
        self.declare_parameter("stuck_goal_retry_limit", 1)

        self.declare_parameter("score_distance_weight", 1.0)
        self.declare_parameter("score_size_weight", 2.0)

        # map readiness in meters
        self.declare_parameter("min_map_size_x_m", 8.0)
        self.declare_parameter("min_map_size_y_m", 8.0)

        # warmup motion
        self.declare_parameter("warmup_enable", True)
        self.declare_parameter("warmup_max_duration_s", 30.0)
        self.declare_parameter("warmup_angular_z", 0.6)
        self.declare_parameter("warmup_linear_x", 0.0)

        # if true, take frame_id from map header
        self.declare_parameter("use_map_header_frame", True)

        self.map_topic = self.get_parameter("map_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)

        self.min_cluster_size = int(self.get_parameter("min_frontier_cluster_size").value)
        self.blacklist_radius = float(self.get_parameter("goal_blacklist_radius_m").value)

        self.period_s = float(self.get_parameter("planning_period_s").value)
        self.goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)
        self.stuck_retry_limit = int(self.get_parameter("stuck_goal_retry_limit").value)

        self.w_dist = float(self.get_parameter("score_distance_weight").value)
        self.w_size = float(self.get_parameter("score_size_weight").value)

        self.min_map_x_m = float(self.get_parameter("min_map_size_x_m").value)
        self.min_map_y_m = float(self.get_parameter("min_map_size_y_m").value)

        self.warmup_enable = bool(self.get_parameter("warmup_enable").value)
        self.warmup_max_duration_s = float(self.get_parameter("warmup_max_duration_s").value)
        self.warmup_angular_z = float(self.get_parameter("warmup_angular_z").value)
        self.warmup_linear_x = float(self.get_parameter("warmup_linear_x").value)

        self.use_map_header_frame = bool(self.get_parameter("use_map_header_frame").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.latest_map: Optional[OccupancyGrid] = None

        self.current_goal: Optional[PoseStamped] = None
        self.current_goal_sent_wall_time = 0.0
        self.current_goal_retry_count = 0
        self.blacklisted_goals: List[Tuple[float, float]] = []

        self.active_nav_goal_handle = None
        self.active_nav_future = None

        self.in_warmup = self.warmup_enable
        self.warmup_start_wall_time = time.time()

        self.timer = self.create_timer(self.period_s, self.tick)
        self.get_logger().info("FrontierExplorer started")

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        if self.use_map_header_frame and msg.header.frame_id:
            self.global_frame = msg.header.frame_id

    def tick(self) -> None:
        if self.latest_map is None:
            return

        if not self.tf_available():
            self.get_logger().info("Waiting for TF map->base_link")
            return

        if self.in_warmup:
            if self.warmup_done():
                self.in_warmup = False
                self.stop_robot()
                self.get_logger().info("Warmup complete, starting exploration")
            else:
                self.publish_warmup_cmd()
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().info("Nav2 NavigateToPose not ready yet")
            return

        if not self.map_big_enough(self.latest_map):
            self.get_logger().info("Map still small, waiting for SLAM to grow map")
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("No robot pose, waiting")
            return
        rx, ry = robot_xy

        if not self.world_in_map(rx, ry, self.latest_map):
            self.get_logger().warn("Robot pose outside /map bounds, waiting for map to include robot")
            return

        if self.current_goal is not None:
            if time.time() - self.current_goal_sent_wall_time > self.goal_timeout_s:
                self.get_logger().warn("Goal timeout, blacklisting and replanning")
                self.blacklist_goal(self.current_goal)
                self.cancel_navigation()
                self.current_goal = None
            return

        frontiers = self.find_frontier_cells(self.latest_map)
        if not frontiers:
            self.get_logger().info("No frontiers found, exploration likely complete")
            return

        clusters = self.cluster_frontiers(frontiers, self.latest_map.info.width, self.latest_map.info.height)
        clusters = [c for c in clusters if len(c) >= self.min_cluster_size]
        if not clusters:
            self.get_logger().info("Only tiny frontiers, exploration likely complete")
            return

        best_goal = self.select_best_goal(clusters, rx, ry, self.latest_map)
        if best_goal is None:
            self.get_logger().warn("No valid goal found, all candidates blacklisted or invalid")
            return

        self.send_goal(best_goal)

    # ---------------- TF ----------------

    def tf_available(self) -> bool:
        try:
            self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
            return True
        except TransformException:
            return False

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ---------------- Warmup ----------------

    def warmup_done(self) -> bool:
        elapsed = time.time() - self.warmup_start_wall_time
        if elapsed >= self.warmup_max_duration_s:
            return True
        if self.latest_map is not None and self.map_big_enough(self.latest_map):
            return True
        return False

    def publish_warmup_cmd(self) -> None:
        cmd = Twist()
        cmd.linear.x = float(self.warmup_linear_x)
        cmd.angular.z = float(self.warmup_angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ---------------- Map gating ----------------

    def map_big_enough(self, grid: OccupancyGrid) -> bool:
        size_x = float(grid.info.width) * float(grid.info.resolution)
        size_y = float(grid.info.height) * float(grid.info.resolution)
        return (size_x >= self.min_map_x_m) and (size_y >= self.min_map_y_m)

    def world_in_map(self, wx: float, wy: float, grid: OccupancyGrid) -> bool:
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        w = grid.info.width
        h = grid.info.height
        return (ox <= wx < ox + w * res) and (oy <= wy < oy + h * res)

    # ---------------- Frontier extraction ----------------

    def grid_index(self, x: int, y: int, width: int) -> int:
        return y * width + x

    def in_bounds(self, x: int, y: int, width: int, height: int) -> bool:
        return 0 <= x < width and 0 <= y < height

    def is_free(self, v: int) -> bool:
        if v < 0:
            return False
        return v <= self.free_threshold

    def is_unknown(self, v: int) -> bool:
        return v < 0

    def find_frontier_cells(self, grid: OccupancyGrid) -> List[Tuple[int, int]]:
        w = grid.info.width
        h = grid.info.height
        data = grid.data

        frontiers: List[Tuple[int, int]] = []
        for y in range(h):
            for x in range(w):
                v = data[self.grid_index(x, y, w)]
                if not self.is_free(v):
                    continue
                if self.has_unknown_neighbor(x, y, w, h, data):
                    frontiers.append((x, y))
        return frontiers

    def has_unknown_neighbor(self, x: int, y: int, w: int, h: int, data: List[int]) -> bool:
        for ny in (y - 1, y, y + 1):
            for nx in (x - 1, x, x + 1):
                if nx == x and ny == y:
                    continue
                if not self.in_bounds(nx, ny, w, h):
                    continue
                if self.is_unknown(data[self.grid_index(nx, ny, w)]):
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
                        if not self.in_bounds(nx, ny, w, h):
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

    def cluster_representative(self, cluster: List[Tuple[int, int]]) -> Tuple[int, int]:
        sx = 0.0
        sy = 0.0
        for x, y in cluster:
            sx += x
            sy += y
        return int(round(sx / len(cluster))), int(round(sy / len(cluster)))

    # ---------------- Goal selection ----------------

    def is_blacklisted_world(self, wx: float, wy: float) -> bool:
        for bx, by in self.blacklisted_goals:
            if math.hypot(wx - bx, wy - by) <= self.blacklist_radius:
                return True
        return False

    def blacklist_goal(self, goal: PoseStamped) -> None:
        self.blacklisted_goals.append((goal.pose.position.x, goal.pose.position.y))

    def select_best_goal(
        self,
        clusters: List[List[Tuple[int, int]]],
        rx: float,
        ry: float,
        grid: OccupancyGrid
    ) -> Optional[PoseStamped]:
        best_score = -1e18
        best_pose: Optional[PoseStamped] = None

        for cluster in clusters:
            cx, cy = self.cluster_representative(cluster)
            wx, wy = self.cell_to_world(cx, cy, grid)

            if not self.world_in_map(wx, wy, grid):
                continue
            if self.is_blacklisted_world(wx, wy):
                continue

            dist = math.hypot(wx - rx, wy - ry)
            size = float(len(cluster))

            score = self.w_size * size - self.w_dist * dist
            if score > best_score:
                best_score = score
                best_pose = self.make_goal_pose(wx, wy, rx, ry, grid.header.frame_id or self.global_frame)

        return best_pose

    def make_goal_pose(self, gx: float, gy: float, rx: float, ry: float, frame_id: str) -> PoseStamped:
        yaw = math.atan2(gy - ry, gx - rx)
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quat(yaw)
        return ps

    # ---------------- Nav2 ----------------

    def send_goal(self, goal_pose: PoseStamped) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server not available")
            return

        self.get_logger().info(
            f"Sending exploration goal x={goal_pose.pose.position.x:.2f} y={goal_pose.pose.position.y:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.current_goal = goal_pose
        self.current_goal_sent_wall_time = time.time()

        self.active_nav_future = self.nav_client.send_goal_async(goal_msg)
        self.active_nav_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected, blacklisting")
            if self.current_goal is not None:
                self.blacklist_goal(self.current_goal)
            self.current_goal = None
            return

        self.active_nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future) -> None:
        res = future.result()
        status = res.status if res is not None else None

        if status in (4, 5, 6):
            self.get_logger().warn(f"Navigation failed with status {status}")
            if self.current_goal is not None:
                self.current_goal_retry_count += 1
                if self.current_goal_retry_count >= self.stuck_retry_limit:
                    self.get_logger().warn("Retry limit reached, blacklisting goal")
                    self.blacklist_goal(self.current_goal)
                    self.current_goal_retry_count = 0
                    self.current_goal = None
                else:
                    self.get_logger().info("Retrying same goal")
                    old = self.current_goal
                    self.current_goal = None
                    self.send_goal(old)
                    return
            else:
                self.current_goal = None
        else:
            self.get_logger().info(f"Navigation finished with status {status}")
            self.current_goal_retry_count = 0
            self.current_goal = None

        self.active_nav_goal_handle = None
        self.active_nav_future = None

    def cancel_navigation(self) -> None:
        if self.active_nav_goal_handle is None:
            return
        try:
            cancel_future = self.active_nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: None)
        except Exception:
            pass
        self.active_nav_goal_handle = None
        self.active_nav_future = None


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()