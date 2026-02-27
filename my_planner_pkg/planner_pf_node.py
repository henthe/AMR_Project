#!/usr/bin/env python3
import math
import heapq
import yaml
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


# ----------------------------
# Map loading (YAML + PGM)
# ----------------------------
def load_map_from_yaml(yaml_path: str):
    with open(yaml_path, "r") as f:
        info = yaml.safe_load(f)

    image_path = info["image"]
    if not image_path.startswith("/"):
        import os
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    resolution = float(info["resolution"])
    origin_list = info["origin"]  # [x,y,yaw]
    origin = (float(origin_list[0]), float(origin_list[1]), float(origin_list[2]))

    occupied_thresh = float(info.get("occupied_thresh", 0.65))
    free_thresh = float(info.get("free_thresh", 0.196))
    negate = int(info.get("negate", 0))

    try:
        import imageio.v2 as imageio
        img = imageio.imread(image_path)
    except Exception:
        from PIL import Image
        img = np.array(Image.open(image_path))

    if img.ndim == 3:
        img = img[:, :, 0]
    img = img.astype(np.uint8)

    if negate == 1:
        img = 255 - img

    intensity = img.astype(np.float32) / 255.0
    p_occ = 1.0 - intensity

    occ_grid = np.full(img.shape, 255, dtype=np.uint8)  # unknown
    occ_grid[p_occ >= occupied_thresh] = 100
    occ_grid[p_occ <= free_thresh] = 0

    return occ_grid, resolution, origin


# ----------------------------
# A* global planning
# ----------------------------
@dataclass(order=True)
class PQItem:
    f: float
    g: float
    node: Tuple[int, int]


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    allow_diagonal: bool = True,
) -> Optional[List[Tuple[int, int]]]:
    H, W = grid.shape

    def in_bounds(n):
        r, c = n
        return 0 <= r < H and 0 <= c < W

    def is_free(n):
        r, c = n
        return grid[r, c] == 0

    if not in_bounds(start) or not in_bounds(goal):
        return None
    if not is_free(start) or not is_free(goal):
        return None

    if allow_diagonal:
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]
    else:
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    open_heap: List[PQItem] = []
    heapq.heappush(open_heap, PQItem(f=heuristic(start, goal), g=0.0, node=start))

    came_from = {}
    gscore = {start: 0.0}
    closed = set()

    while open_heap:
        cur = heapq.heappop(open_heap)
        if cur.node in closed:
            continue
        closed.add(cur.node)

        if cur.node == goal:
            path = [cur.node]
            n = cur.node
            while n in came_from:
                n = came_from[n]
                path.append(n)
            path.reverse()
            return path

        cr, cc = cur.node
        for dr, dc in nbrs:
            nn = (cr + dr, cc + dc)
            if not in_bounds(nn):
                continue
            if not is_free(nn):
                continue

            step = math.hypot(dr, dc)
            tentative_g = gscore[cur.node] + step

            if nn not in gscore or tentative_g < gscore[nn]:
                gscore[nn] = tentative_g
                came_from[nn] = cur.node
                f = tentative_g + heuristic(nn, goal)
                heapq.heappush(open_heap, PQItem(f=f, g=tentative_g, node=nn))

    return None


# ----------------------------
# Path post processing
# ----------------------------
def extract_waypoints(
    cell_path: List[Tuple[int, int]],
    take_every_n: int = 10,
    include_turning_points: bool = True,
    min_segment_len_cells: int = 4,
) -> List[Tuple[int, int]]:
    if not cell_path:
        return []

    wps = [cell_path[0]]
    if include_turning_points and len(cell_path) >= 3:
        last_added_idx = 0
        for i in range(1, len(cell_path) - 1):
            r0, c0 = cell_path[i - 1]
            r1, c1 = cell_path[i]
            r2, c2 = cell_path[i + 1]
            d1 = (r1 - r0, c1 - c0)
            d2 = (r2 - r1, c2 - c1)
            if d2 != d1 and (i - last_added_idx) >= min_segment_len_cells:
                wps.append((r1, c1))
                last_added_idx = i

    if take_every_n > 0:
        for i in range(0, len(cell_path), take_every_n):
            if cell_path[i] != wps[-1]:
                wps.append(cell_path[i])

    if cell_path[-1] != wps[-1]:
        wps.append(cell_path[-1])

    out, seen = [], set()
    for w in wps:
        if w not in seen:
            out.append(w)
            seen.add(w)
    return out


# ----------------------------
# Grid/world transforms
# ----------------------------
def world_to_grid(x: float, y: float, origin: Tuple[float, float, float], resolution: float, H: int):
    ox, oy, _ = origin
    c = int(math.floor((x - ox) / resolution))
    r_from_bottom = int(math.floor((y - oy) / resolution))
    r = (H - 1) - r_from_bottom
    return (r, c)


def grid_to_world(r: int, c: int, origin: Tuple[float, float, float], resolution: float, H: int):
    ox, oy, _ = origin
    r_from_bottom = (H - 1) - r
    x = ox + (c + 0.5) * resolution
    y = oy + (r_from_bottom + 0.5) * resolution
    return (x, y)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GlobalAStarPotentialFieldNode(Node):
    def __init__(self):
        super().__init__("global_astar_potential_field")

        # Params
        self.declare_parameter("map_yaml", "first_try.yaml")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("inflation_radius_m", 0.4)
        self.declare_parameter("unknown_is_obstacle", True)

        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("waypoint_every_n_cells", 30)
        self.declare_parameter("waypoints_include_turns", True)

        self.declare_parameter("k_att", 1.0)
        self.declare_parameter("k_rep", 0.8)
        self.declare_parameter("repulsion_range_m", 0.5)
        self.declare_parameter("stop_range_m", 0.20)

        self.declare_parameter("k_heading", 1.8)
        self.declare_parameter("max_lin", 1.0)
        self.declare_parameter("max_ang", 1.5)
        self.declare_parameter("wp_reached_dist_m", 0.20)
        self.declare_parameter("lin_scale_on_heading", 1.0)

        # Load map
        map_yaml = self.get_parameter("map_yaml").value
        self.occ, self.resolution, self.origin = load_map_from_yaml(map_yaml)
        self.H, self.W = self.occ.shape

        self.unknown_is_obstacle = self.get_parameter("unknown_is_obstacle").value
        self.inflation_radius_m = self.get_parameter("inflation_radius_m").value
        self.grid = self._build_planning_grid(self.occ)

        self.goal_world: Optional[Tuple[float, float]] = None
        self.global_path_cells: List[Tuple[int, int]] = []
        self.waypoints_world: List[Tuple[float, float]] = []
        self.wp_index = 0

        self.scan: Optional[LaserScan] = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS pubs/subs
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/planner_markers", 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Loaded map {self.W}x{self.H}, res={self.resolution:.3f}, origin={self.origin}"
        )

    def _build_planning_grid(self, occ: np.ndarray) -> np.ndarray:
        grid = np.copy(occ)

        if self.unknown_is_obstacle:
            grid[grid == 255] = 100
        else:
            grid[grid == 255] = 0

        inflation_cells = int(math.ceil(self.inflation_radius_m / self.resolution))
        if inflation_cells <= 0:
            grid[grid != 0] = 100
            return grid

        obstacle = (grid == 100).astype(np.uint8)
        try:
            from scipy.ndimage import binary_dilation
            structure = np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1), dtype=bool)
            inflated = binary_dilation(obstacle.astype(bool), structure=structure)
            grid[inflated] = 100
            grid[~inflated] = 0
        except Exception:
            inflated = np.zeros_like(obstacle, dtype=np.uint8)
            ys, xs = np.where(obstacle == 1)
            for y, x in zip(ys, xs):
                y0 = max(0, y - inflation_cells)
                y1 = min(self.H, y + inflation_cells + 1)
                x0 = max(0, x - inflation_cells)
                x1 = min(self.W, x + inflation_cells + 1)
                inflated[y0:y1, x0:x1] = 1
            grid[inflated == 1] = 100
            grid[inflated == 0] = 0

        return grid

    def on_goal(self, msg: PoseStamped):
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Goal: ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f})")
        self.plan_global()

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def get_robot_pose_map(self) -> Optional[Tuple[float, float, float]]:
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tfm = self.tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                rclpy.time.Time()
            )
        except TransformException:
            return None

        x = tfm.transform.translation.x
        y = tfm.transform.translation.y
        q = tfm.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (x, y, yaw)

    def plan_global(self):
        pose = self.get_robot_pose_map()
        if pose is None or self.goal_world is None:
            self.get_logger().warn("Cannot plan yet: missing TF pose or goal.")
            return

        sx, sy, _ = pose
        gx, gy = self.goal_world

        start = world_to_grid(sx, sy, self.origin, self.resolution, self.H)
        goal = world_to_grid(gx, gy, self.origin, self.resolution, self.H)

        allow_diag = self.get_parameter("allow_diagonal").value
        path = astar(self.grid, start, goal, allow_diagonal=allow_diag)
        if path is None:
            self.get_logger().error("A* failed: no path found (start/goal may be in obstacle).")
            self.global_path_cells = []
            self.waypoints_world = []
            self.wp_index = 0
            self.publish_markers()
            return

        self.global_path_cells = path

        every_n = int(self.get_parameter("waypoint_every_n_cells").value)
        include_turns = bool(self.get_parameter("waypoints_include_turns").value)
        wp_cells = extract_waypoints(path, take_every_n=every_n, include_turning_points=include_turns)

        self.waypoints_world = [grid_to_world(r, c, self.origin, self.resolution, self.H) for (r, c) in wp_cells]
        self.wp_index = 0

        self.get_logger().info(f"Global path: {len(path)} cells, waypoints: {len(self.waypoints_world)}")
        self.publish_markers()

    def publish_markers(self):
        ma = MarkerArray()
        frame = self.get_parameter("map_frame").value

        path_m = Marker()
        path_m.header.frame_id = frame
        path_m.header.stamp = self.get_clock().now().to_msg()
        path_m.ns = "global_path"
        path_m.id = 1
        path_m.type = Marker.LINE_STRIP
        path_m.action = Marker.ADD
        path_m.scale.x = 0.03

        path_m.color.r = 0.0
        path_m.color.g = 1.0
        path_m.color.b = 0.0
        path_m.color.a = 1.0



        for (r, c) in self.global_path_cells:
            x, y = grid_to_world(r, c, self.origin, self.resolution, self.H)
            path_m.points.append(Point(x=x, y=y, z=0.0))

        ma.markers.append(path_m)

        wps_m = Marker()
        wps_m.header.frame_id = frame
        wps_m.header.stamp = self.get_clock().now().to_msg()
        wps_m.ns = "waypoints"
        wps_m.id = 2
        wps_m.type = Marker.SPHERE_LIST
        wps_m.action = Marker.ADD
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
        self.marker_pub.publish(ma)

    def control_loop(self):
        if self.scan is None:
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            return

        if not self.waypoints_world or self.wp_index >= len(self.waypoints_world):
            self.cmd_pub.publish(Twist())
            return

        x, y, yaw = pose
        # If we have already passed a waypoint (e.g., skirted around an obstacle),
        # and never got within the reached radius, skip it so we never backtrack.
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

            # Signed progress along the segment current waypoint -> next waypoint.
            # If progress is positive and we've gone beyond the current waypoint by more
            # than the reach distance, consider it missed and skip forward.
            relx = x - wx0
            rely = y - wy0
            progress = (relx * dirx) + (rely * diry)
            dist_to_current = math.hypot(wx0 - x, wy0 - y)

            if progress > wp_reached_dist and dist_to_current > wp_reached_dist:
                total_wps = len(self.waypoints_world)
                self.get_logger().info(
                    f"Skipping waypoint {self.wp_index + 1}/{total_wps} (missed/passed)."
                )
                self.wp_index += 1
                continue
            break

        wx, wy = self.waypoints_world[self.wp_index]

        dist_wp = math.hypot(wx - x, wy - y)
        if dist_wp <= wp_reached_dist:
            total_wps = len(self.waypoints_world)
            reached_idx = self.wp_index

            # Log once per waypoint hit (including the final waypoint/goal)
            if reached_idx >= total_wps - 1:
                gx, gy = (self.goal_world if self.goal_world is not None else (wx, wy))
                self.get_logger().info(
                    f"Goal reached at ({gx:.2f}, {gy:.2f})."
                )
                self.cmd_pub.publish(Twist())
                self.wp_index = total_wps
                return

            self.get_logger().info(
                f"Waypoint {reached_idx + 1}/{total_wps} reached at ({wx:.2f}, {wy:.2f})."
            )

            self.wp_index += 1
            wx, wy = self.waypoints_world[self.wp_index]
            dist_wp = math.hypot(wx - x, wy - y)

        # Attractive in robot frame
        dx_w = wx - x
        dy_w = wy - y
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        dx_r = c * dx_w - s * dy_w
        dy_r = s * dx_w + c * dy_w

        k_att = self.get_parameter("k_att").value
        F_att = np.array([k_att * dx_r, k_att * dy_r], dtype=np.float32)

        # Repulsive from scan
        k_rep = self.get_parameter("k_rep").value
        rep_range = self.get_parameter("repulsion_range_m").value
        stop_range = self.get_parameter("stop_range_m").value

        ranges = np.array(self.scan.ranges, dtype=np.float32)
        angles = self.scan.angle_min + np.arange(len(ranges), dtype=np.float32) * self.scan.angle_increment

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        front_cone = np.abs(angles) < math.radians(35.0)
        if np.any(ranges[front_cone] < stop_range):
            self.cmd_pub.publish(Twist())
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

        ang = clamp(self.get_parameter("k_heading").value * heading_err,
                    -self.get_parameter("max_ang").value,
                    self.get_parameter("max_ang").value)

        heading_factor = max(0.0, math.cos(self.get_parameter("lin_scale_on_heading").value * heading_err))
        lin = clamp(0.6 * heading_factor * self.get_parameter("max_lin").value,
                    0.0,
                    self.get_parameter("max_lin").value)

        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GlobalAStarPotentialFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()