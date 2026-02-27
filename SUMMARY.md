# Project Summary

> Auto-generated codebase summary for AI context. Keep in sync with code changes.

## Overview

University bachelor thesis project — an **Autonomous Mobile Robot (AMR)** navigation stack built as a single ROS 2 (Humble) Python package called `my_planner_pkg`. The package provides localization, path planning, and autonomous exploration for a robot equipped with a 2D LiDAR and wheel odometry.

## Package Structure

```
AMR_Project/                          # ROS 2 ament_python package root
├── package.xml                       # ROS 2 package manifest
├── setup.py                          # Entry points for all 3 nodes
├── setup.cfg
├── config/
│   └── slam_toolbox_override.yaml    # SLAM Toolbox parameter overrides
├── maps/
│   ├── first_try.yaml                # Pre-built map metadata (0.05 m/px)
│   └── first_try.pgm                 # Pre-built occupancy grid image
├── my_planner_pkg/                   # Python source package
│   ├── __init__.py
│   ├── planner_pf_node.py            # A* planner + potential field controller
│   ├── particle_filter_localization.py  # Monte Carlo Localization (particle filter)
│   └── frontier_pf_explorer_node.py  # Frontier exploration via potential fields
└── test/                             # Default ament lint tests (copyright, flake8, pep257)
```

## ROS 2 Nodes

### 1. `planner_pf_node` — A* Global Planner + Potential Field Local Planner
- **Entry point:** `planner_pf_node = my_planner_pkg.planner_pf_node:main`
- **Class:** `GlobalAStarPotentialFieldNode`
- **Purpose:** Navigate to a user-specified goal pose using a pre-loaded static map.
- **How it works:**
  1. Loads a static map from a YAML/PGM file at startup.
  2. Inflates obstacles (configurable radius, uses scipy if available).
  3. On receiving a goal, runs A* on the inflated grid to get a cell path.
  4. Extracts sparse waypoints (turning points + every-N-cells sampling).
  5. A 20 Hz control loop drives toward the current waypoint using an attractive potential field force toward the waypoint and repulsive forces from LiDAR scan obstacles.
  6. Skips waypoints that the robot has already passed.
- **Subscribes:** `/goal_pose` (PoseStamped), `/scan` (LaserScan)
- **Publishes:** `/cmd_vel` (Twist), `/planner_markers` (MarkerArray — path + waypoints for RViz)
- **TF:** Reads `map → base_link`
- **Key parameters:** `map_yaml`, `inflation_radius_m`, `k_att`, `k_rep`, `repulsion_range_m`, `stop_range_m`, `max_lin`, `max_ang`, `wp_reached_dist_m`

### 2. `pf_localization` — Particle Filter (Monte Carlo Localization)
- **Entry point:** `pf_localization = my_planner_pkg.particle_filter_localization:main`
- **Class:** `ParticleFilterLocalization`
- **Purpose:** Localize the robot on a known map using a particle filter.
- **How it works:**
  1. Loads a static map from YAML or subscribes to `/map` topic.
  2. Initializes particles globally (uniform on free cells) or as a Gaussian around a given pose.
  3. **Motion model:** Thrun et al. 3-phase odometry model (rot1 → translation → rot2) with 4 noise parameters (alpha1–4).
  4. **Measurement model:** Beam model comparing LiDAR ranges against raycasted expected ranges on the map. Uses z_hit (Gaussian) + z_rand (uniform) mixture.
  5. **Resampling:** Systematic resampling triggered when N_eff drops below threshold, with random particle injection to prevent particle deprivation.
  6. Publishes the `map → odom` TF transform computed from the weighted particle mean.
- **Subscribes:** `/scan` (LaserScan), `/odom` (Odometry), `/initialpose` (PoseWithCovarianceStamped)
- **Publishes:** `/map` (OccupancyGrid, transient local), `/particle_cloud` (PoseArray), `/pf_pose` (PoseWithCovarianceStamped)
- **TF:** Broadcasts `map → odom`, reads `odom → base_link` and `base_link → laser`
- **Key parameters:** `map_yaml`, `num_particles`, `sigma_z`, `alpha1`–`alpha4`, `laser_step`, `ray_step`, `init_mode`
- **Language note:** Code comments are in German.

### 3. `frontier_pf_explorer` — Frontier Exploration (Potential-Field-based)
- **Entry point:** `frontier_pf_explorer = my_planner_pkg.frontier_pf_explorer_node:main`
- **Class:** `FrontierPotentialFieldExplorer`
- **Purpose:** Autonomous frontier-based exploration using SLAM. Detects frontiers on the SLAM-generated map, plans A* paths, and navigates via potential fields. Fully self-contained — no Nav2 dependency.
- **Builds on:** Assignment 1 (`planner_pf_node.py`) — imports `astar`, `extract_waypoints`, `clamp`, `wrap_angle`.
- **State machine:** `WARMUP → FIND_FRONTIER → NAVIGATE → RECOVERY → DONE`
- **How it works:**
  1. **Warmup phase** (~5 s): rotates in place so SLAM Toolbox builds an initial map.
  2. **Frontier detection:** finds free cells (value 0) adjacent to unknown cells (value −1) on the OccupancyGrid, clusters them via BFS, filters by minimum size.
  3. **Frontier scoring:** `w_size × cluster_size − w_dist × distance_to_robot`. Picks the highest-scoring non-blacklisted cluster; computes centroid as goal.
  4. **Path planning:** builds an inflated planning grid from the SLAM map (unknown → obstacle), runs A* from robot to frontier centroid, extracts sparse waypoints.
  5. **Potential field navigation:** attractive force toward current waypoint + repulsive forces from LiDAR obstacles. Skips passed waypoints.
  6. **Tangential wall-sliding:** when attractive and repulsive forces oppose (dot product < −0.3), a perpendicular component is added to the repulsive force so the robot slides along walls instead of oscillating.
  7. **Front-blocked rotation:** when an obstacle is within `stop_range_m` in the front cone, forward motion is suppressed but the robot can still rotate to escape.
  8. **Stuck detection:** if total distance traveled in a sliding time window falls below threshold, triggers recovery.
  9. **Recovery:** back up briefly, then rotate toward the more open side (LiDAR left vs right range sums).
  10. **Periodic reselection:** every `reselect_goal_every_s` seconds, picks a new frontier goal using the latest SLAM map.
  11. **Blacklist management:** unreachable or stuck-at goals are blacklisted; blacklist is cleared if all candidates become blacklisted.
- **Subscribes:** `/map` (OccupancyGrid, QoS: reliable + transient local), `/scan` (LaserScan, QoS: sensor data)
- **Publishes:** `/cmd_vel` (Twist)
- **TF:** Reads `map → base_link`
- **Key parameters:** `inflation_radius_m`, `k_att`, `k_rep`, `repulsion_range_m`, `stop_range_m`, `k_heading`, `max_lin`, `max_ang`, `goal_reached_dist_m`, `min_frontier_cluster_size`, `score_size_weight`, `score_distance_weight`, `reselect_goal_every_s`, `warmup_duration_s`, `waypoint_every_n_cells`, `stuck_window_s`, `stuck_threshold_m`, `goal_blacklist_radius_m`, `recovery_back_duration_s`, `recovery_turn_duration_s`

## Shared Algorithms & Patterns

- **A* path planning** — Defined in `planner_pf_node.py`, imported by `frontier_pf_explorer_node.py`. Standard A* on an inflated 2D occupancy grid (0=free, 100=obstacle). 8-connected neighborhoods, Euclidean heuristic.
- **Waypoint extraction** — Defined in `planner_pf_node.py`, imported by `frontier_pf_explorer_node.py`. Reduces dense cell path to sparse waypoints via turning-point detection + every-N-cells sampling.
- **Potential field controller** — Used in nodes 1 and 3. Attractive force toward goal, repulsive forces from nearby LiDAR points. In node 3, a tangential component is added when forces oppose (corner escape), and front-blocked only suppresses linear speed (rotation continues). Linear speed scaled by heading error cosine.
- **Frontier detection** — Used in node 3. Vectorized NumPy detection of free cells adjacent to unknown cells, then BFS clustering.
- **Map loading from YAML/PGM** — Used in nodes 1 and 2. Parses slam_toolbox-style map files (image path, resolution, origin, thresholds). Node 2 also supports receiving the map via topic.
- **TF lookups** — All nodes use tf2_ros to get the robot pose in the map frame.

## External Dependencies

| Dependency | Used by |
|---|---|
| `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs` | All nodes |
| `tf2_ros`, `tf_transformations` | All nodes |
| `visualization_msgs` | planner_pf_node |
| `numpy` | All nodes |
| `scipy.ndimage` | planner_pf_node, frontier_pf_explorer_node (optional, for obstacle inflation) |
| `pyyaml` | planner_pf_node, particle_filter_localization |
| `Pillow` / `imageio` | planner_pf_node, particle_filter_localization (map image loading) |

## Configuration Files

- **`config/slam_toolbox_override.yaml`** — Overrides for SLAM Toolbox: sim time enabled, map update interval 0.05s, laser range 0.12–5.6m.
- **`maps/first_try.yaml`** — Map metadata: 0.05 m/px resolution, origin at (-1.55, -4.93, 0).

## Typical Usage Scenarios

1. **Localization + goal navigation:** Run `pf_localization` (provides `map → odom` TF) + `planner_pf_node` (navigates to goals). Requires a pre-built map.
2. **Autonomous exploration:** Run SLAM Toolbox + `frontier_pf_explorer`. The explorer handles frontier detection, A* path planning, and potential field driving — all in one node.
