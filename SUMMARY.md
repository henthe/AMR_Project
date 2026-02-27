# Project Summary

> Auto-generated codebase summary for AI context. Keep in sync with code changes.

## Overview

University bachelor thesis project — an **Autonomous Mobile Robot (AMR)** navigation stack built as a single ROS 2 (Humble) Python package called `my_planner_pkg`. The package provides localization, path planning, and autonomous exploration for a robot equipped with a 2D LiDAR and wheel odometry.

## Package Structure

```
AMR_Project/                          # ROS 2 ament_python package root
├── package.xml                       # ROS 2 package manifest
├── setup.py                          # Entry points for all 4 nodes
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
│   ├── frontier_explorer_node.py     # Frontier exploration via Nav2
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

### 3. `frontier_explorer` — Frontier Exploration (Nav2-based)
- **Entry point:** `frontier_explorer = my_planner_pkg.frontier_explorer_node:main`
- **Class:** `FrontierExplorer`
- **Purpose:** Autonomous exploration by detecting frontiers (boundaries between known-free and unknown space) and sending goals to Nav2.
- **How it works:**
  1. Optional warmup phase: rotates in place to let SLAM Toolbox build an initial map.
  2. Gates on: TF available, map large enough, Nav2 server ready.
  3. Finds frontier cells (free cells adjacent to unknown cells), clusters them via BFS.
  4. Scores clusters by `w_size * cluster_size - w_dist * distance_to_robot`, picks the best.
  5. Sends the goal to Nav2's `NavigateToPose` action server.
  6. Blacklists goals that time out, get rejected, or fail repeatedly.
- **Subscribes:** `/map` (OccupancyGrid)
- **Publishes:** `/cmd_vel` (Twist — warmup only)
- **Action client:** `navigate_to_pose` (NavigateToPose)
- **TF:** Reads `map → base_link`
- **Key parameters:** `min_frontier_cluster_size`, `goal_blacklist_radius_m`, `score_distance_weight`, `score_size_weight`, `warmup_enable`, `min_map_size_x_m/y_m`

### 4. `frontier_pf_explorer` — Frontier Exploration (Potential-Field-based)
- **Entry point:** `frontier_pf_explorer = my_planner_pkg.frontier_pf_explorer_node:main`
- **Class:** `FrontierPotentialFieldExplorer`
- **Purpose:** Same frontier exploration concept but drives toward frontiers using a potential field controller instead of Nav2 — fully self-contained, no Nav2 dependency.
- **How it works:**
  1. Warmup phase (rotate + slight forward motion).
  2. Picks the frontier cluster point farthest from the robot, scored by `2.0 * size + 0.6 * distance`.
  3. Drives toward the goal using attractive + repulsive potential fields from LiDAR (same approach as planner_pf_node).
  4. **Tangential wall-sliding force:** When repulsive and attractive forces oppose each other (corners/dead-ends), a perpendicular component is added to the repulsive force so the robot slides along walls instead of oscillating.
  5. **Front-blocked rotation:** When an obstacle is within stop_range in the front cone, forward motion is suppressed but the robot can still rotate to escape (no hard stop).
  6. Low-pass filters heading and angular velocity commands for smooth motion.
  7. Stuck detection: if path traveled in a time window is below threshold, triggers recovery.
  8. **Oscillation detection:** If the angular command rapidly alternates sign (5+ reversals in 2.5 s), recovery is triggered immediately — faster than general stuck detection.
  9. Recovery behavior: back up, then turn toward the more open side (based on LiDAR).
  10. Periodically reselects frontier goal (every N seconds).
  11. Clears blacklist and retries if all candidates are blacklisted.
- **Subscribes:** `/map` (OccupancyGrid), `/scan` (LaserScan)
- **Publishes:** `/cmd_vel` (Twist)
- **TF:** Reads `map → base_link`
- **Key parameters:** `k_att`, `k_rep`, `repulsion_range_m`, `max_lin`, `max_ang`, `goal_reached_dist_m`, `stuck_check_enable`, `recovery_enable`, `reselect_goal_every_s`

## Shared Algorithms & Patterns

- **Potential field controller** — Used in nodes 1 and 4. Attractive force toward goal, repulsive forces from nearby LiDAR points. In node 4, a tangential component is added when forces oppose (corner escape), and front-blocked only suppresses linear speed (rotation continues). Linear speed scaled by heading error cosine.
- **Frontier detection** — Used in nodes 3 and 4. Iterates occupancy grid, finds free cells with unknown neighbors, clusters via BFS flood-fill.
- **Map loading from YAML/PGM** — Used in nodes 1 and 2. Parses slam_toolbox-style map files (image path, resolution, origin, thresholds). Node 2 also supports receiving the map via topic.
- **TF lookups** — All nodes use tf2_ros to get the robot pose in the map frame.

## External Dependencies

| Dependency | Used by |
|---|---|
| `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs` | All nodes |
| `tf2_ros`, `tf_transformations` | All nodes |
| `visualization_msgs` | planner_pf_node |
| `nav2_msgs` | frontier_explorer_node |
| `numpy` | All nodes |
| `scipy.ndimage` | planner_pf_node (optional, for obstacle inflation) |
| `pyyaml` | planner_pf_node, particle_filter_localization |
| `Pillow` / `imageio` | planner_pf_node, particle_filter_localization (map image loading) |

## Configuration Files

- **`config/slam_toolbox_override.yaml`** — Overrides for SLAM Toolbox: sim time enabled, map update interval 0.05s, laser range 0.12–5.6m.
- **`maps/first_try.yaml`** — Map metadata: 0.05 m/px resolution, origin at (-1.55, -4.93, 0).

## Typical Usage Scenarios

1. **Localization + goal navigation:** Run `pf_localization` (provides `map → odom` TF) + `planner_pf_node` (navigates to goals). Requires a pre-built map.
2. **Exploration with Nav2:** Run SLAM Toolbox + Nav2 stack + `frontier_explorer`. The explorer autonomously picks frontiers and delegates navigation to Nav2.
3. **Exploration without Nav2:** Run SLAM Toolbox + `frontier_pf_explorer`. The explorer handles both frontier selection and driving via potential fields.
