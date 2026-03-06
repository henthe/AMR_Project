# Project Guidelines

- This repo is a university project (AMR — Autonomous Mobile Robots)
- The code needs to work; coding tests or advanced use cases are not required
- Keep the code simple and clean
- For ROS2 documentation, use the Context7 MCP server

# Assignments

The project has three assignments (see README.md for full descriptions):

1. **Path & Motion Planning** (SOLVED) — `my_planner_pkg/planner_pf_node.py`
   - A* global planner + potential field local planner with waypoint following
   - Loads a static map from YAML, plans A* path, extracts waypoints, follows them with attractive/repulsive potential fields using LaserScan

2. **Localisation** (SOLVED) — `my_planner_pkg/particle_filter_localization.py`
   - Monte Carlo Localization (particle filter)
   - Thrun et al. motion model, beam-based measurement model with raycast, systematic resampling with random injection

3. **Environment Exploration** (CURRENT) 
   - Frontier-based exploration combined with SLAM (slam_toolbox)
   - Select poses at the boundary between explored and unexplored regions
   - **IMPORTANT: Must build on Assignment 1's solution** (`planner_pf_node.py`) — reuse/adapt the potential field planner for navigating to frontier goals

# Active Focus

- The actively developed node is (Assignment 3)
- `planner_pf_node.py` is the foundation to build upon for the explorer's navigation

# Development Environment

- **ROS2 distro:** Humble (on Ubuntu 22.04)
- ROS 2 is **not installed** on this development PC — code cannot be built or run here
- Code will be tested on a separate machine with the full ROS2 workspace
- Because of this: after finishing any coding task, **always re-read and carefully verify** the changed code to make sure it will work (correct imports, matching topic names, proper types, no typos, consistent logic)

# Target Machine Setup

- **Workspace:** `~/ros2_ws/` (colcon workspace root)
- **Source packages live in:** `~/ros2_ws/src/`
- **Build command:** `cd ~/ros2_ws && colcon build` (always run from workspace root)
- **Source after build:** `source ~/ros2_ws/install/setup.bash`
- **ROS2 base setup:** `source /opt/ros/humble/setup.bash` (should be in `.bashrc`)

## Key Dependencies (apt)

- `ros-humble-gazebo-ros`, `ros-humble-turtlebot3-gazebo`
- `ros-humble-xacro`, `ros-humble-tf2-geometry-msgs`, `ros-humble-tf-transformations`
- `ros-humble-joint-state-publisher-gui`, `ros-humble-joint-state-publisher`
- `ros-humble-joy-linux`, `ros-humble-urg-node`, `ros-humble-urg-node-msgs`
- `ros-humble-rosbag2-*`, `ros-humble-ros2bag`
- `python3-colcon-common-extensions`

## HBRS-AMR Repositories (in `~/ros2_ws/src/`)

| Repository | Branch | Purpose |
|---|---|---|
| `HBRS-AMR/Robile` | `ros2` | Core packages to drive the robot |
| `HBRS-AMR/robile_description` | `ros2` | Robot URDF/description |
| `HBRS-AMR/robile_gazebo` | `ros2` | Gazebo simulation |
| `HBRS-AMR/robile_navigation` | `ros2` | Navigation stack (nav2) |
| `HBRS-AMR/robile_interfaces` | `main` | Custom message/service definitions |

**Note:** In `robile_navigation/config/nav2_params.yaml`, the `lattice_filepath` under `planner_server → GridBased` must be updated to the absolute path on the target machine.

# GitHub MCP Server

Use the GitHub MCP server tools for interacting with the upstream HBRS-AMR repositories:

- **`get_file_contents`** — Read files from HBRS-AMR repos (e.g., launch files, configs, message definitions) without needing them cloned locally
- **`search_code`** — Search for specific code patterns, topic names, message types, or parameters across HBRS-AMR repos
- **`list_branches` / `list_tags`** — Check available branches/tags (e.g., confirm `ros2` branch exists)
- **`list_commits`** — Check recent changes to upstream repos

Example use cases:
- Look up a custom message definition: `get_file_contents` from `HBRS-AMR/robile_interfaces`
- Find which launch file spawns a specific node: `search_code` in `HBRS-AMR/robile_gazebo`
- Check how a topic is published in the robot driver: `search_code` in `HBRS-AMR/Robile`
- Review nav2 parameter configuration: `get_file_contents` from `HBRS-AMR/robile_navigation`

# Summary File

- `SUMMARY.md` contains a detailed codebase overview for AI context
- **After any code change** (new files, modified nodes, changed topics/parameters, added dependencies), update `SUMMARY.md` to reflect the current state of the code
- Keep the summary accurate — it is the primary reference for understanding the project
