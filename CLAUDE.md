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

# Testing

- ROS 2 is **not installed** on this development PC — code cannot be built or run here
- Code will be tested on a separate machine afterwards
- Because of this: after finishing any coding task, **always re-read and carefully verify** the changed code to make sure it will work (correct imports, matching topic names, proper types, no typos, consistent logic)

# Summary File

- `SUMMARY.md` contains a detailed codebase overview for AI context
- **After any code change** (new files, modified nodes, changed topics/parameters, added dependencies), update `SUMMARY.md` to reflect the current state of the code
- Keep the summary accurate — it is the primary reference for understanding the project
