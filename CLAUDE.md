# Project Guidelines

- This repo is a university project
- The code needs to work; coding tests or advanced use cases are not required
- Keep the code simple and clean
- For ROS2 documentation, use the Context7 MCP server

# Active Focus

- The only actively developed node is `my_planner_pkg/frontier_pf_explorer_node.py` — other nodes are not currently in use

# Testing

- ROS 2 is **not installed** on this development PC — code cannot be built or run here
- Code will be tested on a separate machine afterwards
- Because of this: after finishing any coding task, **always re-read and carefully verify** the changed code to make sure it will work (correct imports, matching topic names, proper types, no typos, consistent logic)

## Test commands (run on the ROS 2 machine)

Terminal 1 — SLAM Toolbox (online async mapping mode):
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Terminal 2 — Frontier explorer:
```
ros2 run my_planner_pkg frontier_pf_explorer --ros-args \
  -p use_sim_time:=true \
  -p max_lin:=0.75 \
  -p max_ang:=2.0 \
  -p reselect_goal_every_s:=2.5 \
  -p k_rep:=1.15 \
  -p repulsion_range_m:=0.90 \
  -p stop_range_m:=0.22 \
  -p k_att:=0.85 \
  -p lin_scale_on_heading:=1.4
```

# Summary File

- `SUMMARY.md` contains a detailed codebase overview for AI context
- **After any code change** (new files, modified nodes, changed topics/parameters, added dependencies), update `SUMMARY.md` to reflect the current state of the code
- Keep the summary accurate — it is the primary reference for understanding the project
