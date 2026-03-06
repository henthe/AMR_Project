-we need to solve assignment 3
-we already did a previous version of our solution (frontier_pf_explorer_node.py) which you should not look at so dont look at old commits!
-the worklfow we have in mind includes the following:

-the robot starts with an unexplored map (gridcells) in which it is centered
-for slam, the built in slam toolbox should be used with the parameters in slam_toolbox_override.yaml
-the robot will iteratively do path planning (from assignment 1) to unexplored cells (choose the one which is closest to him and is in a cluster of at least 5)
-so after the robot has reached a goal, it will assign a new goal (unexplored cell)
-if the robot is stuck, he has to perform a random walk (rotate around the own axis for 6 seconds with 0.8 rad/s and move forward for 6s with 0.7 m/s)
>the robot is stuck if he has not moved more than 0.05m in the last 10 seconds
-the algorithm terminates (explortaion is finished) if the robot cant create a path to the next chosen goal (e.g. he is a room with no exit option)

logging (the robot should log different steps):
-when he choses a new goal and when he has reached a goal
-when he choses a new milestone from the path
-when he starts randomwalk (and also which phase exactly: turn, move)
-when he has finished randomwalk


rviz:
-we want to always display the goal and waypoints of the current path in rviz so this needs to be published