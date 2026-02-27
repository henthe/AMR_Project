A* + Potential Field Planner (ROS 2 Humble)
Build

Vom Workspace Root:

cd ~/ros2_ws
colcon build --packages-select my_planner_pkg
source install/setup.bash

Wichtig: Bei jedem neuen Terminal zuerst sourcen:

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
Run

Da dein TF-Tree nur odom -> base_footprint enthält, muss der Planner im odom Frame laufen:

ros2 run my_planner_pkg planner_pf_node --ros-args \
  -p map_yaml:=$HOME/ros2_ws/src/my_planner_pkg/maps/first_try.yaml \
  -p map_frame:=odom \
  -p base_frame:=base_footprint

Voraussetzung:

Simulator läuft

/tf, /scan, /odom, /cmd_vel existieren

Prüfen mit:

ros2 topic list
Optional: Geschwindigkeit anpassen

Beispiel für schnellere Bewegung:

ros2 run my_planner_pkg planner_pf_node --ros-args \
  -p map_yaml:=$HOME/ros2_ws/src/my_planner_pkg/maps/first_try.yaml \
  -p map_frame:=odom \
  -p base_frame:=base_footprint \
  -p max_lin:=0.7 \
  -p max_ang:=2.0