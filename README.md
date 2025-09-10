This repository is a ROS 2 workspace focused on robot navigation.
It contains packages for autonomous exploration and a custom global path planner.

📂 Packages
1. auto_explo

Provides autonomous exploration capabilities.

Uses SLAM Toolbox for map creation.

Includes the maps generated during SLAM runs.

Integrates the explore-lite package for frontier-based exploration, enabling autonomous map building.

2. custom_planner

Implements a custom global planner plugin.

Uses the A* algorithm for path planning.

Can be plugged into the ROS 2 Navigation stack as an alternative to default planners (e.g., NavFn or SmacPlanner).

🚀 Features

Map building with SLAM Toolbox.

Autonomous frontier-based exploration with explore-lite.

Custom global path planning using A*.

Compatible with the ROS 2 Navigation Stack (Nav2).
