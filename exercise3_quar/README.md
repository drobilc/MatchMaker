# Exercise 3 - Quarantine edition
To use this exercise, extract the zip contents to your ROS workspace.

Change the `map_file` argument in file `launch/amcl_simulation.launch` to your map file. The map can be found at `maps/my_map.yaml`.

```bash
# Launch ROS core
roscore

# Launch Gizmo with our 3d models
roslaunch exercise3_quar rins_world.launch

# Run BaseMove server, be sure to change the map path
roslaunch exercise3_quar amcl_simulation.launch

# Start the map visualisation
roslaunch turtlebot_rviz_launchers view_navigation.launch

# Actually run our script that moves robot to harcoded locations
# This runs the move_better.py script inside scripts folder
roslaunch exercise3_quar move_to_hardcoded_locations.launch
```