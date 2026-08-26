# FTCLocalPlanner

This repository contains a very simple "follow the carrot" local planner implementation.

## SimplePathTracker

`ftc_local_planner/SimplePathTracker` is an alternative MBF controller intended for precise mowing rows and curved perimeter paths. It projects the robot onto the path and creates one angular command from path-curvature feed-forward, heading error, and signed cross-track error, avoiding competing lateral and angular PID loops. Forward speed is constrained by curvature, tracking error, distance to the goal, acceleration limits, and mower current/RPM. Smooth row-end arcs are driven continuously; sharp vertices slow to a stop and rotate in place.

The controller checks the costmap's configured robot footprint at the current pose and along the predicted `(linear, angular)` trajectory. Set the footprint and padding in the standard costmap configuration; no controller-specific polygon is required.

Path projection is monotonic and limited by measured pose-to-pose travel. `projection_initial_allowance` permits a small initial offset, `projection_distance_factor` allows for localization and path-geometry error, `projection_pose_deadband` rejects stationary pose noise, and `projection_max_pose_step` limits the effect of localization jumps. Repeated controller updates cannot advance through a closed or spatially adjacent path when the robot is stationary.

Parameters are loaded from `open_mower/params/simple_path_tracker.yaml`. Mower logic selects it by default through the private `mowing_controller` parameter. Set that parameter to `FTCPlanner` to switch mowing back without changing code. Docking continues to use `DockingFTCPlanner`.

All `SimplePathTracker` parameters are refreshed from the ROS parameter server once per second, using roscpp's local parameter cache. They can therefore be tuned while the controller is running, for example:

```bash
rosparam set /move_base_flex/SimplePathTracker/heading_gain 2.5
```

Tune in this order:

1. Set `mowing_speed`, acceleration limits, and angular speed limits.
2. Tune `heading_gain` on straight rows with `cross_track_gain` low.
3. Raise `cross_track_gain` until offsets converge without oscillation.
4. Use `cross_track_slowdown_gain` only to control how aggressively large errors reduce speed.

## Description
The planner follows a given path as exactly as possible. It calculate the position of the carrot by
taking robots velocity limits into account. If deviation between carrot and robot is above a given
threshold, the planner stops.

Also it checks for collisions of carrot with obstacles as well as collisions of robot footprint at actual robot pose. It doesn't implement obstacle aviodance but obstacle detection. Following situation will cause the 
planner to exit:
- goal reached
- goal (pose) timeout reached
- deviation between carrot and robot 
- collision of carrot with obstacles
- collision of robot footprint with obstacles

## Published Topics
- **`global_point`** (geometry_msgs/PoseStamped) pose of carrot
- **`global_plan`** (nav_msgs/Path) global path to follow
- **`debug_pid`** (ftc_local_planner/PID) debug information of PID calculation
- **`costmap_marker`** (visualization_msgs/Marker) debug information of costmap check
