# FTCLocalPlanner

This repository contains a very simple "follow the carrot" local planner implementation.

## PurePursuitTracker

`ftc_local_planner/PurePursuitTracker` uses the same projection, safety, speed regulation, sharp-corner handling, endpoint behavior, diagnostics, and MBF integration as `SimplePathTracker`, but generates its tracking angular command from an adaptive pure-pursuit lookahead point. Configure it in `open_mower/params/pure_pursuit_tracker.yaml` and select it with the mower logic `mowing_controller` parameter set to `PurePursuitTracker`.

The lookahead distance is `clamp(minimum_lookahead + lookahead_time * commanded_speed, minimum_lookahead, maximum_lookahead)`. Shorter lookahead follows tight geometry more closely; longer lookahead is smoother.

## SimplePathTracker

`ftc_local_planner/SimplePathTracker` is an alternative MBF controller intended for precise mowing rows and curved perimeter paths. It projects the robot onto the path and creates one angular command from path-curvature feed-forward, heading error, and signed cross-track error, avoiding competing lateral and angular PID loops. Forward speed is constrained by curvature, tracking error, distance to the goal, acceleration limits, and mower current/RPM. Smooth row-end arcs are driven continuously; sharp vertices slow to a stop and rotate in place using the raw outgoing-segment heading rather than the preview-smoothed tracking heading.

Angular commands use separate acceleration and deceleration limits. When acceleration limiting prevents the requested angular speed during forward tracking, linear speed is reduced by the same ratio to preserve path curvature. A pending angular sign reversal stops forward motion until the command reaches the requested sign. Safety stops and straight rotation-recovery movements bypass the slew limiter. Rotation progress remains armed until a complete controller update stays in tracking, so an immediate return to rotation cannot continually reset the stall watchdog.

Lateral feedback is reduced smoothly on curved paths using `effective_cross_track_gain = cross_track_gain / (1 + cross_track_curvature_scale * abs(curvature))`. This preserves full lateral correction on straight mowing rows while allowing curvature feed-forward to dominate tight turns. Set `cross_track_curvature_scale` to zero to disable the scaling.

`boundary_slowdown_distance` optionally caps tracking speed near lethal costmap cells (and unknown cells when `unknown_is_obstacle` is enabled). The cap changes linearly from `boundary_minimum_speed` at zero clearance to `mowing_speed` at the configured distance. Set the distance to zero to disable it.

Mower current and RPM speed limits use asymmetric first-order filtering. `mow_load_filter_attack_time_constant` controls the fast response to increasing load, while `mow_load_filter_release_time_constant` controls the smoother recovery as load falls. Set either time constant to zero to disable filtering in that direction.

The controller checks the costmap's configured robot footprint at the current pose and along the predicted `(linear, angular)` trajectory. Set the footprint and padding in the standard costmap configuration; no controller-specific polygon is required.

Path projection is monotonic and each update is limited by measured forward motion along the current path tangent, so lateral motion across the inside of a curve cannot advance the tracking reference. `projection_initial_allowance` permits a small initial offset, `projection_distance_factor` scales accepted along-path motion, `projection_pose_deadband` rejects stationary pose noise, and `projection_max_pose_step` limits the effect of localization jumps. Unused allowance is discarded rather than accumulated, preventing later closed or spatially adjacent loops from becoming eligible early.

Projection scans only segments whose cumulative start distance is within the adaptive distance limit, making the search independent of path-point density. Beyond the immediate successor, `projection_heading_tolerance` rejects segments whose direction differs too much from the current segment, preventing a nearby closing or transition segment from winning solely by Euclidean distance. Set the tolerance to zero to disable this continuity check. Heading and curvature previews are clamped to the outgoing side of an already-processed sharp vertex so the completed corner cannot pull tracking back into rotation.

If the projected path is exhausted while the mower remains outside `goal_distance_tolerance`, the controller rotates toward and drives to the endpoint. It returns MBF `MISSED_GOAL` only after goal distance stops improving for `goal_position_timeout` seconds. The timeout uses ROS time so simulation and bag playback remain deterministic.

Parameters are loaded from `open_mower/params/simple_path_tracker.yaml`. Mower logic selects it by default through the private `mowing_controller` parameter. Set that parameter to `FTCPlanner` to switch mowing back without changing code. Docking continues to use `DockingFTCPlanner`.

All `SimplePathTracker` parameters are refreshed from the ROS parameter server once per second, using roscpp's local parameter cache. They can therefore be tuned while the controller is running, for example:

```bash
rosparam set /move_base_flex/SimplePathTracker/heading_gain 2.5
```

The controller publishes scalar tracking diagnostics for time-series tools such as PlotJuggler:

- `cross_track_error` (metres, signed)
- `heading_error` (radians, signed)
- `path_curvature` (inverse metres, signed)
- `remaining_distance` (metres along the path)

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
