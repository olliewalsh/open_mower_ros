# slic3r_coverage_planner
A coverage planner for ROS using libslic3r as core logic

Perimeter loops are joined at collision-free points that minimize the maximum
heading angle relative to the destination loop. This reduces how far the front of
the mower points towards the boundary during a transition. Locally straight seam
locations are preferred next, with accumulated turning used only as a final
tie-breaker.

The transition search length and Bezier control limit scale with the requested
coverage spacing, while retaining minimum values for smaller mowers.
