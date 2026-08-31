# slic3r_coverage_planner
A coverage planner for ROS using libslic3r as core logic

Perimeter loops are joined at collision-free points that favor locally straight
outline sections. This keeps loop seams away from polygon corners while retaining
the existing approach-clearance and smooth-transition checks.
