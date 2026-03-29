// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.

#ifndef SRC_MOWINGREENTRYPLANNER_H
#define SRC_MOWINGREENTRYPLANNER_H

#include <vector>

#include "geometry_msgs/Polygon.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "slic3r_coverage_planner/Path.h"
#include "xbot_msgs/AbsolutePose.h"

class MowingReentryPlanner {
 public:
  struct Plan {
    bool valid = false;
    geometry_msgs::PoseStamped approach_pose;
    nav_msgs::Path lead_in_path;
  };

  Plan plan(const slic3r_coverage_planner::Path& path, int path_index, const geometry_msgs::Polygon& area_outline,
            const std::vector<geometry_msgs::Polygon>& area_obstacles, const xbot_msgs::AbsolutePose& current_pose,
            const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
            const nav_msgs::OccupancyGrid& global_costmap, bool has_global_costmap,
            const std::vector<geometry_msgs::Point>& footprint, double approach_distance, double inset_distance) const;

 private:
  static bool pointInPolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
  static bool isPointInFreeMowingSpace(const geometry_msgs::Point& point, const geometry_msgs::Polygon& area,
                                       const std::vector<geometry_msgs::Polygon>& obstacles);
  static bool isSegmentInFreeMowingSpace(const geometry_msgs::Point& start, const geometry_msgs::Point& end,
                                         const geometry_msgs::Polygon& area,
                                         const std::vector<geometry_msgs::Polygon>& obstacles);
  static double distanceToSegment(const geometry_msgs::Point& point, const geometry_msgs::Point32& start,
                                  const geometry_msgs::Point32& end);
  static double distanceToPolygonEdges(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
  static double computeSegmentClearance(const geometry_msgs::Point& start, const geometry_msgs::Point& end,
                                        const geometry_msgs::Polygon& area,
                                        const std::vector<geometry_msgs::Polygon>& obstacles);
  static bool worldToMap(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Point& point, int& mx, int& my);
  static bool isBlockingCost(int8_t cost);
  static const nav_msgs::OccupancyGrid* getCostmapForPoint(const geometry_msgs::Point& point,
                                                           const nav_msgs::OccupancyGrid& local_costmap,
                                                           bool has_local_costmap,
                                                           const nav_msgs::OccupancyGrid& global_costmap,
                                                           bool has_global_costmap);
  static std::vector<geometry_msgs::Point> transformFootprint(const geometry_msgs::PoseStamped& pose,
                                                              const std::vector<geometry_msgs::Point>& footprint);
  static std::vector<geometry_msgs::Point> sampleFootprint(const geometry_msgs::PoseStamped& pose,
                                                           const std::vector<geometry_msgs::Point>& footprint);
  static bool isPoseFootprintClear(const geometry_msgs::PoseStamped& pose, const geometry_msgs::Polygon& area,
                                   const std::vector<geometry_msgs::Polygon>& obstacles,
                                   const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
                                   const nav_msgs::OccupancyGrid& global_costmap, bool has_global_costmap,
                                   const std::vector<geometry_msgs::Point>& footprint);
  static double computePoseClearance(const geometry_msgs::PoseStamped& pose, const geometry_msgs::Polygon& area,
                                     const std::vector<geometry_msgs::Polygon>& obstacles,
                                     const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
                                     const nav_msgs::OccupancyGrid& global_costmap, bool has_global_costmap,
                                     const std::vector<geometry_msgs::Point>& footprint);
  static double distanceToCostmapObstacle(const geometry_msgs::Point& point, const nav_msgs::OccupancyGrid& grid,
                                          double max_distance);
  static bool isLeadInNeeded(const geometry_msgs::PoseStamped& target_pose, const geometry_msgs::Polygon& area,
                             const std::vector<geometry_msgs::Polygon>& obstacles,
                             const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
                             const nav_msgs::OccupancyGrid& global_costmap, bool has_global_costmap,
                             const std::vector<geometry_msgs::Point>& footprint, double clearance_threshold);
};

#endif  // SRC_MOWINGREENTRYPLANNER_H
