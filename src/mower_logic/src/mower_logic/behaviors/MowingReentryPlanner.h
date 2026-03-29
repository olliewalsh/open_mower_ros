// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.

#ifndef SRC_MOWINGREENTRYPLANNER_H
#define SRC_MOWINGREENTRYPLANNER_H

#include <vector>

#include "geometry_msgs/Polygon.h"
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
            double approach_distance, double inset_distance) const;

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
};

#endif  // SRC_MOWINGREENTRYPLANNER_H
