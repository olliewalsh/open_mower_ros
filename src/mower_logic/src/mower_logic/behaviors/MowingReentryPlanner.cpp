// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.

#include "MowingReentryPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool MowingReentryPlanner::pointInPolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon) {
  if (polygon.points.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++) {
    const auto& pi = polygon.points[i];
    const auto& pj = polygon.points[j];
    bool intersects = ((pi.y > point.y) != (pj.y > point.y)) &&
                      (point.x < (pj.x - pi.x) * (point.y - pi.y) / ((pj.y - pi.y) + 1e-9) + pi.x);
    if (intersects) inside = !inside;
  }
  return inside;
}

bool MowingReentryPlanner::isPointInFreeMowingSpace(const geometry_msgs::Point& point,
                                                    const geometry_msgs::Polygon& area,
                                                    const std::vector<geometry_msgs::Polygon>& obstacles) {
  if (!pointInPolygon(point, area)) {
    return false;
  }
  for (const auto& obstacle : obstacles) {
    if (pointInPolygon(point, obstacle)) {
      return false;
    }
  }
  return true;
}

bool MowingReentryPlanner::isSegmentInFreeMowingSpace(const geometry_msgs::Point& start,
                                                      const geometry_msgs::Point& end,
                                                      const geometry_msgs::Polygon& area,
                                                      const std::vector<geometry_msgs::Polygon>& obstacles) {
  double dx = end.x - start.x;
  double dy = end.y - start.y;
  double length = std::hypot(dx, dy);
  int samples = std::max(2, static_cast<int>(std::ceil(length / 0.05)) + 1);

  for (int i = 0; i < samples; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    geometry_msgs::Point sample;
    sample.x = start.x + dx * t;
    sample.y = start.y + dy * t;
    if (!isPointInFreeMowingSpace(sample, area, obstacles)) {
      return false;
    }
  }

  return true;
}

double MowingReentryPlanner::distanceToSegment(const geometry_msgs::Point& point, const geometry_msgs::Point32& start,
                                               const geometry_msgs::Point32& end) {
  double dx = end.x - start.x;
  double dy = end.y - start.y;
  double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1e-9) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }

  double t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq;
  t = std::max(0.0, std::min(1.0, t));
  double closest_x = start.x + t * dx;
  double closest_y = start.y + t * dy;
  return std::hypot(point.x - closest_x, point.y - closest_y);
}

double MowingReentryPlanner::distanceToPolygonEdges(const geometry_msgs::Point& point,
                                                    const geometry_msgs::Polygon& polygon) {
  if (polygon.points.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < polygon.points.size(); ++i) {
    const auto& start = polygon.points[i];
    const auto& end = polygon.points[(i + 1) % polygon.points.size()];
    min_distance = std::min(min_distance, distanceToSegment(point, start, end));
  }
  return min_distance;
}

double MowingReentryPlanner::computeSegmentClearance(const geometry_msgs::Point& start, const geometry_msgs::Point& end,
                                                     const geometry_msgs::Polygon& area,
                                                     const std::vector<geometry_msgs::Polygon>& obstacles) {
  double dx = end.x - start.x;
  double dy = end.y - start.y;
  double length = std::hypot(dx, dy);
  int samples = std::max(2, static_cast<int>(std::ceil(length / 0.05)) + 1);
  double min_clearance = std::numeric_limits<double>::infinity();

  for (int i = 0; i < samples; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    geometry_msgs::Point sample;
    sample.x = start.x + dx * t;
    sample.y = start.y + dy * t;

    if (!isPointInFreeMowingSpace(sample, area, obstacles)) {
      return 0.0;
    }

    min_clearance = std::min(min_clearance, distanceToPolygonEdges(sample, area));
    for (const auto& obstacle : obstacles) {
      min_clearance = std::min(min_clearance, distanceToPolygonEdges(sample, obstacle));
    }
  }

  return min_clearance;
}

MowingReentryPlanner::Plan MowingReentryPlanner::plan(const slic3r_coverage_planner::Path& path, int path_index,
                                                      const geometry_msgs::Polygon& area_outline,
                                                      const std::vector<geometry_msgs::Polygon>& area_obstacles,
                                                      const xbot_msgs::AbsolutePose& current_pose,
                                                      double approach_distance, double inset_distance) const {
  Plan result;
  if (path_index < 0 || path_index >= path.path.poses.size() || approach_distance <= 0.0) {
    return result;
  }

  const auto& reentry_pose = path.path.poses[path_index];
  tf2::Quaternion reentry_quat;
  tf2::fromMsg(reentry_pose.pose.orientation, reentry_quat);
  double unused_roll;
  double unused_pitch;
  double yaw;
  tf2::Matrix3x3(reentry_quat).getRPY(unused_roll, unused_pitch, yaw);
  double tangent_x = std::cos(yaw);
  double tangent_y = std::sin(yaw);
  double normal_x = -tangent_y;
  double normal_y = tangent_x;

  geometry_msgs::Point robot_point = current_pose.pose.pose.position;

  struct Candidate {
    geometry_msgs::PoseStamped pose;
    double min_clearance = 0.0;
    double distance_to_robot = std::numeric_limits<double>::infinity();
  };
  std::vector<Candidate> valid_candidates;

  for (double side_sign : {1.0, -1.0}) {
    geometry_msgs::PoseStamped candidate = reentry_pose;
    candidate.pose.position.x -= tangent_x * approach_distance;
    candidate.pose.position.y -= tangent_y * approach_distance;
    candidate.pose.position.x += normal_x * side_sign * inset_distance;
    candidate.pose.position.y += normal_y * side_sign * inset_distance;

    if (!isSegmentInFreeMowingSpace(candidate.pose.position, reentry_pose.pose.position, area_outline,
                                    area_obstacles)) {
      continue;
    }

    double dx = candidate.pose.position.x - robot_point.x;
    double dy = candidate.pose.position.y - robot_point.y;
    double min_clearance =
        computeSegmentClearance(candidate.pose.position, reentry_pose.pose.position, area_outline, area_obstacles);
    valid_candidates.push_back({candidate, min_clearance, std::hypot(dx, dy)});
  }

  if (valid_candidates.empty()) {
    return result;
  }

  auto best =
      std::min_element(valid_candidates.begin(), valid_candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.min_clearance - b.min_clearance) > 1e-3) {
          return a.min_clearance > b.min_clearance;
        }
        return a.distance_to_robot < b.distance_to_robot;
      });

  result.approach_pose = best->pose;
  result.lead_in_path.header = path.path.header;
  result.lead_in_path.poses.push_back(result.approach_pose);
  for (size_t i = path_index; i < path.path.poses.size() && result.lead_in_path.poses.size() < 3; ++i) {
    result.lead_in_path.poses.push_back(path.path.poses[i]);
  }
  while (result.lead_in_path.poses.size() < 3) {
    result.lead_in_path.poses.push_back(result.lead_in_path.poses.back());
  }
  result.valid = true;
  return result;
}
