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

bool MowingReentryPlanner::worldToMap(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Point& point, int& mx,
                                      int& my) {
  double origin_x = grid.info.origin.position.x;
  double origin_y = grid.info.origin.position.y;
  double resolution = grid.info.resolution;
  if (resolution <= 0.0) {
    return false;
  }

  double local_x = point.x - origin_x;
  double local_y = point.y - origin_y;
  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }

  mx = static_cast<int>(std::floor(local_x / resolution));
  my = static_cast<int>(std::floor(local_y / resolution));
  return mx >= 0 && my >= 0 && mx < static_cast<int>(grid.info.width) && my < static_cast<int>(grid.info.height);
}

bool MowingReentryPlanner::isBlockingCost(int8_t cost) {
  return cost >= 50;
}

const nav_msgs::OccupancyGrid* MowingReentryPlanner::getCostmapForPoint(const geometry_msgs::Point& point,
                                                                        const nav_msgs::OccupancyGrid& local_costmap,
                                                                        bool has_local_costmap,
                                                                        const nav_msgs::OccupancyGrid& global_costmap,
                                                                        bool has_global_costmap) {
  int mx;
  int my;
  if (has_local_costmap && worldToMap(local_costmap, point, mx, my)) {
    return &local_costmap;
  }
  if (has_global_costmap && worldToMap(global_costmap, point, mx, my)) {
    return &global_costmap;
  }
  return nullptr;
}

std::vector<geometry_msgs::Point> MowingReentryPlanner::transformFootprint(
    const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::Point>& footprint) {
  std::vector<geometry_msgs::Point> points;
  if (footprint.empty()) {
    points.push_back(pose.pose.position);
    return points;
  }

  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.orientation, quat);
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  points.reserve(footprint.size());
  for (const auto& fp : footprint) {
    geometry_msgs::Point world;
    world.x = pose.pose.position.x + fp.x * cos_yaw - fp.y * sin_yaw;
    world.y = pose.pose.position.y + fp.x * sin_yaw + fp.y * cos_yaw;
    points.push_back(world);
  }
  return points;
}

std::vector<geometry_msgs::Point> MowingReentryPlanner::sampleFootprint(
    const geometry_msgs::PoseStamped& pose, const std::vector<geometry_msgs::Point>& footprint) {
  std::vector<geometry_msgs::Point> vertices = transformFootprint(pose, footprint);
  if (vertices.empty()) {
    vertices.push_back(pose.pose.position);
    return vertices;
  }

  std::vector<geometry_msgs::Point> samples = vertices;
  if (vertices.size() > 1) {
    for (size_t i = 0; i < vertices.size(); ++i) {
      const auto& start = vertices[i];
      const auto& end = vertices[(i + 1) % vertices.size()];
      double dx = end.x - start.x;
      double dy = end.y - start.y;
      double length = std::hypot(dx, dy);
      int edge_samples = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
      for (int j = 1; j < edge_samples; ++j) {
        double t = static_cast<double>(j) / static_cast<double>(edge_samples);
        geometry_msgs::Point sample;
        sample.x = start.x + dx * t;
        sample.y = start.y + dy * t;
        samples.push_back(sample);
      }
    }
  }
  samples.push_back(pose.pose.position);
  return samples;
}

double MowingReentryPlanner::distanceToCostmapObstacle(const geometry_msgs::Point& point,
                                                       const nav_msgs::OccupancyGrid& grid, double max_distance) {
  int mx;
  int my;
  if (!worldToMap(grid, point, mx, my)) {
    return 0.0;
  }

  int index = my * static_cast<int>(grid.info.width) + mx;
  if (index < 0 || index >= static_cast<int>(grid.data.size())) {
    return 0.0;
  }
  if (isBlockingCost(grid.data[index])) {
    return 0.0;
  }

  int max_radius_cells = std::max(1, static_cast<int>(std::ceil(max_distance / grid.info.resolution)));
  double min_distance = max_distance;
  for (int dy = -max_radius_cells; dy <= max_radius_cells; ++dy) {
    for (int dx = -max_radius_cells; dx <= max_radius_cells; ++dx) {
      int cx = mx + dx;
      int cy = my + dy;
      if (cx < 0 || cy < 0 || cx >= static_cast<int>(grid.info.width) || cy >= static_cast<int>(grid.info.height)) {
        continue;
      }
      int cell_index = cy * static_cast<int>(grid.info.width) + cx;
      if (!isBlockingCost(grid.data[cell_index])) {
        continue;
      }
      double distance = std::hypot(dx * grid.info.resolution, dy * grid.info.resolution);
      min_distance = std::min(min_distance, distance);
    }
  }
  return min_distance;
}

bool MowingReentryPlanner::isPoseFootprintClear(const geometry_msgs::PoseStamped& pose,
                                                const geometry_msgs::Polygon& area,
                                                const std::vector<geometry_msgs::Polygon>& obstacles,
                                                const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
                                                const nav_msgs::OccupancyGrid& global_costmap, bool has_global_costmap,
                                                const std::vector<geometry_msgs::Point>& footprint) {
  for (const auto& sample : sampleFootprint(pose, footprint)) {
    if (!isPointInFreeMowingSpace(sample, area, obstacles)) {
      return false;
    }
    const nav_msgs::OccupancyGrid* grid =
        getCostmapForPoint(sample, local_costmap, has_local_costmap, global_costmap, has_global_costmap);
    if (grid != nullptr) {
      int mx;
      int my;
      if (!worldToMap(*grid, sample, mx, my)) {
        return false;
      }
      int idx = my * static_cast<int>(grid->info.width) + mx;
      if (isBlockingCost(grid->data[idx])) {
        return false;
      }
    }
  }
  return true;
}

double MowingReentryPlanner::computePoseClearance(const geometry_msgs::PoseStamped& pose,
                                                  const geometry_msgs::Polygon& area,
                                                  const std::vector<geometry_msgs::Polygon>& obstacles,
                                                  const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
                                                  const nav_msgs::OccupancyGrid& global_costmap,
                                                  bool has_global_costmap,
                                                  const std::vector<geometry_msgs::Point>& footprint) {
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto& sample : sampleFootprint(pose, footprint)) {
    if (!isPointInFreeMowingSpace(sample, area, obstacles)) {
      return 0.0;
    }
    min_clearance = std::min(min_clearance, distanceToPolygonEdges(sample, area));
    for (const auto& obstacle : obstacles) {
      min_clearance = std::min(min_clearance, distanceToPolygonEdges(sample, obstacle));
    }
    const nav_msgs::OccupancyGrid* grid =
        getCostmapForPoint(sample, local_costmap, has_local_costmap, global_costmap, has_global_costmap);
    if (grid != nullptr) {
      min_clearance = std::min(min_clearance, distanceToCostmapObstacle(sample, *grid, 0.6));
    }
  }
  return min_clearance;
}

bool MowingReentryPlanner::isLeadInNeeded(const geometry_msgs::PoseStamped& target_pose,
                                          const geometry_msgs::Polygon& area,
                                          const std::vector<geometry_msgs::Polygon>& obstacles,
                                          const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap,
                                          const nav_msgs::OccupancyGrid& global_costmap, bool has_global_costmap,
                                          const std::vector<geometry_msgs::Point>& footprint,
                                          double clearance_threshold) {
  if (!isPoseFootprintClear(target_pose, area, obstacles, local_costmap, has_local_costmap, global_costmap,
                            has_global_costmap, footprint)) {
    return true;
  }
  return computePoseClearance(target_pose, area, obstacles, local_costmap, has_local_costmap, global_costmap,
                              has_global_costmap, footprint) < clearance_threshold;
}

MowingReentryPlanner::Plan MowingReentryPlanner::plan(
    const slic3r_coverage_planner::Path& path, int path_index, const geometry_msgs::Polygon& area_outline,
    const std::vector<geometry_msgs::Polygon>& area_obstacles, const xbot_msgs::AbsolutePose& current_pose,
    const nav_msgs::OccupancyGrid& local_costmap, bool has_local_costmap, const nav_msgs::OccupancyGrid& global_costmap,
    bool has_global_costmap, const std::vector<geometry_msgs::Point>& footprint, double approach_distance,
    double inset_distance) const {
  Plan result;
  if (path_index < 0 || path_index >= path.path.poses.size() || approach_distance <= 0.0) {
    return result;
  }

  const auto& reentry_pose = path.path.poses[path_index];
  if (!isLeadInNeeded(reentry_pose, area_outline, area_obstacles, local_costmap, has_local_costmap, global_costmap,
                      has_global_costmap, footprint, std::max(0.15, inset_distance * 0.6))) {
    return result;
  }

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
    double score = 0.0;
    double distance_to_robot = std::numeric_limits<double>::infinity();
  };
  std::vector<Candidate> valid_candidates;

  double distance_step = 0.1;
  double max_search_distance = std::max(approach_distance + 0.3, 0.4);
  double lateral_step = 0.1;
  int lateral_count = std::max(1, static_cast<int>(std::ceil(std::max(inset_distance, 0.1) / lateral_step)));

  for (double distance = distance_step; distance <= max_search_distance + 1e-6; distance += distance_step) {
    for (int lateral_index = -lateral_count; lateral_index <= lateral_count; ++lateral_index) {
      double lateral_offset = lateral_index * lateral_step;
      if (std::abs(lateral_offset) > inset_distance + 0.15) {
        continue;
      }

      geometry_msgs::PoseStamped candidate = reentry_pose;
      candidate.pose.position.x -= tangent_x * distance;
      candidate.pose.position.y -= tangent_y * distance;
      candidate.pose.position.x += normal_x * lateral_offset;
      candidate.pose.position.y += normal_y * lateral_offset;

      if (!isPoseFootprintClear(candidate, area_outline, area_obstacles, local_costmap, has_local_costmap,
                                global_costmap, has_global_costmap, footprint)) {
        continue;
      }

      bool segment_clear = true;
      double min_segment_clearance = std::numeric_limits<double>::infinity();
      int segment_samples = std::max(2, static_cast<int>(std::ceil(distance / 0.05)) + 1);
      for (int i = 0; i < segment_samples; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(segment_samples - 1);
        geometry_msgs::PoseStamped sample_pose = reentry_pose;
        sample_pose.pose.position.x =
            candidate.pose.position.x + (reentry_pose.pose.position.x - candidate.pose.position.x) * t;
        sample_pose.pose.position.y =
            candidate.pose.position.y + (reentry_pose.pose.position.y - candidate.pose.position.y) * t;
        if (!isPoseFootprintClear(sample_pose, area_outline, area_obstacles, local_costmap, has_local_costmap,
                                  global_costmap, has_global_costmap, footprint)) {
          segment_clear = false;
          break;
        }
        min_segment_clearance =
            std::min(min_segment_clearance,
                     computePoseClearance(sample_pose, area_outline, area_obstacles, local_costmap, has_local_costmap,
                                          global_costmap, has_global_costmap, footprint));
      }
      if (!segment_clear) {
        continue;
      }

      double dx = candidate.pose.position.x - robot_point.x;
      double dy = candidate.pose.position.y - robot_point.y;
      double distance_to_robot = std::hypot(dx, dy);
      double score = min_segment_clearance + 0.2 * distance;
      valid_candidates.push_back({candidate, score, distance_to_robot});
    }
  }

  if (valid_candidates.empty()) {
    return result;
  }

  auto best =
      std::min_element(valid_candidates.begin(), valid_candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (std::abs(a.score - b.score) > 1e-3) {
          return a.score > b.score;
        }
        return a.distance_to_robot < b.distance_to_robot;
      });

  result.approach_pose = best->pose;
  result.staging_pose = best->pose;
  tf2::Quaternion staging_quat;
  staging_quat.setRPY(0.0, 0.0, 0.0);
  result.staging_pose.pose.orientation = tf2::toMsg(staging_quat);
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
