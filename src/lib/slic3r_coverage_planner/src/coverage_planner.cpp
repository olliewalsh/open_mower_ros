//
// Created by Clemens Elflein on 27.08.21.
//

#include "ros/ros.h"

#include <boost/range/adaptor/reversed.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"


#include "slic3r_coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <Fill/FillPlanePath.hpp>
#include <PerimeterGenerator.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"


bool visualize_plan;
ros::Publisher marker_array_publisher;

namespace {
constexpr double kSourceSpacing = 0.05;
constexpr double kMaxStraightSpacing = 0.50;
constexpr double kMaxChordError = 0.01;
constexpr double kMaxHeadingChange = 5.0 * M_PI / 180.0;
constexpr double kSeamTurnWindow = 0.40;

double pointDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double normalizedAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Scores how much the outline turns near a possible split/join point. Points
// are sampled at kSourceSpacing, so the fixed index window represents a stable
// physical distance and prevents an otherwise arbitrary polygon start from
// placing every perimeter seam in a corner.
double seamTurnScore(const Points &points, int index) {
    if (points.size() < 3) return std::numeric_limits<double>::infinity();
    const int size = static_cast<int>(points.size());
    const int radius = std::max(1, static_cast<int>(std::ceil(kSeamTurnWindow / kSourceSpacing)));
    double peak_turn = 0.0;
    double weighted_turn = 0.0;
    for (int offset = -radius; offset <= radius; ++offset) {
        const int current = ((index + offset) % size + size) % size;
        const int before = (current + size - 1) % size;
        const int after = (current + 1) % size;
        const double first_x = unscale(points[current].x - points[before].x);
        const double first_y = unscale(points[current].y - points[before].y);
        const double second_x = unscale(points[after].x - points[current].x);
        const double second_y = unscale(points[after].y - points[current].y);
        if (std::hypot(first_x, first_y) < 1e-6 ||
            std::hypot(second_x, second_y) < 1e-6)
            continue;
        const double turn = std::abs(normalizedAngle(
            std::atan2(second_y, second_x) - std::atan2(first_y, first_x)));
        const double weight =
            1.0 - static_cast<double>(std::abs(offset)) / static_cast<double>(radius + 1);
        peak_turn = std::max(peak_turn, turn);
        weighted_turn += weight * turn;
    }
    return peak_turn + weighted_turn;
}

double pointLineDistance(const geometry_msgs::Point &point, const geometry_msgs::Point &start,
                         const geometry_msgs::Point &end) {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < 1e-12) return pointDistance(point, start);
    const double t = std::max(0.0, std::min(1.0,
            ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared));
    geometry_msgs::Point projection;
    projection.x = start.x + t * dx;
    projection.y = start.y + t * dy;
    return pointDistance(point, projection);
}

void updatePathOrientations(nav_msgs::Path &path) {
    if (path.poses.size() < 2) return;
    for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto &before = path.poses[i == 0 ? 0 : i - 1].pose.position;
        const auto &after = path.poses[i + 1 < path.poses.size() ? i + 1 : i].pose.position;
        const double yaw = std::atan2(after.y - before.y, after.x - before.x);
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        path.poses[i].pose.orientation = tf2::toMsg(orientation);
    }
}

void adaptivelySimplifyPath(nav_msgs::Path &path) {
    if (path.poses.size() < 3) {
        updatePathOrientations(path);
        return;
    }

    std::vector<geometry_msgs::PoseStamped> unique;
    unique.reserve(path.poses.size());
    for (const auto &pose: path.poses) {
        if (unique.empty() || pointDistance(unique.back().pose.position, pose.pose.position) > 1e-5)
            unique.push_back(pose);
    }
    if (unique.size() < 3) {
        path.poses = unique;
        updatePathOrientations(path);
        return;
    }

    std::vector<geometry_msgs::PoseStamped> simplified;
    simplified.reserve(unique.size());
    size_t anchor = 0;
    simplified.push_back(unique.front());
    while (anchor + 1 < unique.size()) {
        size_t best = anchor + 1;
        for (size_t candidate = anchor + 2; candidate < unique.size(); ++candidate) {
            const auto &start = unique[anchor].pose.position;
            const auto &end = unique[candidate].pose.position;
            if (pointDistance(start, end) > kMaxStraightSpacing) break;

            double max_error = 0.0;
            for (size_t i = anchor + 1; i < candidate; ++i)
                max_error = std::max(max_error, pointLineDistance(unique[i].pose.position, start, end));

            const auto &first = unique[anchor + 1].pose.position;
            const auto &penultimate = unique[candidate - 1].pose.position;
            const double entry_heading = std::atan2(first.y - start.y, first.x - start.x);
            const double exit_heading = std::atan2(end.y - penultimate.y, end.x - penultimate.x);
            if (max_error > kMaxChordError ||
                std::abs(normalizedAngle(exit_heading - entry_heading)) > kMaxHeadingChange)
                break;
            best = candidate;
        }
        simplified.push_back(unique[best]);
        anchor = best;
    }
    path.poses = simplified;
    updatePathOrientations(path);
}

Points subdivideStraightSegments(const Points &vertices) {
    Points result;
    if (vertices.empty()) return result;
    result.push_back(vertices.front());
    for (size_t i = 1; i < vertices.size(); ++i) {
        const Point &start = vertices[i - 1];
        const Point &end = vertices[i];
        const double length = std::hypot(unscale(end.x - start.x), unscale(end.y - start.y));
        const int segments = std::max(1, static_cast<int>(std::ceil(length / kMaxStraightSpacing)));
        for (int segment = 1; segment <= segments; ++segment) {
            const double fraction = static_cast<double>(segment) / segments;
            result.emplace_back(start.x + static_cast<coord_t>((end.x - start.x) * fraction),
                                start.y + static_cast<coord_t>((end.y - start.y) * fraction));
        }
    }
    return result;
}

Points cubicTransition(const Point &previous, const Point &start, const Point &end, const Point &next) {
    const double sx = unscale(start.x), sy = unscale(start.y);
    const double ex = unscale(end.x), ey = unscale(end.y);
    double in_x = sx - unscale(previous.x), in_y = sy - unscale(previous.y);
    double out_x = unscale(next.x) - ex, out_y = unscale(next.y) - ey;
    const double in_length = std::hypot(in_x, in_y);
    const double out_length = std::hypot(out_x, out_y);
    const double chord = std::hypot(ex - sx, ey - sy);
    Points result;
    if (in_length < 1e-6 || out_length < 1e-6 || chord < 1e-6) return result;
    in_x /= in_length; in_y /= in_length;
    out_x /= out_length; out_y /= out_length;
    const double control_distance = std::min(0.30, chord * 0.45);
    const double c1x = sx + in_x * control_distance, c1y = sy + in_y * control_distance;
    const double c2x = ex - out_x * control_distance, c2y = ey - out_y * control_distance;
    const int steps = std::max(2, static_cast<int>(std::ceil(
            (chord + 2.0 * control_distance) / kSourceSpacing)));
    for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        const double u = 1.0 - t;
        const double x = u*u*u*sx + 3*u*u*t*c1x + 3*u*t*t*c2x + t*t*t*ex;
        const double y = u*u*u*sy + 3*u*u*t*c1y + 3*u*t*t*c2y + t*t*t*ey;
        result.emplace_back(scale_(x), scale_(y));
    }
    return result;
}

struct TransitionQuality {
    double score{std::numeric_limits<double>::infinity()};
    bool has_inflection{false};
};

TransitionQuality transitionQuality(const Point &start, const Points &curve,
                                    const Point &end, const Point &next) {
    Points samples;
    samples.reserve(curve.size() + 3);
    samples.push_back(start);
    samples.insert(samples.end(), curve.begin(), curve.end());
    samples.push_back(end);
    samples.push_back(next);

    TransitionQuality quality;
    double peak_curvature = 0.0;
    double total_turn = 0.0;
    int previous_sign = 0;
    for (size_t i = 1; i + 1 < samples.size(); ++i) {
        const double ax = unscale(samples[i].x - samples[i - 1].x);
        const double ay = unscale(samples[i].y - samples[i - 1].y);
        const double bx = unscale(samples[i + 1].x - samples[i].x);
        const double by = unscale(samples[i + 1].y - samples[i].y);
        const double first_length = std::hypot(ax, ay);
        const double second_length = std::hypot(bx, by);
        if (first_length < 1e-6 || second_length < 1e-6) continue;
        const double turn = normalizedAngle(std::atan2(by, bx) - std::atan2(ay, ax));
        const double curvature = std::abs(turn) / (0.5 * (first_length + second_length));
        peak_curvature = std::max(peak_curvature, curvature);
        total_turn += std::abs(turn);
        if (std::abs(turn) > 0.5 * M_PI / 180.0) {
            const int sign = turn > 0.0 ? 1 : -1;
            if (previous_sign != 0 && sign != previous_sign) quality.has_inflection = true;
            previous_sign = sign;
        }
    }
    quality.score = peak_curvature + 0.25 * total_turn;
    return quality;
}

bool transitionIsSafe(const Point &start, const Points &curve, const Point &end,
                      const Polygon &boundary) {
    Point intersection;
    Point previous = start;
    for (const auto &point: curve) {
        Line segment(previous, point);
        if (boundary.intersection(segment, &intersection)) return false;
        previous = point;
    }
    Line final_segment(previous, end);
    return !boundary.intersection(final_segment, &intersection);
}

double distanceToPolygonBoundary(const Point &point, const Polygon &polygon) {
    double distance = std::numeric_limits<double>::infinity();
    for (const auto &line: polygon.lines())
        distance = std::min(distance, unscale(line.distance_to(point)));
    return distance;
}

// Returns the largest approach scale that fits at this directed path point.
double outlineApproachScale(const Point &goal, const Point &next, const Polygon &area,
                            const Polygons &obstacles, double configured_length,
                            double configured_inset) {
    const double gx = unscale(goal.x), gy = unscale(goal.y);
    double tx = unscale(next.x - goal.x), ty = unscale(next.y - goal.y);
    const double tangent_length = std::hypot(tx, ty);
    if (tangent_length < 1e-6) return 0.0;
    tx /= tangent_length;
    ty /= tangent_length;
    const double requested_length = configured_length > 0.0 ? configured_length : 1.5;
    const double requested_inset = configured_inset > 0.0 ? configured_inset : 0.4;
    for (double scale : {1.0, 0.75, 0.5, 0.25}) {
        const double length = std::max(0.5, requested_length * scale);
        const double inset = std::max(0.1, requested_inset * scale);
        const int samples = std::max(3, static_cast<int>(std::ceil((length + inset) / 0.1)));
        for (double side : {1.0, -1.0}) {
            const double nx = -ty * side, ny = tx * side;
            const double p0x = gx - tx * length + nx * inset;
            const double p0y = gy - ty * length + ny * inset;
            const Point staging_point(scale_(p0x), scale_(p0y));
            if (!area.contains(staging_point) ||
                distanceToPolygonBoundary(staging_point, area) + 1e-6 < requested_inset)
                continue;
            bool staging_point_clear = true;
            for (const auto &obstacle : obstacles) {
                if (obstacle.contains(staging_point) ||
                    distanceToPolygonBoundary(staging_point, obstacle) + 1e-6 < requested_inset) {
                    staging_point_clear = false;
                    break;
                }
            }
            if (!staging_point_clear) continue;
            const double handle = length * 0.45;
            const double p1x = p0x + tx * handle, p1y = p0y + ty * handle;
            const double p2x = gx - tx * handle, p2y = gy - ty * handle;
            bool valid = true;
            for (int sample = 0; sample < samples; ++sample) {
                const double u = static_cast<double>(sample) / samples;
                const double v = 1.0 - u;
                const Point point(scale_(v*v*v*p0x + 3*v*v*u*p1x + 3*v*u*u*p2x + u*u*u*gx),
                                  scale_(v*v*v*p0y + 3*v*v*u*p1y + 3*v*u*u*p2y + u*u*u*gy));
                if (!area.contains(point)) { valid = false; break; }
                for (const auto &obstacle : obstacles) {
                    if (obstacle.contains(point)) { valid = false; break; }
                }
                if (!valid) break;
            }
            if (valid) return scale;
        }
    }
    return 0.0;
}
}  // namespace


void
createMarkers(const slic3r_coverage_planner::PlanPathRequest &planning_request,
              const slic3r_coverage_planner::PlanPathResponse &planning_result,
              visualization_msgs::MarkerArray &markerArray) {

    std::vector<std_msgs::ColorRGBA> colors;

    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }

    // Create markers for the input polygon
    {
        // Walk through the paths we send to the navigation stack
        auto &path = planning_request.outline.points;
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.color = colors[0];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path) {

            geometry_msgs::Point vpt;
            vpt.x = point.x;
            vpt.y = point.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start and end
        if (!path.empty()) {
            visualization_msgs::Marker marker{};

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color = colors[0];
            marker.pose.position.x = path.front().x;
            marker.pose.position.y = path.front().y;
            marker.scale.x = 0.1;
            marker.scale.y = marker.scale.z = 0.1;
            markerArray.markers.push_back(marker);
        }
    }


    // keep track of the color used last, so that we can use a new one for each path
    uint32_t cidx = 0;

    // Walk through the paths we send to the navigation stack
    for (auto &path: planning_result.paths) {
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.color = colors[cidx];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path.path.poses) {

            geometry_msgs::Point vpt;
            vpt.x = point.pose.position.x;
            vpt.y = point.pose.position.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start
        if (!path.path.poses.empty()) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.color = colors[cidx];
            marker.pose = path.path.poses.front().pose;
            marker.scale.x = 0.2;
            marker.scale.y = marker.scale.z = 0.05;
            markerArray.markers.push_back(marker);
        }

        // New color for a new path
        cidx = (cidx + 1) % colors.size();
    }
}

void traverse_from_left(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: contours) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_left(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

void traverse_from_right(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: boost::adaptors::reverse(contours)) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_right(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

slic3r_coverage_planner::Path determinePathForOutline(std_msgs::Header &header, Slic3r::Polygon &outline_poly,
                                                        const Polygons &obstacles, Slic3r::Polygons &group,
                                                        bool isObstacle, double approach_length,
                                                        double approach_inset, Point *areaLastPoint) {
    slic3r_coverage_planner::Path path;
    path.is_outline = true;
    path.path.header = header;

    Point lastPoint;
    Point previousPoint;
    bool has_previous_point = false;
    bool is_first_point = true;
    for (int i = 0; i < group.size(); i++) {
        auto points = group[i].equally_spaced_points(scale_(kSourceSpacing));
        if (points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << points.size() << " points");

        if (is_first_point && !isObstacle) {
            double best_scale = 0.0;
            double best_turn_score = std::numeric_limits<double>::infinity();
            int best_idx = 0;
            for (int idx = 0; idx < points.size(); ++idx) {
                const int next_idx = (idx + 1) % points.size();
                const double scale = outlineApproachScale(points[idx], points[next_idx], outline_poly,
                                                          obstacles, approach_length, approach_inset);
                if (scale <= 0.0) continue;
                const double turn_score = seamTurnScore(points, idx);
                if (scale > best_scale ||
                    (scale == best_scale && turn_score < best_turn_score)) {
                    best_scale = scale;
                    best_turn_score = turn_score;
                    best_idx = idx;
                }
            }
            if (best_idx > 0) std::rotate(points.begin(), points.begin() + best_idx, points.end());
            if (best_scale > 0.0)
                ROS_INFO_STREAM("Selected area outline start with approach scale " << best_scale);
        }
        else if (is_first_point && isObstacle && group.size() == 1) {
            double best_scale = 0.0;
            double best_turn_score = std::numeric_limits<double>::infinity();
            int best_idx = 0;
            for (int idx = 0; idx < points.size(); ++idx) {
                const int start_idx = (idx + points.size() - 1) % points.size();
                const int start_next_idx = (start_idx + points.size() - 1) % points.size();
                const double scale = outlineApproachScale(points[start_idx], points[start_next_idx], outline_poly,
                                                          obstacles, approach_length, approach_inset);
                if (scale <= 0.0) continue;
                const double turn_score = seamTurnScore(points, start_idx);
                if (scale > best_scale ||
                    (scale == best_scale && turn_score < best_turn_score)) {
                    best_scale = scale;
                    best_turn_score = turn_score;
                    best_idx = idx;
                }
            }
            if (best_idx > 0) std::rotate(points.begin(), points.begin() + best_idx, points.end());
            if (best_scale > 0.0)
                ROS_INFO_STREAM("Selected obstacle outline start with approach scale " << best_scale);
        }

        if (!is_first_point) {
            // Find a good transition point between the loops.
            // It should be close to the last split point, so that we don't need to traverse a lot.

            // Find the point in the current poly which is closest to the last point of the last group
            // (which is the next inner poly from this point of view).
            const auto last_x = unscale(lastPoint.x);
            const auto last_y = unscale(lastPoint.y);
            double min_distance = INFINITY;
            int closest_idx = 0;
            for (int idx = 0; idx < points.size(); ++idx) {
                const auto &pt = points[idx];
                const auto pt_x = unscale(pt.x);
                const auto pt_y = unscale(pt.y);
                double distance = sqrt((pt_x - last_x) * (pt_x - last_x) + (pt_y - last_y) * (pt_y - last_y));
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_idx = idx;
                }
            }

            const Polygon *next_outer_poly;
            if (i < group.size() - 1) {
                next_outer_poly = &group[i + 1];
            } else {
                // we are in the outermost line, use outline for collision check
                next_outer_poly = &outline_poly;
            }
            int selected_idx = closest_idx;
            Points selected_transition;
            TransitionQuality selected_quality;
            bool selected_without_inflection = false;
            double selected_turn_score = std::numeric_limits<double>::infinity();
            if (has_previous_point && points.size() > 1) {
                const bool selecting_obstacle_start = isObstacle && i == static_cast<int>(group.size()) - 1;
                const int candidate_count = selecting_obstacle_start ? points.size() : std::min<int>(11, points.size() - 1);
                double selected_approach_scale = 0.0;
                for (int offset = 0; offset < candidate_count; ++offset) {
                    const int candidate_idx = (closest_idx + offset) % points.size();
                    const int next_idx = (candidate_idx + 1) % points.size();
                    auto transition = cubicTransition(previousPoint, lastPoint,
                                                      points[candidate_idx], points[next_idx]);
                    if (transition.empty() || !transitionIsSafe(lastPoint, transition,
                                                                points[candidate_idx], *next_outer_poly))
                        continue;
                    const auto quality = transitionQuality(lastPoint, transition,
                                                           points[candidate_idx], points[next_idx]);
                    const double turn_score = seamTurnScore(points, candidate_idx);
                    double approach_scale = 0.0;
                    if (selecting_obstacle_start) {
                        const int start_idx = (candidate_idx + points.size() - 1) % points.size();
                        const int start_next_idx = (start_idx + points.size() - 1) % points.size();
                        approach_scale = outlineApproachScale(points[start_idx], points[start_next_idx], outline_poly,
                                                             obstacles, approach_length, approach_inset);
                        if (approach_scale <= 0.0) continue;
                    }
                    const bool candidate_without_inflection = !quality.has_inflection;
                    const bool prefer_approach = approach_scale > selected_approach_scale;
                    const bool same_approach = approach_scale == selected_approach_scale;
                    const bool prefer_inflection =
                        candidate_without_inflection && !selected_without_inflection;
                    const bool same_inflection =
                        candidate_without_inflection == selected_without_inflection;
                    const bool prefer_turn = turn_score < selected_turn_score;
                    const bool same_turn = turn_score == selected_turn_score;
                    if (selected_transition.empty() || prefer_approach || (same_approach &&
                        (prefer_inflection || (same_inflection &&
                         (prefer_turn || (same_turn && quality.score < selected_quality.score)))))) {
                        selected_idx = candidate_idx;
                        selected_transition = transition;
                        selected_quality = quality;
                        selected_without_inflection = candidate_without_inflection;
                        selected_approach_scale = approach_scale;
                        selected_turn_score = turn_score;
                    }
                }
            }
            if (selected_idx > 0)
                std::rotate(points.begin(), points.begin() + selected_idx, points.end());
            if (!selected_transition.empty())
                points.insert(points.begin(), selected_transition.begin(), selected_transition.end());
        }

        for (auto &pt: points) {
            if (is_first_point) {
                lastPoint = pt;
                is_first_point = false;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            // Direction for obstacle needs to be inversed compared to area outline, because we will reverse the point order later.
            auto dir = isObstacle ? lastPoint - pt : pt - lastPoint;

            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint.x);
            pose.pose.position.y = unscale(lastPoint.y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
            previousPoint = lastPoint;
            has_previous_point = true;
            lastPoint = pt;
        }
    }

    if (is_first_point) {
        // there wasn't any usable point, so return the empty path
        return path;
    }

    // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation = path.path.poses.back().pose.orientation;
    pose.pose.position.x = unscale(lastPoint.x);
    pose.pose.position.y = unscale(lastPoint.y);
    pose.pose.position.z = 0;
    path.path.poses.push_back(pose);
    adaptivelySimplifyPath(path.path);

    if (areaLastPoint != nullptr) {
        *areaLastPoint = lastPoint;
    }

    return path;
}

bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res) {

    Slic3r::Polygon outline_poly;
    for (auto &pt: req.outline.points) {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_counter_clockwise();

    // This ExPolygon contains our input area with holes.
    Slic3r::ExPolygon expoly(outline_poly);

    for (auto &hole: req.holes) {
        Slic3r::Polygon hole_poly;
        for (auto &pt: hole.points) {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_clockwise();

        // Clip the hole to the outline instead of using it as-is. An obstacle that only
        // partially overlaps the area (or extends beyond it) can otherwise own the extreme
        // vertex of the whole path set, which makes Clipper's offset engine (ClipperOffset::
        // FixOrientations) flip the orientation of every path, including the outline itself.
        // That makes the planner fill inside the obstacle instead of inside the area.
        Polygons clipped_holes = intersection(outline_poly, hole_poly);
        for (auto &clipped_hole: clipped_holes) {
            clipped_hole.make_clockwise();
            expoly.holes.push_back(clipped_hole);
        }
    }





    // Results are stored here
    std::vector<Polygons> area_outlines;
    Polylines fill_lines;
    std::vector<Polygons> obstacle_outlines;


    coord_t distance = scale_(req.distance);
    coord_t outer_distance = scale_(req.outer_offset);

    // detect how many perimeters must be generated for this island
    int loops = req.outline_count;

    ROS_INFO_STREAM("generating " << loops << " outlines");

    const int loop_number = loops - 1;  // 0-indexed loops
    const int inner_loop_number = loop_number - req.outline_overlap_count;


    Polygons gaps;

    Polygons last = expoly;
    Polygons inner = last;
    if (loop_number >= 0) {  // no loops = -1

        std::vector<PerimeterGeneratorLoops> contours(loop_number + 1);    // depth => loops
        std::vector<PerimeterGeneratorLoops> holes(loop_number + 1);       // depth => loops

        for (int i = 0; i <= loop_number; ++i) {  // outer loop is 0
            Polygons offsets;

            if (i == 0) {
                offsets = offset(
                        last,
                        -outer_distance
                );
            } else {
                offsets = offset(
                        last,
                        -distance
                );
            }

            if (offsets.empty()) break;


            last = offsets;
            if (i <= inner_loop_number) {
                inner = last;
            }

            for (Polygons::const_iterator polygon = offsets.begin(); polygon != offsets.end(); ++polygon) {
                PerimeterGeneratorLoop loop(*polygon, i);
                loop.is_contour = polygon->is_counter_clockwise();
                if (loop.is_contour) {
                    contours[i].push_back(loop);
                } else {
                    holes[i].push_back(loop);
                }
            }
        }

        // nest loops: holes first
        for (int d = 0; d <= loop_number; ++d) {
            PerimeterGeneratorLoops &holes_d = holes[d];

            // loop through all holes having depth == d
            for (int i = 0; i < (int) holes_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = holes_d[i];

                // find the hole loop that contains this one, if any
                for (int t = d + 1; t <= loop_number; ++t) {
                    for (int j = 0; j < (int) holes[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = holes[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            holes_d.erase(holes_d.begin() + i);
                            --i;
                            goto NEXT_LOOP;
                        }
                    }
                }

                NEXT_LOOP:;
            }
        }

        // nest contour loops
        for (int d = loop_number; d >= 1; --d) {
            PerimeterGeneratorLoops &contours_d = contours[d];

            // loop through all contours having depth == d
            for (int i = 0; i < (int) contours_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = contours_d[i];

                // find the contour loop that contains it
                for (int t = d - 1; t >= 0; --t) {
                    for (size_t j = 0; j < contours[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = contours[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            contours_d.erase(contours_d.begin() + i);
                            --i;
                            goto NEXT_CONTOUR;
                        }
                    }
                }

                NEXT_CONTOUR:;
            }
        }

        if (!req.skip_area_outline) {
            traverse_from_right(contours[0], area_outlines);
        }

        if (!req.skip_obstacle_outlines) {
            for (auto &hole: holes) {
                traverse_from_left(hole, obstacle_outlines);
            }
            for (auto &obstacle_group: obstacle_outlines) {
                for (auto &poly: obstacle_group) {
                    std::reverse(poly.points.begin(), poly.points.end());
                }
            }
        }
    }


    if (!req.skip_fill) {
        ExPolygons expp = union_ex(inner);


        // Go through the innermost poly and create the fill path using a Fill object
        for (auto &poly: expp) {
            Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);

            Slic3r::Fill *fill;
            if (req.fill_type == slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR) {
                fill = new Slic3r::FillRectilinear();
            } else {
                fill = new Slic3r::FillConcentric();
            }
            fill->link_max_length = scale_(1.0);
            fill->angle = req.angle;
            fill->z = scale_(1.0);
            fill->endpoints_overlap = 0;
            fill->density = 1.0;
            fill->dont_connect = false;
            fill->dont_adjust = true;
            fill->min_spacing = req.distance;
            fill->complete = false;
            fill->link_max_length = 0;

            ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

            Slic3r::Polylines lines = fill->fill_surface(surface);
            append_to(fill_lines, lines);
            delete fill;
            fill = nullptr;

            ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
            for (int i = 0; i < lines.size(); i++) {
                ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
            }
        }
    }


    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    /**
     * Some postprocessing is done here. Until now we just have polygons (just points), but the ROS
     * navigation stack requires an orientation for each of those points as well.
     *
     * In order to achieve this, we split the polygon at some point to make it into a line with start and end.
     * Then we can calculate the orientation at each point by looking at the connection line between two points.
     */

    Point areaLastPoint;
    for (auto &group: area_outlines) {
        auto path = determinePathForOutline(header, outline_poly, expoly.holes, group, false, req.outline_approach_length, req.outline_approach_inset, &areaLastPoint);
        if (!path.path.poses.empty()) {
            res.paths.push_back(path);
        }
    }

    // The order for 3d printing seems to be to sweep across the X and then up the Y axis
    // which is very inefficient for a mower. Order the holes by distance to the previous end-point instead.
    std::vector<Slic3r::Polygons> ordered_obstacle_outlines;
    if (obstacle_outlines.size() > 0) {
        // If no prev point set to the first point in first obstacle
        // Note: back() polygon is the first (outer) loop
        auto prev_point = area_outlines.size() > 0 ? &areaLastPoint :
            &obstacle_outlines.front().back().points.front();

        while (obstacle_outlines.size()) {
            // Sort be desc distance then pop closest outline from the back of the vector
            std::sort(obstacle_outlines.begin(), obstacle_outlines.end(),
                      [prev_point](Slic3r::Polygons &a, Slic3r::Polygons &b) {
                          // Note: back() polygon is the first (outer) loop
                          auto a_firstPoint = a.back().points.front();
                          double distance_a = sqrt(
                                  (a_firstPoint.x - prev_point->x) * (a_firstPoint.x - prev_point->x) +
                                  (a_firstPoint.y - prev_point->y) * (a_firstPoint.y - prev_point->y)
                          );
                          auto b_firstPoint = b.back().points.front();
                          double distance_b = sqrt(
                                  (b_firstPoint.x - prev_point->x) * (b_firstPoint.x - prev_point->x) +
                                  (b_firstPoint.y - prev_point->y) * (b_firstPoint.y - prev_point->y)
                          );
                          return distance_a >= distance_b;
                      });
            ordered_obstacle_outlines.push_back(obstacle_outlines.back());
            obstacle_outlines.pop_back();
            // Note: front() polygon is the last (inner) loop
            prev_point = &ordered_obstacle_outlines.back().front().points.back();
        }
    }

    // At this point, the obstacles outlines are still "the wrong way" (i.e. inner first, then outer ...),
    // this is intentional, because then it's easier to find good traversal points.
    // In order to make the mower approach the obstacle, we will reverse the path later.
    for (auto &group: ordered_obstacle_outlines) {
        // Reverse here to make the mower approach the obstacle instead of starting close to the obstacle
        auto path = determinePathForOutline(header, outline_poly, expoly.holes, group, true, req.outline_approach_length, req.outline_approach_inset, nullptr);
        if (!path.path.poses.empty()) {
            std::reverse(path.path.poses.begin(), path.path.poses.end());
            updatePathOrientations(path.path);
            res.paths.push_back(path);
        }
    }

    if (!req.skip_fill) {
        for (int i = 0; i < fill_lines.size(); i++) {
            auto &line = fill_lines[i];
            slic3r_coverage_planner::Path path;
            path.is_outline = false;
            path.path.header = header;

            line.remove_duplicate_points();


            auto path_points = subdivideStraightSegments(line.points);
            if (path_points.size() < 2) {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << path_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt: path_points) {
                if (lastPoint == nullptr) {
                    lastPoint = &pt;
                    continue;
                }

                // calculate pose for "lastPoint" pointing to current point

                auto dir = pt - *lastPoint;
                double orientation = atan2(dir.y, dir.x);
                tf2::Quaternion q(0.0, 0.0, orientation);

                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);

            updatePathOrientations(path.path);

            res.paths.push_back(path);
        }
    }

    if (visualize_plan) {
        visualization_msgs::MarkerArray arr;
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = -1;
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::DELETEALL;
            arr.markers.push_back(marker);
        }
        createMarkers(req, res, arr);
        marker_array_publisher.publish(arr);
    }


    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "slic3r_coverage_planner");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    visualize_plan = paramNh.param("visualize_plan", true);

    if (visualize_plan) {
        marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>(
                "slic3r_coverage_planner/path_marker_array", 100, true);
    }

    ros::ServiceServer plan_path_srv = n.advertiseService("slic3r_coverage_planner/plan_path", planPath);

    ros::spin();
    return 0;
}
