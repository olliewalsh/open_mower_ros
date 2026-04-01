#include <mower_map/nav_point_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mower_map::NavPointLayer, costmap_2d::Layer)

namespace mower_map {

namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;

NavPointLayer::NavPointLayer() : service_nh_("/mower_map_service") {
}

void NavPointLayer::onInitialize() {
  current_ = true;
  default_value_ = costmap_2d::FREE_SPACE;
  matchSize();

  ros::NodeHandle private_nh("~/" + name_);
  private_nh.param("enabled", enabled_, true);
  private_nh.param("side_padding", side_padding_, 0.1);
  private_nh.param("front_padding", front_padding_, 0.15);
  private_nh.param("rear_opening_offset", rear_opening_offset_, 0.05);
  private_nh.param("wall_thickness", wall_thickness_, 0.12);
  private_nh.param("robot_clearance_padding", robot_clearance_padding_, 0.05);
  private_nh.param("apply_timeout_seconds", apply_timeout_seconds_, 5.0);

  refreshFootprintMetrics();

  set_nav_point_srv_ = service_nh_.advertiseService("set_nav_point", &NavPointLayer::setNavPoint, this);
  clear_nav_point_srv_ = service_nh_.advertiseService("clear_nav_point", &NavPointLayer::clearNavPoint, this);
}

void NavPointLayer::matchSize() {
  CostmapLayer::matchSize();
  resetMaps();
}

bool NavPointLayer::isDiscretized() {
  return true;
}

void NavPointLayer::refreshFootprintMetrics() {
  const auto footprint = layered_costmap_->getFootprint();
  if (footprint.empty()) {
    return;
  }

  footprint_front_ = footprint.front().x;
  footprint_rear_ = footprint.front().x;
  footprint_left_ = footprint.front().y;
  footprint_right_ = footprint.front().y;
  for (const auto& point : footprint) {
    footprint_front_ = std::max(footprint_front_, static_cast<double>(point.x));
    footprint_rear_ = std::min(footprint_rear_, static_cast<double>(point.x));
    footprint_left_ = std::max(footprint_left_, static_cast<double>(point.y));
    footprint_right_ = std::min(footprint_right_, static_cast<double>(point.y));
  }
}

std::vector<geometry_msgs::Point> NavPointLayer::makeWorldPolygon(
    const std::vector<std::pair<double, double>>& local_polygon) const {
  tf2::Quaternion q;
  tf2::fromMsg(nav_pose_.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  std::vector<geometry_msgs::Point> world_polygon;
  world_polygon.reserve(local_polygon.size());
  for (const auto& point : local_polygon) {
    geometry_msgs::Point transformed;
    transformed.x = nav_pose_.position.x + point.first * c - point.second * s;
    transformed.y = nav_pose_.position.y + point.first * s + point.second * c;
    transformed.z = 0.0;
    world_polygon.push_back(transformed);
  }
  return world_polygon;
}

std::vector<std::pair<double, double>> NavPointLayer::getRobotFootprintInNavFrame(double robot_x, double robot_y,
                                                                                  double robot_yaw) const {
  std::vector<std::pair<double, double>> local_points;
  const auto footprint = layered_costmap_->getFootprint();
  if (footprint.empty()) {
    return local_points;
  }

  tf2::Quaternion nav_q;
  tf2::fromMsg(nav_pose_.orientation, nav_q);
  double nav_roll = 0.0;
  double nav_pitch = 0.0;
  double nav_yaw = 0.0;
  tf2::Matrix3x3(nav_q).getRPY(nav_roll, nav_pitch, nav_yaw);

  const double robot_c = std::cos(robot_yaw);
  const double robot_s = std::sin(robot_yaw);
  const double nav_c = std::cos(-nav_yaw);
  const double nav_s = std::sin(-nav_yaw);

  local_points.reserve(footprint.size());
  for (const auto& point : footprint) {
    const double world_x = robot_x + point.x * robot_c - point.y * robot_s;
    const double world_y = robot_y + point.x * robot_s + point.y * robot_c;

    const double dx = world_x - nav_pose_.position.x;
    const double dy = world_y - nav_pose_.position.y;

    const double local_x = dx * nav_c - dy * nav_s;
    const double local_y = dx * nav_s + dy * nav_c;
    local_points.emplace_back(local_x, local_y);
  }

  return local_points;
}

bool NavPointLayer::polygonsIntersect(const std::vector<geometry_msgs::Point>& a,
                                      const std::vector<geometry_msgs::Point>& b) const {
  if (a.size() < 3 || b.size() < 3) {
    return false;
  }

  auto toBoostPolygon = [](const std::vector<geometry_msgs::Point>& polygon) {
    BoostPolygon result;
    auto& outer = result.outer();
    outer.reserve(polygon.size() + 1);
    for (const auto& point : polygon) {
      outer.emplace_back(point.x, point.y);
    }
    outer.emplace_back(polygon.front().x, polygon.front().y);
    bg::correct(result);
    return result;
  };

  return bg::intersects(toBoostPolygon(a), toBoostPolygon(b));
}

std::vector<std::vector<geometry_msgs::Point>> NavPointLayer::buildNavObstaclePolygons() const {
  double inner_left = footprint_left_ + side_padding_;
  double inner_right = footprint_right_ - side_padding_;
  const double side_wall_front = footprint_front_ + front_padding_;
  const double side_wall_rear = footprint_rear_ + rear_opening_offset_;
  const double front_wall_front = side_wall_front + wall_thickness_;
  bool include_front_wall = true;

  const auto make_left_wall = [&](double current_inner_left) {
    return makeWorldPolygon({
        {side_wall_rear, current_inner_left},
        {side_wall_front, current_inner_left},
        {side_wall_front, current_inner_left + wall_thickness_},
        {side_wall_rear, current_inner_left + wall_thickness_},
    });
  };
  const auto make_right_wall = [&](double current_inner_right) {
    return makeWorldPolygon({
        {side_wall_rear, current_inner_right - wall_thickness_},
        {side_wall_front, current_inner_right - wall_thickness_},
        {side_wall_front, current_inner_right},
        {side_wall_rear, current_inner_right},
    });
  };
  const auto make_front_wall = [&](double current_inner_left, double current_inner_right) {
    return makeWorldPolygon({
        {side_wall_front, current_inner_right - wall_thickness_},
        {front_wall_front, current_inner_right - wall_thickness_},
        {front_wall_front, current_inner_left + wall_thickness_},
        {side_wall_front, current_inner_left + wall_thickness_},
    });
  };

  if (nav_set_robot_pose_valid_) {
    const auto robot_footprint_local =
        getRobotFootprintInNavFrame(nav_set_robot_x_, nav_set_robot_y_, nav_set_robot_yaw_);
    if (!robot_footprint_local.empty()) {
      double robot_left = robot_footprint_local.front().second;
      double robot_right = robot_footprint_local.front().second;
      for (const auto& point : robot_footprint_local) {
        robot_left = std::max(robot_left, point.second);
        robot_right = std::min(robot_right, point.second);
      }

      const auto robot_footprint_world = makeWorldPolygon(robot_footprint_local);
      if (polygonsIntersect(robot_footprint_world, make_left_wall(inner_left))) {
        inner_left = std::max(inner_left, robot_left + robot_clearance_padding_);
      }
      if (polygonsIntersect(robot_footprint_world, make_right_wall(inner_right))) {
        inner_right = std::min(inner_right, robot_right - robot_clearance_padding_);
      }
      if (polygonsIntersect(robot_footprint_world, make_front_wall(inner_left, inner_right))) {
        include_front_wall = false;
      }
    }
  }

  const double outer_left = inner_left + wall_thickness_;
  const double outer_right = inner_right - wall_thickness_;

  std::vector<std::vector<geometry_msgs::Point>> polygons;

  if (side_wall_rear >= side_wall_front || inner_right >= inner_left || outer_right >= outer_left) {
    return polygons;
  }

  polygons.push_back(make_left_wall(inner_left));
  polygons.push_back(make_right_wall(inner_right));

  if (include_front_wall) {
    polygons.push_back(make_front_wall(inner_left, inner_right));
  }

  return polygons;
}

NavPointLayer::Bounds NavPointLayer::computeObstacleBounds() const {
  Bounds bounds;
  if (!nav_point_active_) {
    return bounds;
  }

  const auto polygons = buildNavObstaclePolygons();
  for (const auto& polygon : polygons) {
    for (const auto& point : polygon) {
      if (!bounds.valid) {
        bounds.min_x = bounds.max_x = point.x;
        bounds.min_y = bounds.max_y = point.y;
        bounds.valid = true;
        continue;
      }
      bounds.min_x = std::min(bounds.min_x, static_cast<double>(point.x));
      bounds.min_y = std::min(bounds.min_y, static_cast<double>(point.y));
      bounds.max_x = std::max(bounds.max_x, static_cast<double>(point.x));
      bounds.max_y = std::max(bounds.max_y, static_cast<double>(point.y));
    }
  }
  return bounds;
}

void NavPointLayer::updateBoundsFrom(const Bounds& bounds, double* min_x, double* min_y, double* max_x,
                                     double* max_y) const {
  if (!bounds.valid) {
    return;
  }
  *min_x = std::min(*min_x, bounds.min_x);
  *min_y = std::min(*min_y, bounds.min_y);
  *max_x = std::max(*max_x, bounds.max_x);
  *max_y = std::max(*max_y, bounds.max_y);
}

void NavPointLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                 double* max_x, double* max_y) {
  if (!enabled_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_yaw_ = robot_yaw;
    robot_pose_valid_ = true;
  }

  refreshFootprintMetrics();

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (nav_point_dirty_) {
    updateBoundsFrom(last_bounds_, min_x, min_y, max_x, max_y);
    const Bounds current_bounds = computeObstacleBounds();
    updateBoundsFrom(current_bounds, min_x, min_y, max_x, max_y);
    last_bounds_ = current_bounds;
    nav_point_dirty_ = false;
    return;
  }

  updateBoundsFrom(last_bounds_, min_x, min_y, max_x, max_y);
}

void NavPointLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/,
                                int /*max_j*/) {
  if (!enabled_) {
    return;
  }

  std::unique_lock<std::mutex> lock(state_mutex_);
  const auto polygons =
      nav_point_active_ ? buildNavObstaclePolygons() : std::vector<std::vector<geometry_msgs::Point>>{};
  applied_generation_ = requested_generation_;
  lock.unlock();
  state_cv_.notify_all();

  for (const auto& poly : polygons) {
    master_grid.setConvexPolygonCost(poly, costmap_2d::LETHAL_OBSTACLE);
  }
}

bool NavPointLayer::setNavPoint(mower_map::SetNavPointSrvRequest& req, mower_map::SetNavPointSrvResponse& /*res*/) {
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    nav_pose_ = req.nav_pose;
    nav_set_robot_x_ = robot_x_;
    nav_set_robot_y_ = robot_y_;
    nav_set_robot_yaw_ = robot_yaw_;
    nav_set_robot_pose_valid_ = robot_pose_valid_;
    nav_point_active_ = true;
    nav_point_dirty_ = true;
    current_ = false;
    generation = ++requested_generation_;
  }
  ROS_INFO_STREAM("NavPointLayer: Set temporary nav obstacle.");
  return waitForAppliedGeneration(generation);
}

bool NavPointLayer::clearNavPoint(mower_map::ClearNavPointSrvRequest& /*req*/,
                                  mower_map::ClearNavPointSrvResponse& /*res*/) {
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!nav_point_active_) {
      return true;
    }
    nav_point_active_ = false;
    nav_set_robot_pose_valid_ = false;
    nav_point_dirty_ = true;
    current_ = false;
    generation = ++requested_generation_;
  }
  ROS_INFO_STREAM("NavPointLayer: Cleared temporary nav obstacle.");
  return waitForAppliedGeneration(generation);
}

bool NavPointLayer::waitForAppliedGeneration(uint64_t generation) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (applied_generation_ >= generation) {
    return true;
  }
  const bool applied = state_cv_.wait_for(lock, std::chrono::duration<double>(apply_timeout_seconds_),
                                          [&]() { return applied_generation_ >= generation; });
  if (!applied) {
    ROS_WARN_STREAM("NavPointLayer: Timed out waiting for nav point generation " << generation << " to apply.");
  }
  return applied;
}

}  // namespace mower_map
