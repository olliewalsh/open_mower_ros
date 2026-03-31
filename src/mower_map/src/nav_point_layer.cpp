#include <mower_map/nav_point_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mower_map::NavPointLayer, costmap_2d::Layer)

namespace mower_map {

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

std::vector<std::vector<geometry_msgs::Point>> NavPointLayer::buildNavObstaclePolygons() const {
  const double inner_left = footprint_left_ + side_padding_;
  const double inner_right = footprint_right_ - side_padding_;
  const double outer_left = inner_left + wall_thickness_;
  const double outer_right = inner_right - wall_thickness_;
  const double side_wall_front = footprint_front_ + front_padding_;
  const double side_wall_rear = footprint_rear_ + rear_opening_offset_;
  const double front_wall_front = side_wall_front + wall_thickness_;

  std::vector<std::vector<geometry_msgs::Point>> polygons;

  const auto append_polygon = [&](const std::vector<std::pair<double, double>>& local_polygon) {
    polygons.push_back(makeWorldPolygon(local_polygon));
  };

  append_polygon({
      {side_wall_rear, inner_left},
      {side_wall_front, inner_left},
      {side_wall_front, outer_left},
      {side_wall_rear, outer_left},
  });

  append_polygon({
      {side_wall_rear, outer_right},
      {side_wall_front, outer_right},
      {side_wall_front, inner_right},
      {side_wall_rear, inner_right},
  });

  append_polygon({
      {side_wall_front, outer_right},
      {front_wall_front, outer_right},
      {front_wall_front, outer_left},
      {side_wall_front, outer_left},
  });

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

void NavPointLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x,
                                 double* min_y, double* max_x, double* max_y) {
  if (!enabled_) {
    return;
  }

  refreshFootprintMetrics();

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
  if (!enabled_ || !nav_point_active_) {
    return;
  }

  const auto polygons = buildNavObstaclePolygons();
  for (const auto& poly : polygons) {
    master_grid.setConvexPolygonCost(poly, costmap_2d::LETHAL_OBSTACLE);
  }
}

bool NavPointLayer::setNavPoint(mower_map::SetNavPointSrvRequest& req, mower_map::SetNavPointSrvResponse& /*res*/) {
  nav_pose_ = req.nav_pose;
  nav_point_active_ = true;
  nav_point_dirty_ = true;
  current_ = false;
  ROS_INFO_STREAM("NavPointLayer: Set temporary nav obstacle.");
  return true;
}

bool NavPointLayer::clearNavPoint(mower_map::ClearNavPointSrvRequest& /*req*/,
                                  mower_map::ClearNavPointSrvResponse& /*res*/) {
  if (!nav_point_active_) {
    return true;
  }
  nav_point_active_ = false;
  nav_point_dirty_ = true;
  current_ = false;
  ROS_INFO_STREAM("NavPointLayer: Cleared temporary nav obstacle.");
  return true;
}

}  // namespace mower_map
