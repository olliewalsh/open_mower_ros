#ifndef MOWER_MAP_NAV_POINT_LAYER_H_
#define MOWER_MAP_NAV_POINT_LAYER_H_

#include <costmap_2d/costmap_layer.h>
#include <geometry_msgs/Pose.h>
#include <mower_map/ClearNavPointSrv.h>
#include <mower_map/SetNavPointSrv.h>
#include <ros/ros.h>

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace mower_map {

class NavPointLayer : public costmap_2d::CostmapLayer {
 public:
  NavPointLayer();

  void onInitialize() override;
  void matchSize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  bool isDiscretized();

 private:
  struct Bounds {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool valid = false;
  };

  bool setNavPoint(mower_map::SetNavPointSrvRequest& req, mower_map::SetNavPointSrvResponse& res);
  bool clearNavPoint(mower_map::ClearNavPointSrvRequest& req, mower_map::ClearNavPointSrvResponse& res);
  std::vector<std::vector<geometry_msgs::Point>> buildNavObstaclePolygons() const;
  std::vector<geometry_msgs::Point> makeWorldPolygon(const std::vector<std::pair<double, double>>& local_polygon) const;
  std::vector<std::pair<double, double>> getRobotFootprintInNavFrame() const;
  Bounds computeObstacleBounds() const;
  void updateBoundsFrom(const Bounds& bounds, double* min_x, double* min_y, double* max_x, double* max_y) const;
  void refreshFootprintMetrics();
  bool waitForAppliedGeneration(uint64_t generation);

  ros::NodeHandle service_nh_;
  ros::ServiceServer set_nav_point_srv_;
  ros::ServiceServer clear_nav_point_srv_;
  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;

  geometry_msgs::Pose nav_pose_;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_yaw_ = 0.0;
  bool robot_pose_valid_ = false;
  bool nav_point_active_ = false;
  bool nav_point_dirty_ = false;
  Bounds last_bounds_;

  double footprint_front_ = 0.0;
  double footprint_rear_ = 0.0;
  double footprint_left_ = 0.0;
  double footprint_right_ = 0.0;
  double side_padding_ = 0.1;
  double front_padding_ = 0.15;
  double rear_opening_offset_ = 0.05;
  double wall_thickness_ = 0.12;
  double robot_clearance_padding_ = 0.05;
  double apply_timeout_seconds_ = 5.0;
  uint64_t requested_generation_ = 0;
  uint64_t applied_generation_ = 0;
};

}  // namespace mower_map

#endif
