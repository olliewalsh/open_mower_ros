#ifndef FTC_LOCAL_PLANNER_SIMPLE_PATH_TRACKER_H_
#define FTC_LOCAL_PLANNER_SIMPLE_PATH_TRACKER_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <mower_msgs/Status.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/buffer.h>

#include "ftc_local_planner/PlannerGetProgress.h"

namespace ftc_local_planner
{
class SimplePathTracker : public mbf_costmap_core::CostmapController
{
public:
  SimplePathTracker() = default;
  explicit SimplePathTracker(bool pure_pursuit) : pure_pursuit_(pure_pursuit) {}
  ~SimplePathTracker() override = default;
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
      const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped& cmd_vel,
      std::string& message) override;
  bool isGoalReached(double dist_tolerance, double angle_tolerance) override;
  bool cancel() override;

private:
  enum class State { PRE_ROTATE, ROTATE_ESCAPE, TRACKING, FINAL_ROTATE, FINISHED };
  struct Projection
  {
    size_t segment{0};
    double fraction{0.0}, path_distance{0.0}, x{0.0}, y{0.0}, heading{0.0}, curvature{0.0};
    double cross_track_error{0.0}, heading_error{0.0}, remaining_distance{0.0};
    bool sharp_corner_ahead{false};
    double corner_distance{0.0};
  };
  struct PathSample { double x{0.0}, y{0.0}, heading{0.0}; size_t segment{0}; };
  bool projectToPath(double x, double y, double yaw, Projection& projection) const;
  PathSample samplePath(double distance) const;
  bool trajectoryIsSafe(double x, double y, double yaw, double linear, double angular,
      std::string& reason) const;
  bool straightEscapeIsSafe(double x, double y, double yaw, double distance) const;
  bool footprintIsSafe(double x, double y, double yaw) const;
  double obstacleClearance(double x, double y, double maximum_distance) const;
  void resetRotationProgress();
  bool rotationHasStalled(double heading_error, const ros::Time& now);
  void resetTrackingProgress();
  bool trackingHasStalled(double path_distance, const ros::Time& now);
  void resetMotionProgress();
  bool motionHasStalled(double x, double y, double yaw, bool motion_commanded, const ros::Time& now);
  double applyAccelerationLimit(double target, double dt);
  double applyAngularAccelerationLimit(double target, double dt);
  void refreshParameters(bool cached);
  bool getProgress(PlannerGetProgressRequest&, PlannerGetProgressResponse& response);
  void statusReceived(const mower_msgs::Status::ConstPtr& status);
  static double yawOf(const geometry_msgs::Quaternion& orientation);
  static double normalizeAngle(double angle);

  bool initialized_{false}, cancelled_{false};
  const bool pure_pursuit_{false};
  State state_{State::FINISHED};
  std::string name_;
  std::unique_ptr<ros::NodeHandle> parameter_nh_;
  ros::WallTime last_parameter_refresh_;
  costmap_2d::Costmap2DROS* costmap_ros_{nullptr};
  std::unique_ptr<base_local_planner::CostmapModel> collision_model_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  std::vector<double> cumulative_distance_;
  size_t current_index_{0};
  double projection_distance_limit_{0.0};
  double last_projected_path_distance_{0.0};
  bool have_last_projection_pose_{false};
  double last_projection_x_{0.0}, last_projection_y_{0.0};
  bool goal_miss_active_{false};
  ros::Time goal_miss_started_;
  double best_goal_distance_{std::numeric_limits<double>::infinity()};
  double last_linear_command_{0.0}, last_angular_command_{0.0};
  ros::Time last_command_time_;
  bool rotate_progress_active_{false};
  double best_rotate_error_{std::numeric_limits<double>::infinity()};
  ros::Time rotate_progress_started_;
  double rotate_escape_start_x_{0.0}, rotate_escape_start_y_{0.0};
  double rotate_escape_direction_{1.0};
  ros::Time rotate_escape_started_;
  int rotate_escape_attempt_count_{0};
  bool tracking_progress_active_{false};
  double tracking_progress_reference_distance_{0.0};
  ros::Time tracking_progress_started_;
  bool motion_progress_active_{false};
  double motion_progress_reference_x_{0.0}, motion_progress_reference_y_{0.0};
  double motion_progress_reference_yaw_{0.0};
  ros::Time motion_progress_started_;
  mower_msgs::Status mower_status_;
  ros::Publisher plan_publisher_, tracking_point_publisher_;
  ros::Publisher cross_track_error_publisher_, heading_error_publisher_;
  ros::Publisher path_curvature_publisher_, remaining_distance_publisher_;
  ros::ServiceServer progress_server_;
  ros::Subscriber status_subscriber_;

  double heading_gain_{2.0}, cross_track_gain_{2.0}, cross_track_curvature_scale_{0.25};
  double softening_speed_{0.1};
  double mowing_speed_{0.38}, minimum_tracking_speed_{0.1}, max_angular_speed_{1.8};
  double boundary_slowdown_distance_{0.5}, boundary_minimum_speed_{0.1};
  double max_acceleration_{0.15}, max_deceleration_{0.3}, cross_track_slowdown_gain_{3.0};
  double max_angular_acceleration_{2.0}, max_angular_deceleration_{3.0};
  double rotate_threshold_{0.7}, rotate_tolerance_{0.17};
  double rotate_progress_timeout_{5.0}, rotate_progress_angle_{0.05};
  double rotate_escape_distance_{0.1}, rotate_escape_speed_{0.1}, rotate_escape_timeout_{5.0};
  int rotate_escape_attempts_{1};
  double tracking_progress_timeout_{5.0}, tracking_progress_distance_{0.05};
  double tracking_progress_min_speed_{0.05};
  double motion_progress_timeout_{10.0}, motion_progress_distance_{0.03}, motion_progress_angle_{0.05};
  double motion_progress_min_linear_{0.05}, motion_progress_min_angular_{0.1};
  double curvature_preview_distance_{0.35}, curvature_angular_fraction_{0.7};
  double minimum_lookahead_{0.15}, maximum_lookahead_{0.35}, lookahead_time_{0.4};
  double sharp_corner_angle_{1.22}, turnaround_angle_threshold_{2.1}, turnaround_preview_distance_{1.5};
  double corner_slowdown_distance_{0.4}, corner_position_tolerance_{0.05};
  double goal_distance_tolerance_{0.1}, goal_angle_tolerance_{0.17}, goal_slowdown_distance_{0.5};
  double goal_position_timeout_{2.0};
  double projection_initial_allowance_{0.5};
  double projection_distance_factor_{1.5};
  double projection_heading_tolerance_{0.52};
  double projection_pose_deadband_{0.002}, projection_max_pose_step_{0.25};
  bool check_collisions_{true}, unknown_is_obstacle_{true};
  double collision_horizon_{2.0}, collision_time_step_{0.1};
  double braking_deceleration_{0.4}, reaction_time_{0.2}, collision_margin_{0.1};
  bool mow_load_filter_initialized_{false};
  double filtered_mow_current_{0.0}, filtered_mow_rpm_{0.0};
  ros::Time last_mow_load_filter_time_;
  double mow_load_filter_attack_time_constant_{0.25}, mow_load_filter_release_time_constant_{1.5};
  double max_mow_motor_current_{6.0}, mow_current_gain_{0.125};
  double min_mow_motor_rpm_{2000.0}, mow_rpm_gain_{0.0002};
};

class PurePursuitTracker : public SimplePathTracker
{
public:
  PurePursuitTracker() : SimplePathTracker(true) {}
};
}  // namespace ftc_local_planner
#endif
