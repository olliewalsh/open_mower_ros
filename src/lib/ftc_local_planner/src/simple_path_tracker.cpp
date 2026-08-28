#include "ftc_local_planner/simple_path_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::SimplePathTracker, mbf_costmap_core::CostmapController)
PLUGINLIB_EXPORT_CLASS(ftc_local_planner::PurePursuitTracker, mbf_costmap_core::CostmapController)

namespace ftc_local_planner
{
namespace { constexpr uint32_t SUCCESS = 0, CANCELED = 101, COLLISION = 104, MISSED_GOAL = 107, INVALID_PATH = 111; }

void SimplePathTracker::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_) return;
  name_ = std::move(name);
  costmap_ros_ = costmap_ros;
  collision_model_.reset(new base_local_planner::CostmapModel(*costmap_ros_->getCostmap()));
  ros::NodeHandle nh("~/" + name_);
  parameter_nh_.reset(new ros::NodeHandle(nh));
#define LOAD(p) nh.param(#p, p##_, p##_)
  LOAD(heading_gain); LOAD(cross_track_gain); LOAD(cross_track_curvature_scale); LOAD(softening_speed);
  LOAD(mowing_speed); LOAD(minimum_tracking_speed); LOAD(max_angular_speed);
  LOAD(max_acceleration); LOAD(max_deceleration); LOAD(cross_track_slowdown_gain);
  LOAD(rotate_threshold); LOAD(rotate_tolerance); LOAD(goal_distance_tolerance);
  LOAD(rotate_progress_timeout); LOAD(rotate_progress_angle);
  LOAD(rotate_escape_distance); LOAD(rotate_escape_speed); LOAD(rotate_escape_timeout); LOAD(rotate_escape_attempts);
  LOAD(tracking_progress_timeout); LOAD(tracking_progress_distance); LOAD(tracking_progress_min_speed);
  LOAD(motion_progress_timeout); LOAD(motion_progress_distance); LOAD(motion_progress_angle);
  LOAD(motion_progress_min_linear); LOAD(motion_progress_min_angular);
  LOAD(curvature_preview_distance); LOAD(curvature_angular_fraction);
  LOAD(minimum_lookahead); LOAD(maximum_lookahead); LOAD(lookahead_time); LOAD(sharp_corner_angle);
  LOAD(turnaround_angle_threshold); LOAD(turnaround_preview_distance);
  LOAD(corner_slowdown_distance); LOAD(corner_position_tolerance);
  LOAD(goal_angle_tolerance); LOAD(goal_slowdown_distance); LOAD(goal_position_timeout);
  LOAD(projection_search_window);
  LOAD(projection_initial_allowance);
  LOAD(projection_distance_factor); LOAD(projection_pose_deadband); LOAD(projection_max_pose_step);
  LOAD(check_collisions); LOAD(unknown_is_obstacle); LOAD(collision_horizon);
  LOAD(collision_time_step); LOAD(braking_deceleration); LOAD(reaction_time); LOAD(collision_margin);
  LOAD(max_mow_motor_current); LOAD(mow_current_gain); LOAD(min_mow_motor_rpm); LOAD(mow_rpm_gain);
#undef LOAD
  last_parameter_refresh_ = ros::WallTime::now();
  plan_publisher_ = nh.advertise<nav_msgs::Path>("global_plan", 1, true);
  tracking_point_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("tracking_point", 1);
  cross_track_error_publisher_ = nh.advertise<std_msgs::Float64>("cross_track_error", 1);
  heading_error_publisher_ = nh.advertise<std_msgs::Float64>("heading_error", 1);
  path_curvature_publisher_ = nh.advertise<std_msgs::Float64>("path_curvature", 1);
  remaining_distance_publisher_ = nh.advertise<std_msgs::Float64>("remaining_distance", 1);
  progress_server_ = nh.advertiseService("planner_get_progress", &SimplePathTracker::getProgress, this);
  status_subscriber_ = nh.subscribe<mower_msgs::Status>("/ll/mower_status", 1,
      &SimplePathTracker::statusReceived, this, ros::TransportHints().tcpNoDelay(true));
  initialized_ = true;
  ROS_INFO_STREAM((pure_pursuit_ ? "PurePursuitTracker '" : "SimplePathTracker '")
      << name_ << "' initialized");
}

bool SimplePathTracker::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (plan.size() < 2) { plan_.clear(); state_ = State::FINISHED; return false; }
  plan_ = plan; current_index_ = 0; last_linear_command_ = 0.0;
  projection_distance_limit_ = projection_initial_allowance_;
  last_projected_path_distance_ = 0.0;
  have_last_projection_pose_ = false;
  goal_miss_active_ = false;
  best_goal_distance_ = std::numeric_limits<double>::infinity();
  resetRotationProgress();
  resetTrackingProgress();
  resetMotionProgress();
  rotate_escape_attempt_count_ = 0;
  cumulative_distance_.assign(plan_.size(), 0.0);
  for (size_t i = 1; i < plan_.size(); ++i)
    cumulative_distance_[i] = cumulative_distance_[i - 1] +
        std::hypot(plan_[i].pose.position.x - plan_[i - 1].pose.position.x,
                   plan_[i].pose.position.y - plan_[i - 1].pose.position.y);
  last_command_time_ = ros::Time::now(); cancelled_ = false; state_ = State::PRE_ROTATE;
  nav_msgs::Path path; path.header = plan.front().header; path.poses = plan; plan_publisher_.publish(path);
  return true;
}

uint32_t SimplePathTracker::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped&, geometry_msgs::TwistStamped& command,
    std::string& message)
{
  const ros::WallTime now = ros::WallTime::now();
  if ((now - last_parameter_refresh_).toSec() >= 1.0) {
    refreshParameters(true);
    last_parameter_refresh_ = now;
  }
  command.header.stamp = ros::Time::now(); command.header.frame_id = "base_link";
  command.twist = geometry_msgs::Twist();
  if (cancelled_) return CANCELED;
  if (state_ == State::FINISHED) return SUCCESS;
  if (plan_.size() < 2) { message = "No valid path"; return INVALID_PATH; }
  const double x = pose.pose.position.x, y = pose.pose.position.y, yaw = yawOf(pose.pose.orientation);
  if (have_last_projection_pose_) {
    const double pose_step = std::hypot(x - last_projection_x_, y - last_projection_y_);
    const double accepted_step = pose_step >= std::max(0.0, projection_pose_deadband_) ?
        std::min(pose_step, std::max(0.0, projection_max_pose_step_)) : 0.0;
    projection_distance_limit_ = last_projected_path_distance_ +
        std::max(0.0, projection_distance_factor_) * accepted_step;
  }
  last_projection_x_ = x; last_projection_y_ = y; have_last_projection_pose_ = true;
  if (current_index_ < cumulative_distance_.size()) {
    last_projected_path_distance_ = std::max(
        last_projected_path_distance_, cumulative_distance_[current_index_]);
    projection_distance_limit_ = std::max(
        projection_distance_limit_, last_projected_path_distance_);
  }
  Projection p;
  if (!projectToPath(x, y, yaw, p)) { message = "Cannot project pose onto path"; return INVALID_PATH; }
  last_projected_path_distance_ = p.path_distance;
  std_msgs::Float64 diagnostic;
  diagnostic.data = p.cross_track_error; cross_track_error_publisher_.publish(diagnostic);
  diagnostic.data = p.heading_error; heading_error_publisher_.publish(diagnostic);
  diagnostic.data = p.curvature; path_curvature_publisher_.publish(diagnostic);
  diagnostic.data = p.remaining_distance; remaining_distance_publisher_.publish(diagnostic);
  current_index_ = std::max(current_index_, p.segment);
  double control_curvature = p.curvature;
  if (pure_pursuit_) {
    const double minimum = std::max(0.01, minimum_lookahead_);
    const double maximum = std::max(minimum, maximum_lookahead_);
    const double lookahead = std::max(minimum, std::min(maximum,
        minimum + std::max(0.0, lookahead_time_) * std::abs(last_linear_command_)));
    const PathSample target = samplePath(p.path_distance + lookahead);
    const double dx = target.x - x, dy = target.y - y;
    const double distance2 = dx * dx + dy * dy;
    const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    control_curvature = distance2 > 1e-8 ? 2.0 * lateral / distance2 : 0.0;
    if (p.sharp_corner_ahead) control_curvature = 0.0;
  }
  geometry_msgs::PoseStamped point = pose; point.pose.position.x = p.x; point.pose.position.y = p.y;
  tf2::Quaternion q; q.setRPY(0, 0, p.heading); point.pose.orientation = tf2::toMsg(q);
  tracking_point_publisher_.publish(point);
  const auto& goal = plan_.back().pose.position;
  const double goal_distance = std::hypot(goal.x - x, goal.y - y);
  const ros::Time control_time = ros::Time::now();

  if (state_ == State::PRE_ROTATE) {
    if (std::abs(p.heading_error) <= rotate_tolerance_) {
      state_ = State::TRACKING;
      resetRotationProgress();
      rotate_escape_attempt_count_ = 0;
    } else if (rotationHasStalled(p.heading_error, control_time)) {
      if (rotate_escape_attempt_count_ >= std::max(0, rotate_escape_attempts_)) {
        message = "Rotation stalled after escape recovery";
        return MISSED_GOAL;
      }
      const double distance = std::max(0.0, rotate_escape_distance_);
      const bool forward_safe = distance > 0.0 && straightEscapeIsSafe(x, y, yaw, distance);
      const bool reverse_safe = distance > 0.0 && straightEscapeIsSafe(x, y, yaw, -distance);
      if (!forward_safe && !reverse_safe) {
        message = "Rotation stalled and no collision-free escape is available";
        return COLLISION;
      }
      rotate_escape_direction_ = forward_safe ? 1.0 : -1.0;
      rotate_escape_start_x_ = x;
      rotate_escape_start_y_ = y;
      rotate_escape_started_ = control_time;
      rotate_escape_attempt_count_++;
      resetRotationProgress();
      state_ = State::ROTATE_ESCAPE;
      last_linear_command_ = 0.0;
      ROS_WARN_STREAM("SimplePathTracker: rotation stalled; escaping "
          << (rotate_escape_direction_ > 0.0 ? "forward" : "in reverse"));
    } else {
      command.twist.angular.z = std::max(-max_angular_speed_,
          std::min(max_angular_speed_, heading_gain_ * p.heading_error));
    }
  }
  if (state_ == State::ROTATE_ESCAPE) {
    const double travelled = std::hypot(x - rotate_escape_start_x_, y - rotate_escape_start_y_);
    if (travelled >= std::max(0.0, rotate_escape_distance_)) {
      state_ = State::PRE_ROTATE;
      resetRotationProgress();
      last_linear_command_ = 0.0;
    } else if ((control_time - rotate_escape_started_).toSec() >=
               std::max(0.0, rotate_escape_timeout_)) {
      message = "Rotation escape stalled after moving " + std::to_string(travelled) + " m";
      return MISSED_GOAL;
    } else {
      const double remaining = std::max(0.0, rotate_escape_distance_ - travelled);
      if (!straightEscapeIsSafe(x, y, yaw, rotate_escape_direction_ * remaining)) {
        message = "Rotation escape path became unsafe";
        return COLLISION;
      }
      command.twist.linear.x = rotate_escape_direction_ * std::max(0.0, rotate_escape_speed_);
    }
  }
  if (state_ == State::TRACKING) {
    if (goal_distance <= goal_distance_tolerance_ &&
        p.remaining_distance <= goal_distance_tolerance_) {
      goal_miss_active_ = false;
      state_ = State::FINAL_ROTATE;
      last_linear_command_ = 0;
    }
    else if (p.remaining_distance <= goal_distance_tolerance_) {
      const double goal_heading = std::atan2(goal.y - y, goal.x - x);
      const double goal_heading_error = normalizeAngle(goal_heading - yaw);
      if (!goal_miss_active_ || goal_distance + 0.01 < best_goal_distance_) {
        goal_miss_active_ = true;
        goal_miss_started_ = ros::Time::now();
        best_goal_distance_ = goal_distance;
      }
      if ((ros::Time::now() - goal_miss_started_).toSec() >=
          std::max(0.0, goal_position_timeout_)) {
        message = "Final approach stalled with goal distance " + std::to_string(goal_distance) +
            " m, tolerance " + std::to_string(goal_distance_tolerance_) + " m";
        return MISSED_GOAL;
      }
      if (std::abs(goal_heading_error) > rotate_tolerance_) {
        goal_miss_started_ = ros::Time::now();
        last_linear_command_ = 0.0;
        command.twist.angular.z = std::max(-max_angular_speed_,
            std::min(max_angular_speed_, heading_gain_ * goal_heading_error));
      } else {
        const double braking_distance = std::max(0.0, goal_distance - goal_distance_tolerance_);
        const double target = std::min(mowing_speed_,
            std::sqrt(2.0 * max_deceleration_ * braking_distance));
        const ros::Time command_time = ros::Time::now();
        double dt = (command_time - last_command_time_).toSec();
        last_command_time_ = command_time;
        if (!std::isfinite(dt) || dt <= 0 || dt > 1) dt = 0.1;
        command.twist.linear.x = applyAccelerationLimit(target, dt);
        command.twist.angular.z = std::max(-max_angular_speed_,
            std::min(max_angular_speed_, heading_gain_ * goal_heading_error));
      }
    }
    else if (std::abs(p.heading_error) > rotate_threshold_) {
      goal_miss_active_ = false;
      state_ = State::PRE_ROTATE; last_linear_command_ = 0;
      resetRotationProgress();
      command.twist.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, heading_gain_ * p.heading_error));
    } else {
      goal_miss_active_ = false;
      const double heading_scale = std::pow(std::max(0.0, std::cos(p.heading_error)), 2);
      const double tracking_scale = heading_scale / (1.0 + cross_track_slowdown_gain_ * std::abs(p.cross_track_error));
      double target = mowing_speed_ * tracking_scale;
      if (tracking_scale <= 0.05)
        target = 0.0;
      else if (p.remaining_distance >= goal_slowdown_distance_)
        target = std::max(minimum_tracking_speed_, target);
      if (std::abs(control_curvature) > 1e-6)
        target = std::min(target, curvature_angular_fraction_ * max_angular_speed_ /
            std::abs(control_curvature));
      if (p.sharp_corner_ahead) {
        if (p.corner_distance <= corner_position_tolerance_) {
          current_index_ = std::min(p.segment + 1, plan_.size() - 2);
          last_projected_path_distance_ = cumulative_distance_[p.segment + 1];
          projection_distance_limit_ = last_projected_path_distance_;
          state_ = State::PRE_ROTATE;
          resetRotationProgress();
          last_linear_command_ = 0.0;
          target = 0.0;
        } else if (p.corner_distance < corner_slowdown_distance_) {
          const double braking_distance = std::max(0.0,
              p.corner_distance - corner_position_tolerance_);
          target = std::min(target, std::sqrt(2.0 * max_deceleration_ * braking_distance));
        }
      }
      if (p.remaining_distance < goal_slowdown_distance_) target *= std::max(0.0, p.remaining_distance / goal_slowdown_distance_);
      if (mower_status_.mow_enabled) {
        const double amps = std::max(0.0, double(mower_status_.mower_esc_current) - max_mow_motor_current_);
        const double rpm = std::max(0.0, min_mow_motor_rpm_ - std::abs(double(mower_status_.mower_motor_rpm)));
        target = std::min(target, std::max(minimum_tracking_speed_, mowing_speed_ - amps * mow_current_gain_));
        target = std::min(target, std::max(minimum_tracking_speed_, mowing_speed_ - rpm * mow_rpm_gain_));
      }
      const ros::Time now = ros::Time::now(); double dt = (now - last_command_time_).toSec(); last_command_time_ = now;
      if (!std::isfinite(dt) || dt <= 0 || dt > 1) dt = 0.1;
      const double linear = applyAccelerationLimit(target, dt);
      const double effective_cross_track_gain = cross_track_gain_ /
          (1.0 + std::max(0.0, cross_track_curvature_scale_) * std::abs(p.curvature));
      const double angular = pure_pursuit_ ? linear * control_curvature :
          linear * p.curvature + heading_gain_ * p.heading_error -
          std::atan2(effective_cross_track_gain * p.cross_track_error,
                     std::abs(linear) + softening_speed_);
      command.twist.linear.x = linear;
      command.twist.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular));
    }
  }
  if (state_ == State::FINAL_ROTATE) {
    const double error = normalizeAngle(yawOf(plan_.back().pose.orientation) - yaw);
    if (std::abs(error) <= goal_angle_tolerance_) {
      state_ = State::FINISHED;
      resetRotationProgress();
    } else if (rotationHasStalled(error, control_time)) {
      message = "Final rotation stalled";
      return MISSED_GOAL;
    } else {
      command.twist.angular.z = std::max(-max_angular_speed_,
          std::min(max_angular_speed_, heading_gain_ * error));
    }
  }
  if (state_ == State::FINISHED) command.twist = geometry_msgs::Twist();
  if (state_ != State::ROTATE_ESCAPE &&
      !trajectoryIsSafe(x, y, yaw, command.twist.linear.x, command.twist.angular.z, message)) {
    command.twist = geometry_msgs::Twist(); last_linear_command_ = 0; return COLLISION;
  }
  const bool normal_tracking = state_ == State::TRACKING &&
      std::abs(command.twist.linear.x) >= std::max(0.0, tracking_progress_min_speed_);
  if (!normal_tracking) {
    resetTrackingProgress();
  } else if (trackingHasStalled(p.path_distance, control_time)) {
    command.twist = geometry_msgs::Twist();
    last_linear_command_ = 0.0;
    message = "Tracking stalled with no path progress for " +
        std::to_string(std::max(0.0, tracking_progress_timeout_)) + " s";
    return MISSED_GOAL;
  }
  if (state_ == State::FINISHED) {
    resetMotionProgress();
  } else {
    const bool motion_commanded =
        std::abs(command.twist.linear.x) >= std::max(0.0, motion_progress_min_linear_) ||
        std::abs(command.twist.angular.z) >= std::max(0.0, motion_progress_min_angular_);
    if (motionHasStalled(x, y, yaw, motion_commanded, control_time)) {
      command.twist = geometry_msgs::Twist();
      last_linear_command_ = 0.0;
      message = "Controller stalled with no position or heading progress for " +
          std::to_string(std::max(0.0, motion_progress_timeout_)) + " s";
      return MISSED_GOAL;
    }
  }
  return SUCCESS;
}

bool SimplePathTracker::projectToPath(double x, double y, double yaw, Projection& result) const
{
  const size_t first = current_index_;
  const size_t last = std::min(plan_.size() - 2, first + size_t(std::max(1, projection_search_window_)));
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = first; i <= last; ++i) {
    const auto& a = plan_[i].pose.position; const auto& b = plan_[i + 1].pose.position;
    const double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx + dy*dy;
    if (length2 < 1e-10) continue;
    if (cumulative_distance_[i + 1] < last_projected_path_distance_)
      continue;
    if (cumulative_distance_[i] > projection_distance_limit_)
      continue;
    const double segment_length = std::sqrt(length2);
    const double minimum_fraction = std::max(0.0, std::min(1.0,
        (last_projected_path_distance_ - cumulative_distance_[i]) / segment_length));
    const double reachable_fraction = std::min(1.0,
        (projection_distance_limit_ - cumulative_distance_[i]) / segment_length);
    const double t = std::max(minimum_fraction, std::min(reachable_fraction,
        ((x-a.x)*dx + (y-a.y)*dy)/length2));
    const double px=a.x+t*dx, py=a.y+t*dy, distance2=(x-px)*(x-px)+(y-py)*(y-py);
    if (distance2 < best) {
      best=distance2; result.segment=i; result.fraction=t; result.x=px; result.y=py;
      result.heading=std::atan2(dy,dx);
    }
  }
  if (!std::isfinite(best)) return false;
  const double segment_length = cumulative_distance_[result.segment + 1] -
      cumulative_distance_[result.segment];
  const double path_distance = cumulative_distance_[result.segment] +
      result.fraction * segment_length;
  result.path_distance = path_distance;
  const double half_preview = std::max(0.02, curvature_preview_distance_ * 0.5);
  const PathSample behind = samplePath(path_distance - half_preview);
  const PathSample ahead = samplePath(path_distance + half_preview);
  if (std::hypot(ahead.x - behind.x, ahead.y - behind.y) > 1e-6)
    result.heading = std::atan2(ahead.y - behind.y, ahead.x - behind.x);
  const PathSample curvature_behind = samplePath(path_distance - curvature_preview_distance_);
  const PathSample curvature_ahead = samplePath(path_distance + curvature_preview_distance_);
  const double curvature_span = std::max(0.04, std::min(cumulative_distance_.back(),
      path_distance + curvature_preview_distance_) - std::max(0.0,
      path_distance - curvature_preview_distance_));
  result.curvature = normalizeAngle(curvature_ahead.heading - curvature_behind.heading) /
      curvature_span;
  result.cross_track_error=-std::sin(result.heading)*(x-result.x)+
      std::cos(result.heading)*(y-result.y);
  result.heading_error=normalizeAngle(result.heading-yaw);
  const auto& end=plan_[result.segment+1].pose.position;
  result.remaining_distance=std::hypot(end.x-result.x,end.y-result.y);
  for(size_t i=result.segment+1;i+1<plan_.size();++i)
    result.remaining_distance+=std::hypot(plan_[i+1].pose.position.x-plan_[i].pose.position.x,plan_[i+1].pose.position.y-plan_[i].pose.position.y);
  result.corner_distance = std::hypot(end.x - result.x, end.y - result.y);
  const double current_heading = std::atan2(
      end.y - plan_[result.segment].pose.position.y,
      end.x - plan_[result.segment].pose.position.x);
  if (result.segment + 2 < plan_.size()) {
    const auto& next = plan_[result.segment + 2].pose.position;
    const double next_dx = next.x - end.x, next_dy = next.y - end.y;
    if (std::hypot(next_dx, next_dy) > 1e-6) {
      result.sharp_corner_ahead = std::abs(normalizeAngle(
          std::atan2(next_dy, next_dx) - current_heading)) >= sharp_corner_angle_;
    }
  }
  if (!result.sharp_corner_ahead) {
    double accumulated_turn = 0.0;
    double previous_heading = current_heading;
    double vertex_distance = result.corner_distance;
    double first_turn_distance = 0.0;
    bool have_first_turn = false;
    for (size_t segment = result.segment + 1; segment + 1 < plan_.size(); ++segment) {
      if (vertex_distance > std::max(0.0, turnaround_preview_distance_)) break;
      const auto& a = plan_[segment].pose.position;
      const auto& b = plan_[segment + 1].pose.position;
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double length = std::hypot(dx, dy);
      if (length > 1e-6) {
        const double heading = std::atan2(dy, dx);
        const double turn = normalizeAngle(heading - previous_heading);
        if (!have_first_turn && std::abs(turn) > 0.05) {
          first_turn_distance = vertex_distance;
          have_first_turn = true;
        }
        accumulated_turn += turn;
        previous_heading = heading;
        if (have_first_turn && std::abs(accumulated_turn) >= turnaround_angle_threshold_) {
          result.sharp_corner_ahead = true;
          result.corner_distance = first_turn_distance;
          break;
        }
      }
      vertex_distance += length;
    }
  }
  if (result.sharp_corner_ahead) {
    result.heading = current_heading;
    result.heading_error = normalizeAngle(result.heading - yaw);
    result.cross_track_error = -std::sin(result.heading) * (x - result.x) +
        std::cos(result.heading) * (y - result.y);
    result.curvature = 0.0;
  }
  return true;
}

SimplePathTracker::PathSample SimplePathTracker::samplePath(double distance) const
{
  PathSample sample;
  distance = std::max(0.0, std::min(cumulative_distance_.back(), distance));
  auto upper = std::upper_bound(cumulative_distance_.begin(), cumulative_distance_.end(), distance);
  size_t segment = upper == cumulative_distance_.begin() ? 0 :
      static_cast<size_t>(std::distance(cumulative_distance_.begin(), upper) - 1);
  segment = std::min(segment, plan_.size() - 2);
  while (segment + 1 < plan_.size() - 1 &&
         cumulative_distance_[segment + 1] - cumulative_distance_[segment] < 1e-10)
    ++segment;
  const double length = cumulative_distance_[segment + 1] - cumulative_distance_[segment];
  const double fraction = length > 1e-10 ?
      (distance - cumulative_distance_[segment]) / length : 0.0;
  const auto& a = plan_[segment].pose.position;
  const auto& b = plan_[segment + 1].pose.position;
  sample.x = a.x + fraction * (b.x - a.x);
  sample.y = a.y + fraction * (b.y - a.y);
  sample.heading = std::atan2(b.y - a.y, b.x - a.x);
  sample.segment = segment;
  return sample;
}

bool SimplePathTracker::trajectoryIsSafe(double x,double y,double yaw,double linear,double angular,std::string& reason) const
{
  if (!check_collisions_) return true;
  if (!footprintIsSafe(x,y,yaw)) { reason="Current footprint is in collision"; return false; }
  const double stopping = braking_deceleration_ > 1e-6 ? std::abs(linear)/braking_deceleration_+reaction_time_ : collision_horizon_;
  const double horizon=std::max(collision_horizon_,stopping+collision_margin_/std::max(std::abs(linear),0.05));
  const double step=std::max(0.01,collision_time_step_);
  for(double elapsed=step;elapsed<=horizon+1e-9;elapsed+=step) {
    if(std::abs(angular)<1e-6){x+=linear*std::cos(yaw)*step;y+=linear*std::sin(yaw)*step;}
    else {const double next=yaw+angular*step;x+=linear/angular*(std::sin(next)-std::sin(yaw));y-=linear/angular*(std::cos(next)-std::cos(yaw));yaw=next;}
    if(!footprintIsSafe(x,y,yaw)){reason="Predicted footprint trajectory is in collision";return false;}
  }
  return true;
}

bool SimplePathTracker::straightEscapeIsSafe(double x, double y, double yaw, double distance) const
{
  if (!footprintIsSafe(x, y, yaw)) return false;
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(distance) / 0.02)));
  const double step = distance / steps;
  for (int i = 0; i < steps; ++i) {
    x += step * std::cos(yaw);
    y += step * std::sin(yaw);
    if (!footprintIsSafe(x, y, yaw)) return false;
  }
  return true;
}

bool SimplePathTracker::footprintIsSafe(double x,double y,double yaw) const
{
  auto* layered=costmap_ros_->getLayeredCostmap();
  const double cost=collision_model_->footprintCost(x,y,yaw,costmap_ros_->getRobotFootprint(),layered->getInscribedRadius(),layered->getCircumscribedRadius());
  return cost == -2.0 ? !unknown_is_obstacle_ : cost >= 0.0;
}
void SimplePathTracker::resetRotationProgress()
{
  rotate_progress_active_ = false;
  best_rotate_error_ = std::numeric_limits<double>::infinity();
  rotate_progress_started_ = ros::Time();
}

bool SimplePathTracker::rotationHasStalled(double heading_error, const ros::Time& now)
{
  const double error = std::abs(heading_error);
  if (!rotate_progress_active_ || now < rotate_progress_started_) {
    rotate_progress_active_ = true;
    best_rotate_error_ = error;
    rotate_progress_started_ = now;
    return false;
  }
  if (error <= best_rotate_error_ - std::max(0.0, rotate_progress_angle_)) {
    best_rotate_error_ = error;
    rotate_progress_started_ = now;
  }
  return rotate_progress_timeout_ > 0.0 &&
      (now - rotate_progress_started_).toSec() >= rotate_progress_timeout_;
}

void SimplePathTracker::resetTrackingProgress()
{
  tracking_progress_active_ = false;
  tracking_progress_reference_distance_ = 0.0;
  tracking_progress_started_ = ros::Time();
}

bool SimplePathTracker::trackingHasStalled(double path_distance, const ros::Time& now)
{
  if (!tracking_progress_active_ || now < tracking_progress_started_) {
    tracking_progress_active_ = true;
    tracking_progress_reference_distance_ = path_distance;
    tracking_progress_started_ = now;
    return false;
  }
  if (path_distance >= tracking_progress_reference_distance_ +
                       std::max(0.0, tracking_progress_distance_)) {
    tracking_progress_reference_distance_ = path_distance;
    tracking_progress_started_ = now;
  }
  return tracking_progress_timeout_ > 0.0 &&
      (now - tracking_progress_started_).toSec() >= tracking_progress_timeout_;
}

void SimplePathTracker::resetMotionProgress()
{
  motion_progress_active_ = false;
  motion_progress_started_ = ros::Time();
}

bool SimplePathTracker::motionHasStalled(double x, double y, double yaw, bool motion_commanded, const ros::Time& now)
{
  if (!motion_progress_active_) {
    if (!motion_commanded) return false;
    motion_progress_active_ = true;
    motion_progress_reference_x_ = x;
    motion_progress_reference_y_ = y;
    motion_progress_reference_yaw_ = yaw;
    motion_progress_started_ = now;
    return false;
  }
  if (now < motion_progress_started_) {
    motion_progress_reference_x_ = x;
    motion_progress_reference_y_ = y;
    motion_progress_reference_yaw_ = yaw;
    motion_progress_started_ = now;
    return false;
  }
  const double distance = std::hypot(x - motion_progress_reference_x_, y - motion_progress_reference_y_);
  const double angle = std::abs(normalizeAngle(yaw - motion_progress_reference_yaw_));
  if (distance >= std::max(0.0, motion_progress_distance_) ||
      angle >= std::max(0.0, motion_progress_angle_)) {
    motion_progress_reference_x_ = x;
    motion_progress_reference_y_ = y;
    motion_progress_reference_yaw_ = yaw;
    motion_progress_started_ = now;
  }
  return motion_progress_timeout_ > 0.0 &&
      (now - motion_progress_started_).toSec() >= motion_progress_timeout_;
}

void SimplePathTracker::refreshParameters(bool cached)
{
#define LOAD(p) do { \
  if (cached) parameter_nh_->getParamCached(#p, p##_); \
  else parameter_nh_->param(#p, p##_, p##_); \
} while (false)
  LOAD(heading_gain); LOAD(cross_track_gain); LOAD(cross_track_curvature_scale); LOAD(softening_speed);
  LOAD(mowing_speed); LOAD(minimum_tracking_speed); LOAD(max_angular_speed);
  LOAD(max_acceleration); LOAD(max_deceleration); LOAD(cross_track_slowdown_gain);
  LOAD(rotate_threshold); LOAD(rotate_tolerance); LOAD(goal_distance_tolerance);
  LOAD(rotate_progress_timeout); LOAD(rotate_progress_angle);
  LOAD(rotate_escape_distance); LOAD(rotate_escape_speed); LOAD(rotate_escape_timeout); LOAD(rotate_escape_attempts);
  LOAD(tracking_progress_timeout); LOAD(tracking_progress_distance); LOAD(tracking_progress_min_speed);
  LOAD(motion_progress_timeout); LOAD(motion_progress_distance); LOAD(motion_progress_angle);
  LOAD(motion_progress_min_linear); LOAD(motion_progress_min_angular);
  LOAD(curvature_preview_distance); LOAD(curvature_angular_fraction);
  LOAD(minimum_lookahead); LOAD(maximum_lookahead); LOAD(lookahead_time); LOAD(sharp_corner_angle);
  LOAD(turnaround_angle_threshold); LOAD(turnaround_preview_distance);
  LOAD(corner_slowdown_distance); LOAD(corner_position_tolerance);
  LOAD(goal_angle_tolerance); LOAD(goal_slowdown_distance); LOAD(goal_position_timeout);
  LOAD(projection_search_window);
  LOAD(projection_initial_allowance); LOAD(projection_distance_factor);
  LOAD(projection_pose_deadband); LOAD(projection_max_pose_step);
  LOAD(check_collisions); LOAD(unknown_is_obstacle); LOAD(collision_horizon);
  LOAD(collision_time_step); LOAD(braking_deceleration); LOAD(reaction_time); LOAD(collision_margin);
  LOAD(max_mow_motor_current); LOAD(mow_current_gain); LOAD(min_mow_motor_rpm); LOAD(mow_rpm_gain);
#undef LOAD
}
double SimplePathTracker::applyAccelerationLimit(double target,double dt){last_linear_command_=std::max(last_linear_command_-max_deceleration_*dt,std::min(last_linear_command_+max_acceleration_*dt,target));return last_linear_command_;}
bool SimplePathTracker::getProgress(PlannerGetProgressRequest&,PlannerGetProgressResponse& response){response.index=uint32_t(current_index_);return true;}
void SimplePathTracker::statusReceived(const mower_msgs::Status::ConstPtr& status){mower_status_=*status;}
double SimplePathTracker::yawOf(const geometry_msgs::Quaternion& orientation){tf2::Quaternion q;tf2::fromMsg(orientation,q);double r,p,y;tf2::Matrix3x3(q).getRPY(r,p,y);return y;}
double SimplePathTracker::normalizeAngle(double angle){return std::atan2(std::sin(angle),std::cos(angle));}
bool SimplePathTracker::isGoalReached(double,double){return state_==State::FINISHED&&!cancelled_;}
bool SimplePathTracker::cancel(){cancelled_=true;state_=State::FINISHED;last_linear_command_=0;resetMotionProgress();return true;}
}  // namespace ftc_local_planner
