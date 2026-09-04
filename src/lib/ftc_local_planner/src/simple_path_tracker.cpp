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
  LOAD(boundary_slowdown_distance); LOAD(boundary_minimum_speed);
  LOAD(max_acceleration); LOAD(max_deceleration); LOAD(deceleration_reaction_time);
  LOAD(cross_track_slowdown_gain);
  LOAD(max_angular_acceleration); LOAD(max_angular_deceleration);
  LOAD(rotate_threshold); LOAD(rotate_tolerance); LOAD(goal_distance_tolerance);
  LOAD(rotate_start_speed_tolerance); LOAD(rotate_stop_settle_time); LOAD(rotate_stop_timeout);
  LOAD(rotate_progress_timeout); LOAD(rotate_progress_angle);
  LOAD(rotate_escape_distance); LOAD(rotate_escape_speed); LOAD(rotate_escape_timeout); LOAD(rotate_escape_attempts);
  LOAD(tracking_progress_timeout); LOAD(tracking_progress_distance); LOAD(tracking_progress_min_speed);
  LOAD(motion_progress_timeout); LOAD(motion_progress_distance); LOAD(motion_progress_angle);
  LOAD(motion_progress_min_linear); LOAD(motion_progress_min_angular);
  LOAD(curvature_preview_distance); LOAD(curvature_angular_fraction);
  LOAD(minimum_lookahead); LOAD(maximum_lookahead); LOAD(lookahead_time); LOAD(sharp_corner_angle);
  LOAD(turnaround_angle_threshold); LOAD(turnaround_preview_distance);
  LOAD(corner_position_tolerance); LOAD(vertex_reapproach_distance); LOAD(vertex_reapproach_attempts);
  LOAD(goal_angle_tolerance);
  LOAD(projection_initial_allowance);
  LOAD(projection_distance_factor); LOAD(projection_pose_deadband); LOAD(projection_max_pose_step);
  LOAD(projection_heading_tolerance);
  LOAD(check_collisions); LOAD(unknown_is_obstacle); LOAD(collision_horizon);
  LOAD(collision_time_step); LOAD(braking_deceleration); LOAD(reaction_time); LOAD(collision_margin);
  LOAD(max_mow_motor_current); LOAD(mow_current_gain); LOAD(min_mow_motor_rpm); LOAD(mow_rpm_gain);
  LOAD(mow_load_filter_attack_time_constant); LOAD(mow_load_filter_release_time_constant);
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
  plan_ = plan; current_index_ = 0; last_linear_command_ = 0.0; last_angular_command_ = 0.0;
  projection_distance_limit_ = projection_initial_allowance_;
  last_projected_path_distance_ = 0.0;
  have_last_projection_pose_ = false;
  resetRotationProgress();
  resetRotateStopWait();
  stop_target_ = StopTarget();
  vertex_reapproach_attempt_count_ = 0;
  resetTrackingProgress();
  resetMotionProgress();
  rotate_escape_attempt_count_ = 0;
  mow_load_filter_initialized_ = false;
  cumulative_distance_.assign(plan_.size(), 0.0);
  for (size_t i = 1; i < plan_.size(); ++i)
    cumulative_distance_[i] = cumulative_distance_[i - 1] +
        std::hypot(plan_[i].pose.position.x - plan_[i - 1].pose.position.x,
                   plan_[i].pose.position.y - plan_[i - 1].pose.position.y);
  cacheStopVertices();
  last_command_time_ = ros::Time::now(); cancelled_ = false; state_ = State::PRE_ROTATE;
  nav_msgs::Path path; path.header = plan.front().header; path.poses = plan; plan_publisher_.publish(path);
  return true;
}

uint32_t SimplePathTracker::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped& command,
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
    const double pose_dx = x - last_projection_x_;
    const double pose_dy = y - last_projection_y_;
    const double pose_step = std::hypot(pose_dx, pose_dy);
    if (pose_step >= std::max(0.0, projection_pose_deadband_)) {
      const PathSample projection_reference = samplePath(last_projected_path_distance_);
      const double along_path_step = std::max(0.0,
          pose_dx * std::cos(projection_reference.heading) +
          pose_dy * std::sin(projection_reference.heading));
      const double accepted_step = std::min(
          along_path_step, std::max(0.0, projection_max_pose_step_));
      projection_distance_limit_ = last_projected_path_distance_ +
          std::max(0.0, projection_distance_factor_) * accepted_step;
      last_projection_x_ = x;
      last_projection_y_ = y;
    }
  } else {
    last_projection_x_ = x;
    last_projection_y_ = y;
    have_last_projection_pose_ = true;
  }
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

  geometry_msgs::PoseStamped point = pose;
  point.pose.position.x = p.x;
  point.pose.position.y = p.y;
  tf2::Quaternion q; q.setRPY(0, 0, p.heading); point.pose.orientation = tf2::toMsg(q);
  tracking_point_publisher_.publish(point);

  const ros::Time control_time = ros::Time::now();
  double command_dt = (control_time - last_command_time_).toSec();
  last_command_time_ = control_time;
  if (!std::isfinite(command_dt) || command_dt <= 0 || command_dt > 1) command_dt = 0.1;
  const bool measured_speed_valid = std::isfinite(velocity.twist.linear.x);
  const double measured_speed = measured_speed_valid ? std::abs(velocity.twist.linear.x) : 0.0;
  const double approach_speed = std::max(measured_speed, std::abs(last_linear_command_));
  const double minimum_tracking_command = std::min(
      std::max(0.0, mowing_speed_), std::max(0.0, minimum_tracking_speed_));

  const auto stoppingSpeed = [&](double distance, double tolerance) {
    const double usable_distance = std::max(0.0,
        distance - std::max(0.0, tolerance) -
        approach_speed * std::max(0.0, deceleration_reaction_time_));
    return max_deceleration_ > 1e-6 ?
        std::sqrt(2.0 * max_deceleration_ * usable_distance) : 0.0;
  };
  const auto linearMotionSettled = [&]() {
    if (std::abs(last_linear_command_) > 1e-6) {
      resetRotateStopWait();
      return false;
    }
    last_linear_command_ = 0.0;
    if (!rotate_stop_wait_active_ || control_time < rotate_stop_wait_started_) {
      rotate_stop_wait_active_ = true;
      rotate_stop_wait_started_ = control_time;
      rotate_stop_settle_active_ = false;
    }
    const bool measured_stopped = measured_speed_valid &&
        measured_speed <= std::max(0.0, rotate_start_speed_tolerance_);
    if (!measured_stopped) {
      rotate_stop_settle_active_ = false;
      return false;
    }
    if (!rotate_stop_settle_active_ || control_time < rotate_stop_settle_started_) {
      rotate_stop_settle_active_ = true;
      rotate_stop_settle_started_ = control_time;
    }
    return (control_time - rotate_stop_settle_started_).toSec() >=
        std::max(0.0, rotate_stop_settle_time_);
  };
  const auto stopWaitTimedOut = [&]() {
    return rotate_stop_wait_active_ && rotate_stop_timeout_ > 0.0 &&
        (control_time - rotate_stop_wait_started_).toSec() >= rotate_stop_timeout_;
  };
  const auto lineAngularCommand = [&](double tangent_x, double tangent_y,
                                      double reference_x, double reference_y,
                                      double linear) {
    const double path_heading = std::atan2(tangent_y, tangent_x);
    const double heading_error = normalizeAngle(path_heading - yaw);
    const double cross_track_error = -tangent_y * (x - reference_x) +
        tangent_x * (y - reference_y);
    const double direction = linear < 0.0 ? -1.0 : 1.0;
    double angular = heading_gain_ * heading_error - direction *
        std::atan2(cross_track_gain_ * cross_track_error,
                   std::abs(linear) + std::max(0.0, softening_speed_));
    if (minimum_tracking_command > 1e-6 && std::abs(linear) < minimum_tracking_command)
      angular *= std::abs(linear) / minimum_tracking_command;
    return std::max(-max_angular_speed_, std::min(max_angular_speed_, angular));
  };

  const auto& segment_start = plan_[p.segment].pose.position;
  const auto& segment_end = plan_[p.segment + 1].pose.position;
  const double segment_dx = segment_end.x - segment_start.x;
  const double segment_dy = segment_end.y - segment_start.y;
  const double segment_heading = std::hypot(segment_dx, segment_dy) > 1e-6 ?
      std::atan2(segment_dy, segment_dx) : p.heading;
  const double segment_heading_error = normalizeAngle(segment_heading - yaw);

  if (state_ == State::APPROACH_STOP) {
    if (!stop_target_.active) {
      message = "Stop approach has no target";
      return INVALID_PATH;
    }
    const double target_dx = stop_target_.x - x;
    const double target_dy = stop_target_.y - y;
    const double longitudinal_error = target_dx * stop_target_.tangent_x +
        target_dy * stop_target_.tangent_y;
    const double lateral_error = -stop_target_.tangent_y * (x - stop_target_.x) +
        stop_target_.tangent_x * (y - stop_target_.y);
    const double position_error = std::hypot(target_dx, target_dy);
    const bool position_reached = !stop_target_.require_position ||
        position_error <= std::max(0.0, stop_target_.position_tolerance);

    double target = 0.0;
    if (!position_reached) {
      const double direction = longitudinal_error >= 0.0 ? 1.0 : -1.0;
      if (stop_correction_active_) {
        const double braking_distance =
            measured_speed * std::max(0.0, deceleration_reaction_time_) +
            (max_deceleration_ > 1e-6 ?
                measured_speed * measured_speed / (2.0 * max_deceleration_) :
                std::numeric_limits<double>::infinity());
        const bool reached_braking_point = stop_correction_direction_ > 0.0 ?
            longitudinal_error <= braking_distance : longitudinal_error >= -braking_distance;
        if (reached_braking_point) {
          stop_correction_active_ = false;
        } else {
          target = stop_correction_direction_ * minimum_tracking_command;
        }
      } else {
        target = direction * std::min(std::max(0.0, mowing_speed_),
            stoppingSpeed(std::abs(longitudinal_error), stop_target_.position_tolerance));
      }
    }
    // A positive braking-profile command below the usable tracking minimum can
    // leave the mower stationary forever without entering the settled branch.
    // Stop commanding first; once settled, the correction mode below latches a
    // minimum-speed command until its measured braking point.
    if (!stop_correction_active_ && measured_speed_valid &&
        measured_speed <= std::max(0.0, rotate_start_speed_tolerance_) &&
        std::abs(target) > 1e-6 && std::abs(target) < minimum_tracking_command) {
      target = 0.0;
    }
    const double linear = applyAccelerationLimit(target, command_dt);
    command.twist.linear.x = linear;
    if (std::abs(linear) > 1e-6) {
      command.twist.angular.z = lineAngularCommand(
          stop_target_.tangent_x, stop_target_.tangent_y,
          stop_target_.x, stop_target_.y, linear);
    }

    if (linearMotionSettled()) {
      if (position_reached) {
        completeStopApproach();
      } else if (std::abs(lateral_error) > stop_target_.position_tolerance) {
        if (vertex_reapproach_attempt_count_ >= std::max(0, vertex_reapproach_attempts_)) {
          message = "Unable to align with stop vertex after " +
              std::to_string(vertex_reapproach_attempt_count_) + " re-approach attempts";
          return MISSED_GOAL;
        }
        const double reverse_distance = std::max(0.0, vertex_reapproach_distance_);
        if (reverse_distance <= 1e-6 || minimum_tracking_command <= 1e-6 ||
            !straightEscapeIsSafe(x, y, yaw, -reverse_distance)) {
          message = "Unable to safely realign with stop vertex";
          return MISSED_GOAL;
        }
        reapproach_start_x_ = x;
        reapproach_start_y_ = y;
        reapproach_heading_ = yaw;
        vertex_reapproach_attempt_count_++;
        stop_target_reapproaching_ = true;
        stop_correction_active_ = false;
        resetRotateStopWait();
        resetRotationProgress();
        resetTrackingProgress();
        state_ = State::REAPPROACH_REVERSE;
        ROS_WARN_STREAM("SimplePathTracker: stop vertex lateral error "
            << std::abs(lateral_error) << " m; starting re-approach "
            << vertex_reapproach_attempt_count_ << " / "
            << std::max(0, vertex_reapproach_attempts_));
      } else {
        if (minimum_tracking_command <= 1e-6) {
          message = "Stop correction requires a positive minimum tracking speed";
          return MISSED_GOAL;
        }
        stop_correction_direction_ = longitudinal_error >= 0.0 ? 1.0 : -1.0;
        stop_correction_active_ = true;
        resetRotateStopWait();
        ROS_INFO_STREAM("SimplePathTracker: stopped "
            << std::abs(longitudinal_error) << " m "
            << (stop_correction_direction_ > 0.0 ? "before" : "past")
            << " stop target; correcting at " << minimum_tracking_command << " m/s");
      }
    } else if (stopWaitTimedOut()) {
      message = "Timed out waiting for linear velocity to settle at stop target";
      return MISSED_GOAL;
    }
  } else if (state_ == State::REAPPROACH_REVERSE) {
    const double travelled = std::hypot(x - reapproach_start_x_, y - reapproach_start_y_);
    const bool reverse_complete = travelled >= std::max(0.0, vertex_reapproach_distance_);
    const double target = reverse_complete ? 0.0 : -minimum_tracking_command;
    command.twist.linear.x = applyAccelerationLimit(target, command_dt);
    if (std::abs(command.twist.linear.x) > 1e-6) {
      const double heading_error = normalizeAngle(reapproach_heading_ - yaw);
      command.twist.angular.z = std::max(-max_angular_speed_,
          std::min(max_angular_speed_, heading_gain_ * heading_error));
    }
    if (reverse_complete && linearMotionSettled()) {
      state_ = State::REAPPROACH_AIM;
      resetRotateStopWait();
      resetRotationProgress();
      ROS_INFO("SimplePathTracker: re-approach reverse complete; aiming at stop vertex");
    } else if (stopWaitTimedOut()) {
      message = "Timed out stopping after vertex re-approach reverse";
      return MISSED_GOAL;
    }
  } else if (state_ == State::REAPPROACH_AIM) {
    const double target_dx = stop_target_.x - x;
    const double target_dy = stop_target_.y - y;
    const double target_distance = std::hypot(target_dx, target_dy);
    if (target_distance <= stop_target_.position_tolerance) {
      completeStopApproach();
    } else {
      const double target_heading = std::atan2(target_dy, target_dx);
      const double heading_error = normalizeAngle(target_heading - yaw);
      if (std::abs(heading_error) <= rotate_tolerance_) {
        if (!straightEscapeIsSafe(x, y, target_heading, target_distance)) {
          message = "Unable to safely return to stop vertex";
          return MISSED_GOAL;
        }
        stop_target_.tangent_x = std::cos(target_heading);
        stop_target_.tangent_y = std::sin(target_heading);
        stop_correction_active_ = false;
        resetRotateStopWait();
        resetRotationProgress();
        state_ = State::APPROACH_STOP;
        ROS_INFO_STREAM("SimplePathTracker: aligned for " << target_distance
            << " m straight re-approach to stop vertex");
      } else if (rotationHasStalled(heading_error, control_time)) {
        message = "Rotation towards stop vertex stalled";
        return MISSED_GOAL;
      } else {
        command.twist.angular.z = std::max(-max_angular_speed_,
            std::min(max_angular_speed_, heading_gain_ * heading_error));
      }
    }
  } else if (state_ == State::PRE_ROTATE) {
    if (std::abs(segment_heading_error) <= rotate_tolerance_) {
      state_ = State::TRACKING;
      rotate_escape_attempt_count_ = 0;
    } else if (rotationHasStalled(segment_heading_error, control_time)) {
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
          std::min(max_angular_speed_, heading_gain_ * segment_heading_error));
    }
  } else if (state_ == State::ROTATE_ESCAPE) {
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
  } else if (state_ == State::TRACKING) {
    if (std::abs(segment_heading_error) > rotate_threshold_) {
      beginStopForRotate(x, y, segment_heading);
      command.twist.linear.x = applyAccelerationLimit(0.0, command_dt);
    } else {
      const double heading_scale = std::pow(std::max(0.0, std::cos(p.heading_error)), 2);
      const double tracking_scale = heading_scale /
          (1.0 + cross_track_slowdown_gain_ * std::abs(p.cross_track_error));
      double target = mowing_speed_ * tracking_scale;
      if (tracking_scale <= 0.05)
        target = 0.0;
      else
        target = std::max(minimum_tracking_command, target);
      if (std::abs(control_curvature) > 1e-6)
        target = std::min(target, curvature_angular_fraction_ * max_angular_speed_ /
            std::abs(control_curvature));

      if (mower_status_.mow_enabled) {
        const double raw_current = double(mower_status_.mower_esc_current);
        const double raw_rpm = std::abs(double(mower_status_.mower_motor_rpm));
        const ros::Time filter_time = ros::Time::now();
        if (!mow_load_filter_initialized_ || filter_time < last_mow_load_filter_time_) {
          filtered_mow_current_ = raw_current;
          filtered_mow_rpm_ = raw_rpm;
          mow_load_filter_initialized_ = true;
        } else {
          const double filter_dt = std::max(0.0, std::min(
              1.0, (filter_time - last_mow_load_filter_time_).toSec()));
          const auto update_filter = [filter_dt](double filtered, double sample, double time_constant) {
            const double tau = std::max(0.0, time_constant);
            const double alpha = tau > 0.0 ? filter_dt / (tau + filter_dt) : 1.0;
            return filtered + alpha * (sample - filtered);
          };
          const double current_tau = raw_current > filtered_mow_current_ ?
              mow_load_filter_attack_time_constant_ : mow_load_filter_release_time_constant_;
          const double rpm_tau = raw_rpm < filtered_mow_rpm_ ?
              mow_load_filter_attack_time_constant_ : mow_load_filter_release_time_constant_;
          filtered_mow_current_ = update_filter(filtered_mow_current_, raw_current, current_tau);
          filtered_mow_rpm_ = update_filter(filtered_mow_rpm_, raw_rpm, rpm_tau);
        }
        last_mow_load_filter_time_ = filter_time;
        const double amps = std::max(0.0, filtered_mow_current_ - max_mow_motor_current_);
        const double rpm = std::max(0.0, min_mow_motor_rpm_ - filtered_mow_rpm_);
        target = std::min(target,
            std::max(minimum_tracking_command, mowing_speed_ - amps * mow_current_gain_));
        target = std::min(target,
            std::max(minimum_tracking_command, mowing_speed_ - rpm * mow_rpm_gain_));
      } else {
        mow_load_filter_initialized_ = false;
      }

      const double boundary_distance = std::max(0.0, boundary_slowdown_distance_);
      if (boundary_distance > 0.0) {
        const double clearance = obstacleClearance(x, y, boundary_distance);
        const double scale = std::max(0.0, std::min(1.0, clearance / boundary_distance));
        const double normal_speed = std::max(0.0, mowing_speed_);
        const double minimum_speed = std::min(
            normal_speed, std::max(0.0, boundary_minimum_speed_));
        target = std::min(target,
            minimum_speed + scale * (normal_speed - minimum_speed));
      }

      const bool stopping_at_corner = p.sharp_corner_ahead;
      const double stop_distance = stopping_at_corner ? p.corner_distance : p.remaining_distance;
      const double stop_tolerance = stopping_at_corner ?
          corner_position_tolerance_ : goal_distance_tolerance_;
      const double stop_speed = stoppingSpeed(stop_distance, stop_tolerance);
      if (stop_speed <= target) {
        if (stopping_at_corner)
          beginStopApproach(p.corner_index, false);
        else
          beginStopApproach(plan_.size() - 1, true);
        target = std::min(target, stop_speed);
      }

      const double linear = applyAccelerationLimit(target, command_dt);
      const double effective_cross_track_gain = cross_track_gain_ /
          (1.0 + std::max(0.0, cross_track_curvature_scale_) * std::abs(p.curvature));
      const double angular = pure_pursuit_ ? linear * control_curvature :
          linear * p.curvature + heading_gain_ * p.heading_error -
          std::atan2(effective_cross_track_gain * p.cross_track_error,
                     std::abs(linear) + softening_speed_);
      command.twist.linear.x = linear;
      command.twist.angular.z = std::max(
          -max_angular_speed_, std::min(max_angular_speed_, angular));
    }
  } else if (state_ == State::FINAL_ROTATE) {
    const double error = normalizeAngle(yawOf(plan_.back().pose.orientation) - yaw);
    if (std::abs(error) <= goal_angle_tolerance_) {
      current_index_ = plan_.size() - 1;
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

  if (state_ == State::TRACKING)
    resetRotationProgress();
  if (state_ == State::FINISHED) {
    command.twist = geometry_msgs::Twist();
    last_angular_command_ = 0.0;
  } else if (state_ == State::ROTATE_ESCAPE) {
    command.twist.angular.z = 0.0;
    last_angular_command_ = 0.0;
  } else {
    const double requested_angular = command.twist.angular.z;
    const double limited_angular = applyAngularAccelerationLimit(requested_angular, command_dt);
    if ((state_ == State::TRACKING || state_ == State::APPROACH_STOP) &&
        std::abs(command.twist.linear.x) > 1e-6) {
      double linear_scale = 1.0;
      if (requested_angular * limited_angular < 0.0 ||
          (std::abs(requested_angular) <= 1e-6 && std::abs(limited_angular) > 1e-6)) {
        linear_scale = 0.0;
      } else if (std::abs(requested_angular) > 1e-6 &&
                 std::abs(limited_angular) < std::abs(requested_angular)) {
        linear_scale = std::abs(limited_angular / requested_angular);
      }
      command.twist.linear.x *= linear_scale;
      if (state_ == State::TRACKING && std::abs(command.twist.linear.x) > 1e-6 &&
          std::abs(command.twist.linear.x) < minimum_tracking_command) {
        command.twist.linear.x = std::copysign(
            minimum_tracking_command, command.twist.linear.x);
      }
      last_linear_command_ = command.twist.linear.x;
    }
    command.twist.angular.z = limited_angular;
  }

  const bool vertex_recovery = state_ == State::REAPPROACH_REVERSE ||
      state_ == State::REAPPROACH_AIM || stop_target_reapproaching_;
  if (state_ != State::ROTATE_ESCAPE &&
      !trajectoryIsSafe(x, y, yaw, command.twist.linear.x, command.twist.angular.z, message)) {
    command.twist = geometry_msgs::Twist();
    last_linear_command_ = 0.0;
    last_angular_command_ = 0.0;
    if (vertex_recovery) {
      message = "Unable to safely realign with stop vertex: " + message;
      return MISSED_GOAL;
    }
    return COLLISION;
  }

  const bool normal_tracking = state_ == State::TRACKING &&
      std::abs(command.twist.linear.x) >= std::max(0.0, tracking_progress_min_speed_);
  if (!normal_tracking) {
    resetTrackingProgress();
  } else if (trackingHasStalled(p.path_distance, control_time)) {
    command.twist = geometry_msgs::Twist();
    last_linear_command_ = 0.0;
    last_angular_command_ = 0.0;
    message = "Tracking stalled with no path progress for " +
        std::to_string(std::max(0.0, tracking_progress_timeout_)) + " s";
    return MISSED_GOAL;
  }

  if (state_ == State::FINISHED) {
    resetMotionProgress();
  } else {
    const bool recovery_motion_expected = state_ == State::REAPPROACH_REVERSE ||
        state_ == State::REAPPROACH_AIM || stop_target_reapproaching_;
    const bool motion_commanded = recovery_motion_expected ||
        std::abs(command.twist.linear.x) >= std::max(0.0, motion_progress_min_linear_) ||
        std::abs(command.twist.angular.z) >= std::max(0.0, motion_progress_min_angular_);
    if (motionHasStalled(x, y, yaw, motion_commanded, control_time)) {
      command.twist = geometry_msgs::Twist();
      last_linear_command_ = 0.0;
      last_angular_command_ = 0.0;
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
  double reference_heading = 0.0;
  bool have_reference_heading = false;
  size_t reference_segment = first;
  double search_distance_limit = projection_distance_limit_;
  for (size_t i = first; i + 1 < plan_.size(); ++i) {
    if (cumulative_distance_[i] > projection_distance_limit_) break;
    const auto& a = plan_[i].pose.position;
    const auto& b = plan_[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 > 1e-10) {
      reference_heading = std::atan2(dy, dx);
      reference_segment = i;
      have_reference_heading = true;
      const double segment_length = std::sqrt(length2);
      const double minimum_fraction = std::max(0.0, std::min(1.0,
          (last_projected_path_distance_ - cumulative_distance_[i]) / segment_length));
      const double pose_fraction = std::max(minimum_fraction, std::min(1.0,
          ((x - a.x) * dx + (y - a.y) * dy) / length2));
      // The active segment is topologically unambiguous, so let its projection
      // catch up to the robot. The distance gate remains in force before the
      // projection can advance onto any later segment.
      search_distance_limit = std::max(search_distance_limit,
          cumulative_distance_[i] + pose_fraction * segment_length);
      break;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = first; i + 1 < plan_.size(); ++i) {
    if (cumulative_distance_[i] > search_distance_limit) break;
    const auto& a = plan_[i].pose.position; const auto& b = plan_[i + 1].pose.position;
    const double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx + dy*dy;
    if (length2 < 1e-10) continue;
    // Keep the immediate successor eligible so projection can advance normally
    // through an ordinary corner. Do not skip across it onto a nearby folded
    // segment whose direction is discontinuous with the active segment.
    if (i > reference_segment + 1 && have_reference_heading && projection_heading_tolerance_ > 0.0 &&
        std::abs(normalizeAngle(std::atan2(dy, dx) - reference_heading)) > projection_heading_tolerance_) {
      continue;
    }
    // Allow round-off at a segment endpoint. Computing start + segment
    // length can otherwise land a few ulps beyond the identical cumulative
    // endpoint and reject the final segment on the next controller update.
    if (cumulative_distance_[i + 1] + 1e-9 < last_projected_path_distance_)
      continue;
    const double segment_length = std::sqrt(length2);
    const double minimum_fraction = std::max(0.0, std::min(1.0,
        (last_projected_path_distance_ - cumulative_distance_[i]) / segment_length));
    const double reachable_fraction = std::min(1.0,
        (search_distance_limit - cumulative_distance_[i]) / segment_length);
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
  result.path_distance = std::max(cumulative_distance_[result.segment],
      std::min(cumulative_distance_[result.segment + 1], path_distance));
  // Do not let a symmetric preview reach backward across a sharp vertex that
  // has already been processed. Otherwise the outgoing segment starts with a
  // bisector heading and curvature from the completed corner, which can send
  // the controller straight back into PRE_ROTATE.
  bool sharp_corner_behind = false;
  for (size_t segment = result.segment; segment > 0; --segment) {
    const auto& previous_start = plan_[segment - 1].pose.position;
    const auto& previous_end = plan_[segment].pose.position;
    const double previous_dx = previous_end.x - previous_start.x;
    const double previous_dy = previous_end.y - previous_start.y;
    if (std::hypot(previous_dx, previous_dy) <= 1e-6)
      continue;
    const double previous_heading = std::atan2(previous_dy, previous_dx);
    sharp_corner_behind =
        std::abs(normalizeAngle(result.heading - previous_heading)) >= sharp_corner_angle_;
    break;
  }
  const double preview_floor = sharp_corner_behind ?
      cumulative_distance_[result.segment] : 0.0;
  const double half_preview = std::max(0.02, curvature_preview_distance_ * 0.5);
  const PathSample behind = samplePath(std::max(preview_floor, path_distance - half_preview));
  const PathSample ahead = samplePath(path_distance + half_preview);
  if (std::hypot(ahead.x - behind.x, ahead.y - behind.y) > 1e-6)
    result.heading = std::atan2(ahead.y - behind.y, ahead.x - behind.x);
  const double curvature_behind_distance =
      std::max(preview_floor, path_distance - curvature_preview_distance_);
  const double curvature_ahead_distance = std::min(
      cumulative_distance_.back(), path_distance + curvature_preview_distance_);
  const PathSample curvature_behind = samplePath(curvature_behind_distance);
  const PathSample curvature_ahead = samplePath(curvature_ahead_distance);
  const double curvature_span =
      std::max(0.04, curvature_ahead_distance - curvature_behind_distance);
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
  result.corner_index = result.segment + 1;
  const double current_heading = std::atan2(
      end.y - plan_[result.segment].pose.position.y,
      end.x - plan_[result.segment].pose.position.x);
  for (size_t vertex = result.segment + 1; vertex + 1 < plan_.size(); ++vertex) {
    if (vertex >= stop_vertices_.size() || !stop_vertices_[vertex]) continue;
    const double distance = std::max(0.0, cumulative_distance_[vertex] - path_distance);
    if (vertex == result.segment + 1 ||
        distance <= std::max(0.0, turnaround_preview_distance_)) {
      result.sharp_corner_ahead = true;
      result.corner_distance = distance;
      result.corner_index = vertex;
    }
    break;
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

double SimplePathTracker::obstacleClearance(double x, double y, double maximum_distance) const
{
  auto* costmap = costmap_ros_->getCostmap();
  unsigned int center_x, center_y;
  if (!costmap->worldToMap(x, y, center_x, center_y)) return 0.0;
  const double resolution = costmap->getResolution();
  const int radius = static_cast<int>(std::ceil(std::max(0.0, maximum_distance) / resolution));
  double clearance = std::max(0.0, maximum_distance);
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int cell_x = static_cast<int>(center_x) + dx;
      const int cell_y = static_cast<int>(center_y) + dy;
      if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(costmap->getSizeInCellsX()) ||
          cell_y >= static_cast<int>(costmap->getSizeInCellsY()))
        continue;
      const unsigned char cost = costmap->getCost(cell_x, cell_y);
      if (cost != costmap_2d::LETHAL_OBSTACLE &&
          !(unknown_is_obstacle_ && cost == costmap_2d::NO_INFORMATION))
        continue;
      double cell_world_x, cell_world_y;
      costmap->mapToWorld(cell_x, cell_y, cell_world_x, cell_world_y);
      clearance = std::min(clearance, std::hypot(x - cell_world_x, y - cell_world_y));
    }
  }
  return clearance;
}

void SimplePathTracker::resetRotationProgress()
{
  rotate_progress_active_ = false;
  best_rotate_error_ = std::numeric_limits<double>::infinity();
  rotate_progress_started_ = ros::Time();
}

void SimplePathTracker::cacheStopVertices()
{
  stop_vertices_.assign(plan_.size(), false);
  if (plan_.size() < 3) return;

  const auto segmentHeading = [&](size_t segment, double& heading) {
    if (segment + 1 >= plan_.size()) return false;
    const auto& a = plan_[segment].pose.position;
    const auto& b = plan_[segment + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    if (std::hypot(dx, dy) <= 1e-6) return false;
    heading = std::atan2(dy, dx);
    return true;
  };

  for (size_t vertex = 1; vertex + 1 < plan_.size(); ++vertex) {
    size_t incoming = vertex;
    double incoming_heading = 0.0;
    while (incoming > 0 && !segmentHeading(incoming - 1, incoming_heading)) --incoming;
    size_t outgoing = vertex;
    double outgoing_heading = 0.0;
    while (outgoing + 1 < plan_.size() && !segmentHeading(outgoing, outgoing_heading)) ++outgoing;
    if (incoming > 0 && outgoing + 1 < plan_.size() &&
        std::abs(normalizeAngle(outgoing_heading - incoming_heading)) >= sharp_corner_angle_) {
      stop_vertices_[vertex] = true;
    }
  }

  const double preview = std::max(0.0, turnaround_preview_distance_);
  for (size_t start = 0; start + 2 < plan_.size(); ++start) {
    double previous_heading = 0.0;
    if (!segmentHeading(start, previous_heading)) continue;
    double accumulated_turn = 0.0;
    size_t first_turn = plan_.size();
    for (size_t segment = start + 1; segment + 1 < plan_.size(); ++segment) {
      if (cumulative_distance_[segment] - cumulative_distance_[start + 1] > preview) break;
      double heading = 0.0;
      if (!segmentHeading(segment, heading)) continue;
      const double turn = normalizeAngle(heading - previous_heading);
      if (first_turn == plan_.size() && std::abs(turn) > 0.05) first_turn = segment;
      accumulated_turn += turn;
      previous_heading = heading;
      if (first_turn < plan_.size() &&
          std::abs(accumulated_turn) >= std::max(0.0, turnaround_angle_threshold_)) {
        stop_vertices_[first_turn] = true;
        break;
      }
    }
  }
}

void SimplePathTracker::beginStopApproach(size_t target_index, bool final)
{
  if (target_index == 0 || target_index >= plan_.size()) return;
  size_t incoming = target_index;
  double dx = 0.0, dy = 0.0, length = 0.0;
  while (incoming > 0 && length <= 1e-6) {
    const auto& a = plan_[incoming - 1].pose.position;
    const auto& b = plan_[incoming].pose.position;
    dx = b.x - a.x;
    dy = b.y - a.y;
    length = std::hypot(dx, dy);
    --incoming;
  }
  if (length <= 1e-6) return;

  const auto& target = plan_[target_index].pose.position;
  stop_target_.active = true;
  stop_target_.require_position = true;
  stop_target_.final = final;
  stop_target_.index = target_index;
  stop_target_.x = target.x;
  stop_target_.y = target.y;
  stop_target_.tangent_x = dx / length;
  stop_target_.tangent_y = dy / length;
  stop_target_.position_tolerance = std::max(
      0.0, final ? goal_distance_tolerance_ : corner_position_tolerance_);
  stop_correction_active_ = false;
  stop_target_reapproaching_ = false;
  vertex_reapproach_attempt_count_ = 0;
  projection_distance_limit_ = cumulative_distance_[target_index];
  resetRotateStopWait();
  resetRotationProgress();
  resetTrackingProgress();
  state_ = State::APPROACH_STOP;
}

void SimplePathTracker::beginStopForRotate(double x, double y, double heading)
{
  stop_target_.active = true;
  stop_target_.require_position = false;
  stop_target_.final = false;
  stop_target_.index = current_index_;
  stop_target_.x = x;
  stop_target_.y = y;
  stop_target_.tangent_x = std::cos(heading);
  stop_target_.tangent_y = std::sin(heading);
  stop_target_.position_tolerance = 0.0;
  stop_correction_active_ = false;
  stop_target_reapproaching_ = false;
  vertex_reapproach_attempt_count_ = 0;
  resetRotateStopWait();
  resetRotationProgress();
  state_ = State::APPROACH_STOP;
}

void SimplePathTracker::completeStopApproach()
{
  if (stop_target_.require_position) {
    if (stop_target_.final) {
      state_ = State::FINAL_ROTATE;
    } else {
      current_index_ = std::min(stop_target_.index, plan_.size() - 2);
      last_projected_path_distance_ = cumulative_distance_[stop_target_.index];
      projection_distance_limit_ = last_projected_path_distance_;
      state_ = State::PRE_ROTATE;
    }
  } else {
    state_ = State::PRE_ROTATE;
  }
  stop_target_ = StopTarget();
  stop_correction_active_ = false;
  stop_target_reapproaching_ = false;
  resetRotateStopWait();
  resetRotationProgress();
}

void SimplePathTracker::resetRotateStopWait()
{
  rotate_stop_wait_active_ = false;
  rotate_stop_settle_active_ = false;
  rotate_stop_wait_started_ = ros::Time();
  rotate_stop_settle_started_ = ros::Time();
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
  if (!motion_commanded) {
    resetMotionProgress();
    return false;
  }
  if (!motion_progress_active_) {
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
  LOAD(boundary_slowdown_distance); LOAD(boundary_minimum_speed);
  LOAD(max_acceleration); LOAD(max_deceleration); LOAD(deceleration_reaction_time);
  LOAD(cross_track_slowdown_gain);
  LOAD(max_angular_acceleration); LOAD(max_angular_deceleration);
  LOAD(rotate_threshold); LOAD(rotate_tolerance); LOAD(goal_distance_tolerance);
  LOAD(rotate_start_speed_tolerance); LOAD(rotate_stop_settle_time); LOAD(rotate_stop_timeout);
  LOAD(rotate_progress_timeout); LOAD(rotate_progress_angle);
  LOAD(rotate_escape_distance); LOAD(rotate_escape_speed); LOAD(rotate_escape_timeout); LOAD(rotate_escape_attempts);
  LOAD(tracking_progress_timeout); LOAD(tracking_progress_distance); LOAD(tracking_progress_min_speed);
  LOAD(motion_progress_timeout); LOAD(motion_progress_distance); LOAD(motion_progress_angle);
  LOAD(motion_progress_min_linear); LOAD(motion_progress_min_angular);
  LOAD(curvature_preview_distance); LOAD(curvature_angular_fraction);
  LOAD(minimum_lookahead); LOAD(maximum_lookahead); LOAD(lookahead_time); LOAD(sharp_corner_angle);
  LOAD(turnaround_angle_threshold); LOAD(turnaround_preview_distance);
  LOAD(corner_position_tolerance); LOAD(vertex_reapproach_distance); LOAD(vertex_reapproach_attempts);
  LOAD(goal_angle_tolerance);
  LOAD(projection_initial_allowance); LOAD(projection_distance_factor);
  LOAD(projection_pose_deadband); LOAD(projection_max_pose_step);
  LOAD(projection_heading_tolerance);
  LOAD(check_collisions); LOAD(unknown_is_obstacle); LOAD(collision_horizon);
  LOAD(collision_time_step); LOAD(braking_deceleration); LOAD(reaction_time); LOAD(collision_margin);
  LOAD(max_mow_motor_current); LOAD(mow_current_gain); LOAD(min_mow_motor_rpm); LOAD(mow_rpm_gain);
  LOAD(mow_load_filter_attack_time_constant); LOAD(mow_load_filter_release_time_constant);
#undef LOAD
}
double SimplePathTracker::applyAccelerationLimit(double target, double dt)
{
  // Decelerate to zero before changing direction, then use the acceleration
  // limit while building speed in either direction.
  const bool reversing = last_linear_command_ * target < 0.0;
  const double limited_target = reversing ? 0.0 : target;
  const bool accelerating = std::abs(limited_target) > std::abs(last_linear_command_);
  const double rate = std::max(0.0, accelerating ? max_acceleration_ : max_deceleration_);
  const double maximum_step = rate * std::max(0.0, dt);
  const double delta = std::max(-maximum_step,
      std::min(maximum_step, limited_target - last_linear_command_));
  last_linear_command_ += delta;
  return last_linear_command_;
}
double SimplePathTracker::applyAngularAccelerationLimit(double target,double dt)
{
  const double acceleration = std::max(0.0, max_angular_acceleration_);
  const double deceleration = std::max(0.0, max_angular_deceleration_);
  if (!std::isfinite(target) || !std::isfinite(dt) || dt <= 0.0) return last_angular_command_;

  if (last_angular_command_ * target < 0.0) {
    if (deceleration <= 0.0) {
      last_angular_command_ = target;
      return last_angular_command_;
    }
    const double time_to_zero = std::abs(last_angular_command_) / deceleration;
    if (dt <= time_to_zero) {
      last_angular_command_ -= std::copysign(deceleration * dt, last_angular_command_);
      return last_angular_command_;
    }
    last_angular_command_ = 0.0;
    dt -= time_to_zero;
  }

  const bool accelerating = std::abs(target) > std::abs(last_angular_command_);
  const double rate = accelerating ? acceleration : deceleration;
  if (rate <= 0.0) {
    last_angular_command_ = target;
    return last_angular_command_;
  }
  const double maximum_change = rate * dt;
  last_angular_command_ = std::max(last_angular_command_ - maximum_change,
      std::min(last_angular_command_ + maximum_change, target));
  return last_angular_command_;
}
bool SimplePathTracker::getProgress(PlannerGetProgressRequest&,PlannerGetProgressResponse& response){response.index=uint32_t(current_index_);return true;}
void SimplePathTracker::statusReceived(const mower_msgs::Status::ConstPtr& status){mower_status_=*status;}
double SimplePathTracker::yawOf(const geometry_msgs::Quaternion& orientation){tf2::Quaternion q;tf2::fromMsg(orientation,q);double r,p,y;tf2::Matrix3x3(q).getRPY(r,p,y);return y;}
double SimplePathTracker::normalizeAngle(double angle){return std::atan2(std::sin(angle),std::cos(angle));}
bool SimplePathTracker::isGoalReached(double,double){return state_==State::FINISHED&&!cancelled_;}
bool SimplePathTracker::cancel(){cancelled_=true;state_=State::FINISHED;last_linear_command_=0;last_angular_command_=0;resetMotionProgress();return true;}
}  // namespace ftc_local_planner
