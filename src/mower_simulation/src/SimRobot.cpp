//
// Created by clemens on 29.11.24.
//

#include "SimRobot.h"

#include <nav_msgs/Odometry.h>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <xbot_msgs/AbsolutePose.h>

#include <boost/thread/pthread/thread_data.hpp>
#include <cmath>

constexpr double SimRobot::BATTERY_VOLTS_MIN;
constexpr double SimRobot::BATTERY_VOLTS_MAX;
constexpr double SimRobot::CHARGE_CURRENT;
constexpr double SimRobot::CHARGE_VOLTS;

namespace {
double clampUnit(double value) {
  if (value < -1.0) {
    return -1.0;
  }
  if (value > 1.0) {
    return 1.0;
  }
  return value;
}
}  // namespace

SimRobot::SimRobot(ros::NodeHandle& nh, const ros::Duration& simulation_step_period)
    : nh_{nh}, simulation_step_period_{simulation_step_period} {
  nh_.param("drive_max_wheel_speed_mps", drive_max_wheel_speed_mps_, drive_max_wheel_speed_mps_);
  nh_.param("drive_time_constant_s", drive_time_constant_s_, drive_time_constant_s_);
  nh_.param("drive_deadband", drive_deadband_, drive_deadband_);
  nh_.param("drive_current_per_duty_amp", drive_current_per_duty_amp_, drive_current_per_duty_amp_);
}

void SimRobot::Start() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  if (started_) {
    return;
  }
  started_ = true;
  gps_service_ = nh_.advertiseService("/xbot_positioning/set_gps_state", &SimRobot::OnSetGpsState, this);
  pose_service_ = nh_.advertiseService("/xbot_positioning/set_robot_pose", &SimRobot::OnSetPose, this);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_out", 50);
  xbot_absolute_pose_pub_ = nh_.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
  timer_ = nh_.createTimer(simulation_step_period_, &SimRobot::SimulationStep, this);
  timer_.start();
}

bool SimRobot::OnSetGpsState(xbot_positioning::GPSControlSrvRequest& req,
                             xbot_positioning::GPSControlSrvResponse& res) {
  gps_enabled_ = req.gps_enabled;
  return true;
}

bool SimRobot::OnSetPose(xbot_positioning::SetPoseSrvRequest& req, xbot_positioning::SetPoseSrvResponse& res) {
  // Ignored, because calling this service won't move a real mower either, it just improves the estimation.
  return true;
}

void SimRobot::GetTwist(double& vx, double& vr) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  vx = vx_;
  vr = vr_;
  vx += linear_speed_noise(generator);
  vr += angular_speed_noise(generator);
}

void SimRobot::ConfigureDiffDrive(double wheel_distance_m, double wheel_ticks_per_meter) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  if (wheel_distance_m > 0.0) {
    wheel_distance_m_ = wheel_distance_m;
  }
  wheel_ticks_per_meter_ = wheel_ticks_per_meter;
}

void SimRobot::ResetEmergency() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  emergency_active_ = false;
  emergency_latch_ = false;
  emergency_reason_ = 0;
}

void SimRobot::SetEmergency(bool active, const uint16_t& reason) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  emergency_active_ = active;
  emergency_latch_ |= active;
  emergency_reason_ = reason;
}

void SimRobot::GetEmergencyState(bool& active, bool& latch, uint16_t& reason) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  active = emergency_active_;
  latch = emergency_latch_;
  reason = emergency_reason_;
  if (latch) {
    reason |= EmergencyReason::LATCH;
  }
}

void SimRobot::SetWheelDuty(double left_duty, double right_duty) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  left_wheel_target_duty_ = clampUnit(left_duty);
  right_wheel_target_duty_ = clampUnit(right_duty);
}

SimRobot::DiffDriveState SimRobot::GetDiffDriveState() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  DiffDriveState state;
  state.left_speed_mps = left_wheel_speed_mps_;
  state.right_speed_mps = right_wheel_speed_mps_;
  state.linear_speed_mps = vx_;
  state.angular_speed_radps = vr_;
  state.left_duty = left_wheel_target_duty_;
  state.right_duty = right_wheel_target_duty_;
  state.left_current_a = left_wheel_current_a_;
  state.right_current_a = right_wheel_current_a_;
  state.left_tacho = static_cast<uint32_t>(left_tacho_ticks_);
  state.right_tacho = static_cast<uint32_t>(right_tacho_ticks_);
  return state;
}

void SimRobot::GetPosition(double& x, double& y, double& heading) {
  std::lock_guard<std::mutex> lk{state_mutex_};

  x = pos_x_;
  y = pos_y_;
  heading = pos_heading_;

  x += position_noise(generator);
  y += position_noise(generator);
  heading += heading_noise(generator);
  heading = fmod(heading, M_PI * 2.0);
  while (heading < 0) {
    heading += M_PI * 2.0;
  }
}

void SimRobot::SetPosition(const double x, const double y, const double heading) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  pos_x_ = x;
  pos_y_ = y;
  pos_heading_ = heading;
}

void SimRobot::SetDockingPose(const double x, const double y, const double heading) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  docking_pos_x_ = x;
  docking_pos_y_ = y;
  docking_pos_heading_ = heading;
}

void SimRobot::GetIsCharging(bool& charging, double& seconds_since_start, std::string& charging_status,
                             double& charger_volts, double& battery_volts, double& charging_current) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  charging = is_charging_;
  seconds_since_start = (ros::Time::now() - charging_started_time).toSec();
  charging_status = charger_state_;
  charger_volts = charger_volts_;
  charging_current = charge_current_;
  battery_volts = battery_volts_;
}

void SimRobot::SimulationStep(const ros::TimerEvent& te) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  const auto now = ros::Time::now();
  double time_diff_s = (now - last_update_).toSec();
  const auto duty_to_speed = [this](double duty) {
    const double magnitude = std::abs(duty);
    if (magnitude <= drive_deadband_) {
      return 0.0;
    }
    const double normalized = (magnitude - drive_deadband_) / (1.0 - drive_deadband_);
    return std::copysign(normalized * drive_max_wheel_speed_mps_, duty);
  };

  const double commanded_left_duty = emergency_latch_ ? 0.0 : left_wheel_target_duty_;
  const double commanded_right_duty = emergency_latch_ ? 0.0 : right_wheel_target_duty_;
  const double left_target_speed = duty_to_speed(commanded_left_duty);
  const double right_target_speed = duty_to_speed(commanded_right_duty);
  const double alpha = drive_time_constant_s_ > 1e-6 ? std::min(1.0, time_diff_s / drive_time_constant_s_) : 1.0;
  left_wheel_speed_mps_ += alpha * (left_target_speed - left_wheel_speed_mps_);
  right_wheel_speed_mps_ += alpha * (right_target_speed - right_wheel_speed_mps_);

  left_wheel_current_a_ = drive_current_per_duty_amp_ * std::abs(commanded_left_duty);
  right_wheel_current_a_ = drive_current_per_duty_amp_ * std::abs(commanded_right_duty);

  vx_ = 0.5 * (left_wheel_speed_mps_ + right_wheel_speed_mps_);
  vr_ = wheel_distance_m_ > 1e-6 ? (right_wheel_speed_mps_ - left_wheel_speed_mps_) / wheel_distance_m_ : 0.0;

  if (wheel_ticks_per_meter_ > 0.0) {
    left_tacho_accum_ += left_wheel_speed_mps_ * time_diff_s * wheel_ticks_per_meter_;
    right_tacho_accum_ -= right_wheel_speed_mps_ * time_diff_s * wheel_ticks_per_meter_;
    left_tacho_ticks_ = static_cast<int32_t>(std::llround(left_tacho_accum_));
    right_tacho_ticks_ = static_cast<int32_t>(std::llround(right_tacho_accum_));
  }

  double delta_x = (vx_ * cos(pos_heading_)) * time_diff_s;
  double delta_y = (vx_ * sin(pos_heading_)) * time_diff_s;
  double delta_th = vr_ * time_diff_s;
  pos_x_ += delta_x;
  pos_y_ += delta_y;
  pos_heading_ += delta_th;
  pos_heading_ = fmod(pos_heading_, M_PI * 2.0);
  if (pos_heading_ < 0) {
    pos_heading_ += M_PI * 2.0;
  }

  // Update Charger Status
  if (sqrt((docking_pos_x_ - pos_x_) * (docking_pos_x_ - pos_x_) +
           (docking_pos_y_ - pos_y_) * (docking_pos_y_ - pos_y_)) < 0.5) {
    if (!is_charging_) {
      spdlog::info("Charging");
      is_charging_ = true;
      charging_started_time = ros::Time::now();
    }
  } else {
    if (is_charging_) {
      spdlog::info("Stopped Charging");
      is_charging_ = false;
    }
  }

  if (is_charging_) {
    if (battery_volts_ < BATTERY_VOLTS_MAX) {
      charger_state_ = "CC";
      battery_volts_ += 0.05;
      if (battery_volts_ > BATTERY_VOLTS_MAX) {
        battery_volts_ = BATTERY_VOLTS_MAX;
      }
      charger_volts_ = CHARGE_VOLTS;
      charge_current_ = CHARGE_CURRENT;
    } else if (charge_current_ > 0.2) {
      charger_state_ = "CV";
      battery_volts_ = BATTERY_VOLTS_MAX;
      charger_volts_ = CHARGE_VOLTS;
      charge_current_ = charge_current_ * 0.99;
    } else {
      charger_state_ = "Done";
      battery_volts_ = BATTERY_VOLTS_MAX;
      charger_volts_ = CHARGE_VOLTS;
      charge_current_ = 0;
    }
  } else {
    charger_state_ = "Not Charging";
    battery_volts_ = std::max(BATTERY_VOLTS_MIN, battery_volts_ - 0.001);
    charger_volts_ = 0.0;
    charge_current_ = 0.0;
  }

  PublishPosition();

  last_update_ = now;
}

void SimRobot::PublishPosition() {
  nav_msgs::Odometry odometry;
  geometry_msgs::TransformStamped odom_trans;
  xbot_msgs::AbsolutePose xb_absolute_pose_msg;
  static tf2_ros::TransformBroadcaster transform_broadcaster;

  odometry.header.seq++;
  odometry.header.stamp = ros::Time::now();
  odometry.header.frame_id = "map";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.x = pos_x_;
  odometry.pose.pose.position.y = pos_y_;
  tf2::Quaternion q_mag(0.0, 0.0, pos_heading_);
  odometry.pose.pose.orientation = tf2::toMsg(q_mag);
  odometry.twist.twist.linear.x = vx_;
  odometry.twist.twist.linear.y = 0.0;
  odometry.twist.twist.linear.z = 0.0;
  odometry.twist.twist.angular.x = 0.0;
  odometry.twist.twist.angular.y = 0.0;
  odometry.twist.twist.angular.z = vr_;

  odom_trans.header = odometry.header;
  odom_trans.child_frame_id = odometry.child_frame_id;
  odom_trans.transform.translation.x = odometry.pose.pose.position.x;
  odom_trans.transform.translation.y = odometry.pose.pose.position.y;
  odom_trans.transform.translation.z = odometry.pose.pose.position.z;
  odom_trans.transform.rotation = odometry.pose.pose.orientation;

  xb_absolute_pose_msg.header = odometry.header;
  xb_absolute_pose_msg.sensor_stamp = 0;
  xb_absolute_pose_msg.received_stamp = 0;
  xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
  xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE |
                               xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;
  xb_absolute_pose_msg.orientation_valid = true;
  xb_absolute_pose_msg.motion_vector_valid = false;
  xb_absolute_pose_msg.position_accuracy = gps_enabled_ ? 0.05 : 999;
  xb_absolute_pose_msg.orientation_accuracy = 0.01;
  xb_absolute_pose_msg.pose = odometry.pose;
  xb_absolute_pose_msg.vehicle_heading = pos_heading_;
  xb_absolute_pose_msg.motion_heading = pos_heading_;

  odometry_pub_.publish(odometry);
  transform_broadcaster.sendTransform(odom_trans);
  xbot_absolute_pose_pub_.publish(xb_absolute_pose_msg);

  spdlog::debug("Position: x:{}, y:{}, heading:{}", pos_x_, pos_y_, pos_heading_);
}
