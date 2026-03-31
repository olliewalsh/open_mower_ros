//
// Created by clemens on 26.07.24.
//

#include "diff_drive_service.hpp"

#include <algorithm>

WheelSpeedController::Gains DiffDriveService::GetConfiguredWheelSpeedControllerGains() const {
  WheelSpeedController::Gains gains{
      static_cast<float>(WheelSpeedFeedforward.value),
      static_cast<float>(WheelSpeedKp.value),
      static_cast<float>(WheelSpeedKi.value),
  };
  if (gains.feedforward <= 0.0f) {
    gains.feedforward = DefaultWheelSpeedControllerGains().feedforward;
  }
  if (gains.kp < 0.0f) {
    gains.kp = DefaultWheelSpeedControllerGains().kp;
  }
  if (gains.ki < 0.0f) {
    gains.ki = DefaultWheelSpeedControllerGains().ki;
  }
  return gains;
}

float DiffDriveService::GetNominalWheelSpeedLimit() const {
  const auto gains = GetConfiguredWheelSpeedControllerGains();
  return 1.0f / gains.feedforward;
}

void DiffDriveService::UpdateControllerGains() {
  const auto gains = GetConfiguredWheelSpeedControllerGains();
  left_wheel_controller_.SetGains(gains);
  right_wheel_controller_.SetGains(gains);
}

void DiffDriveService::UpdateDutyFromMeasuredSpeeds(float dt) {
  UpdateControllerGains();
  const float scale = wheel_speed_limit_estimator_.ScaleForTargets(desired_speed_l_, desired_speed_r_);
  left_wheel_controller_.SetTargetSpeed(desired_speed_l_ * scale);
  right_wheel_controller_.SetTargetSpeed(desired_speed_r_ * scale);
  left_wheel_controller_.Update(dt);
  right_wheel_controller_.Update(dt);
  wheel_speed_limit_estimator_.Update(dt, GetNominalWheelSpeedLimit(), left_wheel_controller_.target_speed(),
                                      right_wheel_controller_.target_speed(), left_wheel_controller_.measured_speed(),
                                      right_wheel_controller_.measured_speed(), left_wheel_controller_.duty(),
                                      right_wheel_controller_.duty());
}

void DiffDriveService::ResetDriveCommandState() {
  desired_speed_l_ = 0.0f;
  desired_speed_r_ = 0.0f;
  wheel_speed_limit_estimator_.Reset(GetNominalWheelSpeedLimit());
  left_wheel_controller_.Reset();
  right_wheel_controller_.Reset();
}

void DiffDriveService::OnStop() {
  ResetDriveCommandState();
  last_ticks_valid_ = false;
  last_control_twist_received_ = ros::Time(0);
  robot_.SetWheelDuty(0.0, 0.0);
}

void DiffDriveService::tick() {
  if (!last_control_twist_received_.isZero() &&
      (ros::Time::now() - last_control_twist_received_).toSec() > kControlTwistTimeoutSeconds) {
    ResetDriveCommandState();
    SetDuty();
  }
  ProcessStatusUpdate();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  last_control_twist_received_ = ros::Time::now();
  const auto linear = static_cast<float>(new_value[0]);
  const auto angular = static_cast<float>(new_value[5]);
  desired_speed_r_ = linear + 0.5f * static_cast<float>(WheelDistance.value) * angular;
  desired_speed_l_ = linear - 0.5f * static_cast<float>(WheelDistance.value) * angular;
  UpdateDutyFromMeasuredSpeeds(0.0f);
  SetDuty();
}

bool DiffDriveService::OnStart() {
  if (WheelDistance.value == 0 || WheelTicksPerMeter.value == 0.0) {
    return false;
  }
  robot_.ConfigureDiffDrive(static_cast<double>(WheelDistance.value), static_cast<double>(WheelTicksPerMeter.value));
  UpdateControllerGains();
  ResetDriveCommandState();
  last_ticks_valid_ = false;
  last_control_twist_received_ = ros::Time::now();
  return true;
}

void DiffDriveService::SetDuty() {
  robot_.SetWheelDuty(left_wheel_controller_.duty(), right_wheel_controller_.duty());
}

void DiffDriveService::ProcessStatusUpdate() {
  const auto state = robot_.GetDiffDriveState();
  const ros::Time now = ros::Time::now();

  if (last_ticks_valid_) {
    const double dt = std::max(1e-6, (now - last_status_update_).toSec());
    const int32_t d_left = static_cast<int32_t>(state.left_tacho - last_ticks_left_);
    const int32_t d_right = static_cast<int32_t>(state.right_tacho - last_ticks_right_);
    const float wheel_ticks_per_meter = static_cast<float>(WheelTicksPerMeter.value);
    left_wheel_controller_.SetMeasuredSpeed(static_cast<float>(d_left) /
                                            static_cast<float>(dt * wheel_ticks_per_meter));
    right_wheel_controller_.SetMeasuredSpeed(-static_cast<float>(d_right) /
                                             static_cast<float>(dt * wheel_ticks_per_meter));
    UpdateDutyFromMeasuredSpeeds(static_cast<float>(dt));
    SetDuty();
  } else {
    left_wheel_controller_.SetMeasuredSpeed(static_cast<float>(state.left_speed_mps));
    right_wheel_controller_.SetMeasuredSpeed(static_cast<float>(state.right_speed_mps));
    wheel_speed_limit_estimator_.Reset(GetNominalWheelSpeedLimit());
  }

  last_ticks_left_ = state.left_tacho;
  last_ticks_right_ = state.right_tacho;
  last_ticks_valid_ = true;
  last_status_update_ = now;

  StartTransaction();
  SendLeftESCStatus(200u);
  SendRightESCStatus(200u);
  double twist[6]{};
  twist[0] = state.linear_speed_mps;
  twist[5] = state.angular_speed_radps;
  SendActualTwist(twist, sizeof(twist) / sizeof(double));
  CommitTransaction();
}
