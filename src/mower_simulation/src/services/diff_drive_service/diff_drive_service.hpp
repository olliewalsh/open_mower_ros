//
// Created by clemens on 26.07.24.
//

#ifndef DIFF_DRIVE_SERVICE_HPP
#define DIFF_DRIVE_SERVICE_HPP

#include <ros/ros.h>

#include <DiffDriveServiceBase.hpp>
#include <cmath>

#include "../../SimRobot.h"
#include "wheel_speed_controller.hpp"
#include "wheel_speed_limit_estimator.hpp"

using namespace xbot::service;

class DiffDriveService : public DiffDriveServiceBase {
 public:
  explicit DiffDriveService(uint16_t service_id, SimRobot& robot, uint32_t tick_interval_us)
      : DiffDriveServiceBase(service_id),
        robot_(robot),
        tick_schedule_{scheduler_, IsRunning(), tick_interval_us,
                       XBOT_FUNCTION_FOR_METHOD(DiffDriveService, &DiffDriveService::tick, this)} {
  }

  void OnMowerStatusChanged(uint32_t new_status);

 protected:
  bool OnStart() override;
  void OnStop() override;

 private:
  static constexpr double kControlTwistTimeoutSeconds = 1.0;
  static WheelSpeedController::Gains DefaultWheelSpeedControllerGains() {
    return WheelSpeedController::Gains{1.5f, 0.35f, 1.5f};
  }
  SimRobot& robot_;
  ros::NodeHandle ll_diff_drive_param_nh_{"/ll/services/diff_drive"};
  float desired_speed_l_ = 0.0f;
  float desired_speed_r_ = 0.0f;
  float wheel_distance_m_ = 0.0f;
  float wheel_ticks_per_meter_ = 0.0f;
  uint32_t last_ticks_left_ = 0;
  uint32_t last_ticks_right_ = 0;
  bool last_ticks_valid_ = false;
  ros::Time last_status_update_{0};
  ros::Time last_control_twist_received_{0};
  WheelSpeedController left_wheel_controller_{DefaultWheelSpeedControllerGains()};
  WheelSpeedController right_wheel_controller_{DefaultWheelSpeedControllerGains()};
  WheelSpeedLimitEstimator wheel_speed_limit_estimator_{};

  WheelSpeedController::Gains GetConfiguredWheelSpeedControllerGains() const;
  bool LoadDiffDriveGeometryParams();
  float GetNominalWheelSpeedLimit() const;
  void UpdateControllerGains();
  void UpdateDutyFromMeasuredSpeeds(float dt);
  void ResetDriveCommandState();
  void tick();
  ManagedSchedule tick_schedule_;
  void SetDuty();
  void ProcessStatusUpdate();

 protected:
  void OnControlTwistChanged(const double* new_value, uint32_t length) override;
};

#endif  // DIFF_DRIVE_SERVICE_HPP
