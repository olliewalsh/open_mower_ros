//
// Created by clemens on 26.07.24.
//

#ifndef EMERGENCY_SERVICE_HPP
#define EMERGENCY_SERVICE_HPP

#include <ros/time.h>

#include <EmergencyServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class EmergencyService : public EmergencyServiceBase {
 private:
 public:
  explicit EmergencyService(uint16_t service_id, SimRobot& robot, uint32_t tick_interval_us)
      : EmergencyServiceBase(service_id),
        tick_schedule_{scheduler_, IsRunning(), tick_interval_us,
                       XBOT_FUNCTION_FOR_METHOD(EmergencyService, &EmergencyService::tick, this)},
        robot_(robot) {
  }

 protected:
  bool OnStart() override;
  void OnStop() override;

 private:
  void tick();
  ManagedSchedule tick_schedule_;

  SimRobot& robot_;

  ros::Time last_clear_emergency_message_{0};
  uint16_t emergency_reason = EmergencyReason::LATCH;

 protected:
  void OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) override;
};

#endif  // EMERGENCY_SERVICE_HPP
