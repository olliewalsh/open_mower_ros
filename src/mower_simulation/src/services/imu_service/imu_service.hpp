//
// Created by clemens on 31.07.24.
//

#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

#include <ImuServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class ImuService : public ImuServiceBase {
 public:
  explicit ImuService(const uint16_t service_id, SimRobot& robot, uint32_t tick_interval_us)
      : ImuServiceBase(service_id),
        robot_(robot),
        tick_schedule_{scheduler_, IsRunning(), tick_interval_us,
                       XBOT_FUNCTION_FOR_METHOD(ImuService, &ImuService::tick, this)} {
  }

 private:
  SimRobot& robot_;
  void tick();
  ManagedSchedule tick_schedule_;
};

#endif  // IMU_SERVICE_HPP
