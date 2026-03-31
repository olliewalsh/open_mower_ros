//
// Created by clemens on 02.08.24.
//

#ifndef GPS_SERVICE_HPP
#define GPS_SERVICE_HPP
#include <GpsServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class GpsService : public GpsServiceBase {
 public:
  explicit GpsService(uint16_t service_id, SimRobot& robot, uint32_t tick_interval_us)
      : GpsServiceBase(service_id),
        tick_schedule_{scheduler_, IsRunning(), tick_interval_us,
                       XBOT_FUNCTION_FOR_METHOD(GpsService, &GpsService::tick, this)},
        robot_(robot) {
  }

 private:
  void tick();
  ManagedSchedule tick_schedule_;

 private:
  SimRobot& robot_;
};

#endif  // GPS_SERVICE_HPP
