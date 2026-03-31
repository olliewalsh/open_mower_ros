//
// Created by clemens on 31.07.24.
//

#ifndef MOWER_SERVICE_HPP
#define MOWER_SERVICE_HPP

#include <MowerServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class MowerService : public MowerServiceBase {
 public:
  explicit MowerService(const uint16_t service_id, SimRobot& robot, uint32_t tick_interval_us)
      : MowerServiceBase(service_id),
        robot_(robot),
        tick_schedule_{scheduler_, IsRunning(), tick_interval_us,
                       XBOT_FUNCTION_FOR_METHOD(MowerService, &MowerService::tick, this)} {
  }

 private:
  SimRobot& robot_;
  bool mower_running_ = false;
  void tick();
  ManagedSchedule tick_schedule_;

 protected:
  bool OnStart() override;
  void OnMowerEnabledChanged(const uint8_t& new_value) override;
};

#endif  // MOWER_SERVICE_HPP
