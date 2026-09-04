#include "wheel_speed_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void CheckNear(float actual, float expected, const char* message) {
  if (std::fabs(actual - expected) > 1e-6f) {
    std::cerr << message << ": expected " << expected << ", got " << actual << std::endl;
    std::exit(1);
  }
}

void TestIntegralIsDisabledAtZeroTarget() {
  const WheelSpeedController::Gains gains{1.0f, 0.5f, 2.0f, 0.0f};
  WheelSpeedController controller{gains};

  controller.SetTargetSpeed(0.1f);
  controller.SetMeasuredSpeed(0.0f);
  CheckNear(controller.Update(1.0f), 0.35f, "controller did not accumulate integral");

  controller.SetTargetSpeed(0.0f);
  controller.SetMeasuredSpeed(0.2f);
  CheckNear(controller.Update(0.1f), -0.1f, "zero target accumulated braking integral");

  controller.SetMeasuredSpeed(0.0f);
  CheckNear(controller.Update(0.1f), 0.0f, "zero target retained drive duty at rest");
}

void TestIntegralIsResetOnReversal() {
  const WheelSpeedController::Gains gains{1.0f, 0.5f, 2.0f, 0.0f};
  WheelSpeedController controller{gains};

  controller.SetTargetSpeed(0.1f);
  controller.SetMeasuredSpeed(0.0f);
  CheckNear(controller.Update(1.0f), 0.35f, "controller did not accumulate integral");

  controller.SetTargetSpeed(-0.1f);
  CheckNear(controller.Update(0.0f), -0.15f, "reversal retained the previous integral");
}

}  // namespace

int main() {
  TestIntegralIsDisabledAtZeroTarget();
  TestIntegralIsResetOnReversal();
  return 0;
}
