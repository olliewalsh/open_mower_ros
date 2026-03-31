//
// Created by clemens on 26.07.24.
//

#ifndef MOWER_SIMULATION_WHEEL_SPEED_LIMIT_ESTIMATOR_HPP
#define MOWER_SIMULATION_WHEEL_SPEED_LIMIT_ESTIMATOR_HPP

#include <cmath>

class WheelSpeedLimitEstimator {
 public:
  void Reset(float nominal_limit) {
    estimated_limit_ = nominal_limit;
  }

  float ScaleForTargets(float desired_speed_l, float desired_speed_r) const {
    const float peak_target = MaxFloat(std::fabs(desired_speed_l), std::fabs(desired_speed_r));
    if (estimated_limit_ <= 0.0f || peak_target <= estimated_limit_) {
      return 1.0f;
    }
    return estimated_limit_ / peak_target;
  }

  void Update(float dt, float nominal_limit, float target_speed_l, float target_speed_r, float measured_speed_l,
              float measured_speed_r, float duty_l, float duty_r) {
    if (dt <= 0.0f) {
      return;
    }

    const float min_limit = nominal_limit * 0.25f;
    const float measured_peak = MaxFloat(std::fabs(measured_speed_l), std::fabs(measured_speed_r));
    const bool saturated_and_limited =
        ((std::fabs(duty_l) > 0.98f) && (std::fabs(target_speed_l - measured_speed_l) > nominal_limit * 0.05f)) ||
        ((std::fabs(duty_r) > 0.98f) && (std::fabs(target_speed_r - measured_speed_r) > nominal_limit * 0.05f));

    if (saturated_and_limited && measured_peak > 0.0f) {
      const float alpha_down = MinFloat(1.0f, dt / 0.5f);
      const float target_limit = MaxFloat(min_limit, measured_peak);
      estimated_limit_ += alpha_down * (target_limit - estimated_limit_);
    } else {
      const float alpha_up = MinFloat(1.0f, dt / 10.0f);
      estimated_limit_ += alpha_up * (nominal_limit - estimated_limit_);
    }

    estimated_limit_ = ClampFloat(estimated_limit_, min_limit, nominal_limit);
  }

 private:
  static float MaxFloat(float a, float b) {
    return a >= b ? a : b;
  }

  static float MinFloat(float a, float b) {
    return a <= b ? a : b;
  }

  static float ClampFloat(float value, float min_value, float max_value) {
    return MaxFloat(min_value, MinFloat(value, max_value));
  }

  float estimated_limit_ = 0.0f;
};

#endif
