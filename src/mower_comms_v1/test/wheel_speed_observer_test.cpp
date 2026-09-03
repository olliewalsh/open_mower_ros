#include "wheel_speed_observer.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << std::endl;
    std::exit(1);
  }
}

int32_t QuantizedTicks(double position, double ticks_per_meter) {
  return static_cast<int32_t>(std::lround(position * ticks_per_meter));
}

void TestConstantSpeedWithQuantizedTicks(float ticks_per_meter, float speed) {
  const float resolution = 1.0f / ticks_per_meter;
  const float sample_intervals[] = {0.007f, 0.011f, 0.016f, 0.009f, 0.014f};

  WheelSpeedObserver observer;
  double position = 0.0;
  int32_t previous_ticks = 0;
  double observer_squared_error = 0.0;
  double window_squared_error = 0.0;
  int32_t window_ticks[3]{};
  float window_dt[3]{};
  int32_t window_tick_sum = 0;
  float window_dt_sum = 0.0f;
  int window_index = 0;
  int window_count = 0;
  int samples = 0;

  // Run far enough to exercise internal position recentering as well as steady-state filtering.
  for (int index = 0; index < 4000; ++index) {
    const float dt = sample_intervals[index % 5];
    position += speed * dt;
    const int32_t ticks = QuantizedTicks(position, ticks_per_meter);
    const int32_t delta_ticks = ticks - previous_ticks;
    previous_ticks = ticks;
    const float distance_delta = delta_ticks * resolution;
    Check(observer.Update(distance_delta, dt, resolution), "constant-speed update was rejected");

    if (window_count == 3) {
      window_tick_sum -= window_ticks[window_index];
      window_dt_sum -= window_dt[window_index];
    } else {
      ++window_count;
    }
    window_ticks[window_index] = delta_ticks;
    window_dt[window_index] = dt;
    window_tick_sum += delta_ticks;
    window_dt_sum += dt;
    window_index = (window_index + 1) % 3;

    if (index > 100) {
      const double observer_error = observer.velocity() - speed;
      const double window_speed = window_tick_sum / (window_dt_sum * ticks_per_meter);
      const double window_error = window_speed - speed;
      observer_squared_error += observer_error * observer_error;
      window_squared_error += window_error * window_error;
      ++samples;
    }
  }

  const double observer_rms = std::sqrt(observer_squared_error / samples);
  const double window_rms = std::sqrt(window_squared_error / samples);
  Check(observer_rms < 0.01, "constant-speed observer noise is too high");
  Check(observer_rms < window_rms * 0.6, "observer did not improve on the three-sample estimator");
}

void TestAccelerationAndStopResponse(float ticks_per_meter) {
  constexpr float kDt = 0.01f;
  constexpr float kAcceleration = 0.35f;
  const float resolution = 1.0f / ticks_per_meter;

  WheelSpeedObserver observer;
  double position = 0.0;
  float speed = 0.0f;
  int32_t previous_ticks = 0;

  for (int index = 0; index < 100; ++index) {
    speed += kAcceleration * kDt;
    position += speed * kDt;
    const int32_t ticks = QuantizedTicks(position, ticks_per_meter);
    Check(observer.Update((ticks - previous_ticks) * resolution, kDt, resolution), "acceleration update was rejected");
    previous_ticks = ticks;
  }
  Check(std::fabs(observer.velocity() - speed) < 0.03f, "observer response to acceleration is too slow");

  for (int index = 0; index < 100; ++index) {
    speed -= kAcceleration * kDt;
    position += speed * kDt;
    const int32_t ticks = QuantizedTicks(position, ticks_per_meter);
    Check(observer.Update((ticks - previous_ticks) * resolution, kDt, resolution), "deceleration update was rejected");
    previous_ticks = ticks;
  }
  Check(std::fabs(observer.velocity()) < 0.03f, "observer response to stopping is too slow");

  for (int index = 0; index < 100; ++index) {
    speed -= kAcceleration * kDt;
    position += speed * kDt;
    const int32_t ticks = QuantizedTicks(position, ticks_per_meter);
    Check(observer.Update((ticks - previous_ticks) * resolution, kDt, resolution), "reversal update was rejected");
    previous_ticks = ticks;
  }
  Check(std::fabs(observer.velocity() - speed) < 0.03f, "observer response to reversal is too slow");
}

void TestPureRotationCancellation(float ticks_per_meter) {
  constexpr float kDt = 0.011f;
  constexpr float kWheelSpeed = 0.2f;
  const float resolution = 1.0f / ticks_per_meter;

  WheelSpeedObserver left;
  WheelSpeedObserver right;
  double left_position = 0.0;
  double right_position = 0.0;
  int32_t previous_left_ticks = 0;
  int32_t previous_right_ticks = 0;

  for (int index = 0; index < 500; ++index) {
    left_position -= kWheelSpeed * kDt;
    right_position += kWheelSpeed * kDt;
    const int32_t left_ticks = QuantizedTicks(left_position, ticks_per_meter);
    const int32_t right_ticks = QuantizedTicks(right_position, ticks_per_meter);
    Check(left.Update((left_ticks - previous_left_ticks) * resolution, kDt, resolution),
          "left rotation update was rejected");
    Check(right.Update((right_ticks - previous_right_ticks) * resolution, kDt, resolution),
          "right rotation update was rejected");
    previous_left_ticks = left_ticks;
    previous_right_ticks = right_ticks;
  }

  Check(std::fabs(0.5f * (left.velocity() + right.velocity())) < 0.005f,
        "pure rotation produced excessive linear velocity");
}

void TestInvalidSamplesResetObserver() {
  constexpr float kResolution = 1.0f / 1195.0f;
  WheelSpeedObserver observer;
  Check(observer.Update(0.003f, 0.01f, kResolution), "valid update was rejected");
  Check(!observer.Update(0.0f, 0.75f, kResolution), "long sample interval was accepted");
  Check(observer.velocity() == 0.0f, "long sample interval did not reset velocity");
  Check(!observer.Update(1.0f, 0.01f, kResolution), "impossible wheel speed was accepted");
}

void TestTachoRolloverDelta() {
  const uint32_t previous = UINT32_MAX - 1;
  const uint32_t current = 1;
  Check(static_cast<int32_t>(current - previous) == 3, "tacho rollover delta is incorrect");
}

}  // namespace

int main() {
  TestConstantSpeedWithQuantizedTicks(1195.0f, 0.35f);
  TestConstantSpeedWithQuantizedTicks(600.0f, 0.35f);
  TestConstantSpeedWithQuantizedTicks(600.0f, 0.05f);
  TestAccelerationAndStopResponse(1195.0f);
  TestAccelerationAndStopResponse(600.0f);
  TestPureRotationCancellation(1195.0f);
  TestPureRotationCancellation(600.0f);
  TestInvalidSamplesResetObserver();
  TestTachoRolloverDelta();
  return 0;
}
