#ifndef OPEN_MOWER_WHEEL_SPEED_OBSERVER_HPP
#define OPEN_MOWER_WHEEL_SPEED_OBSERVER_HPP

#include <cmath>

class WheelSpeedObserver {
 public:
  struct Config {
    Config(float acceleration_noise_value = 0.5f, float max_sample_interval_value = 0.5f, float max_speed_value = 2.0f)
        : acceleration_noise(acceleration_noise_value),
          max_sample_interval(max_sample_interval_value),
          max_speed(max_speed_value) {
    }

    float acceleration_noise;   // Standard deviation of unmodelled wheel acceleration in m/s^2.
    float max_sample_interval;  // Reinitialize after a status gap longer than this many seconds.
    float max_speed;            // Reject a tacho delta implying a faster wheel speed in m/s.
  };

  explicit WheelSpeedObserver(Config config = Config()) : config_(config) {
    Reset();
  }

  void Reset() {
    initialized_ = false;
    measured_position_ = 0.0f;
    position_ = 0.0f;
    velocity_ = 0.0f;
    covariance_position_ = 0.0f;
    covariance_position_velocity_ = 0.0f;
    covariance_velocity_ = 0.0f;
  }

  bool Update(float distance_delta, float dt, float position_resolution) {
    if (!std::isfinite(distance_delta) || !std::isfinite(dt) || !std::isfinite(position_resolution) || dt <= 0.0f ||
        position_resolution <= 0.0f || dt > config_.max_sample_interval ||
        std::fabs(distance_delta / dt) > config_.max_speed) {
      Reset();
      return false;
    }

    const float measurement_variance = position_resolution * position_resolution / 12.0f;
    if (!initialized_) {
      initialized_ = true;
      covariance_position_ = measurement_variance;
      covariance_velocity_ = 1.0f;
    }

    measured_position_ += distance_delta;
    // Keep the single-precision position small without changing the innovation.
    if (std::fabs(measured_position_) > 10.0f) {
      position_ -= measured_position_;
      measured_position_ = 0.0f;
    }

    const float predicted_position = position_ + velocity_ * dt;
    const float predicted_velocity = velocity_;

    const float acceleration_variance = config_.acceleration_noise * config_.acceleration_noise;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float predicted_covariance_position = covariance_position_ + 2.0f * dt * covariance_position_velocity_ +
                                                dt2 * covariance_velocity_ + 0.25f * dt4 * acceleration_variance;
    const float predicted_covariance_position_velocity =
        covariance_position_velocity_ + dt * covariance_velocity_ + 0.5f * dt3 * acceleration_variance;
    const float predicted_covariance_velocity = covariance_velocity_ + dt2 * acceleration_variance;

    const float innovation_variance = predicted_covariance_position + measurement_variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0f) {
      Reset();
      return false;
    }

    const float position_gain = predicted_covariance_position / innovation_variance;
    const float velocity_gain = predicted_covariance_position_velocity / innovation_variance;
    const float innovation = measured_position_ - predicted_position;
    position_ = predicted_position + position_gain * innovation;
    velocity_ = predicted_velocity + velocity_gain * innovation;

    // Joseph covariance update preserves symmetry and positive semi-definiteness under rounding.
    const float one_minus_position_gain = 1.0f - position_gain;
    covariance_position_ = one_minus_position_gain * one_minus_position_gain * predicted_covariance_position +
                           position_gain * position_gain * measurement_variance;
    covariance_position_velocity_ = one_minus_position_gain * (predicted_covariance_position_velocity -
                                                               velocity_gain * predicted_covariance_position) +
                                    position_gain * velocity_gain * measurement_variance;
    covariance_velocity_ = velocity_gain * velocity_gain * predicted_covariance_position -
                           2.0f * velocity_gain * predicted_covariance_position_velocity +
                           predicted_covariance_velocity + velocity_gain * velocity_gain * measurement_variance;

    if (!std::isfinite(position_) || !std::isfinite(velocity_) || !std::isfinite(covariance_position_) ||
        !std::isfinite(covariance_position_velocity_) || !std::isfinite(covariance_velocity_) ||
        covariance_position_ < 0.0f || covariance_velocity_ < 0.0f) {
      Reset();
      return false;
    }
    return true;
  }

  float velocity() const {
    return velocity_;
  }

 private:
  Config config_;
  bool initialized_ = false;
  float measured_position_ = 0.0f;
  float position_ = 0.0f;
  float velocity_ = 0.0f;
  float covariance_position_ = 0.0f;
  float covariance_position_velocity_ = 0.0f;
  float covariance_velocity_ = 0.0f;
};

#endif  // OPEN_MOWER_WHEEL_SPEED_OBSERVER_HPP
