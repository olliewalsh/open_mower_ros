#ifndef MOWER_COMMS_V1_WHEEL_SPEED_CONTROLLER_HPP
#define MOWER_COMMS_V1_WHEEL_SPEED_CONTROLLER_HPP

class WheelSpeedController {
 public:
  struct Gains {
    Gains(float feedforward_value = 1.5f, float kp_value = 0.35f, float ki_value = 1.5f, float kd_value = 0.0f)
        : feedforward(feedforward_value), kp(kp_value), ki(ki_value), kd(kd_value) {
    }

    float feedforward;
    float kp;
    float ki;
    float kd;
  };

  explicit WheelSpeedController(Gains gains) : gains_(gains) {
  }

  void SetGains(Gains gains) {
    gains_ = gains;
  }

  void SetMaxDuty(float max_duty) {
    max_duty_ = max_duty;
  }

  void Reset() {
    target_speed_ = 0;
    measured_speed_ = 0;
    integral_ = 0;
    prev_error_ = 0;
    duty_ = 0;
  }

  void SetTargetSpeed(float speed) {
    // Integral learned in one direction works against the controller after stopping or reversing.
    // Seed the previous error for the new target so a configured derivative term does not kick.
    const bool stops_or_reverses = target_speed_ != 0.0f && (speed == 0.0f || (target_speed_ > 0.0f) != (speed > 0.0f));
    target_speed_ = speed;
    if (stops_or_reverses) {
      integral_ = 0.0f;
      prev_error_ = target_speed_ - measured_speed_;
    }
  }

  void SetMeasuredSpeed(float speed) {
    measured_speed_ = speed;
  }

  float Update(float dt) {
    const float error = target_speed_ - measured_speed_;
    float next_integral = integral_;
    if (dt > 0.0f) {
      next_integral += error * dt;
    }

    float derivative = 0.0f;
    if (dt > 0.0f) {
      derivative = (error - prev_error_) / dt;
    }
    prev_error_ = error;

    float unsaturated =
        gains_.feedforward * target_speed_ + gains_.kp * error + gains_.ki * next_integral + gains_.kd * derivative;

    // Only integrate while unsaturated, or when the error drives the controller back out of saturation.
    if ((unsaturated <= max_duty_ && unsaturated >= -max_duty_) || (unsaturated > max_duty_ && error < 0.0f) ||
        (unsaturated < -max_duty_ && error > 0.0f)) {
      integral_ = next_integral;
      unsaturated =
          gains_.feedforward * target_speed_ + gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    }

    duty_ = Clamp(unsaturated);
    return duty_;
  }

  float measured_speed() const {
    return measured_speed_;
  }

  float duty() const {
    return duty_;
  }

 private:
  float Clamp(float value) const {
    if (value >= max_duty_) {
      return max_duty_;
    }
    if (value <= -max_duty_) {
      return -max_duty_;
    }
    return value;
  }

  Gains gains_;
  float max_duty_ = 0.95f;
  float target_speed_ = 0;
  float measured_speed_ = 0;
  float integral_ = 0;
  float prev_error_ = 0;
  float duty_ = 0;
};

#endif  // MOWER_COMMS_V1_WHEEL_SPEED_CONTROLLER_HPP
