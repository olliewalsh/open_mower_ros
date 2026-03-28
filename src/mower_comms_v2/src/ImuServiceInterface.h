//
// Created by clemens on 26.07.24.
//

#ifndef IMUSERVICEINTERFACE_H
#define IMUSERVICEINTERFACE_H

#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>

#include <ImuServiceInterfaceBase.hpp>

class ImuServiceInterface : public ImuServiceInterfaceBase {
 public:
  struct CollisionConfig {
    uint8_t disable_detection = 1;
    float accel_threshold = 0.0f;
    float gyro_threshold = 0.0f;
    float jerk_threshold = 0.0f;
    float gravity_filter_hz = 0.0f;
    float signal_filter_hz = 0.0f;
    float wheel_current_threshold = 0.0f;
    float actual_linear_speed_threshold = 0.0f;
    float actual_angular_speed_threshold = 0.0f;
    float actual_speed_drop_threshold = 0.0f;
    uint16_t consecutive_samples = 0;
  };

  ImuServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& imu_publisher,
                      const ros::Publisher& collision_debug_publisher, const std::string& axis_config,
                      const CollisionConfig& collision_config)
      : ImuServiceInterfaceBase(service_id, ctx),
        imu_publisher_(imu_publisher),
        collision_debug_publisher_(collision_debug_publisher),
        axis_config_(axis_config),
        collision_config_(collision_config) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnAxesChanged(const double* new_value, uint32_t length) override;
  void OnCollisionDebugChanged(const double* new_value, uint32_t length) override;

 private:
  const ros::Publisher& imu_publisher_;
  const ros::Publisher& collision_debug_publisher_;
  std::string axis_config_;
  CollisionConfig collision_config_;

  sensor_msgs::Imu imu_msg{};
  std_msgs::Float64MultiArray collision_debug_msg_{};
  bool validateAxisConfig();
};

#endif  // IMUSERVICEINTERFACE_H
