/*
 * OpenMower
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Created by clemens on 09.09.24.
 * Copyright (C) 2024, 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "PowerServiceInterface.h"

void PowerServiceInterface::OnChargeVoltageChanged(const float& new_value) {
  power_msg_.charge_voltage = new_value;
}

void PowerServiceInterface::OnChargeCurrentChanged(const float& new_value) {
  power_msg_.charge_current = new_value;
}

void PowerServiceInterface::OnBatteryVoltageChanged(const float& new_value) {
  power_msg_.battery_voltage = new_value;
}

void PowerServiceInterface::OnChargingStatusChanged(const char* new_value, uint32_t length) {
  power_msg_.charger_status = std::string(new_value, length);
}

void PowerServiceInterface::OnChargerEnabledChanged(const uint8_t& new_value) {
  power_msg_.charger_enabled = new_value;
}

void PowerServiceInterface::OnBatteryPercentageChanged(const float& new_value) {
  power_msg_.battery_pct = new_value;
}

void PowerServiceInterface::OnChargeVoltageADCChanged(const float& new_value) {
  power_msg_.charge_voltage_adc = new_value;
}

void PowerServiceInterface::OnBatteryVoltageADCChanged(const float& new_value) {
  power_msg_.battery_voltage_adc = new_value;
}

void PowerServiceInterface::OnDCDCInputCurrentChanged(const float& new_value) {
  power_msg_.dcdc_input_current = new_value;
}

void PowerServiceInterface::OnChargerInputCurrentChanged(const float& new_value) {
  power_msg_.charger_input_current = new_value;
}

bool PowerServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  std::unique_lock<std::mutex> lk{config_mutex_};
  StartTransaction(true);
  SetRegisterBatteryEmptyVoltage(battery_empty_voltage_);
  SetRegisterBatteryFullVoltage(battery_full_voltage_);
  SetRegisterCriticalBatteryLowVoltage(battery_critical_voltage_);
  SetRegisterCriticalBatteryHighVoltage(battery_critical_high_voltage_);
  // Optional registers
  if (charge_voltage_ > 0.0f) SetRegisterChargeVoltage(charge_voltage_);
  if (charge_current_ > 0.0f) SetRegisterChargeCurrent(charge_current_);
  if (charge_termination_current_ > 0.0f) SetRegisterTerminationCurrent(charge_termination_current_);
  if (charge_precharge_current_ > 0.0f) SetRegisterPreChargeCurrent(charge_precharge_current_);
  if (charge_recharge_voltage_ >= 0) {
    SetRegisterReChargeVoltage(static_cast<ReChargeVoltages>(charge_recharge_voltage_));
  }
  if (system_current_ > 0.0f) SetRegisterSystemCurrent(system_current_);
  SetRegisterDangerouslyOverrideHardwareChargeCurrentLimit(override_hardware_charge_current_limit_ ? 1 : 0);
  CommitTransaction();
  return true;
}

void PowerServiceInterface::UpdateConfig(float battery_full_voltage, float battery_empty_voltage,
                                         float battery_critical_voltage, float battery_critical_high_voltage,
                                         float charge_current, float system_current) {
  std::unique_lock<std::mutex> lk{config_mutex_};
  battery_full_voltage_ = battery_full_voltage;
  battery_empty_voltage_ = battery_empty_voltage;
  battery_critical_voltage_ = battery_critical_voltage;
  battery_critical_high_voltage_ = battery_critical_high_voltage;
  charge_current_ = charge_current;
  system_current_ = system_current;

  StartTransaction(true);
  SetRegisterBatteryFullVoltage(battery_full_voltage_);
  SetRegisterBatteryEmptyVoltage(battery_empty_voltage_);
  SetRegisterCriticalBatteryLowVoltage(battery_critical_voltage_);
  SetRegisterCriticalBatteryHighVoltage(battery_critical_high_voltage_);
  SetRegisterChargeCurrent(charge_current_);
  SetRegisterSystemCurrent(system_current_);
  CommitTransaction();
}

void PowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  power_msg_ = {};
  power_msg_.stamp = ros::Time::now();
}

void PowerServiceInterface::OnTransactionEnd() {
  status_publisher_.publish(power_msg_);
}
