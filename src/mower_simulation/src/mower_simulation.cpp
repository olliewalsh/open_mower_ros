// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "ros/ros.h"

// Include messages for mower control
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/Power.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <xbot-service/Io.hpp>
#include <xbot-service/portable/system.hpp>

#include "../../../services/service_ids.h"
#include "SimRobot.h"
#include "dynamic_reconfigure/server.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_simulation/MowerSimulationConfig.h"
#include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"

namespace {
double positiveRateParam(ros::NodeHandle& nh, const std::string& name, double default_value) {
  double rate_hz = default_value;
  nh.param(name, rate_hz, default_value);
  if (rate_hz <= 0.0) {
    ROS_WARN_STREAM("Invalid mower_simulation parameter '" << name << "'=" << rate_hz << ". Falling back to "
                                                           << default_value << " Hz.");
    rate_hz = default_value;
  }
  return rate_hz;
}

uint32_t rateHzToIntervalUs(double rate_hz) {
  return static_cast<uint32_t>(std::llround(1'000'000.0 / rate_hz));
}
}  // namespace

ros::Publisher status_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher pose_pub;
ros::Publisher initial_pose_publisher;
ros::ServiceClient docking_point_client;

dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig>* reconfig_server;

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_simulation");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  reconfig_server = new dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig>(paramNh);
  // reconfig_server->setCallback(reconfigureCB);

  docking_point_client = n.serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");

  ros::NodeHandle llParamNh("/ll");
  std::string bind_ip = "0.0.0.0";
  llParamNh.getParam("bind_ip", bind_ip);
  ROS_INFO_STREAM("Bind IP (Mower Simulation): " << bind_ip);

  xbot::service::system::initSystem();
  xbot::service::Io::start(bind_ip.c_str());

  const double simulation_rate_hz = positiveRateParam(paramNh, "simulation_rate_hz", 25.0);
  const double diff_drive_service_rate_hz = positiveRateParam(paramNh, "diff_drive_service_rate_hz", 25.0);
  const double imu_service_rate_hz = positiveRateParam(paramNh, "imu_service_rate_hz", 100.0);
  const double gps_service_rate_hz = positiveRateParam(paramNh, "gps_service_rate_hz", 10.0);
  const double emergency_service_rate_hz = positiveRateParam(paramNh, "emergency_service_rate_hz", 1.0);
  const double mower_service_rate_hz = positiveRateParam(paramNh, "mower_service_rate_hz", 2.0);
  const double power_service_rate_hz = positiveRateParam(paramNh, "power_service_rate_hz", 1.0);

  ROS_INFO_STREAM("Mower simulation rates [Hz] sim="
                  << simulation_rate_hz << " diff_drive=" << diff_drive_service_rate_hz << " imu="
                  << imu_service_rate_hz << " gps=" << gps_service_rate_hz << " emergency=" << emergency_service_rate_hz
                  << " mower=" << mower_service_rate_hz << " power=" << power_service_rate_hz);

  SimRobot robot{paramNh, ros::Duration(1.0 / simulation_rate_hz)};

  // Move the robot to the docking station.
  // TODO: Use a better way to make sure that the docking position is loaded.
  mower_map::GetDockingPointSrv get_docking_point_srv;
  for (int i = 0; i < 20; ++i) {
    if (docking_point_client.call(get_docking_point_srv)) {
      const auto& docking_pose = get_docking_point_srv.response.docking_pose;
      if (docking_pose.position.x != 0 || docking_pose.position.y != 0) {
        tf2::Quaternion quat;
        tf2::fromMsg(docking_pose.orientation, quat);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robot.SetDockingPose(docking_pose.position.x, docking_pose.position.y, yaw);
        robot.SetPosition(docking_pose.position.x, docking_pose.position.y, yaw);
        break;
      }
    }
    sleep(1);
  }

  EmergencyService emergency_service{xbot::service_ids::EMERGENCY, robot,
                                     rateHzToIntervalUs(emergency_service_rate_hz)};
  DiffDriveService diff_drive_service{xbot::service_ids::DIFF_DRIVE, robot,
                                      rateHzToIntervalUs(diff_drive_service_rate_hz)};
  MowerService mower_service{xbot::service_ids::MOWER, robot, rateHzToIntervalUs(mower_service_rate_hz)};
  ImuService imu_service{xbot::service_ids::IMU, robot, rateHzToIntervalUs(imu_service_rate_hz)};
  PowerService power_service{xbot::service_ids::POWER, robot, rateHzToIntervalUs(power_service_rate_hz)};
  GpsService gps_service{xbot::service_ids::GPS, robot, rateHzToIntervalUs(gps_service_rate_hz)};

  emergency_service.start();
  diff_drive_service.start();
  mower_service.start();
  imu_service.start();
  power_service.start();
  gps_service.start();

  robot.Start();

  ros::spin();
  delete (reconfig_server);
  return 0;
}
