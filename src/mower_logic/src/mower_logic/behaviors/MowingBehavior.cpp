// Created by Clemens Elflein on 2/21/22.
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
#include "MowingBehavior.h"

#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <mbf_msgs/RecoveryAction.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <EmergencyServiceInterfaceBase.hpp>
#include <cmath>
#include <limits>

#include "../StateSubscriber.h"
#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mower_msgs/Status.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern std::string mowingController;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern actionlib::SimpleActionClient<mbf_msgs::RecoveryAction>* mbfClientRecovery;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);
extern void setEmergencyMode(uint16_t reason);

extern StateSubscriber<mower_msgs::Status> status_state_subscriber;

extern std::string current_job_id;
extern bool current_job_finished;

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  while (ros::ok() && !aborted) {
    if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return &DockingBehavior::INSTANCE;
    }

    // No plan will be created if the area is skipped
    if (currentMowingPaths.empty()) {
      currentMowingArea++;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
      currentMowingArea++;
      currentMowingPaths.clear();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }
  // we got aborted, go to docking station
  return &DockingBehavior::INSTANCE;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  mowerEnabled = false;
  paused = aborted = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  mowerEnabled = false;
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
  publishMowerEvent("JOB_COMPLETE");
  current_job_finished = true;
  currentMowingPaths.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
  // Delete old plan and progress.
  currentMowingPaths.clear();

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return false;
  }

  currentMowingAreaId = mapSrv.response.area.id;
  currentMowingAreaName = mapSrv.response.area.name;

  if (!mapSrv.response.area.active) {
    ROS_INFO_STREAM("MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  // Area orientation is the same as the first point, unless explicitly specified in the area attributes
  double angle = 0;
  if (!std::isnan(mapSrv.response.area.angle)) {
    angle = mapSrv.response.area.angle;
    ROS_INFO_STREAM("MowingBehavior: Using explicitly specified mow angle: " << angle);
  } else {
    auto points = mapSrv.response.area.area.points;
    if (points.size() >= 2) {
      tf2::Vector3 first(points[0].x, points[0].y, 0);
      for (auto point : points) {
        tf2::Vector3 second(point.x, point.y, 0);
        auto diff = second - first;
        if (diff.length() > 2.0) {
          // we have found a point that has a distance of > 2 m, calculate the angle
          angle = atan2(diff.y(), diff.x());
          ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
          break;
        }
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
  }

  // calculate coverage
  const auto& area = mapSrv.response.area;
  auto overrideOrGlobal = [](auto override, auto global, auto sentinel) {
    return (override != sentinel) ? override : global;
  };

  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  const int outline_count = std::max(0, overrideOrGlobal(area.outline_count, config.outline_count, -1));
  const double outline_approach_boundary_buffer = config.tool_width * outline_count;
  pathSrv.request.outline_count = outline_count;
  pathSrv.request.outline_overlap_count =
      overrideOrGlobal(area.outline_overlap_count, config.outline_overlap_count, -1);
  pathSrv.request.outline = area.area;
  pathSrv.request.holes = area.obstacles;
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = std::isnan(area.outline_offset) ? config.outline_offset : area.outline_offset;
  pathSrv.request.distance = config.tool_width;
  pathSrv.request.outline_approach_length = config.outline_approach_length;
  pathSrv.request.outline_approach_inset = std::max(config.outline_approach_inset, outline_approach_boundary_buffer);
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return false;
  }

  currentMowingPaths = pathSrv.response.paths;

  // Calculate mowing plan digest from the poses
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& path : currentMowingPaths) {
    for (const auto& pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte*)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath
                                                                      << " index: " << currentMowingPathIndex);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  return true;
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

namespace {
bool pointInPolygon(double x, double y, const geometry_msgs::Polygon& polygon) {
  bool inside = false;
  const auto& points = polygon.points;
  if (points.size() < 3) return false;
  for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    const auto& a = points[i];
    const auto& b = points[j];
    const bool crosses = ((a.y > y) != (b.y > y)) && (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x);
    if (crosses) inside = !inside;
  }
  return inside;
}

double distanceToPolygonBoundary(double x, double y, const geometry_msgs::Polygon& polygon) {
  const auto& points = polygon.points;
  if (points.size() < 2) return 0.0;
  double distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    const double dx = points[i].x - points[j].x;
    const double dy = points[i].y - points[j].y;
    const double length_squared = dx * dx + dy * dy;
    const double fraction =
        length_squared > 0.0
            ? std::max(0.0, std::min(1.0, ((x - points[j].x) * dx + (y - points[j].y) * dy) / length_squared))
            : 0.0;
    const double closest_x = points[j].x + fraction * dx;
    const double closest_y = points[j].y + fraction * dy;
    distance = std::min(distance, std::hypot(x - closest_x, y - closest_y));
  }
  return distance;
}

geometry_msgs::PoseStamped bezierPose(const geometry_msgs::PoseStamped& reference, double u, double p0x, double p0y,
                                      double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
  const double v = 1.0 - u;
  geometry_msgs::PoseStamped pose = reference;
  pose.pose.position.x = v * v * v * p0x + 3 * v * v * u * p1x + 3 * v * u * u * p2x + u * u * u * p3x;
  pose.pose.position.y = v * v * v * p0y + 3 * v * v * u * p1y + 3 * v * u * u * p2y + u * u * u * p3y;
  const double dx = 3 * v * v * (p1x - p0x) + 6 * v * u * (p2x - p1x) + 3 * u * u * (p3x - p2x);
  const double dy = 3 * v * v * (p1y - p0y) + 6 * v * u * (p2y - p1y) + 3 * u * u * (p3y - p2y);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, atan2(dy, dx));
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

// Returns the names of the recovery behaviors configured on move_base_flex
// (its "recovery_behaviors" param), in order, or an empty list if none are
// configured. This avoids hardcoding behavior names and makes recovery a no-op
// when MBF has no recovery configured.
std::vector<std::string> getConfiguredRecoveryBehaviors() {
  std::vector<std::string> names;
  XmlRpc::XmlRpcValue behaviors;
  if (!ros::param::get("/move_base_flex/recovery_behaviors", behaviors)) {
    return names;
  }
  if (behaviors.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return names;
  }
  for (int i = 0; i < behaviors.size(); i++) {
    XmlRpc::XmlRpcValue& b = behaviors[i];
    if (b.getType() == XmlRpc::XmlRpcValue::TypeStruct && b.hasMember("name")) {
      names.push_back(static_cast<std::string>(b["name"]));
    }
  }
  return names;
}
}  // namespace

bool MowingBehavior::build_outline_approach(const geometry_msgs::PoseStamped& goal, nav_msgs::Path& approach,
                                            geometry_msgs::PoseStamped& staging_pose) {
  mower_map::GetMowingAreaSrv map_srv;
  map_srv.request.index = currentMowingArea;
  if (!mapClient.call(map_srv)) {
    ROS_ERROR_STREAM("MowingBehavior: Could not load area geometry for outline approach validation.");
    return false;
  }
  const auto& area_outline = map_srv.response.area.area;
  const auto& area_obstacles = map_srv.response.area.obstacles;
  const int outline_count = std::max(
      0, map_srv.response.area.outline_count >= 0 ? map_srv.response.area.outline_count : config.outline_count);
  const double boundary_buffer = config.tool_width * outline_count;

  tf2::Quaternion goal_q;
  tf2::fromMsg(goal.pose.orientation, goal_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(goal_q).getRPY(roll, pitch, yaw);
  const double tx = cos(yaw);
  const double ty = sin(yaw);
  // Try the configured geometry first, then progressively reduce its length in
  // narrow gaps. Never reduce the staging-point boundary clearance.
  for (double scale : {1.0, 0.75, 0.5, 0.25}) {
    const double length = std::max(0.5, config.outline_approach_length * scale);
    const double inset = std::max(boundary_buffer, std::max(0.1, config.outline_approach_inset * scale));
    const int sample_count = std::max(3, static_cast<int>(ceil((length + inset) / 0.1)));

    // Try both path normals. The valid side is inside the mowing area and outside every obstacle.
    for (double side : {1.0, -1.0}) {
      const double nx = -ty * side;
      const double ny = tx * side;
      const double p3x = goal.pose.position.x;
      const double p3y = goal.pose.position.y;
      const double p0x = p3x - tx * length + nx * inset;
      const double p0y = p3y - ty * length + ny * inset;
      if (!pointInPolygon(p0x, p0y, area_outline) ||
          distanceToPolygonBoundary(p0x, p0y, area_outline) + 1e-6 < boundary_buffer) {
        continue;
      }
      bool staging_pose_clear = true;
      for (const auto& obstacle : area_obstacles) {
        if (pointInPolygon(p0x, p0y, obstacle) ||
            distanceToPolygonBoundary(p0x, p0y, obstacle) + 1e-6 < boundary_buffer) {
          staging_pose_clear = false;
          break;
        }
      }
      if (!staging_pose_clear) continue;
      const double handle = length * 0.45;
      const double p1x = p0x + tx * handle;
      const double p1y = p0y + ty * handle;
      const double p2x = p3x - tx * handle;
      const double p2y = p3y - ty * handle;

      nav_msgs::Path candidate;
      candidate.header = goal.header;
      bool valid = true;
      for (int i = 0; i <= sample_count; ++i) {
        const double u = static_cast<double>(i) / sample_count;
        auto pose = bezierPose(goal, u, p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y);
        // The last sample is the existing coverage goal and may be exactly on a polygon boundary.
        if (i != sample_count && !pointInPolygon(pose.pose.position.x, pose.pose.position.y, area_outline)) {
          valid = false;
          break;
        }
        for (const auto& obstacle : area_obstacles) {
          if (pointInPolygon(pose.pose.position.x, pose.pose.position.y, obstacle)) {
            valid = false;
            break;
          }
        }
        if (!valid) break;
        candidate.poses.push_back(pose);
      }
      if (valid) {
        if (scale < 1.0) {
          ROS_INFO_STREAM("MowingBehavior: Using reduced tangent outline approach (length " << length << "m, inset "
                                                                                            << inset << "m).");
        }
        approach = candidate;
        staging_pose = candidate.poses.front();
        return true;
      }
    }
  }
  return false;
}

/// @return true if spinup succeeded or spinup is disabled, false if we should abort/exit
bool MowingBehavior::wait_for_mower_spinup() {
  if (config.mower_spinup_rpm <= 0) {
    return true;  // spinup check disabled
  }

  ROS_INFO_STREAM("MowingBehavior: (MOW) Waiting for mower motor to reach " << config.mower_spinup_rpm
                                                                            << " RPM before driving");
  ros::Time spinup_start = ros::Time::now();
  ros::Rate check_rate(10);
  while (ros::ok()) {
    if (aborted || requested_pause_flag || skip_area || skip_path) {
      if (aborted || requested_pause_flag || skip_area ||
          currentMowingPath + 1 >= static_cast<int>(currentMowingPaths.size())) {
        mowerEnabled = false;
      }
      return false;
    }
    auto last_status = status_state_subscriber.getMessage();
    if (std::abs(last_status.mower_motor_rpm) >= config.mower_spinup_rpm) {
      ROS_INFO_STREAM("MowingBehavior: (MOW) Mower motor reached " << last_status.mower_motor_rpm << " RPM after "
                                                                   << (ros::Time::now() - spinup_start).toSec() << "s");
      return true;
    }
    if (ros::Time::now() - spinup_start > ros::Duration(config.mower_spinup_timeout)) {
      ROS_ERROR_STREAM("MowingBehavior: (MOW) Mower motor failed to reach " << config.mower_spinup_rpm << " RPM within "
                                                                            << config.mower_spinup_timeout
                                                                            << "s. Entering emergency.");
      publishMowerEvent("MOW_MOTOR_SPINUP_FAILED");
      mowerEnabled = false;
      setEmergencyMode(EmergencyReason::MOWER_RPM_TIMEOUT);
      return false;
    }
    check_rate.sleep();
  }
  mowerEnabled = false;
  return false;
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  int outline_approach_attempt_counter = 0;
  int failed_path = -1;
  int failed_path_index = -1;
  int failed_path_attempts = 0;
  double outline_approach_backtrack_distance = 0.0;
  ros::Time paused_time(0.0);
  u_int8_t pause_cause = 0;

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      pause_cause = requested_pause_flag;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & (pauseType::PAUSE_MANUAL | pauseType::PAUSE_OVERTEMP)) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_OVERTEMP) {
          pause_reason += "waiting for cool-down";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MOW_STALL) {
          if (!pause_reason.empty()) pause_reason += " and ";
          pause_reason += "mow motor stalled";
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          if (!pause_reason.empty()) pause_reason += " and ";
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();

      // Skip forward on path after overtemp or stall to avoid re-entering the same spot
      if ((pause_cause & (pauseType::PAUSE_OVERTEMP | pauseType::PAUSE_MOW_STALL)) &&
          config.pause_resume_skip_distance > 0.0 && currentMowingPath < currentMowingPaths.size()) {
        auto& skip_path = currentMowingPaths[currentMowingPath];
        double skipped = 0.0;
        while (currentMowingPathIndex + 1 < skip_path.path.poses.size() &&
               skipped < config.pause_resume_skip_distance) {
          auto& p1 = skip_path.path.poses[currentMowingPathIndex].pose.position;
          auto& p2 = skip_path.path.poses[currentMowingPathIndex + 1].pose.position;
          double dx = p2.x - p1.x;
          double dy = p2.y - p1.y;
          skipped += sqrt(dx * dx + dy * dy);
          currentMowingPathIndex++;
        }
        ROS_INFO_STREAM("MowingBehavior: Skipped " << skipped << "m forward on path after pause");
      }
    }

    auto& path = currentMowingPaths[currentMowingPath];
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= path.path.poses.size()) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that
    //   prevented us from reaching the inital pose
    // * after n attempts, we fail the mow area and skip to the next one
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      nav_msgs::Path outline_approach;
      geometry_msgs::PoseStamped navigation_target = path.path.poses[currentMowingPathIndex];
      bool use_outline_approach = path.is_outline && config.outline_approach_enabled;
      if (use_outline_approach &&
          !build_outline_approach(path.path.poses[currentMowingPathIndex], outline_approach, navigation_target)) {
        if (currentMowingPathIndex == 0) {
          ROS_INFO_STREAM(
              "MowingBehavior: (FIRST POINT) No tangent approach available at the planned outline start; "
              "using the original start pose.");
          use_outline_approach = false;
        } else {
          const auto& current_position = path.path.poses[currentMowingPathIndex].pose.position;
          const auto& previous_position = path.path.poses[currentMowingPathIndex - 1].pose.position;
          const double dx = current_position.x - previous_position.x;
          const double dy = current_position.y - previous_position.y;
          const double backtrack_step_distance = hypot(dx, dy);
          if (outline_approach_backtrack_distance + backtrack_step_distance >
              config.outline_approach_max_backtrack_distance) {
            ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) No safe outline approach found while backtracking "
                             << outline_approach_backtrack_distance
                             << "m; aborting rather than skipping unmowed path.");
            mowerEnabled = false;
            this->abort();
            return false;
          }
          currentMowingPathIndex--;
          outline_approach_backtrack_distance += backtrack_step_distance;
          checkpoint();
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) Could not construct a safe outline approach; backtracking to "
                          << currentMowingPathIndex << " (" << outline_approach_backtrack_distance
                          << "m total) so already-mowed outline may be repeated.");
          continue;
        }
      }
      if (path.is_outline && getConfig().add_fake_obstacle && !use_outline_approach) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = navigation_target;
      moveBaseGoal.controller = mowingController;
      mbfClient->sendGoal(moveBaseGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClient->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
            publishMowerEvent("AREA_SKIPPED");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClient->cancelAllGoals();
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            if (currentMowingPath + 1 >= static_cast<int>(currentMowingPaths.size())) {
              mowerEnabled = false;
            }
            mbfClient->cancelAllGoals();
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
            mbfClient->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
            mbfClient->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                          << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      first_point_attempt_counter++;
      if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
        // we cannot reach the start point
        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: "
                         << current_status.state_);
        // we have 3 attempts to get to the start pose of the mowing area
        if (first_point_attempt_counter < config.max_first_point_attempts) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / "
                                                                     << config.max_first_point_attempts
                                                                     << " Making a little pause ...");
          paused = true;
          update_actions();
        } else {
          // We failed to reach the first point in the mow path by simply repeating the drive to process
          // So now we will trim the path by removing the first pose
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            // We try now to remove the first point so the 2nd, 3rd etc point becomes our target
            // mow path points are offset by 10cm
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_trim_counter << " / "
                                                                       << config.max_first_point_trim_attempts
                                                                       << " Trying an adjacent path point.");
            if (path.is_outline) {
              if (currentMowingPathIndex == 0) {
                ROS_ERROR_STREAM(
                    "MowingBehavior: (FIRST POINT) Cannot backtrack before the start of the outline; "
                    "aborting rather than skipping unmowed path.");
                this->abort();
                return false;
              }
              currentMowingPathIndex--;
              ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) Backtracking to outline index "
                              << currentMowingPathIndex << "; already-mowed path may be repeated.");
            } else {
              currentMowingPathIndex++;
            }
            first_point_trim_counter++;
            first_point_attempt_counter = 0;  // give it another <config.max_first_point_attempts> attempts
            paused = true;
            update_actions();
          } else {
            // Unable to reach the start of the mow path (we tried multiple attempts for the same point, and we skipped
            // points which also didnt work, time to give up)
            ROS_ERROR_STREAM(
                "MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - "
                "aborting at index: "
                << currentMowingPathIndex << " path: " << currentMowingPath << " area: " << currentMowingArea);
            this->abort();
          }
        }
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);

      if (use_outline_approach) {
        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Following tangent outline approach with "
                        << outline_approach.poses.size() << " poses.");
        mbf_msgs::ExePathGoal approach_goal;
        approach_goal.path = outline_approach;
        approach_goal.angle_tolerance = 5.0 * (M_PI / 180.0);
        approach_goal.dist_tolerance = 0.1;
        approach_goal.tolerance_from_action = true;
        approach_goal.controller = mowingController;
        mbfClientExePath->sendGoal(approach_goal);

        actionlib::SimpleClientGoalState approach_status(actionlib::SimpleClientGoalState::PENDING);
        ros::Rate approach_rate(10);
        while (ros::ok()) {
          approach_status = mbfClientExePath->getState();
          if (approach_status.state_ != actionlib::SimpleClientGoalState::ACTIVE &&
              approach_status.state_ != actionlib::SimpleClientGoalState::PENDING) {
            break;
          }
          if (aborted || requested_pause_flag || skip_area || skip_path) {
            if (aborted || requested_pause_flag || skip_area ||
                currentMowingPath + 1 >= static_cast<int>(currentMowingPaths.size())) {
              mowerEnabled = false;
            }
            mbfClientExePath->cancelAllGoals();
            break;
          }
          approach_rate.sleep();
        }

        if (skip_area) {
          publishMowerEvent("AREA_SKIPPED");
          mowerEnabled = false;
          currentMowingPaths.clear();
          skip_area = false;
          return true;
        }
        if (skip_path) {
          if (currentMowingPath + 1 >= static_cast<int>(currentMowingPaths.size())) {
            mowerEnabled = false;
          }
          skip_path = false;
          currentMowingPath++;
          currentMowingPathIndex = 0;
          return false;
        }
        if (aborted || requested_pause_flag) {
          mowerEnabled = false;
          return false;
        }
        if (approach_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
          mowerEnabled = false;
          outline_approach_attempt_counter++;
          if (outline_approach_attempt_counter >= config.max_outline_approach_attempts) {
            ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) Tangent outline approach failed with status "
                             << approach_status.state_ << " after " << outline_approach_attempt_counter
                             << " attempts; aborting mowing.");
            this->abort();
            return false;
          }
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) Tangent outline approach failed with status "
                          << approach_status.state_ << "; retrying (" << outline_approach_attempt_counter << " / "
                          << config.max_outline_approach_attempts << ").");
          paused = true;
          update_actions();
          continue;
        }
        outline_approach_attempt_counter = 0;
      }

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
      outline_approach_backtrack_distance = 0.0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      bool mower_enabled_last = mowerEnabled.exchange(true);

      // Wait for mower motor to spin up to target RPM (if configured and previously disabled)
      if (!mower_enabled_last && !wait_for_mower_spinup()) {
        return false;
      }

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                              path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = mowingController;

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            publishMowerEvent("AREA_SKIPPED");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClientExePath->cancelAllGoals();
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            if (currentMowingPath + 1 >= static_cast<int>(currentMowingPaths.size())) {
              mowerEnabled = false;
            }
            mbfClientExePath->cancelAllGoals();
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              currentMowingPathIndex = exePathStartIndex + currentIndex;
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            if (ros::Time::now() - last_checkpoint > ros::Duration(30.0)) checkpoint();
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        const int finalIndex = getCurrentMowPathIndex();
        if (finalIndex != -1) {
          currentMowingPathIndex = std::max(currentMowingPathIndex, exePathStartIndex + finalIndex);
        }
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (currentMowingPathIndex >= path.path.poses.size() ||
            (path.path.poses.size() - currentMowingPathIndex) < 5)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          currentMowingPath++;
          currentMowingPathIndex = 0;
          // continue with next segment
        } else {
          // we didnt drive all points in the mow path, so we wait for GPS or execute the recovery behaviors.

          if (!requested_pause_flag) {
            if (failed_path == currentMowingPath && failed_path_index == currentMowingPathIndex) {
              failed_path_attempts++;
            } else {
              failed_path = currentMowingPath;
              failed_path_index = currentMowingPathIndex;
              failed_path_attempts = 1;
            }
            if (failed_path_attempts > config.max_mow_path_retries) {
              ROS_WARN_STREAM("MowingBehavior: (MOW) Planner failed at path index "
                              << currentMowingPathIndex << " after " << config.max_mow_path_retries
                              << " retries; advancing one pose.");
              currentMowingPathIndex++;
              failed_path = -1;
              failed_path_index = -1;
              failed_path_attempts = 0;
            } else {
              ROS_WARN_STREAM("MowingBehavior: (MOW) Planner failed at path index "
                              << currentMowingPathIndex << "; retry " << failed_path_attempts << " / "
                              << config.max_mow_path_retries << ".");
            }
            // Path following failed (not a user-requested pause/abort). ExePath, unlike the
            // MoveBase action, never invokes MBF recovery on its own, so trigger it explicitly
            // here. Mirror MBF's move_base behavior: run each configured recovery behavior in
            // order until one succeeds (or we are aborted / they all fail). No-op if MBF has
            // no recovery configured.
            if (hasGoodGPS() && !aborted) {
              mowerEnabled = false;
              std::vector<std::string> recoveryBehaviors = getConfiguredRecoveryBehaviors();
              if (recoveryBehaviors.empty()) {
                ROS_INFO_STREAM(
                    "MowingBehavior: (MOW) No recovery behavior configured - "
                    "skipping recovery.");
              } else if (!mbfClientRecovery->waitForServer(ros::Duration(1.0))) {
                ROS_WARN_STREAM(
                    "MowingBehavior: (MOW) Recovery action server unavailable - "
                    "skipping recovery.");
              } else {
                for (const auto& behavior : recoveryBehaviors) {
                  if (aborted) break;
                  mbf_msgs::RecoveryGoal recoveryGoal;
                  recoveryGoal.behavior = behavior;
                  ROS_INFO_STREAM(
                      "MowingBehavior: (MOW) Path following failed - running recovery "
                      "behavior '"
                      << behavior << "'.");
                  auto recoveryState = sendGoalAndWaitUnlessAborted(mbfClientRecovery, recoveryGoal);
                  ROS_INFO_STREAM("MowingBehavior: (MOW) Recovery behavior '" << behavior << "' finished with state "
                                                                              << recoveryState.toString());
                  if (recoveryState == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    break;
                  }
                }
              }
            }
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
            publishMowerEvent("NAVIGATION_ERROR");
            paused = true;
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
  if (shared_state->active_semiautomatic_task) {
    // We are in semiautomatic task, mark it as manually paused.
    ROS_INFO_STREAM("Manually pausing semiautomatic task");
    auto config = getConfig();
    config.manual_pause_mowing = true;
    setConfig(config);
  }
  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() const {
  return currentMowingArea;
}

std::string MowingBehavior::get_current_area_id() const {
  return currentMowingAreaId;
}

std::string MowingBehavior::get_current_area_name() const {
  return currentMowingAreaName;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    mowerEnabled = false;
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  rosbag::Bag bag;
  mower_logic::CheckPoint cp;
  cp.job_id = current_job_id;
  cp.currentMowingPath = currentMowingPath;
  cp.currentMowingArea = currentMowingArea;
  cp.currentMowingPathIndex = currentMowingPathIndex;
  cp.currentMowingPlanDigest = currentMowingPlanDigest;
  cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
  bag.open("checkpoint.bag", rosbag::bagmode::Write);
  bag.write("checkpoint", ros::Time::now(), cp);
  bag.close();
  last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
  rosbag::Bag bag;
  bool found = false;
  try {
    bag.open("checkpoint.bag");
  } catch (rosbag::BagIOException& e) {
    // Checkpoint does not exist or is corrupt, start at the very beginning
    currentMowingArea = 0;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    currentMowingAngleIncrementSum = 0;
    return false;
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
    for (rosbag::MessageInstance const m : view) {
      auto cp = m.instantiate<mower_logic::CheckPoint>();
      if (cp) {
        ROS_INFO_STREAM("Restoring checkpoint for plan ("
                        << cp->currentMowingPlanDigest << ")"
                        << " job: " << cp->job_id << " area: " << cp->currentMowingArea
                        << " path: " << cp->currentMowingPath << " index: " << cp->currentMowingPathIndex
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
        current_job_id = cp->job_id;
        currentMowingPath = cp->currentMowingPath;
        currentMowingArea = cp->currentMowingArea;
        currentMowingPathIndex = cp->currentMowingPathIndex;
        currentMowingPlanDigest = cp->currentMowingPlanDigest;
        currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
        found = true;
        break;
      }
    }
    bag.close();
  }
  return found;
}
