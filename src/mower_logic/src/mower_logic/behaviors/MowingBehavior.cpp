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
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cmath>
#include <limits>

#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();
extern bool setMowerEnabled(bool enabled);
extern bool mower_stall_latched;
extern bool mower_stall_recovery_in_progress;

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

std::string MowingBehavior::sub_state_name() {
  if (sub_state == 1) {
    return "STALL_RECOVERY";
  }
  return "";
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
  stallRecoveryFailed = false;
  paused = aborted = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
  currentMowingPaths.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  stallRecoveryFailed = false;
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
  currentAreaOutline = mapSrv.response.area.area;
  currentAreaObstacles = mapSrv.response.area.obstacles;

  // Area orientation is the same as the first point
  double angle = 0;
  auto points = mapSrv.response.area.area.points;
  if (points.size() >= 2) {
    tf2::Vector3 first(points[0].x, points[0].y, 0);
    for (auto point : points) {
      tf2::Vector3 second(point.x, point.y, 0);
      auto diff = second - first;
      if (diff.length() > 2.0) {
        // we have found a point that has a distance of > 1 m, calculate the angle
        angle = atan2(diff.y(), diff.x());
        ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
        break;
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
  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  pathSrv.request.outline_count = config.outline_count;
  pathSrv.request.outline_overlap_count = config.outline_overlap_count;
  pathSrv.request.outline = mapSrv.response.area.area;
  pathSrv.request.holes = mapSrv.response.area.obstacles;
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = config.outline_offset;
  pathSrv.request.distance = config.tool_width;
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

bool MowingBehavior::reverse_for_mower_stall_recovery(const nav_msgs::Path& path) {
  const auto cfg = getConfig();
  if (cfg.mower_stall_reverse_distance <= 0.0) {
    return true;
  }

  if (path.poses.empty()) {
    return false;
  }

  int target_index = std::min<int>(currentMowingPathIndex, static_cast<int>(path.poses.size()) - 1);
  double remaining_backtrack = cfg.mower_stall_reverse_distance;
  geometry_msgs::PoseStamped reverse_target = path.poses[target_index];

  while (target_index > 0 && remaining_backtrack > 0.0) {
    const auto& a_pose = path.poses[target_index].pose;
    const auto& b_pose = path.poses[target_index - 1].pose;
    const auto& a = a_pose.position;
    const auto& b = b_pose.position;
    const double segment_length = std::hypot(a.x - b.x, a.y - b.y);
    if (segment_length <= 1e-6) {
      --target_index;
      reverse_target = path.poses[target_index];
      continue;
    }

    if (remaining_backtrack <= segment_length) {
      const double ratio = remaining_backtrack / segment_length;
      reverse_target = path.poses[target_index];
      reverse_target.pose.position.x = a.x + (b.x - a.x) * ratio;
      reverse_target.pose.position.y = a.y + (b.y - a.y) * ratio;
      reverse_target.pose.position.z = a.z + (b.z - a.z) * ratio;
      reverse_target.pose.orientation = b_pose.orientation;
      remaining_backtrack = 0.0;
      break;
    }

    remaining_backtrack -= segment_length;
    --target_index;
    reverse_target = path.poses[target_index];
  }

  mbf_msgs::MoveBaseGoal reverse_goal;
  reverse_goal.target_pose = reverse_target;
  reverse_goal.target_pose.header.stamp = ros::Time::now();
  reverse_goal.controller = "TEBPlanner";

  ROS_INFO_STREAM("MowingBehavior: Backtracking along mowing path by about " << cfg.mower_stall_reverse_distance
                                                                             << " m to index " << target_index
                                                                             << " before mower restart attempt.");
  ros::Time reverse_started = ros::Time::now();
  ros::Rate poll_rate(10.0);
  mbfClient->sendGoal(reverse_goal);
  actionlib::SimpleClientGoalState reverse_state(actionlib::SimpleClientGoalState::PENDING);
  while (ros::ok()) {
    reverse_state = mbfClient->getState();
    if (aborted) {
      mbfClient->cancelAllGoals();
      return false;
    }
    if (reverse_state.isDone()) {
      break;
    }
    if ((ros::Time::now() - reverse_started) >= ros::Duration(cfg.mower_stall_reverse_timeout)) {
      ROS_WARN_STREAM("MowingBehavior: Reverse recovery move timed out after " << cfg.mower_stall_reverse_timeout
                                                                               << " s.");
      mbfClient->cancelAllGoals();
      return false;
    }
    poll_rate.sleep();
  }

  if (reverse_state.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN_STREAM("MowingBehavior: Reverse recovery move finished with status " << reverse_state.state_
                                                                                  << " before mower restart.");
    return false;
  }

  return true;
}

bool MowingBehavior::handle_mower_stall_pause() {
  const auto cfg = getConfig();
  if (cfg.mower_stall_max_restart_attempts <= 0) {
    return false;
  }

  int currentIndex = getCurrentMowPathIndex();
  if (currentIndex >= 0) {
    currentMowingPathIndex = currentIndex;
  }
  mowerEnabled = false;
  mbfClientExePath->cancelAllGoals();
  mower_stall_recovery_in_progress = true;

  for (int attempt = 1; attempt <= cfg.mower_stall_max_restart_attempts && !aborted; ++attempt) {
    ROS_WARN_STREAM("MowingBehavior: Stall recovery attempt " << attempt << " / "
                                                              << cfg.mower_stall_max_restart_attempts);
    setMowerEnabled(false);
    ros::Duration(cfg.mower_stall_restart_delay).sleep();
    if (aborted) {
      mower_stall_recovery_in_progress = false;
      return false;
    }

    if (!reverse_for_mower_stall_recovery(currentMowingPaths[currentMowingPath].path)) {
      if (aborted) {
        mower_stall_recovery_in_progress = false;
        return false;
      }
    }

    setMowerEnabled(true);
    ros::Time restart_started = ros::Time::now();
    ros::Rate poll_rate(5.0);
    while (ros::ok() && !aborted &&
           (ros::Time::now() - restart_started) < ros::Duration(cfg.mower_stall_restart_timeout)) {
      auto status = getStatus();
      if (status.mow_enabled && status.mower_motor_rpm >= cfg.mower_restart_rpm) {
        ROS_INFO_STREAM("MowingBehavior: Stall recovery succeeded at eRPM " << status.mower_motor_rpm);
        mower_stall_latched = false;
        mower_stall_recovery_in_progress = false;
        stallRecoveryFailed = false;
        this->requestContinue(pauseType::PAUSE_MOW_STALL);
        return true;
      }
      poll_rate.sleep();
    }
  }

  mower_stall_recovery_in_progress = false;
  if (currentMowingPath < currentMowingPaths.size()) {
    const double skip_distance = std::max(cfg.mower_stall_skip_distance, 0.0);
    if (skip_distance > 0.0) {
      ROS_ERROR_STREAM("MowingBehavior: Mower stall recovery failed; skipping ahead " << skip_distance
                                                                                      << " m on the mowing path.");
      advance_resume_index_by_distance(currentMowingPaths[currentMowingPath].path, skip_distance);
    } else {
      ROS_ERROR_STREAM("MowingBehavior: Mower stall recovery failed; skipping ahead to the next mowing pose.");
      advance_resume_index_by_distance(currentMowingPaths[currentMowingPath].path, 0.001);
    }
    mower_stall_latched = false;
    stallRecoveryFailed = false;
    this->requestContinue(pauseType::PAUSE_MOW_STALL);
    return true;
  }

  ROS_ERROR_STREAM("MowingBehavior: Mower stall recovery failed with no active mowing path; skipping current path.");
  mower_stall_latched = false;
  stallRecoveryFailed = false;
  currentMowingPath++;
  currentMowingPathIndex = 0;
  this->requestContinue(pauseType::PAUSE_MOW_STALL);
  return true;
}

void MowingBehavior::advance_resume_index_by_distance(const nav_msgs::Path& path, double distance) {
  if (currentMowingPathIndex >= path.poses.size()) {
    return;
  }

  size_t resume_index = static_cast<size_t>(currentMowingPathIndex);
  double accumulated_distance = 0.0;
  while (resume_index + 1 < path.poses.size() && accumulated_distance < distance) {
    const auto& a = path.poses[resume_index].pose.position;
    const auto& b = path.poses[resume_index + 1].pose.position;
    accumulated_distance += std::hypot(b.x - a.x, b.y - a.y);
    ++resume_index;
  }

  if (static_cast<int>(resume_index) != currentMowingPathIndex) {
    ROS_WARN_STREAM("MowingBehavior: Advancing resume index from " << currentMowingPathIndex << " to " << resume_index
                                                                   << " after mower stall recovery failure.");
    currentMowingPathIndex = static_cast<int>(resume_index);
  }
}

void MowingBehavior::update_resume_index_for_current_pose(const nav_msgs::Path& path) {
  if (currentMowingPathIndex >= path.poses.size()) {
    return;
  }

  const auto cfg = getConfig();
  const auto current_pose = getPose();
  const double px = current_pose.pose.pose.position.x;
  const double py = current_pose.pose.pose.position.y;

  size_t best_segment_start = static_cast<size_t>(currentMowingPathIndex);
  double best_segment_t = 0.0;
  double best_distance_sq = std::numeric_limits<double>::max();
  double distance_along_path = 0.0;
  double best_progress_distance = 0.0;

  for (size_t i = static_cast<size_t>(currentMowingPathIndex); i + 1 < path.poses.size(); ++i) {
    const auto& a = path.poses[i].pose.position;
    const auto& b = path.poses[i + 1].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_length_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (segment_length_sq > 1e-9) {
      t = ((px - a.x) * dx + (py - a.y) * dy) / segment_length_sq;
      t = std::max(0.0, std::min(1.0, t));
    }

    const double proj_x = a.x + dx * t;
    const double proj_y = a.y + dy * t;
    const double dist_sq = (px - proj_x) * (px - proj_x) + (py - proj_y) * (py - proj_y);
    if (dist_sq < best_distance_sq) {
      best_distance_sq = dist_sq;
      best_segment_start = i;
      best_segment_t = t;
      best_progress_distance = distance_along_path + std::sqrt(segment_length_sq) * t;
    }

    distance_along_path += std::sqrt(segment_length_sq);
  }

  const double target_progress_distance = best_progress_distance + cfg.mower_resume_lookahead;
  size_t resume_index = best_segment_start;
  double accumulated_distance = 0.0;
  for (size_t i = static_cast<size_t>(currentMowingPathIndex); i + 1 < path.poses.size(); ++i) {
    const auto& a = path.poses[i].pose.position;
    const auto& b = path.poses[i + 1].pose.position;
    const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
    if (accumulated_distance + segment_length >= target_progress_distance) {
      resume_index = i + 1;
      break;
    }
    accumulated_distance += segment_length;
    resume_index = i + 1;
  }

  if (resume_index < path.poses.size() && static_cast<int>(resume_index) != currentMowingPathIndex) {
    ROS_INFO_STREAM("MowingBehavior: Adjusting resume index from " << currentMowingPathIndex << " to " << resume_index
                                                                   << " based on current pose projection.");
    currentMowingPathIndex = static_cast<int>(resume_index);
  }
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

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  ros::Time paused_time(0.0);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    auto& path = currentMowingPaths[currentMowingPath];
    const uint8_t non_stall_pause_flags = requested_pause_flag & ~pauseType::PAUSE_MOW_STALL;

    if ((requested_pause_flag & pauseType::PAUSE_MOW_STALL) && !non_stall_pause_flags && !stallRecoveryFailed) {
      sub_state = 1;
      mowerEnabled = false;
      ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: MOWING (STALL_RECOVERY)");
      if (!handle_mower_stall_pause()) {
        paused = true;
        update_actions();
        continue;
      }
      sub_state = 0;
    }
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      sub_state = 0;
      paused = true;
      mowerEnabled = false;
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
          if (requested_pause_flag & (pauseType::PAUSE_MANUAL | pauseType::PAUSE_MOW_STALL)) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MOW_STALL) {
          pause_reason += "waiting after mower stall";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
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
      update_resume_index_for_current_pose(path.path);
      paused = false;
      stallRecoveryFailed = false;
      update_actions();
    }

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
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      const auto current_pose = getPose();
      const auto& first_point_target = path.path.poses[currentMowingPathIndex];
      double dx = first_point_target.pose.position.x - current_pose.pose.pose.position.x;
      double dy = first_point_target.pose.position.y - current_pose.pose.pose.position.y;
      bool skip_first_point_move_base = std::hypot(dx, dy) <= config.first_point_skip_distance;

      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::SUCCEEDED);
      ros::Rate r(10);

      if (!skip_first_point_move_base) {
        mbf_msgs::MoveBaseGoal moveBaseGoal;
        moveBaseGoal.target_pose = first_point_target;
        moveBaseGoal.controller = "TEBPlanner";
        mbfClient->sendGoal(moveBaseGoal);

        // wait for move-base approach to finish
        while (ros::ok()) {
          current_status = mbfClient->getState();
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
              current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
            // path is being executed, everything seems fine.
            // check if we should pause or abort mowing
            if (skip_area) {
              ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
              // remove all paths in current area and return true
              mowerEnabled = false;
              mbfClient->cancelAllGoals();
              currentMowingPaths.clear();
              skip_area = false;
              return true;
            }
            if (skip_path) {
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
                            << current_status.state_ << " from MBF/TEBPlanner -> Stopping path execution.");
            // we're done, break out of the loop
            break;
          }
          r.sleep();
        }
      } else {
        ROS_INFO_STREAM(
            "MowingBehavior: (FIRST POINT) Skipping separate first-point MoveBase stage because target is already "
            "within "
            << config.first_point_skip_distance << " m.");
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
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt "
                            << first_point_trim_counter << " / " << config.max_first_point_trim_attempts
                            << " Trimming first point off the beginning of the mow path.");
            currentMowingPathIndex++;
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

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      mowerEnabled = true;

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses.insert(exePath.poses.end(), path.path.poses.begin() + currentMowingPathIndex,
                           path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

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
            // remove all paths in current area and return true
            mowerEnabled = false;
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
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
          // we didnt drive all points in the mow path, so we go into pause mode
          // TODO: we should figure out the likely reason for our failure to complete the path
          // if GPS -> PAUSE
          // if something else -> Recovery Behaviour ?

          // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          if (!requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
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
  return sub_state;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return currentMowingArea;
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
                        << " area: " << cp->currentMowingArea << " path: " << cp->currentMowingPath
                        << " index: " << cp->currentMowingPathIndex
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
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
