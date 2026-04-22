
#include <ftc_local_planner/ftc_planner.h>

#include <pluginlib/class_list_macros.h>
#include "mbf_msgs/ExePathAction.h"

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, mbf_costmap_core::CostmapController)

#define RET_SUCCESS 0
#define RET_COLLISION 104
#define RET_BLOCKED 109

namespace ftc_local_planner
{
    void FTCPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        progress_server = private_nh.advertiseService(
            "planner_get_progress", &FTCPlanner::getProgress, this);

        global_point_pub = private_nh.advertise<geometry_msgs::PoseStamped>("global_point", 1);
        global_plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1, true);
        obstacle_marker_pub = private_nh.advertise<visualization_msgs::Marker>("costmap_marker", 10);

        costmap = costmap_ros;
        costmap_map_ = costmap->getCostmap();
        tf_buffer = tf;

        // Parameter for dynamic reconfigure
        reconfig_server = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this,
                                                                                     _1, _2);
        reconfig_server->setCallback(cb);

        // PID Debugging topic
        if (config.debug_pid)
        {
            pubPid = private_nh.advertise<ftc_local_planner::PID>("debug_pid", 1, true);
            pubCp = private_nh.advertise<ftc_local_planner::CP>("debug_cp", 1, true);
        }

        // Recovery behavior initialization
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));

        status_sub = private_nh.subscribe<mower_msgs::Status>("/ll/mower_status", 0, &FTCPlanner::statusReceived, this, ros::TransportHints().tcpNoDelay(true));

        ROS_INFO("FTCLocalPlannerROS: Version 2 Init.");
    }

    void FTCPlanner::statusReceived(const mower_msgs::Status::ConstPtr &msg)
    {
        last_status = *msg;
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &c, uint32_t level)
    {
        if (c.restore_defaults)
        {
            reconfig_server->getConfigDefault(c);
            c.restore_defaults = false;
        }
        config = c;

        // just to be sure
        //current_movement_speed = config.speed_slow;

        // set recovery behavior
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        set_planner_state(PRE_ROTATE);
        is_crashed = false;

        global_plan = plan;
        current_index = 0;
        current_progress = 0.0;

        last_time = ros::Time::now();
        current_movement_speed = 0;
        speed_limit = config.max_cmd_vel_speed;

        lat_error = 0.0;
        lon_error = 0.0;
        angle_error = 0.0;
        i_lon_error = 0.0;
        i_lat_error = 0.0;
        i_angle_error = 0.0;
        lin_speed = 0.0;
        last_rpm_deficit_ = 0.0;

        nav_msgs::Path path;

        if (global_plan.size() > 2)
        {
            // duplicate last point
            global_plan.push_back(global_plan.back());
            // give second from last point last oriantation as the point before that
            global_plan[global_plan.size() - 2].pose.orientation = global_plan[global_plan.size() - 3].pose.orientation;
            path.header = plan.front().header;
            path.poses = plan;
        }
        else
        {
            ROS_WARN_STREAM("FTCLocalPlannerROS: Global plan was too short. Need a minimum of 3 poses - Cancelling.");
            set_planner_state(FINISHED);
            return true;
        }
        global_plan_pub.publish(path);

        ROS_INFO_STREAM("FTCLocalPlannerROS: Got new global plan with " << plan.size() << " points.");

        return true;
    }

    FTCPlanner::~FTCPlanner()
    {
        if (reconfig_server != nullptr)
        {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    double FTCPlanner::distanceLookahead()
    {
        if (global_plan.size() < 2)
        {
            return 0;
        }
        Eigen::Quaternion<double> current_rot(current_control_point.linear());
        double lookahead_distance = 0.0;
        Eigen::Affine3d last_straight_point = current_control_point;
        Eigen::Affine3d current_point;
        for (uint32_t i = current_index + 1; i < global_plan.size(); i++)
        {
            tf2::fromMsg(global_plan[i].pose, current_point);

            // check, if direction is the same. if so, we add the distance
            Eigen::Quaternion<double> rot2(current_point.linear());

            if (lookahead_distance > config.speed_fast_threshold ||
                abs(rot2.angularDistance(current_rot)) > config.speed_fast_threshold_angle * (M_PI / 180.0))
            {
                break;
            }

            lookahead_distance += (current_point.translation() - last_straight_point.translation()).norm();
            last_straight_point = current_point;

        }

        return lookahead_distance;
    }

    uint32_t FTCPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                 const geometry_msgs::TwistStamped &velocity,
                                                 geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {

        ros::Time now = ros::Time::now();
        double dt = now.toSec() - last_time.toSec();
        last_time = now;

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        if (current_state == FINISHED)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_SUCCESS;
        }

        // Control point speed: limited by mow motor current (gradual slowdown as load increases)
        double mow_current_over = std::max(0.0, (double)(last_status.mower_esc_current - config.max_mow_motor_current));
        mow_speed_limit_ = std::max(config.speed_limit_min,
            config.max_cmd_vel_speed - mow_current_over * config.kp_mow_current_lim);

        // Linear speed: PD on mow motor RPM (fast slowdown when motor stalls)
        if (last_status.mow_enabled) {
            double rpm_deficit = std::max(0.0, config.min_mow_motor_rpm - (double)last_status.mower_motor_rpm);
            double d_rpm_deficit = (rpm_deficit - last_rpm_deficit_) / dt;
            last_rpm_deficit_ = rpm_deficit;
            mow_rpm_speed_limit_ = std::min(config.max_cmd_vel_speed, std::max(config.speed_limit_min,
                config.max_cmd_vel_speed - rpm_deficit * config.kp_mow_rpm_lim - d_rpm_deficit * config.kd_mow_rpm_lim));

            // Ensure control point can't outrun the RPM-based lin_speed limit
            mow_speed_limit_ = std::min(mow_speed_limit_, mow_rpm_speed_limit_);
        } else {
            mow_rpm_speed_limit_ = config.max_cmd_vel_speed;
            last_rpm_deficit_ = 0.0;
        }

        // We're not crashed and not finished.
        // First, we update the control point if needed. This is needed since we need the local_control_point to calculate the next state.
        update_control_point(dt);
        // Then, update the planner state.
        update_planner_state();

        if (checkCollision(config.obstacle_lookahead))
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            is_crashed = true;
            return RET_BLOCKED;
        }

        // Finally, we calculate the velocity commands.
        calculate_velocity_commands(dt, cmd_vel);

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        return RET_SUCCESS;
    }


    bool FTCPlanner::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        return current_state == FINISHED && !is_crashed;
    }

    bool FTCPlanner::cancel()
    {
        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC planner was cancelled.");
        set_planner_state(FINISHED);
        return true;
    }

    void FTCPlanner::set_planner_state(PlannerState s) {
        auto last_state = current_state;
        current_state_ = s;
        if( last_state != current_state) {
            ROS_INFO_STREAM("FTCLocalPlannerROS: Switching to state " << current_state);
            state_entered_time = ros::Time::now();
            // Reset oscillation detection
            failure_detector_.clear();
            failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));
        }
    };

    void FTCPlanner::update_planner_state()
    {
        switch (current_state)
        {
        case PRE_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Error reaching goal. config.goal_timeout (" << config.goal_timeout << ") reached - Timeout in PRE_ROTATE phase.");
                is_crashed = true;
                set_planner_state(FINISHED);
                return;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: PRE_ROTATE finished. Starting following");
                set_planner_state(FOLLOWING);
                return;
            }
        }
        break;
        case FOLLOWING:
        {
            double distance = local_control_point.translation().norm();
            // check for crash
            if (distance > config.max_follow_distance)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Robot is far away from global plan. distance (" << distance << ") > config.max_follow_distance (" << config.max_follow_distance << ") It probably has crashed.");
                is_crashed = true;
                set_planner_state(FINISHED);
                return;
            }

            // check if we're done following
            if (current_index == global_plan.size() - 2)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: switching planner to position mode");
                set_planner_state(WAITING_FOR_GOAL_APPROACH);
                return;
            }
        }
        break;
        case WAITING_FOR_GOAL_APPROACH:
        {
            double distance = local_control_point.translation().norm();
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal position. config.goal_timeout (" << config.goal_timeout << ") reached - Attempting final rotation anyways.");
                set_planner_state(POST_ROTATE);
                return;
            }
            if (distance < config.max_goal_distance_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: Reached goal position.");
                set_planner_state(POST_ROTATE);
                return;
            }
        }
        break;
        case POST_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout && abs(angle_error) * (180.0 / M_PI) > config.max_goal_angle_error)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal rotation. config.goal_timeout (" << config.goal_timeout << ") reached");
                set_planner_state(FINISHED);
                return;
            }
            if (abs(angle_error) * (180.0 / M_PI) <= config.target_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: POST_ROTATE finished.");
                set_planner_state(FINISHED);
                return;
            }
        }
        break;
        case FINISHED:
        {
            // Nothing to do here
        }
        case INIT:
        {
            // Nothing to do here
        }
        break;
        }
    }

    double FTCPlanner::velocityLookahead()
    {
        if (global_plan.size() < 2)
        {
            return 0;
        }

        //work out how many points look ahead we need to decelerate to 0 from current speed
        double decelDist = (current_movement_speed * current_movement_speed) / (2.0 * config.acceleration);
        double total_dist = 0.0;
        std::vector<double> distances;
        std::vector<double> rotations;
        Eigen::Affine3d last_point = current_control_point;
        Eigen::Quaternion<double> last_rot(current_control_point.linear());
        uint32_t i = 0;
        double dist = 0;
        for (i = current_index + 1; i < global_plan.size(); i++)
        {
            Eigen::Affine3d next_point;
            tf2::fromMsg(global_plan[i].pose, next_point);
            dist += abs((next_point.translation() - last_point.translation()).norm());
            if (dist < config.velocity_lookahead_min_step) {
                continue;
            }
            distances.push_back(dist);
            Eigen::Quaternion<double> next_rot(next_point.linear());
            rotations.push_back(abs(next_rot.angularDistance(last_rot)));
            total_dist += dist;
            dist = 0;
            last_point = next_point;
            last_rot = next_rot;
            if(total_dist >= decelDist)
                break;
        }

        // Back-off if struggling to keep up
        double max_speed = speed_limit;
        if(i >= global_plan.size())
        {
            max_speed = 0.0; //if we are approaching end of the path finish with zero speed
        }
        if(distances.empty())
        {
            return 0;
        }
        else{
            //now go back through the points to calculate max permissible speed

            for(int32_t i = distances.size()-1;i>=0;i--)
            {
                //calculate max speed to allow time for rotations
                double angle = rotations[i] * (180.0 / M_PI);
                if (angle < config.velocity_lookahead_min_angle) {
                    angle = 0.0;
                }
                double time_to_rotate = angle / config.speed_angular;
                double speed = speed_limit;
                if(time_to_rotate > 0.0)
                    speed = config.velocity_lookahead_scale * distances[i]/time_to_rotate;

                //calculate max speed with acceleration from previous step (actually decel but going backwards)
                max_speed = sqrt((max_speed * max_speed) + (2 * config.acceleration * distances[i]));
                if(max_speed > speed_limit)
                    max_speed = speed_limit;
                if(speed < max_speed)
                    max_speed = speed;
            }
        }

        return max_speed;
    }

    void FTCPlanner::update_control_point(double dt)
    {

        switch (current_state)
        {
        case PRE_ROTATE:
            tf2::fromMsg(global_plan[0].pose, current_control_point);
            break;
        case FOLLOWING:
        {
            double speed = 0.0;
            double straight_dist = distanceLookahead();
            if(config.speed_slow > 0.0)
            {
                // Normal planner operation
                if (straight_dist >= config.speed_fast_threshold)
                {
                    speed = config.speed_fast;
                }
                else
                {
                    speed = config.speed_slow;
                }
            }
            else
            {
                speed = velocityLookahead();
                if(speed < 0.01) speed = 0.01;
            }

            if (speed > current_movement_speed)
            {
                // accelerate
                current_movement_speed += dt * config.acceleration;
                if (current_movement_speed > speed)
                    current_movement_speed = speed;
            }
            else if (speed < current_movement_speed)
            {
                // decelerate
                current_movement_speed -= dt * config.acceleration;
                if (current_movement_speed < speed)
                    current_movement_speed = speed;
            }

            // Hard-cap control point speed so it can't run ahead of the
            // robot when mow current limiting kicks in.
            if (current_movement_speed > mow_speed_limit_)
                current_movement_speed = mow_speed_limit_;

            double distance_to_move = dt * current_movement_speed;
            double angle_to_move = dt * config.speed_angular * (M_PI / 180.0);

            Eigen::Affine3d nextPose, currentPose;
            while (angle_to_move > 0 && distance_to_move > 0 && current_index < global_plan.size() - 2)
            {

                tf2::fromMsg(global_plan[current_index].pose, currentPose);
                tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);

                double pose_distance = (nextPose.translation() - currentPose.translation()).norm();

                Eigen::Quaternion<double> current_rot(currentPose.linear());
                Eigen::Quaternion<double> next_rot(nextPose.linear());

                double pose_distance_angular = current_rot.angularDistance(next_rot);

                if (pose_distance <= 0.0 && std::abs(pose_distance_angular) <= 0.00001)
                {
                    ROS_WARN_STREAM("FTCLocalPlannerROS: Skipping duplicate point in global plan.");
                    current_index++;
                    continue;
                }

                double remaining_distance_to_next_pose = pose_distance * (1.0 - current_progress);
                double remaining_angular_distance_to_next_pose = pose_distance_angular * (1.0 - current_progress);

                if (remaining_distance_to_next_pose < distance_to_move &&
                    remaining_angular_distance_to_next_pose < angle_to_move)
                {
                    // we need to move further than the remaining distance_to_move. Skip to the next point and decrease distance_to_move.
                    current_progress = 0.0;
                    current_index++;
                    distance_to_move -= remaining_distance_to_next_pose;
                    angle_to_move -= remaining_angular_distance_to_next_pose;
                }
                else
                {
                    // we cannot reach the next point yet, so we update the percentage
                    double current_progress_distance =
                        (pose_distance * current_progress + distance_to_move) / pose_distance;
                    double current_progress_angle =
                        (pose_distance_angular * current_progress + angle_to_move) / pose_distance_angular;

                    current_progress = fmin(current_progress_angle, current_progress_distance);
                    if (current_progress > 1.0)
                    {
                        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC PLANNER: Progress > 1.0 dist " << current_progress_distance << "ang " << current_progress_angle);
                        //                    current_progress = 1.0;
                    }
                    distance_to_move = 0;
                    angle_to_move = 0;
                }
            }

            tf2::fromMsg(global_plan[current_index].pose, currentPose);
            tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);
            // interpolate between points
            Eigen::Quaternion<double> rot1(currentPose.linear());
            Eigen::Quaternion<double> rot2(nextPose.linear());

            Eigen::Vector3d trans1 = currentPose.translation();
            Eigen::Vector3d trans2 = nextPose.translation();

            Eigen::Affine3d result;
            result.translation() = (1.0 - current_progress) * trans1 + current_progress * trans2;
            result.linear() = rot1.slerp(current_progress, rot2).toRotationMatrix();

            current_control_point = result;
        }
        break;
        case POST_ROTATE:
            tf2::fromMsg(global_plan[global_plan.size() - 1].pose, current_control_point);
            break;
        case WAITING_FOR_GOAL_APPROACH:
            break;
        case FINISHED:
        case INIT:
            break;
        }

        {
            geometry_msgs::PoseStamped viz;
            viz.header = global_plan[current_index].header;
            viz.pose = tf2::toMsg(current_control_point);
            global_point_pub.publish(viz);
        }

        if (config.debug_pid)
        {
            ftc_local_planner::CP debugCpMsg;

            debugCpMsg.stamp = ros::Time::now();
            debugCpMsg.movement_speed = current_movement_speed;
            debugCpMsg.progress = current_progress;
            pubCp.publish(debugCpMsg);
        }

        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(1.0));
        tf2::doTransform(current_control_point, local_control_point, map_to_base);

        lat_error = local_control_point.translation().y();
        lon_error = local_control_point.translation().x();
        lon_error -= config.follow_distance;
        angle_error = local_control_point.rotation().eulerAngles(0, 1, 2).z();

    }

    void FTCPlanner::calculate_velocity_commands(double dt, geometry_msgs::TwistStamped &cmd_vel)
    {
        // check, if we're completely done
        if (current_state == FINISHED || is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return;
        }

        if (lin_speed < 0)
        {
            // Going backwards, flip error
            lat_error *= -1.0;
            // TODO: Need this too? Don't think so: lon_error *= -1.0;
        }

        auto real_angle_error = angle_error;
        if (config.ang_error_scale) {
            angle_error *= config.ang_error_scale_factor / (config.ang_error_scale_factor + abs(lin_speed));
        }

        i_lon_error += lon_error * dt;
        i_lat_error += lat_error * dt;
        i_angle_error += angle_error * dt;

        if (i_lon_error > config.ki_lon_max)
        {
            i_lon_error = config.ki_lon_max;
        }
        else if (i_lon_error < -config.ki_lon_max)
        {
            i_lon_error = -config.ki_lon_max;
        }
        if (i_lat_error > config.ki_lat_max)
        {
            i_lat_error = config.ki_lat_max;
        }
        else if (i_lat_error < -config.ki_lat_max)
        {
            i_lat_error = -config.ki_lat_max;
        }
        if (current_state == FOLLOWING) {
            if (i_angle_error > config.ki_ang_max)
            {
                i_angle_error = config.ki_ang_max;
            }
            else if (i_angle_error < -config.ki_ang_max)
            {
                i_angle_error = -config.ki_ang_max;
            }
        }
        else {
            if (i_angle_error > config.ki_ang_max_rotate)
            {
                i_angle_error = config.ki_ang_max_rotate;
            }
            else if (i_angle_error < -config.ki_ang_max_rotate)
            {
                i_angle_error = -config.ki_ang_max_rotate;
            }
        }

        double d_input_lon = local_control_point.translation().x() - last_local_control_point.translation().x();
        double d_input_lat = local_control_point.translation().y() - last_local_control_point.translation().y();
        double d_input_angular = local_control_point.rotation().eulerAngles(0, 1, 2).z() - last_local_control_point.rotation().eulerAngles(0, 1, 2).z();
        last_local_control_point = local_control_point;

        double d_lat_input = d_input_lat / dt;
        double d_lon_input = d_input_lon / dt;
        double d_angle_input = d_input_angular / dt;

        double d_lat = (lat_error - last_lat_error) / dt;
        double d_lon = (lon_error - last_lon_error) / dt;
        double d_angle = (angle_error - last_angle_error) / dt;

        last_lat_error = lat_error;
        last_lon_error = lon_error;
        last_angle_error = angle_error;
        double mow_current_error = std::max(0.0, last_status.mower_esc_current - config.max_mow_motor_current);
        speed_limit = std::min(config.max_cmd_vel_speed, std::max(
            config.speed_limit_min,
            std::min(
                config.max_cmd_vel_speed - (lon_error * config.kp_lim + i_lon_error * config.ki_lim + d_lon * config.kd_lim),
                config.max_cmd_vel_speed - (mow_current_error * config.kp_mow_current_lim)
            )
        ));

        double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang + d_angle_input * config.kd_ang_input;
        if (current_state == PRE_ROTATE || current_state == POST_ROTATE)
            ang_speed = angle_error * config.kp_ang_rotate + i_angle_error * config.ki_ang_rotate + d_angle * config.kd_ang_rotate;

        // Only apply lat PID while FOLLOWING
        if ((current_state == FOLLOWING) || (current_state == WAITING_FOR_GOAL_APPROACH)) {
            // reduce angular error gain if there is a large lateral error
            double ang_gain_factor = 1.0;
            if(config.lateral_priority_distance > 0.01)
            {
                if(abs(lat_error) >= config.lateral_priority_distance)
                    ang_gain_factor = 0;
                else
                    ang_gain_factor = (config.lateral_priority_distance - abs(lat_error))/config.lateral_priority_distance;

                if(ang_gain_factor < 0.1)
                    ang_gain_factor = 0.1;
            }
            ang_speed *= ang_gain_factor;
            ang_speed += lat_error * config.kp_lat + i_lat_error * config.ki_lat + d_lat * config.kd_lat + d_lat_input * config.kd_lat_input;
        }

        if (ang_speed > config.max_cmd_vel_ang)
        {
            ang_speed = config.max_cmd_vel_ang;
        }
        else if (ang_speed < -config.max_cmd_vel_ang)
        {
            ang_speed = -config.max_cmd_vel_ang;
        }

        if (current_state == PRE_ROTATE || current_state == POST_ROTATE) {
            // check if robot oscillates
            bool is_oscillating = checkOscillation(cmd_vel);
            if (is_oscillating)
            {
                ang_speed = 0;
            }
        }
        cmd_vel.twist.angular.z = ang_speed;

        // Only allow linear movement while FOLLOWING or WAITING_FOR_GOAL_APPROACH
        if (current_state == FOLLOWING || current_state == WAITING_FOR_GOAL_APPROACH)
        {
            if (config.lon_pid_speed_delta) {
                // TODO: Maybe?
                // double d_lin_speed = (current_movement_speed - lin_speed) + lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon;
                double d_lin_speed = lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon + d_lon_input * config.kd_lon_input;
                double max_acceleration = dt * config.max_cmd_vel_acceleration;
                if (d_lin_speed >= 0) {
                    if (d_lin_speed > max_acceleration) {
                        d_lin_speed = max_acceleration;
                    }
                }
                // else if (d_lin_speed < -max_acceleration) {
                //     d_lin_speed = -max_acceleration;
                // }
                lin_speed += d_lin_speed;
            }
            else {
                lin_speed = lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon + d_lon_input * config.kd_lon_input;
            }
            if (lin_speed < 0 && config.forward_only)
            {
                lin_speed = 0;
            }
            else
            {
                if (lin_speed > mow_rpm_speed_limit_)
                {
                    lin_speed = mow_rpm_speed_limit_;
                }
                else if (lin_speed < -mow_rpm_speed_limit_)
                {
                    lin_speed = -mow_rpm_speed_limit_;
                }
            }
            cmd_vel.twist.linear.x = lin_speed;
        }
        else
        {
            cmd_vel.twist.linear.x = 0.0;
        }

        if (config.debug_pid)
        {
            ftc_local_planner::PID debugPidMsg;

            debugPidMsg.stamp = ros::Time::now();

            // proportional
            debugPidMsg.kp_lat_set = lat_error * config.kp_lat;
            debugPidMsg.kp_lon_set = lon_error * config.kp_lon;
            debugPidMsg.kp_lim_set = lon_error * config.kp_lim;
            debugPidMsg.kp_ang_set = angle_error * config.kp_ang;

            // integral
            debugPidMsg.ki_lat_set = i_lat_error * config.ki_lat;
            debugPidMsg.ki_lon_set = i_lon_error * config.ki_lon;
            debugPidMsg.ki_lim_set = i_lon_error * config.ki_lim;
            debugPidMsg.ki_ang_set = i_angle_error * config.ki_ang;

            // diff
            debugPidMsg.kd_lat_set = d_lat * config.kd_lat;
            debugPidMsg.kd_lon_set = d_lon * config.kd_lon;
            debugPidMsg.kd_lim_set = d_lon * config.kd_lim;
            debugPidMsg.kd_ang_set = d_angle * config.kd_ang;

            // diff input
            debugPidMsg.kd_lat_input_set = d_lat_input * config.kd_lat_input;
            debugPidMsg.kd_lon_input_set = d_lon_input * config.kd_lon_input;
            debugPidMsg.kd_ang_input_set = d_angle_input * config.kd_ang_input;

            // errors
            debugPidMsg.lon_err = lon_error;
            debugPidMsg.lat_err = lat_error;
            debugPidMsg.ang_err = angle_error;
            debugPidMsg.real_ang_err = real_angle_error;

            // speeds
            debugPidMsg.ang_speed = cmd_vel.twist.angular.z;
            debugPidMsg.lin_speed = cmd_vel.twist.linear.x;
            debugPidMsg.speed_limit = speed_limit;

            pubPid.publish(debugPidMsg);
        }
    }

    bool FTCPlanner::getProgress(PlannerGetProgressRequest &req, PlannerGetProgressResponse &res)
    {
        res.index = current_index;
        return true;
    }

    bool FTCPlanner::checkCollision(int max_points)
    {
        unsigned int x;
        unsigned int y;

        std::vector<geometry_msgs::Point> footprint;
        visualization_msgs::Marker obstacle_marker;

        if (!config.check_obstacles)
        {
            return false;
        }
        // maximal costs
        unsigned char previous_cost = 255;
        // ensure look ahead not out of plan
        if (global_plan.size() < max_points)
        {
            max_points = global_plan.size();
        }

        // calculate cost of footprint at robots actual pose
        if (config.obstacle_footprint)
        {
        costmap->getOrientedFootprint(footprint);
        for (int i = 0; i < footprint.size(); i++)
        {
            // check cost of each point of footprint
            if (costmap_map_->worldToMap(footprint[i].x, footprint[i].y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (costs >= costmap_2d::LETHAL_OBSTACLE)
                {
                    ROS_WARN("FTCLocalPlannerROS: Possible collision of footprint at actual pose. Stop local planner.");
                    return true;
                }
            }
        }
        }

        for (int i = 0; i < max_points; i++)
        {
            geometry_msgs::PoseStamped x_pose;
            int index = current_index + i;
            if (index > global_plan.size())
            {
                index = global_plan.size();
            }
            x_pose = global_plan[index];

            if (costmap_map_->worldToMap(x_pose.pose.position.x, x_pose.pose.position.y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (config.debug_obstacle)
                {
                    debugObstacle(obstacle_marker, x, y, costs, max_points);
                }
                // Near at obstacel
                if (costs > 0)
                {
                    // Possible collision
                    if (costs > 127 && costs > previous_cost)
                    {
                        ROS_WARN("FTCLocalPlannerROS: Possible collision. Stop local planner.");
                        return true;
                    }
                }
                previous_cost = costs;
            }
        }
        return false;
    }

    bool FTCPlanner::checkOscillation(geometry_msgs::TwistStamped &cmd_vel)
    {
        bool oscillating = false;
        // detect and resolve oscillations
        if (config.oscillation_recovery)
        {
            // oscillating = true;
            double max_vel_theta = config.max_cmd_vel_ang;
            double max_vel_current = config.max_cmd_vel_speed;

            failure_detector_.update(cmd_vel, config.max_cmd_vel_speed, config.max_cmd_vel_speed, max_vel_theta,
                                     config.oscillation_v_eps, config.oscillation_omega_eps);

            oscillating = failure_detector_.isOscillating();

            if (oscillating) // we are currently oscillating
            {
                if (!oscillation_detected_) // do we already know that robot oscillates?
                {
                    time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                    oscillation_detected_ = true;
                }
                // calculate duration of actual oscillation
                bool oscillation_duration_timeout = !((ros::Time::now() - time_last_oscillation_).toSec() < config.oscillation_recovery_min_duration); // check how long we oscillate
                if (oscillation_duration_timeout)
                {
                    if (!oscillation_warning_) // ensure to send warning just once instead of spamming around
                    {
                        ROS_WARN("FTCLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
                        oscillation_warning_ = true;
                    }
                    return true;
                }
                return false; // oscillating but timeout not reached
            }
            else
            {
                // not oscillating
                time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                oscillation_detected_ = false;
                oscillation_warning_ = false;
                return false;
            }
        }
        return false; // no check for oscillation
    }

    void FTCPlanner::debugObstacle(visualization_msgs::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs)
    {
        if (obstacle_points.points.empty())
        {
            obstacle_points.header.frame_id = costmap->getGlobalFrameID();
            obstacle_points.header.stamp = ros::Time::now();
            obstacle_points.action = visualization_msgs::Marker::ADD;
            obstacle_points.pose.orientation.w = 1.0;
            obstacle_points.type = visualization_msgs::Marker::POINTS;
            obstacle_points.scale.x = 0.2;
            obstacle_points.scale.y = 0.2;
        }
        obstacle_points.id = obstacle_points.points.size() + 1;

        if (cost < 127)
        {
            obstacle_points.color.g = 1.0f;
        }

        if (cost >= 127 && cost < 255)
        {
            obstacle_points.color.r = 1.0f;
        }
        obstacle_points.color.a = 1.0;
        geometry_msgs::Point p;
        costmap_map_->mapToWorld(x, y, p.x, p.y);
        p.z = 0;

        obstacle_points.points.push_back(p);
        if (obstacle_points.points.size() >= maxIDs || cost > 0)
        {
            obstacle_marker_pub.publish(obstacle_points);
            obstacle_points.points.clear();
        }
    }
}
