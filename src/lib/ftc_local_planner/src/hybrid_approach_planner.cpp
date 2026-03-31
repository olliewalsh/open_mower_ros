#include <ftc_local_planner/hybrid_approach_planner.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::HybridApproachPlanner, mbf_costmap_core::CostmapController)

namespace ftc_local_planner
{

HybridApproachPlanner::HybridApproachPlanner()
    : initialized_(false),
      using_near_controller_(false),
      switch_costmap_margin_(0.25),
      min_plan_length_for_near_controller_(1.0),
      near_failure_count_(0),
      near_failure_limit_(3),
      near_retry_cooldown_(2.0),
      tf_buffer_(nullptr),
      costmap_ros_(nullptr),
      controller_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapController")
{
}

void HybridApproachPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    if (initialized_)
    {
        return;
    }

    planner_name_ = name;
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle private_nh("~/" + planner_name_);
    private_nh.param("switch_costmap_margin", switch_costmap_margin_, 0.25);
    private_nh.param("min_plan_length_for_near_controller", min_plan_length_for_near_controller_, 1.0);
    private_nh.param("near_failure_limit", near_failure_limit_, 3);
    private_nh.param("near_retry_cooldown", near_retry_cooldown_, 2.0);
    private_nh.param("far_controller_name", far_controller_name_, std::string("FTCPlanner"));
    private_nh.param("near_controller_name", near_controller_name_, std::string("TEBPlanner"));
    private_nh.param("far_controller_type", far_controller_type_, std::string("ftc_local_planner/FTCPlanner"));
    private_nh.param("near_controller_type", near_controller_type_, std::string("teb_local_planner/TebLocalPlannerROS"));

    const std::string far_child_name = planner_name_ + "/" + far_controller_name_;
    const std::string near_child_name = planner_name_ + "/" + near_controller_name_;

    try
    {
        far_controller_ = controller_loader_.createInstance(far_controller_type_);
        far_controller_->initialize(far_child_name, tf_buffer_, costmap_ros_);

        near_controller_ = controller_loader_.createInstance(near_controller_type_);
        near_controller_->initialize(near_child_name, tf_buffer_, costmap_ros_);
    }
    catch (const pluginlib::PluginlibException &e)
    {
        throw std::runtime_error("HybridApproachPlanner failed to load child controller plugin: " + std::string(e.what()));
    }

    initialized_ = true;
    ROS_INFO_STREAM("HybridApproachPlanner: Initialized with far controller " << far_controller_type_
                    << " and near controller " << near_controller_type_ << ".");
}

bool HybridApproachPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!ensure_initialized())
    {
        return false;
    }

    global_plan_ = plan;
    far_controller_plan_ = simplify_far_plan(plan);
    using_near_controller_ = false;
    near_failure_count_ = 0;
    near_retry_allowed_at_ = ros::Time(0);

    const bool far_ok = far_controller_->setPlan(far_controller_plan_);
    const bool near_ok = near_controller_->setPlan(plan);
    if (far_controller_plan_.size() != plan.size())
    {
        ROS_INFO_STREAM("HybridApproachPlanner: Simplified far-controller approach plan from "
                        << plan.size() << " to " << far_controller_plan_.size() << " poses.");
    }
    return far_ok && near_ok;
}

uint32_t HybridApproachPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                        const geometry_msgs::TwistStamped &velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
{
    if (!ensure_initialized())
    {
        message = "HybridApproachPlanner is not initialized";
        return 102;
    }

    if (!using_near_controller_ &&
        ros::Time::now() >= near_retry_allowed_at_ &&
        global_plan_length() >= min_plan_length_for_near_controller_ &&
        goal_is_within_local_costmap())
    {
        switch_to_near_controller();
    }

    const uint32_t result = active_controller()->computeVelocityCommands(pose, velocity, cmd_vel, message);

    if (using_near_controller_ && near_controller_failed(result))
    {
        near_failure_count_++;
        ROS_WARN_STREAM_THROTTLE(1.0, "HybridApproachPlanner: Near controller " << near_controller_name_
                                 << " returned " << result << " (" << near_failure_count_ << "/"
                                 << near_failure_limit_ << ").");
        if (near_failure_count_ >= near_failure_limit_)
        {
            switch_to_far_controller("near controller repeatedly failed near the goal", true);
        }
    }
    else if (using_near_controller_ && result == 0)
    {
        near_failure_count_ = 0;
    }

    return result;
}

bool HybridApproachPlanner::isGoalReached(double dist_tolerance, double angle_tolerance)
{
    if (!ensure_initialized())
    {
        return false;
    }

    return active_controller()->isGoalReached(dist_tolerance, angle_tolerance);
}

bool HybridApproachPlanner::cancel()
{
    bool far_cancelled = true;
    bool near_cancelled = true;

    if (far_controller_)
    {
        far_cancelled = far_controller_->cancel();
    }

    if (near_controller_)
    {
        near_cancelled = near_controller_->cancel();
    }

    return far_cancelled && near_cancelled;
}

HybridApproachPlanner::ControllerPtr HybridApproachPlanner::active_controller() const
{
    return using_near_controller_ ? near_controller_ : far_controller_;
}

bool HybridApproachPlanner::ensure_initialized() const
{
    return initialized_ && far_controller_ && near_controller_;
}

bool HybridApproachPlanner::goal_is_within_local_costmap() const
{
    if (global_plan_.empty() || costmap_ros_ == nullptr || tf_buffer_ == nullptr)
    {
        return false;
    }

    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    const std::string global_frame = costmap_ros_->getGlobalFrameID();

    if (!goal_pose.header.frame_id.empty() && goal_pose.header.frame_id != global_frame)
    {
        try
        {
            goal_pose = tf_buffer_->transform(goal_pose, global_frame, ros::Duration(0.1));
        }
        catch (const tf2::TransformException &e)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "HybridApproachPlanner: Failed to transform goal pose into "
                                           << global_frame << ": " << e.what());
            return false;
        }
    }

    const costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    const double resolution = costmap->getResolution();
    const double origin_x = costmap->getOriginX();
    const double origin_y = costmap->getOriginY();
    const double max_x = origin_x + costmap->getSizeInCellsX() * resolution;
    const double max_y = origin_y + costmap->getSizeInCellsY() * resolution;
    const double margin = std::max(0.0, switch_costmap_margin_);

    const double goal_x = goal_pose.pose.position.x;
    const double goal_y = goal_pose.pose.position.y;

    return goal_x >= origin_x + margin &&
           goal_x <= max_x - margin &&
           goal_y >= origin_y + margin &&
           goal_y <= max_y - margin;
}

bool HybridApproachPlanner::near_controller_failed(uint32_t result) const
{
    return result != 0;
}

double HybridApproachPlanner::global_plan_length() const
{
    if (global_plan_.size() < 2)
    {
        return 0.0;
    }

    double length = 0.0;
    for (size_t i = 1; i < global_plan_.size(); ++i)
    {
        const auto &a = global_plan_[i - 1].pose.position;
        const auto &b = global_plan_[i].pose.position;
        length += std::hypot(b.x - a.x, b.y - a.y);
    }
    return length;
}

std::vector<geometry_msgs::PoseStamped> HybridApproachPlanner::simplify_far_plan(
    const std::vector<geometry_msgs::PoseStamped> &plan) const
{
    if (plan.size() <= 3)
    {
        return plan;
    }

    constexpr double kMinPointSpacing = 0.02;
    constexpr double kMaxLateralDeviation = 0.03;
    constexpr double kMaxHeadingDeviation = 5.0 * (M_PI / 180.0);

    auto point_distance = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        return std::sqrt(dx * dx + dy * dy);
    };

    auto wrap_angle = [](double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };

    auto point_line_distance = [&](const geometry_msgs::Point &point, const geometry_msgs::Point &line_start,
                                   const geometry_msgs::Point &line_end) {
        const double line_dx = line_end.x - line_start.x;
        const double line_dy = line_end.y - line_start.y;
        const double line_length_sq = line_dx * line_dx + line_dy * line_dy;
        if (line_length_sq <= 1e-9)
        {
            return point_distance(point, line_start);
        }

        const double projection =
            ((point.x - line_start.x) * line_dx + (point.y - line_start.y) * line_dy) / line_length_sq;
        const double clamped_projection = std::max(0.0, std::min(1.0, projection));
        geometry_msgs::Point projected_point;
        projected_point.x = line_start.x + clamped_projection * line_dx;
        projected_point.y = line_start.y + clamped_projection * line_dy;
        return point_distance(point, projected_point);
    };

    std::vector<geometry_msgs::PoseStamped> simplified_plan;
    simplified_plan.reserve(plan.size());
    simplified_plan.push_back(plan.front());

    for (size_t i = 1; i + 1 < plan.size(); ++i)
    {
        const auto &prev_kept = simplified_plan.back().pose.position;
        const auto &current = plan[i].pose.position;
        const auto &next = plan[i + 1].pose.position;

        if (point_distance(prev_kept, current) <= kMinPointSpacing)
        {
            continue;
        }

        const double in_heading = std::atan2(current.y - prev_kept.y, current.x - prev_kept.x);
        const double out_heading = std::atan2(next.y - current.y, next.x - current.x);
        const double heading_delta = std::abs(wrap_angle(out_heading - in_heading));
        const double lateral_deviation = point_line_distance(current, prev_kept, next);

        if (heading_delta <= kMaxHeadingDeviation && lateral_deviation <= kMaxLateralDeviation)
        {
            continue;
        }

        simplified_plan.push_back(plan[i]);
    }

    if (point_distance(simplified_plan.back().pose.position, plan.back().pose.position) > 1e-6)
    {
        simplified_plan.push_back(plan.back());
    }
    else
    {
        simplified_plan.back() = plan.back();
    }

    for (size_t i = 0; i + 1 < simplified_plan.size(); ++i)
    {
        const auto &current = simplified_plan[i].pose.position;
        const auto &next = simplified_plan[i + 1].pose.position;
        const double distance = point_distance(current, next);
        if (distance <= 1e-6)
        {
            continue;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, std::atan2(next.y - current.y, next.x - current.x));
        simplified_plan[i].pose.orientation = tf2::toMsg(q);
    }

    return simplified_plan;
}

void HybridApproachPlanner::switch_to_near_controller()
{
    using_near_controller_ = true;
    near_failure_count_ = 0;
    near_controller_->setPlan(global_plan_);
    ROS_INFO_STREAM("HybridApproachPlanner: Switching to near controller " << near_controller_name_
                    << " because the goal is now within the local costmap window.");
}

void HybridApproachPlanner::switch_to_far_controller(const std::string &reason, bool start_cooldown)
{
    using_near_controller_ = false;
    near_failure_count_ = 0;
    far_controller_->setPlan(far_controller_plan_);
    if (start_cooldown)
    {
        near_retry_allowed_at_ = ros::Time::now() + ros::Duration(std::max(0.0, near_retry_cooldown_));
    }
    ROS_WARN_STREAM("HybridApproachPlanner: Switching back to far controller " << far_controller_name_
                    << " because " << reason << ".");
}

}  // namespace ftc_local_planner
