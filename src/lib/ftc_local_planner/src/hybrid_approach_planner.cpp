#include <ftc_local_planner/hybrid_approach_planner.h>

#include <algorithm>
#include <stdexcept>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::HybridApproachPlanner, mbf_costmap_core::CostmapController)

namespace ftc_local_planner
{

HybridApproachPlanner::HybridApproachPlanner()
    : initialized_(false),
      using_near_controller_(false),
      switch_costmap_margin_(0.25),
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
    using_near_controller_ = false;

    const bool far_ok = far_controller_->setPlan(plan);
    const bool near_ok = near_controller_->setPlan(plan);
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

    if (!using_near_controller_ && goal_is_within_local_costmap())
    {
        using_near_controller_ = true;
        near_controller_->setPlan(global_plan_);
        ROS_INFO_STREAM("HybridApproachPlanner: Switching to near controller " << near_controller_name_
                        << " because the goal is now within the local costmap window.");
    }

    return active_controller()->computeVelocityCommands(pose, velocity, cmd_vel, message);
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

}  // namespace ftc_local_planner
