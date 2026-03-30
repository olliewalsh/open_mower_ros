#ifndef FTC_LOCAL_PLANNER_HYBRID_APPROACH_PLANNER_H_
#define FTC_LOCAL_PLANNER_HYBRID_APPROACH_PLANNER_H_

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <pluginlib/class_loader.h>
#include <tf2_ros/buffer.h>

namespace ftc_local_planner
{

class HybridApproachPlanner : public mbf_costmap_core::CostmapController
{
  public:
    HybridApproachPlanner();

    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

    uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                     const geometry_msgs::TwistStamped &velocity,
                                     geometry_msgs::TwistStamped &cmd_vel,
                                     std::string &message) override;

    bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

    bool cancel() override;

  private:
    using ControllerPtr = boost::shared_ptr<mbf_costmap_core::CostmapController>;

    ControllerPtr active_controller() const;
    bool ensure_initialized() const;
    bool goal_is_within_local_costmap() const;
    bool near_controller_failed(uint32_t result) const;
    double global_plan_length() const;
    void switch_to_near_controller();
    void switch_to_far_controller(const std::string &reason, bool start_cooldown);

    bool initialized_;
    bool using_near_controller_;
    double switch_costmap_margin_;
    double min_plan_length_for_near_controller_;
    int near_failure_count_;
    int near_failure_limit_;
    double near_retry_cooldown_;
    ros::Time near_retry_allowed_at_;

    std::string planner_name_;
    std::string far_controller_name_;
    std::string near_controller_name_;
    std::string far_controller_type_;
    std::string near_controller_type_;

    tf2_ros::Buffer *tf_buffer_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    pluginlib::ClassLoader<mbf_costmap_core::CostmapController> controller_loader_;
    ControllerPtr far_controller_;
    ControllerPtr near_controller_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
};

}  // namespace ftc_local_planner

#endif
