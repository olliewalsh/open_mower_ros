
#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <ros/ros.h>
#include "ftc_local_planner/PlannerGetProgress.h"
#include "ftc_local_planner/oscillation_detector.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <ftc_local_planner/PID.h>
#include <ftc_local_planner/CP.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include "tf2_eigen/tf2_eigen.h"
#include <mbf_costmap_core/costmap_controller.h>
#include <visualization_msgs/Marker.h>
#include "mower_msgs/Status.h"

namespace ftc_local_planner
{

    class FTCPlanner : public mbf_costmap_core::CostmapController
    {

        enum PlannerState
        {
            PRE_ROTATE,
            FOLLOWING,
            WAITING_FOR_GOAL_APPROACH,
            POST_ROTATE,
            FINISHED,
            INIT = 255
        };

    private:
        ros::ServiceServer progress_server;
        // State tracking
        const PlannerState& current_state;
        PlannerState current_state_;
        ros::Time state_entered_time;

        bool is_crashed;

        dynamic_reconfigure::Server<FTCPlannerConfig> *reconfig_server;

        tf2_ros::Buffer *tf_buffer;
        costmap_2d::Costmap2DROS *costmap;
        costmap_2d::Costmap2D* costmap_map_;

        std::vector<geometry_msgs::PoseStamped> global_plan;
        ros::Publisher global_point_pub;
        ros::Publisher global_plan_pub;
        ros::Publisher progress_pub;
        ros::Publisher obstacle_marker_pub;

        ros::Subscriber status_sub;

        FTCPlannerConfig config;

        Eigen::Affine3d current_control_point;

        /**
         * PID State
         */
        double lat_error, lon_error, angle_error, lin_speed = 0.0;
        double last_lon_error = 0.0;
        double last_lat_error = 0.0;
        double last_angle_error = 0.0;
        double i_lon_error = 0.0;
        double i_lat_error = 0.0;
        double i_angle_error = 0.0;
        double speed_limit = 0.0;
        ros::Time last_time;

        mower_msgs::Status last_status;
        /**
         * Speed ramp for acceleration and deceleration
         */
        double current_movement_speed;
        double filtered_actual_linear_speed = 0.0;
        double filtered_actual_angular_speed = 0.0;
        bool actual_twist_initialized_ = false;
        double rotate_collision_line_length_ = 0.0;
        bool angular_lag_reference_initialized_ = false;
        double angular_lag_reference_error_ = 0.0;

        /**
         * State for point interpolation
         */
        uint32_t current_index;
        double current_progress;
        Eigen::Affine3d last_local_control_point, local_control_point;

        /**
         * Private members
         */
        ros::Publisher pubPid, pubCp;
        FailureDetector failure_detector_; //!< Detect if the robot got stucked
        ros::Time time_last_oscillation_;  //!< Store at which time stamp the last oscillation was detected
        bool oscillation_detected_ = false;
        bool oscillation_warning_ = false;
        int rotate_direction_sign_ = 0;

        double distanceLookahead();
        double velocityLookahead();
        void set_planner_state(PlannerState s);
        void update_planner_state();
        void update_actual_twist(const geometry_msgs::TwistStamped &velocity);
        double normalize_angle(double angle) const;
        bool is_cost_blocking(unsigned char cost) const;
        void append_line_samples(const geometry_msgs::Point &start, const geometry_msgs::Point &end, double resolution,
                                 std::vector<geometry_msgs::Point> &samples, bool skip_first_point) const;
        bool is_pose_collision_free(const geometry_msgs::PoseStamped &pose) const;
        bool is_rotation_direction_collision_free(const geometry_msgs::PoseStamped &target_pose, int direction_sign) const;
        void choose_rotate_direction(const geometry_msgs::PoseStamped &target_pose);
        double compute_angular_lag_time(double angular_error, double angular_speed);
        double compute_mow_rpm_error() const;
        double compute_mow_rpm_speed_limit(double mow_rpm_error) const;
        void update_control_point(double dt);
        void calculate_velocity_commands(double dt, geometry_msgs::TwistStamped &cmd_vel);

        /**
         * @brief check for obstacles in path as well as collision at actual pose
         * @param max_points number of path segments (of global path) to check
         * @return true if collision will happen.
         */
        bool checkCollision(int max_points);

        /**
         * @brief check if robot oscillates (only angular). Can be used to do some recovery
         * @param cmd_vel last velocity message send to robot
         * @return true if robot oscillates
         */
        bool checkOscillation(geometry_msgs::TwistStamped &cmd_vel);

        /**
         * @brief publish obstacles on path as marker array.
         * @brief If obstacle_points contains more elements than maxID, marker gets published and
         * @brief cleared afterwards.
         * @param obstacle_points already collected points to visualize
         * @param x X position in costmap
         * @param y Y position in costmap
         * @param cost cost value of cell
         * @param maxIDs num of markers before publishing
         * @return sum of `values`, or 0.0 if `values` is empty.
         */
        void debugObstacle(visualization_msgs::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs);

        double time_in_current_state()
        {
            return (ros::Time::now() - state_entered_time).toSec();
        }

        void reconfigureCB(FTCPlannerConfig &config, uint32_t level);

        void statusReceived(const mower_msgs::Status::ConstPtr &msg);

    public:
        FTCPlanner() : current_state_(INIT), current_state(current_state_) {};

        bool getProgress(ftc_local_planner::PlannerGetProgressRequest &req, ftc_local_planner::PlannerGetProgressResponse &res);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

        ~FTCPlanner() override;

        uint32_t
        computeVelocityCommands(const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &velocity,
                                geometry_msgs::TwistStamped &cmd_vel, std::string &message) override;

        bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

        bool cancel() override;
    };
};
#endif
