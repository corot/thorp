#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/common.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/footprint_helper.h>

#include <mbf_costmap_core/costmap_recovery.h>


namespace thorp_mbf_plugins
{

/**
 * @brief Search tree node
 */
struct CmdVelNode {
  CmdVelNode(int depth, double pose_x, double pose_y, double heading, geometry_msgs::Twist cmd_vel,
             CmdVelNode* prev = nullptr, double cost = std::numeric_limits<double>::infinity()) :
      depth_(depth), pose_x_(pose_x), pose_y_(pose_y), heading_(heading), cmd_vel_(cmd_vel), prev_(prev),
      cost_(cost)
  {}

  void cleanup()
  {
    if (following_.size() != 0)
    {
      for (unsigned int i=0; i<following_.size(); ++i)
      {
        if (following_[i])
          following_[i]->cleanup();
      }
    }
    delete this;
  }

  CmdVelNode* prev_;
  std::vector<CmdVelNode*> following_;

  // the cmd_vel refers to the last command taken to reach (pose_x, pose_y, heading_)
  geometry_msgs::Twist cmd_vel_;
  double pose_x_;
  double pose_y_;
  double heading_;
  // number of preceding commands
  int depth_;
  // cost at pose_x, pose_y, heading_
  double cost_;
};

/**
 * @brief Comparator used on greedy search
 */
struct CmpCmdVelNodePtrs
{
  bool operator()(const CmdVelNode* a, const CmdVelNode* b)
  {
    return (a->cost_ > b->cost_);
  }
};


class SlowEscapeRecovery : public mbf_costmap_core::CostmapRecovery
{
public:
  SlowEscapeRecovery();
  ~SlowEscapeRecovery();

  /// Initialize the parameters of the behavior
  void initialize(std::string n, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);

  /// Run the behavior
  uint32_t runBehavior(std::string& message);

  /// Cancels the behavior
  bool cancel()
  {
    if (!behavior_running_)
      return false;

    cancel_requested_ = true;
    while (ros::ok() && behavior_running_)
      ros::Duration(0.1).sleep();

    return !behavior_running_;
  }

private:
  enum class PlanResult {CANCELED, PLANNING, ZERO_COST, ZERO_VEL, INC_COST, DEC_COST};

  /**
   * @brief One iteration of the controller: publish a velocity command following a decreasing footprint
   * cost gradient on local costmap. Does nothing if the minimum estimated cost is above prev_cost.
   * @param robot_pose Current robot pose on local costmap reference
   * @param time_step Time to project candidate velocities before estimating its cost
   * @param prev_cost Minimum cost obtained on previous iteration
   * @return Planning outcome, as a PlanResult typed enum
   */
  PlanResult makePlan(const geometry_msgs::PoseStamped& robot_pose, double time_step, double prev_cost,
                      double& new_cost, std::vector<geometry_msgs::Twist>& cmd_vels);

  /**
   * Integrate the cost of all the cells covered by the given footprint at the given pose, in the given costmap.
   * We assign a much higher cost to cells containing inflated and specially lethal obstacles. If at least one
   * cell contains a lethal obstacle, we set in_collision flag as true
   * @param x, y, th Target pose
   * @param costmap Costmap on which we want the pose's cost
   * @param footprint Footprint for which we want the pose's cost
   * @param escaped Does any cell within the footprint contain a lethal/inflated obstacle?
   * @return Added cost of the covered cells
   */
  double getPoseCost(double x, double y, double th, costmap_2d::Costmap2DROS* costmap,
                     const std::vector<geometry_msgs::Point>& footprint, bool& escaped);


  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2DROS* global_costmap_;
  base_local_planner::FootprintHelper footprint_helper_;  ///< allows calculating a pose's footprint cost

  std::string name_;
  bool initialized_;
  bool behavior_running_;
  bool cancel_requested_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;

  double v_max_, w_max_;                  ///< @brief Controller maximum linear and angular velocities
  double wheel_separation_;               ///< @brief Wheel separation, required to compute per-wheel velocity
  double controller_frequency_;           ///< @brief Controller execution rate
  double stagnated_patience_;             ///< @brief Time to wait before assuming controller is stagnated
  double vel_projection_time_;            ///< @brief Time to project candidate velocities before estimating its cost
  double max_execution_time_;             ///< @brief Maximum recovery execution time
  double max_travelled_distance_;         ///< @brief Maximum distance to travel during recovery execution
  double escape_from_collision_;          ///< @brief Move away from lethal obstacles by this distance (< 0 to disable)
  double escape_from_inscribed_;          ///< @brief Move away from inflated obstacles by this distance (< 0 to disable)
  double release_bumper_distance_;        ///< @brief Minimum distance required to fully release a pressed bumper
  int release_bumper_depth_;              ///< @brief Minimum depth required to travel release_bumper_distance_
  int max_search_depth_;                  ///< @brief Maximum depth at which the search will be performed
  bool drive_forward_allowed_ = true;     ///< @brief Whether is driving forward allowed (as no bumper is pressed)
  bool drive_backward_allowed_ = true;    ///< @brief Whether is driving backward allowed
};

}; // namespace thorp_mbf_plugins
