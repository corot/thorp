/*
 * Author: Jorge Santos
 */

#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

#include <thorp_msgs/action/pickup_object.hpp>
#include <thorp_msgs/action/place_object.hpp>

#include "thorp_manipulation/arm_targets.hpp"

namespace thorp::manipulation
{
/**
 * Pickup and place object action servers. Each goal becomes a MoveIt Task Constructor task, planned here and
 * executed by move_group's ExecuteTaskSolution capability. Objects are taken from move_group's planning scene.
 */
class PickAndPlaceServer
{
public:
  PickAndPlaceServer(const rclcpp::Node::SharedPtr& node, const rclcpp::Node::SharedPtr& planning_node);
  ~PickAndPlaceServer();

private:
  using PickupObject = thorp_msgs::action::PickupObject;
  using PlaceObject = thorp_msgs::action::PlaceObject;
  using ExecuteTaskSolution = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
  using Feedback = std::function<void(const std::string& stage)>;
  using Canceling = std::function<bool()>;

  template <typename Action>
  typename rclcpp_action::Server<Action>::SharedPtr
  createServer(const std::string& name,
               void (PickAndPlaceServer::*execute)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>));

  void executePickup(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickupObject>> goal_handle);
  void executePlace(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>> goal_handle);

  int32_t pickup(const PickupObject::Goal& goal, const Feedback& feedback, const Canceling& canceling);
  int32_t place(const PlaceObject::Goal& goal, const Feedback& feedback, const Canceling& canceling);

  planning_scene::PlanningScenePtr getPlanningScene();

  /**
   * Target poses for the gripper link, one per attempt, relative to the arm base.
   * @return A ThorpError code
   */
  int32_t makeTargetPoses(const planning_scene::PlanningScene& scene, const Eigen::Vector3d& target,
                          std::vector<geometry_msgs::msg::PoseStamped>& poses);

  moveit::task_constructor::Task makeTask(const std::string& name, const planning_scene::PlanningScenePtr& scene);

  /**
   * Plan the task and execute its best solution.
   * @return A MoveIt error code
   */
  int32_t run(moveit::task_constructor::Task& task, const Feedback& feedback, const Canceling& canceling);

  std::string errorText(int32_t code) const;

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::vector<std::string> gripper_links_;
  // Corners of the open gripper links' bounding boxes, relative to the gripper link
  std::vector<Eigen::Vector3d> open_gripper_corners_;

  moveit::task_constructor::solvers::PipelinePlannerPtr sampling_planner_;
  moveit::task_constructor::solvers::JointInterpolationPlannerPtr interpolation_planner_;
  moveit::task_constructor::solvers::CartesianPathPtr cartesian_planner_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp_action::Client<ExecuteTaskSolution>::SharedPtr execute_client_;
  rclcpp_action::Server<PickupObject>::SharedPtr pickup_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_server_;

  // Pick and place goals are served one at a time, as they share the arm, each on its own thread
  std::atomic<bool> busy_{ false };
  std::thread goal_thread_;
  std::mutex planning_task_mutex_;
  moveit::task_constructor::Task* planning_task_ = nullptr;

  std::string arm_ref_frame_;
  std::string operation_plane_frame_;
  ArmCompensations compensations_;
  GripperModel gripper_model_;
  int attempts_;
  double planning_timeout_;
  int max_solutions_;
};

}  // namespace thorp::manipulation
