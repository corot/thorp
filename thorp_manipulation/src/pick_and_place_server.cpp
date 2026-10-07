/*
 * Author: Jorge Santos
 */

#include "thorp_manipulation/pick_and_place_server.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <thread>

#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/task_constructor/stages.h>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <thorp_msgs/msg/thorp_error.hpp>

namespace mtc = moveit::task_constructor;

using namespace std::chrono_literals;
using moveit_msgs::msg::MoveItErrorCodes;
using thorp_msgs::msg::ThorpError;

namespace thorp::manipulation
{
namespace
{
const std::string ARM_GROUP = "arm";
const std::string GRIPPER_GROUP = "gripper";
const std::string GRIPPER_LINK = "gripper_link";
const std::string GRIPPER_JOINT = "gripper_joint";
// As in moveit_controllers.yaml, so MTC needn't guess which one executes each trajectory
const std::string ARM_CONTROLLER = "arm_controller";
const std::string GRIPPER_CONTROLLER = "gripper_controller";

// Straight gripper motions before and after grasping and releasing objects: minimum and desired distances
constexpr double PICK_APPROACH_MIN = 0.025;
constexpr double PICK_APPROACH_MAX = 0.05;
constexpr double PLACE_APPROACH_MIN = 0.01;
constexpr double PLACE_APPROACH_MAX = 0.05;
// Opening the gripper this much wider than the held object releases it
constexpr double RELEASE_MARGIN = 0.015;
// Object vertices this close to its widest extent across the fingers belong to its widest section
constexpr double WIDEST_SECTION_MARGIN = 0.002;
// Height of the open gripper over the surface supporting the object to grasp
constexpr double SUPPORT_CLEARANCE = 0.005;

template <typename T>
T param(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  // MoveIt configuration parameters come as undeclared overrides, so the node declares them automatically
  if (!node->has_parameter(name))
    node->declare_parameter(name, default_value);
  return node->get_parameter(name).get_value<T>();
}

double yaw(const Eigen::Isometry3d& pose)
{
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

mtc::TrajectoryExecutionInfo executedBy(const std::string& controller)
{
  mtc::TrajectoryExecutionInfo info;
  info.controller_names = { controller };
  return info;
}

/**
 * Pairs in contact where the stages without solutions failed, as MTC's comments say only that a state collides: the
 * last waypoint of each failed trajectory, where MTC's planners stop on a collision, or else the failure's own state,
 * as the current state's.
 */
std::set<std::string> failureContacts(const mtc::Task& task)
{
  std::set<std::string> pairs;
  task.stages()->traverseRecursively([&pairs](const mtc::Stage& stage, unsigned int) {
    if (!stage.solutions().empty())
      return true;
    for (const auto& failure : stage.failures())
    {
      const mtc::InterfaceState* interface_state = failure->start() ? failure->start() : failure->end();
      if (!interface_state)
        continue;
      const auto& scene = interface_state->scene();
      const auto* sub_trajectory = dynamic_cast<const mtc::SubTrajectory*>(failure.get());
      const bool has_waypoints =
          sub_trajectory && sub_trajectory->trajectory() && !sub_trajectory->trajectory()->empty();
      collision_detection::CollisionRequest request;
      request.contacts = true;
      request.max_contacts = 10;
      collision_detection::CollisionResult result;
      scene->checkCollision(request, result,
                            has_waypoints ? sub_trajectory->trajectory()->getLastWayPoint() : scene->getCurrentState());
      for (const auto& [names, contacts] : result.contacts)
        pairs.insert(stage.name() + ": " + names.first + " - " + names.second);
    }
    return true;
  });
  return pairs;
}

geometry_msgs::msg::Vector3Stamped gripperDirection(double x)
{
  geometry_msgs::msg::Vector3Stamped direction;
  direction.header.frame_id = GRIPPER_LINK;
  direction.vector.x = x;
  return direction;
}

std::vector<Eigen::Vector3d> boxCorners(const Eigen::Vector3d& center, const Eigen::Vector3d& half_size)
{
  std::vector<Eigen::Vector3d> corners;
  for (int i = 0; i < 8; ++i)
    corners.push_back(center + Eigen::Vector3d((i & 1 ? 1 : -1) * half_size.x(), (i & 2 ? 1 : -1) * half_size.y(),
                                               (i & 4 ? 1 : -1) * half_size.z()));
  return corners;
}

/**
 * Height of an object's widest section across the gripper fingers, where they close on it: the middle of its vertices
 * farthest to both sides along the fingers' closing axis. Shapes other than meshes count as their bounding boxes.
 * @param shape The object's shape
 * @param pose The shape's pose
 * @param closing_axis The fingers' closing axis, horizontal, on the same frame as the pose
 */
double widestSectionHeight(const shapes::Shape& shape, const Eigen::Isometry3d& pose,
                           const Eigen::Vector3d& closing_axis)
{
  std::vector<Eigen::Vector3d> vertices;
  if (shape.type == shapes::MESH)
  {
    const auto& mesh = static_cast<const shapes::Mesh&>(shape);
    for (unsigned int i = 0; i < mesh.vertex_count; ++i)
      vertices.push_back(pose *
                         Eigen::Vector3d(mesh.vertices[3 * i], mesh.vertices[3 * i + 1], mesh.vertices[3 * i + 2]));
  }
  else
  {
    const Eigen::Vector3d half_size = shapes::computeShapeExtents(&shape) / 2.0;
    for (const Eigen::Vector3d& corner : boxCorners(Eigen::Vector3d::Zero(), half_size))
      vertices.push_back(pose * corner);
  }

  double min_extent = std::numeric_limits<double>::max();
  double max_extent = std::numeric_limits<double>::lowest();
  for (const Eigen::Vector3d& vertex : vertices)
  {
    min_extent = std::min(min_extent, vertex.dot(closing_axis));
    max_extent = std::max(max_extent, vertex.dot(closing_axis));
  }
  double min_side_height = 0.0, max_side_height = 0.0;
  int min_side_count = 0, max_side_count = 0;
  for (const Eigen::Vector3d& vertex : vertices)
  {
    if (vertex.dot(closing_axis) < min_extent + WIDEST_SECTION_MARGIN)
    {
      min_side_height += vertex.z();
      ++min_side_count;
    }
    if (vertex.dot(closing_axis) > max_extent - WIDEST_SECTION_MARGIN)
    {
      max_side_height += vertex.z();
      ++max_side_count;
    }
  }
  return (min_side_height / min_side_count + max_side_height / max_side_count) / 2.0;
}

/**
 * Fixed target poses for the gripper, spawned with it open, so the motion that reaches them opens it on the way.
 */
class OpenGripperPoses : public mtc::stages::FixedCartesianPoses
{
public:
  using FixedCartesianPoses::FixedCartesianPoses;

  void compute() override
  {
    if (upstream_solutions_.empty())
      return;

    planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();
    moveit::core::RobotState& state = scene->getCurrentStateNonConst();
    state.setToDefaultValues(state.getJointModelGroup(GRIPPER_GROUP), "open");
    state.update();
    for (geometry_msgs::msg::PoseStamped pose : properties().get<std::vector<geometry_msgs::msg::PoseStamped>>("poses"))
    {
      if (pose.header.frame_id.empty())
        pose.header.frame_id = scene->getPlanningFrame();
      mtc::InterfaceState target(scene);
      target.properties().set("target_pose", pose);
      spawn(std::move(target), 0.0);
    }
  }
};

}  // namespace

PickAndPlaceServer::PickAndPlaceServer(const rclcpp::Node::SharedPtr& node,
                                       const rclcpp::Node::SharedPtr& planning_node)
  : node_(node)
{
  arm_ref_frame_ = param<std::string>(node_, "arm_ref_frame", "arm_base_link");
  operation_plane_frame_ = param<std::string>(node_, "operation_plane_frame", "arm_shoulder_lift_servo_link");
  compensations_.vertical_backlash_scale = param(node_, "vertical_backlash_scale", 0.0);
  compensations_.vertical_backlash_delta = param(node_, "vertical_backlash_delta", 0.0);
  compensations_.fall_short_distance_delta = param(node_, "fall_short_distance_delta", 0.0);
  compensations_.gripper_asymmetry_yaw_delta = param(node_, "gripper_asymmetry_yaw_delta", 0.0);
  gripper_model_.center = param(node_, "gripper.center", gripper_model_.center);
  gripper_model_.pad_width = param(node_, "gripper.pad_width", gripper_model_.pad_width);
  gripper_model_.finger_length = param(node_, "gripper.finger_length", gripper_model_.finger_length);
  attempts_ = param(node_, "attempts", 5);
  planning_timeout_ = param(node_, "planning_timeout", 10.0);
  max_solutions_ = param(node_, "max_solutions", 3);

  robot_model_ = robot_model_loader::RobotModelLoader(node_).getModel();
  if (!robot_model_)
    throw std::runtime_error("Failed to load the robot model");
  gripper_links_ = robot_model_->getJointModelGroup(GRIPPER_GROUP)->getLinkModelNamesWithCollisionGeometry();
  moveit::core::RobotState open_gripper(robot_model_);
  open_gripper.setToDefaultValues();
  open_gripper.setToDefaultValues(robot_model_->getJointModelGroup(GRIPPER_GROUP), "open");
  open_gripper.update();
  const Eigen::Isometry3d to_gripper_link = open_gripper.getGlobalLinkTransform(GRIPPER_LINK).inverse();
  for (const std::string& name : gripper_links_)
  {
    const moveit::core::LinkModel* link = robot_model_->getLinkModel(name);
    for (const Eigen::Vector3d& corner :
         boxCorners(link->getCenteredBoundingBoxOffset(), link->getShapeExtentsAtOrigin() / 2.0))
      open_gripper_corners_.push_back(to_gripper_link * open_gripper.getGlobalLinkTransform(link) * corner);
  }

  sampling_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(planning_node, "ompl", "RRTConnect");
  // Workspace bounds only matter to planar or floating joints, that the arm lacks; MoveIt warns if there are none
  moveit_msgs::msg::WorkspaceParameters workspace;
  workspace.header.frame_id = robot_model_->getModelFrame();
  workspace.min_corner.x = workspace.min_corner.y = workspace.min_corner.z = -1.0;
  workspace.max_corner.x = workspace.max_corner.y = workspace.max_corner.z = 1.0;
  sampling_planner_->properties().set("workspace_parameters", workspace);
  interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner_->setStepSize(0.005);
  for (const mtc::solvers::PlannerInterfacePtr& planner :
       { mtc::solvers::PlannerInterfacePtr(sampling_planner_), mtc::solvers::PlannerInterfacePtr(interpolation_planner_),
         mtc::solvers::PlannerInterfacePtr(cartesian_planner_) })
  {
    planner->setMaxVelocityScalingFactor(1.0);
    planner->setMaxAccelerationScalingFactor(1.0);
  }

  planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  execute_client_ = rclcpp_action::create_client<ExecuteTaskSolution>(node_, "execute_task_solution");
  pickup_server_ = createServer<PickupObject>("~/pickup_object", &PickAndPlaceServer::executePickup);
  place_server_ = createServer<PlaceObject>("~/place_object", &PickAndPlaceServer::executePlace);
  RCLCPP_INFO(node_->get_logger(), "Pickup and place object action servers ready");
}

template <typename Action>
typename rclcpp_action::Server<Action>::SharedPtr PickAndPlaceServer::createServer(
    const std::string& name,
    void (PickAndPlaceServer::*execute)(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>))
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  return rclcpp_action::create_server<Action>(
      node_, name,
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>) {
        if (busy_.exchange(true))
        {
          RCLCPP_ERROR(node_->get_logger(), "Rejecting goal: already picking or placing an object");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        // Stop planning; execution stops on the executing thread, that polls for cancel requests
        std::lock_guard<std::mutex> lock(planning_task_mutex_);
        if (planning_task_)
          planning_task_->preempt();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this, execute](const std::shared_ptr<GoalHandle> goal_handle) {
        // The previous goal's thread has finished, as goals are accepted only when not busy
        if (goal_thread_.joinable())
          goal_thread_.join();
        goal_thread_ = std::thread([this, execute, goal_handle]() {
          try
          {
            (this->*execute)(goal_handle);
          }
          catch (const std::runtime_error&)
          {
            // Reporting the result fails when shutting down
            if (rclcpp::ok())
              throw;
          }
          busy_ = false;
        });
      });
}

PickAndPlaceServer::~PickAndPlaceServer()
{
  // The goal thread uses this object; on shutdown, it stops waiting for the execution result
  {
    std::lock_guard<std::mutex> lock(planning_task_mutex_);
    if (planning_task_)
      planning_task_->preempt();
  }
  if (goal_thread_.joinable())
    goal_thread_.join();
}

void PickAndPlaceServer::executePickup(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickupObject>> goal_handle)
{
  auto feedback = [goal_handle](const std::string& stage) {
    auto msg = std::make_shared<PickupObject::Feedback>();
    msg->stage = stage;
    goal_handle->publish_feedback(msg);
  };
  auto result = std::make_shared<PickupObject::Result>();
  result->error.code = pickup(*goal_handle->get_goal(), feedback, [&goal_handle]() { return goal_handle->is_canceling(); });
  result->error.text = errorText(result->error.code);
  if (result->error.code == ThorpError::SUCCESS)
    goal_handle->succeed(result);
  else if (goal_handle->is_canceling())
    goal_handle->canceled(result);
  else
    goal_handle->abort(result);
}

void PickAndPlaceServer::executePlace(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>> goal_handle)
{
  auto feedback = [goal_handle](const std::string& stage) {
    auto msg = std::make_shared<PlaceObject::Feedback>();
    msg->stage = stage;
    goal_handle->publish_feedback(msg);
  };
  auto result = std::make_shared<PlaceObject::Result>();
  result->error.code = place(*goal_handle->get_goal(), feedback, [&goal_handle]() { return goal_handle->is_canceling(); });
  result->error.text = errorText(result->error.code);
  if (result->error.code == ThorpError::SUCCESS)
    goal_handle->succeed(result);
  else if (goal_handle->is_canceling())
    goal_handle->canceled(result);
  else
    goal_handle->abort(result);
}

int32_t PickAndPlaceServer::pickup(const PickupObject::Goal& goal, const Feedback& feedback,
                                   const Canceling& canceling)
{
  RCLCPP_INFO(node_->get_logger(), "Pickup object '%s' from support surface '%s'", goal.object_name.c_str(),
              goal.support_surf.c_str());
  planning_scene::PlanningScenePtr scene = getPlanningScene();
  if (!scene)
    return ThorpError::SERVER_NOT_AVAILABLE;

  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  scene->getCurrentState().getAttachedBodies(attached_bodies);
  if (!attached_bodies.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Already have an attached object: '%s'; clear the gripper before picking another",
                 attached_bodies.front()->getName().c_str());
    return ThorpError::OBJECT_ATTACHED;
  }
  collision_detection::World::ObjectConstPtr object = scene->getWorld()->getObject(goal.object_name);
  if (!object)
  {
    RCLCPP_ERROR(node_->get_logger(), "Object '%s' not found in the planning scene", goal.object_name.c_str());
    return ThorpError::OBJECT_NOT_FOUND;
  }
  if (object->shapes_.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Object '%s' has no shapes", goal.object_name.c_str());
    return ThorpError::OBJECT_SIZE_NOT_FOUND;
  }

  // Grasp the object across its widest section, but with the open gripper clear of the support surface, and never
  // above its top; we assume objects are made of a single shape, centered on their pose
  Eigen::Vector3d object_size = shapes::computeShapeExtents(object->shapes_.front().get());
  const Eigen::Isometry3d to_arm_base = scene->getFrameTransform(arm_ref_frame_).inverse();
  Eigen::Isometry3d object_pose = to_arm_base * object->pose_;
  const double top = object_pose.translation().z() + object_size.z() / 2.0;
  const double base = top - object_size.z();
  Eigen::Vector3d grasp_point(object_pose.translation().x(), object_pose.translation().y(), top);
  std::vector<geometry_msgs::msg::PoseStamped> grasp_poses;
  if (int32_t error = makeTargetPoses(*scene, grasp_point, grasp_poses); error != ThorpError::SUCCESS)
    return error;
  Eigen::Isometry3d grasp_pose;
  tf2::fromMsg(grasp_poses.front().pose, grasp_pose);
  // All the grasp poses have the same yaw, and the fingers close along the gripper's y axis
  const double lowering = top - widestSectionHeight(*object->shapes_.front(),
                                                    to_arm_base * object->global_shape_poses_.front(),
                                                    grasp_pose.linear().col(1));
  for (auto& pose : grasp_poses)
  {
    Eigen::Isometry3d gripper;
    tf2::fromMsg(pose.pose, gripper);
    gripper.translation().z() -= lowering;
    double lowest = std::numeric_limits<double>::max();
    for (const Eigen::Vector3d& corner : open_gripper_corners_)
      lowest = std::min(lowest, (gripper * corner).z());
    gripper.translation().z() += std::min(std::max(base + SUPPORT_CLEARANCE - lowest, 0.0), lowering);
    pose.pose = tf2::toMsg(gripper);
  }
  tf2::fromMsg(grasp_poses.front().pose, grasp_pose);
  double opening = graspOpening(yaw(grasp_pose), yaw(object_pose), tf2::toMsg2(object_size), goal.tightening);
  RCLCPP_INFO(node_->get_logger(), "Object size %.1f x %.1f x %.1f cm; grasp %.1f cm over its base, opening %.1f cm",
              object_size.x() * 100, object_size.y() * 100, object_size.z() * 100,
              (grasp_pose.translation().z() - base) * 100, opening * 100);

  mtc::Task task = makeTask("pickup " + goal.object_name, scene);
  mtc::Stage* current_state_stage = task[0];

  // The grasp poses come with the gripper open, so this opens it while moving the arm, merging both motions into a
  // single trajectory; if they can't be merged, it opens the gripper first
  auto move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", mtc::stages::Connect::GroupPlannerVector{ { GRIPPER_GROUP, interpolation_planner_ },
                                                                { ARM_GROUP, sampling_planner_ } });
  move_to_pick->setTimeout(5.0);
  mtc::TrajectoryExecutionInfo arm_and_gripper = executedBy(ARM_CONTROLLER);
  arm_and_gripper.controller_names.push_back(GRIPPER_CONTROLLER);
  move_to_pick->setTrajectoryExecutionInfo(arm_and_gripper);
  move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(move_to_pick));

  auto pick = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(pick->properties(), { "eef", "group" });
  pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner_);
    stage->setTrajectoryExecutionInfo(executedBy(ARM_CONTROLLER));
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(GRIPPER_LINK);
    stage->setMinMaxDistance(PICK_APPROACH_MIN, PICK_APPROACH_MAX);
    stage->setDirection(gripperDirection(+1.0));
    pick->insert(std::move(stage));
  }
  {
    auto generator = std::make_unique<OpenGripperPoses>("grasp poses");
    for (const auto& pose : grasp_poses)
      generator->addPose(pose);
    generator->setMonitoredStage(current_state_stage);
    auto stage = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(generator));
    stage->setMaxIKSolutions(4);
    stage->setMinSolutionDistance(0.5);
    stage->setIKFrame(GRIPPER_LINK);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    stage->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    pick->insert(std::move(stage));
  }
  {
    // The gripper can touch the object and its support surface while grasping and retreating; not before, as the
    // grasp pose IK takes its planning scene from the current state stage
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper contacts");
    stage->allowCollisions(goal.object_name, gripper_links_, true);
    if (!goal.support_surf.empty())
      stage->allowCollisions(goal.support_surf, gripper_links_, true);
    pick->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner_);
    stage->setTrajectoryExecutionInfo(executedBy(GRIPPER_CONTROLLER));
    stage->setGroup(GRIPPER_GROUP);
    stage->setGoal(std::map<std::string, double>{ { GRIPPER_JOINT, gripper_model_.angle(opening) } });
    pick->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(goal.object_name, GRIPPER_LINK);
    if (!goal.support_surf.empty())
      stage->allowCollisions(goal.object_name, goal.support_surf, true);
    pick->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
    stage->setTrajectoryExecutionInfo(executedBy(ARM_CONTROLLER));
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(GRIPPER_LINK);
    stage->setMinMaxDistance(PICK_APPROACH_MIN, PICK_APPROACH_MAX);
    stage->setDirection(gripperDirection(-1.0));
    pick->insert(std::move(stage));
  }
  if (!goal.support_surf.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid support contacts");
    stage->allowCollisions(goal.support_surf, gripper_links_, false);
    stage->allowCollisions(goal.object_name, goal.support_surf, false);
    pick->insert(std::move(stage));
  }
  task.add(std::move(pick));

  return run(task, feedback, canceling);
}

int32_t PickAndPlaceServer::place(const PlaceObject::Goal& goal, const Feedback& feedback, const Canceling& canceling)
{
  RCLCPP_INFO(node_->get_logger(), "Place object '%s' on support surface '%s' at [%.2f, %.2f, %.2f] (%s)",
              goal.object_name.c_str(), goal.support_surf.c_str(), goal.place_pose.pose.position.x,
              goal.place_pose.pose.position.y, goal.place_pose.pose.position.z,
              goal.place_pose.header.frame_id.c_str());
  planning_scene::PlanningScenePtr scene = getPlanningScene();
  if (!scene)
    return ThorpError::SERVER_NOT_AVAILABLE;

  // An empty object name means whatever object we are holding
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  scene->getCurrentState().getAttachedBodies(attached_bodies);
  if (attached_bodies.empty() || (!goal.object_name.empty() && goal.object_name != attached_bodies.front()->getName()))
  {
    RCLCPP_ERROR(node_->get_logger(), "Requested object not in the gripper: '%s'; attached object is '%s'",
                 goal.object_name.c_str(), attached_bodies.empty() ? "" : attached_bodies.front()->getName().c_str());
    return ThorpError::OBJECT_NOT_ATTACHED;
  }
  const std::string object_name = attached_bodies.front()->getName();

  if (!scene->knowsFrameTransform(goal.place_pose.header.frame_id))
  {
    RCLCPP_ERROR(node_->get_logger(), "Unknown place pose frame '%s'", goal.place_pose.header.frame_id.c_str());
    return MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  }
  Eigen::Isometry3d place_pose;
  tf2::fromMsg(goal.place_pose.pose, place_pose);
  place_pose = scene->getFrameTransform(arm_ref_frame_).inverse() *
               scene->getFrameTransform(goal.place_pose.header.frame_id) * place_pose;
  std::vector<geometry_msgs::msg::PoseStamped> gripper_poses;
  if (int32_t error = makeTargetPoses(*scene, place_pose.translation(), gripper_poses); error != ThorpError::SUCCESS)
    return error;
  // The place pose is the object's, so the gripper goes where it puts the object there, as it holds it; but not across
  // the fingers, as the arm, lacking a roll joint, only reaches poses yawed toward their position
  Eigen::Vector3d held_object = attached_bodies.front()->getPose().translation();
  held_object.y() = 0.0;
  for (auto& pose : gripper_poses)
  {
    Eigen::Isometry3d gripper;
    tf2::fromMsg(pose.pose, gripper);
    gripper.translation() -= gripper.linear() * held_object;
    pose.pose = tf2::toMsg(gripper);
  }

  std::vector<std::string> placing_links = gripper_links_;
  placing_links.push_back(object_name);

  mtc::Task task = makeTask("place " + object_name, scene);
  mtc::Stage* place_scene_stage = task[0];
  if (!goal.allowed_touch_objects.empty())
  {
    // The gripper and the object can touch them during the whole placement, as the place pose IK takes its planning
    // scene from here
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow touching objects");
    stage->allowCollisions(goal.allowed_touch_objects, placing_links, true);
    place_scene_stage = stage.get();
    task.add(std::move(stage));
  }

  auto move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place", mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, sampling_planner_ } });
  move_to_place->setTimeout(5.0);
  move_to_place->setTrajectoryExecutionInfo(executedBy(ARM_CONTROLLER));
  move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(move_to_place));

  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group" });
  place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach place", cartesian_planner_);
    stage->setTrajectoryExecutionInfo(executedBy(ARM_CONTROLLER));
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(GRIPPER_LINK);
    stage->setMinMaxDistance(PLACE_APPROACH_MIN, PLACE_APPROACH_MAX);
    stage->setDirection(gripperDirection(+1.0));
    place->insert(std::move(stage));
  }
  {
    auto generator = std::make_unique<mtc::stages::FixedCartesianPoses>("place poses");
    for (const auto& pose : gripper_poses)
      generator->addPose(pose);
    generator->setMonitoredStage(place_scene_stage);
    auto stage = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(generator));
    stage->setMaxIKSolutions(4);
    stage->setMinSolutionDistance(0.5);
    stage->setIKFrame(GRIPPER_LINK);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    stage->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(stage));
  }
  if (!goal.support_surf.empty())
  {
    // The gripper and the object can touch the support surface while releasing and retreating; not before, as the
    // place pose IK takes its planning scene from before the move to place, so place poses need some clearance
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow support contacts");
    stage->allowCollisions(goal.support_surf, gripper_links_, true);
    stage->allowCollisions(object_name, goal.support_surf, true);
    place->insert(std::move(stage));
  }
  {
    // Opening just enough, so the fingers don't hit objects around, as those already on the tray
    std::map<std::string, double> open;
    robot_model_->getJointModelGroup(GRIPPER_GROUP)->getVariableDefaultPositions("open", open);
    const double held = gripper_model_.opening(scene->getCurrentState().getVariablePosition(GRIPPER_JOINT));
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner_);
    stage->setTrajectoryExecutionInfo(executedBy(GRIPPER_CONTROLLER));
    stage->setGroup(GRIPPER_GROUP);
    stage->setGoal(std::map<std::string, double>{
        { GRIPPER_JOINT, std::max(gripper_model_.angle(held + RELEASE_MARGIN), open[GRIPPER_JOINT]) } });
    place->insert(std::move(stage));
  }
  {
    // The released object is still between the fingers until we retreat
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(object_name, GRIPPER_LINK);
    stage->allowCollisions(object_name, gripper_links_, true);
    place->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
    stage->setTrajectoryExecutionInfo(executedBy(ARM_CONTROLLER));
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setIKFrame(GRIPPER_LINK);
    stage->setMinMaxDistance(PLACE_APPROACH_MIN, PLACE_APPROACH_MAX);
    stage->setDirection(gripperDirection(-1.0));
    place->insert(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid contacts");
    stage->allowCollisions(object_name, gripper_links_, false);
    if (!goal.support_surf.empty())
    {
      stage->allowCollisions(goal.support_surf, gripper_links_, false);
      stage->allowCollisions(object_name, goal.support_surf, false);
    }
    if (!goal.allowed_touch_objects.empty())
      stage->allowCollisions(goal.allowed_touch_objects, placing_links, false);
    place->insert(std::move(stage));
  }
  task.add(std::move(place));

  return run(task, feedback, canceling);
}

planning_scene::PlanningScenePtr PickAndPlaceServer::getPlanningScene()
{
  if (!planning_scene_client_->wait_for_service(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "Service '%s' not available", planning_scene_client_->get_service_name());
    return nullptr;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                    moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
                                    moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
                                    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                    moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
                                    moveit_msgs::msg::PlanningSceneComponents::OBJECT_COLORS;
  auto future = planning_scene_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(), "Get planning scene timed out");
    return nullptr;
  }
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  scene->setPlanningSceneMsg(future.get()->scene);
  // Planners reject start states out of bounds, including velocities; the measured state often is, slightly, when a
  // joint has just moved or is pressing an object, as the simulated gripper does
  moveit::core::RobotState& state = scene->getCurrentStateNonConst();
  state.zeroVelocities();
  state.zeroAccelerations();
  state.enforceBounds();
  state.update();
  return scene;
}

int32_t PickAndPlaceServer::makeTargetPoses(const planning_scene::PlanningScene& scene, const Eigen::Vector3d& target,
                                            std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
  double operation_height =
      (scene.getFrameTransform(arm_ref_frame_).inverse() * scene.getFrameTransform(operation_plane_frame_))
          .translation()
          .z();
  for (int attempt = 0; attempt < attempts_; ++attempt)
  {
    std::string error;
    std::optional<geometry_msgs::msg::Pose> pose =
        makeTargetPose(tf2::toMsg(target), operation_height, compensations_, attempt, error);
    if (!pose)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid target [%.2f, %.2f, %.2f]: %s", target.x(), target.y(), target.z(),
                   error.c_str());
      return ThorpError::INVALID_TARGET_POSE;
    }
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = arm_ref_frame_;
    pose_stamped.pose = *pose;
    poses.push_back(pose_stamped);
  }
  return ThorpError::SUCCESS;
}

mtc::Task PickAndPlaceServer::makeTask(const std::string& name, const planning_scene::PlanningScenePtr& scene)
{
  mtc::Task task;
  task.setName(name);
  task.setRobotModel(robot_model_);
  task.setTimeout(planning_timeout_);
  task.setProperty("group", ARM_GROUP);
  task.setProperty("eef", GRIPPER_GROUP);
  task.add(std::make_unique<mtc::stages::FixedState>("current state", scene));
  return task;
}

int32_t PickAndPlaceServer::run(mtc::Task& task, const Feedback& feedback, const Canceling& canceling)
{
  feedback("planning");
  try
  {
    task.init();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Task '%s' initialization failed: %s", task.name().c_str(), e.what());
    return MoveItErrorCodes::FAILURE;
  }
  {
    std::lock_guard<std::mutex> lock(planning_task_mutex_);
    planning_task_ = &task;
  }
  task.plan(max_solutions_);
  {
    std::lock_guard<std::mutex> lock(planning_task_mutex_);
    planning_task_ = nullptr;
  }
  if (canceling())
    return MoveItErrorCodes::PREEMPTED;
  if (task.solutions().empty())
  {
    std::ostringstream explanation;
    task.explainFailure(explanation);
    RCLCPP_ERROR(node_->get_logger(), "Task '%s' planning failed:\n%s", task.name().c_str(),
                 explanation.str().c_str());
    for (const std::string& pair : failureContacts(task))
      RCLCPP_ERROR(node_->get_logger(), "In collision at %s", pair.c_str());
    return MoveItErrorCodes::PLANNING_FAILED;
  }
  const mtc::SolutionBase& solution = *task.solutions().front();
  task.introspection().publishSolution(solution);

  // Name the stage of each sub-trajectory with motion, to report the one being executed
  auto goal = ExecuteTaskSolution::Goal();
  solution.toMsg(goal.solution, &task.introspection());
  // MTC exports the full planning scene, not a diff, for sub-trajectories whose end scene isn't a child of their
  // start one. Applied on execution, it would revert the world to its state when planned, removing e.g. objects
  // detected meanwhile; the task changes the world only on its scene modifying stages, exported as diffs, so of
  // those full scenes we keep just the robot state
  for (auto& sub_trajectory : goal.solution.sub_trajectory)
  {
    if (sub_trajectory.scene_diff.is_diff)
      continue;
    moveit_msgs::msg::PlanningScene robot_state_diff;
    robot_state_diff.is_diff = true;
    robot_state_diff.robot_state = sub_trajectory.scene_diff.robot_state;
    robot_state_diff.robot_state.is_diff = true;
    sub_trajectory.scene_diff = robot_state_diff;
  }
  moveit_task_constructor_msgs::msg::TaskDescription description;
  task.introspection().fillTaskDescription(description);
  std::map<uint32_t, std::string> stage_names;
  for (const auto& stage : description.stages)
    stage_names[stage.id] = stage.name;
  auto sub_trajectory_stages = std::make_shared<std::vector<std::string>>();
  for (const auto& sub_trajectory : goal.solution.sub_trajectory)
    sub_trajectory_stages->push_back(sub_trajectory.trajectory.joint_trajectory.points.empty() ?
                                         "" :
                                         stage_names[sub_trajectory.info.stage_id]);
  // Execution feedback comes for every sub-trajectory, also those without motion, and can arrive after we stop
  // waiting for the result, so the callback owns its data and reports only stage changes
  auto last_stage = std::make_shared<std::string>();
  auto report_stage_from = [sub_trajectory_stages, last_stage, feedback](size_t first) {
    for (size_t i = first; i < sub_trajectory_stages->size(); ++i)
    {
      const std::string& stage = (*sub_trajectory_stages)[i];
      if (stage.empty())
        continue;
      if (stage != *last_stage)
        feedback(stage);
      *last_stage = stage;
      return;
    }
  };

  if (!execute_client_->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server '%s' not available; is move_group's ExecuteTaskSolution "
                 "capability loaded?", "execute_task_solution");
    return MoveItErrorCodes::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "Executing task '%s' (cost %.3f)", task.name().c_str(), solution.cost());
  report_stage_from(0);
  auto options = rclcpp_action::Client<ExecuteTaskSolution>::SendGoalOptions();
  options.feedback_callback = [report_stage_from](auto,
                                                  const std::shared_ptr<const ExecuteTaskSolution::Feedback> execution) {
    report_stage_from(execution->sub_id + 1);
  };
  auto goal_future = execute_client_->async_send_goal(goal, options);
  if (goal_future.wait_for(5s) != std::future_status::ready || !goal_future.get())
  {
    RCLCPP_ERROR(node_->get_logger(), "Task '%s' execution goal rejected", task.name().c_str());
    return MoveItErrorCodes::FAILURE;
  }
  auto goal_handle = goal_future.get();
  auto result_future = execute_client_->async_get_result(goal_handle);
  while (result_future.wait_for(100ms) != std::future_status::ready)
  {
    if (!rclcpp::ok())
    {
      RCLCPP_WARN(node_->get_logger(), "Shutting down while executing task '%s'", task.name().c_str());
      return MoveItErrorCodes::FAILURE;
    }
    if (canceling())
    {
      RCLCPP_WARN(node_->get_logger(), "Task '%s' canceled; stopping execution", task.name().c_str());
      execute_client_->async_cancel_goal(goal_handle).wait_for(5s);
      result_future.wait_for(5s);
      return MoveItErrorCodes::PREEMPTED;
    }
  }
  // The result is an exception if the client couldn't get it, as when shutting down
  try
  {
    result_future.get();
  }
  catch (const rclcpp_action::exceptions::UnawareGoalHandleError& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Task '%s' execution result lost: %s", task.name().c_str(), e.what());
    return MoveItErrorCodes::FAILURE;
  }
  const auto& result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result)
    return MoveItErrorCodes::FAILURE;
  if (result.result->error_code.val != MoveItErrorCodes::SUCCESS)
    RCLCPP_ERROR(node_->get_logger(), "Task '%s' execution failed: %s", task.name().c_str(),
                 moveit::core::errorCodeToString(result.result->error_code).c_str());
  return result.result->error_code.val;
}

std::string PickAndPlaceServer::errorText(int32_t code) const
{
  switch (code)
  {
    case ThorpError::OBJECT_NOT_FOUND:
      return "object not found";
    case ThorpError::OBJECT_SIZE_NOT_FOUND:
      return "object shape not found";
    case ThorpError::INVALID_TARGET_POSE:
      return "invalid target pose";
    case ThorpError::SERVER_NOT_AVAILABLE:
      return "server not available";
    case ThorpError::OBJECT_NOT_ATTACHED:
      return "requested object is not attached";
    case ThorpError::OBJECT_ATTACHED:
      return "already have an object attached";
    default:
      return moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(code));
  }
}

}  // namespace thorp::manipulation
