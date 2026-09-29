#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <thorp_msgs/action/run_subtree.hpp>

namespace thorp::bt
{
/**
 * @brief Loads every behavior tree defined under thorp_bt_cpp/bt and serves them, one at a
 * time, through the RunSubtree action: the goal picks a tree by its <BehaviorTree ID="...">
 * and provides its inputs as a flat json object; the result reports success/failure plus
 * the requested output keys, or else whichever blackboard keys the run added, again as a flat
 * json object.
 *
 * Unlike Runner (bt_runner_node), which loads and ticks a single, fixed tree for the
 * lifetime of the process, Server keeps every tree registered in the shared node factory
 * (Runner::bt_factory_) and creates -- and discards -- a fresh tree, with its own
 * blackboard, per incoming goal.
 */
class Server
{
public:
  explicit Server(const rclcpp::Node::SharedPtr& node);
  ~Server();

  /**
   * @brief Register every *.xml tree file under the bt_dir parameter (except the generated
   * node_models.xml) with the shared node factory.
   *
   * Some files (e.g. navigation.xml, manipulation.xml) are only ever meant to be pulled in
   * through another file's <include>, but also exist standalone in the same directory, so
   * the same tree id ends up registered more than once as we go through every file. That's
   * harmless: BT.CPP just keeps the last (identical) definition it sees.
   */
  bool loadTrees();

  /** Start the RunSubtree action server, on ~/run_subtree. */
  void start();

private:
  using RunSubtree = thorp_msgs::action::RunSubtree;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunSubtree>;

  rclcpp::Node::SharedPtr node_;

  /** Directory containing the *.xml behavior tree files to load. */
  std::string bt_dir_;

  /** Tick rate used while running a subtree. */
  double tick_rate_ = 10.0;

  rclcpp_action::Server<RunSubtree>::SharedPtr as_;

  /** Subtrees run one at a time, each on its own thread */
  std::atomic<bool> busy_{ false };
  std::thread goal_thread_;

  enum class Outcome
  {
    SUCCEEDED,
    CANCELED,
    ABORTED
  };

  /** Run the goal's subtree, discarding it before returning the result to report. */
  std::pair<Outcome, RunSubtree::Result::SharedPtr> execute(const std::shared_ptr<GoalHandle> goal_handle);
};

}  // namespace thorp::bt
