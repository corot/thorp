#pragma once

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <behaviortree_cpp/bt_factory.h>

#include <thorp_msgs/RunSubtreeAction.h>

namespace thorp::bt
{
/**
 * @brief Loads every behavior tree defined under thorp_bt_cpp/bt and serves them, one at a
 * time, through the RunSubtree action: the goal picks a tree by its <BehaviorTree ID="...">
 * and provides its inputs as a flat json object; the result reports success/failure plus
 * whichever blackboard keys the run added, again as a flat json object; feedback carries the
 * same growing set of keys while the subtree is still running.
 *
 * Unlike Runner (bt_runner_node), which loads and ticks a single, fixed tree for the
 * lifetime of the process, Server keeps every tree registered in the shared node factory
 * (Runner::bt_factory_) and creates -- and discards -- a fresh tree, with its own
 * blackboard, per incoming goal.
 */
class Server
{
public:
  Server();

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

  /** Start the RunSubtree action server and spin until shutdown. */
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  /** Directory containing the *.xml behavior tree files to load. */
  std::string bt_dir_;

  /** Tick rate used while running a subtree. */
  double tick_rate_ = 10.0;

  /** Refuse a goal that doesn't provide every input the requested tree needs, rather than
   *  running it and letting some node dereference a port that was never set. See executeCB. */
  bool reject_missing_inputs_ = true;

  actionlib::SimpleActionServer<thorp_msgs::RunSubtreeAction> as_;

  void executeCB(const thorp_msgs::RunSubtreeGoalConstPtr& goal);
};

}  // namespace thorp::bt
