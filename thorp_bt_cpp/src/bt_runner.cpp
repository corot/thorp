#include "thorp_bt_cpp/bt_runner.hpp"

#include <fstream>
#include <mutex>
#include <set>

#include <ament_index_cpp/get_package_prefix.hpp>

// bt tools
#include <behaviortree_cpp/utils/shared_library.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <std_srvs/srv/trigger.hpp>

#include <thorp_toolkit/simulation.hpp>
namespace ttk = thorp::toolkit;

#include "thorp_bt_cpp/node_common.hpp"
#include "thorp_bt_cpp/service_call.hpp"

namespace thorp::bt
{
BT::BehaviorTreeFactory Runner::bt_factory_{};

Runner::Runner(const rclcpp::Node::SharedPtr& node) : node_(node)
{
}

BT::Blackboard::Ptr Runner::makeBlackboard(const rclcpp::Node::SharedPtr& node)
{
  using std::chrono::milliseconds;
  const double tick_rate = node->get_parameter_or("tick_rate", 10.0);
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  // Nav2's BT nodes wait for replies from servers within a tick, for up to half the tick period
  blackboard->set<milliseconds>("bt_loop_duration", milliseconds(static_cast<int>(1000.0 / tick_rate)));
  blackboard->set<milliseconds>("server_timeout", milliseconds(static_cast<int>(500.0 / tick_rate)));
  blackboard->set<milliseconds>("cancel_timeout", milliseconds(1000));
  // Tree creation fails if a server is not available after this long
  blackboard->set<milliseconds>("wait_for_service_timeout",
                                milliseconds(node->get_parameter_or("wait_for_service_timeout", 5000)));
  return blackboard;
}

void Runner::registerNav2Nodes()
{
  static std::once_flag registered;
  std::call_once(registered, [] {
    const std::string lib_dir = ament_index_cpp::get_package_prefix("nav2_behavior_tree") + "/lib/";
    for (const auto* plugin : { "nav2_navigate_to_pose_action_bt_node", "nav2_navigate_through_poses_action_bt_node",
                                "nav2_follow_path_action_bt_node", "nav2_spin_action_bt_node",
                                "nav2_back_up_action_bt_node" })
    {
      std::set<std::string> known;
      for (const auto& [id, builder] : bt_factory_.builders())
        known.insert(id);
      bt_factory_.registerFromPlugin(lib_dir + BT::SharedLibrary::getOSName(plugin));

      std::vector<std::string> added;
      for (const auto& [id, builder] : bt_factory_.builders())
        if (!known.count(id))
          added.push_back(id);
      for (const auto& id : added)
      {
        const BT::TreeNodeManifest manifest = bt_factory_.manifests().at(id);
        const BT::NodeBuilder builder = bt_factory_.builders().at(id);
        bt_factory_.unregisterBuilder(id);
        bt_factory_.registerBuilder(manifest, [builder](const std::string& name, const BT::NodeConfig& config) {
          shareRootEntries(config.blackboard);
          return builder(name, config);
        });
      }
    }
  });
}

bool Runner::loadTree()
{
  registerNav2Nodes();

  // on simulation, wait for the simulated time to start before loading and running the tree
  if (!node_->get_clock()->wait_until_started(rclcpp::Duration::from_seconds(60)))
  {
    RCLCPP_ERROR(node_->get_logger(), "No valid time after 60 seconds");
    return false;
  }

  app_name_ = node_->get_parameter_or<std::string>("app_name", node_->get_name());
  const std::string bt_filepath = node_->get_parameter_or<std::string>("bt_filepath", "");
  const std::string nodes_filepath = node_->get_parameter_or<std::string>("nodes_filepath", "");

  // dump to an XML file to load on Groot
  if (!nodes_filepath.empty())
  {
    std::ofstream ofs(nodes_filepath, std::ios::out | std::ios::trunc);
    if (!ofs)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to open file to write node models file: " << nodes_filepath);
      return false;
    }
    ofs << BT::writeTreeNodesModelXML(bt_factory_) << std::endl;
  }

  try
  {
    bt_factory_.registerBehaviorTreeFromFile(bt_filepath);
    bt_ = bt_factory_.createTree(app_name_, makeBlackboard(node_));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to load behavior tree " << bt_filepath << ": " << e.what());
    return false;
  }

  // init publishers for debugging bt
  if (node_->get_parameter_or("publish_bt", false))
  {
    bt_pub_groot_.emplace(*bt_);
    bt_pub_file_.emplace(*bt_, node_->get_parameter_or<std::string>("publish_bt_filepath",
                                                                     "/tmp/" + app_name_ + ".btlog"));
    bt_pub_topic_.emplace(*bt_, node_);
  }

  tick_rate_ = node_->get_parameter_or("tick_rate", 10.0);
  if (tick_rate_ <= 0)
  {
    RCLCPP_WARN(node_->get_logger(), "Tick rate %.2f is non-positive, defaulting to 10 Hz", tick_rate_);
    tick_rate_ = 10;
  }
  return true;
}

void Runner::waitForNavigation(const rclcpp::Duration& timeout)
{
  // Nav2's servers exist since configured, but reject goals until activated
  const std::string is_active = "/lifecycle_manager_navigation/is_active";
  const auto services = node_->get_service_names_and_types();
  if (services.find(is_active) == services.end())
    return;  // no navigation

  const rclcpp::Time time_start = node_->now();
  while (rclcpp::ok() && (node_->now() - time_start) < timeout)
  {
    const auto response = callService<std_srvs::srv::Trigger>(node_, is_active, std_srvs::srv::Trigger::Request());
    if (response && response->success)
      return;
    node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.5));
  }
  RCLCPP_ERROR_STREAM(node_->get_logger(), "Navigation not active after " << timeout.seconds() << " seconds");
}

void Runner::run()
{
  if (!bt_)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Behavior tree for " << app_name_ << " is not initialized");
    return;
  }

  // Wait start_delay seconds and on simulation for objects spawning to complete
  node_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(node_->get_parameter_or("start_delay", 1.0)));
  ttk::waitForObjectsSpawning(node_, rclcpp::Duration::from_seconds(60));
  waitForNavigation(rclcpp::Duration::from_seconds(60));

  // Callbacks run between ticks, never concurrently with them
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  const rclcpp::Time time_start = node_->now();
  using namespace std::chrono;
  auto tick_period = duration_cast<system_clock::duration>(duration<double>(1.0 / tick_rate_));
  auto status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && !BT::isStatusCompleted(status))
  {
    // Record the start time to subtract the time used on tick and ROS spin from the sleep time
    auto start_time = system_clock::now();

    try
    {
      status = bt_->tickOnce();
    }
    catch (const std::exception& e)
    {
      // e.g. a node missing an input; the tree can't go on
      RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
      bt_->haltTree();
      status = BT::NodeStatus::FAILURE;
      break;
    }
    executor.spin_some();

    // Sleep tick period minus elapsed time
    auto elapsed_time = system_clock::now() - start_time;
    auto sleep_time = tick_period - elapsed_time;
    if (sleep_time > system_clock::duration::zero())
    {
      bt_->sleep(sleep_time);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Missed desired tick rate of %.2fHz, most recent tick actually took %.2f seconds",
                           tick_rate_, duration_cast<duration<double>>(elapsed_time).count());
    }
  }

  const double completion_time = (node_->now() - time_start).seconds();
  RCLCPP_INFO(node_->get_logger(), "%s completed in %.2fs with status %s", app_name_.c_str(), completion_time,
              BT::toStr(status).c_str());
}

}  // namespace thorp::bt
