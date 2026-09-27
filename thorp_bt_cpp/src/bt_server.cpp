#include "thorp_bt_cpp/bt_server.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

#include <boost/bind.hpp>

#include "thorp_bt_cpp/bt_runner.hpp"  // owner of the shared, static bt_factory_
#include "thorp_bt_cpp/blackboard_json.hpp"

namespace fs = std::filesystem;

namespace thorp::bt
{
Server::Server() : pnh_("~"), as_(pnh_, "run_subtree", boost::bind(&Server::executeCB, this, _1), false)
{
}

bool Server::loadTrees()
{
  if (!pnh_.getParam("bt_dir", bt_dir_))
  {
    ROS_ERROR_STREAM_NAMED("bt_server", "Missing required parameter: bt_dir");
    return false;
  }
  tick_rate_ = pnh_.param("tick_rate", 10.0);
  if (tick_rate_ <= 0)
  {
    ROS_WARN_NAMED("bt_server", "Tick rate %.2f is non-positive, defaulting to 10 Hz", tick_rate_);
    tick_rate_ = 10;
  }

  std::error_code ec;
  std::vector<fs::path> tree_files;
  for (const auto& entry : fs::directory_iterator(bt_dir_, ec))
  {
    if (entry.path().extension() != ".xml")
    {
      continue;
    }
    if (entry.path().filename() == "node_models.xml")
    {
      continue;  // generated output (for Groot), not a tree source
    }
    tree_files.push_back(entry.path());
  }
  if (ec)
  {
    ROS_ERROR_STREAM_NAMED("bt_server", "Cannot list bt_dir '" << bt_dir_ << "': " << ec.message());
    return false;
  }
  if (tree_files.empty())
  {
    ROS_ERROR_STREAM_NAMED("bt_server", "No behavior tree xml files found in " << bt_dir_);
    return false;
  }
  std::sort(tree_files.begin(), tree_files.end());

  for (const auto& file : tree_files)
  {
    try
    {
      Runner::bt_factory_.registerBehaviorTreeFromFile(file);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM_NAMED("bt_server", "Failed to load " << file << ": " << e.what());
      return false;
    }
  }

  const auto tree_ids = Runner::bt_factory_.registeredBehaviorTrees();
  std::stringstream ids;
  for (const auto& id : tree_ids)
  {
    ids << id << " ";
  }
  ROS_INFO_STREAM_NAMED("bt_server", "Loaded " << tree_files.size() << " file(s) from " << bt_dir_ << ": "
                                               << tree_ids.size() << " subtree(s) available: " << ids.str());
  return true;
}

void Server::run()
{
  as_.start();
  ROS_INFO_NAMED("bt_server", "Ready to run subtrees on action '%s'", pnh_.resolveName("run_subtree").c_str());
  ros::spin();
}

void Server::executeCB(const thorp_msgs::RunSubtreeGoalConstPtr& goal)
{
  thorp_msgs::RunSubtreeResult result;

  nlohmann::json input_json;
  try
  {
    input_json = goal->json.empty() ? nlohmann::json::object() : nlohmann::json::parse(goal->json);
  }
  catch (const nlohmann::json::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("bt_server", "Invalid input json for subtree '" << goal->subtree << "': " << e.what());
    result.success = false;
    result.json = nlohmann::json{ { "error", std::string("invalid input json: ") + e.what() } }.dump();
    as_.setAborted(result);
    return;
  }

  std::optional<BT::Tree> tree;
  try
  {
    tree = Runner::bt_factory_.createTree(goal->subtree);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("bt_server", "Cannot create subtree '" << goal->subtree << "': " << e.what());
    result.success = false;
    result.json = nlohmann::json{ { "error", std::string("unknown subtree: ") + e.what() } }.dump();
    as_.setAborted(result);
    return;
  }

  auto blackboard = tree->rootBlackboard();
  try
  {
    blackboardFromJson(input_json, *blackboard);
  }
  catch (const std::exception& e)
  {
    // Seeding turns json into real typed values, and a value of the wrong shape throws from
    // inside nlohmann or a BT converter rather than returning an error. Uncaught, that ends
    // the process: one badly shaped argument and every later goal dies with it, which for a
    // server whose callers are LLMs writing json is not a question of if. A refusal names the
    // problem and leaves the server standing.
    ROS_ERROR_STREAM_NAMED("bt_server", "Cannot seed inputs for subtree '" << goal->subtree << "': " << e.what());
    result.success = false;
    result.json = nlohmann::json{ { "error", std::string("cannot seed inputs: ") + e.what() } }.dump();
    as_.setAborted(result);
    return;
  }

  // Normally the caller names the keys it wants back, and we just read those once the run is
  // over. Without them we fall back to reporting whatever the run added, which needs a
  // snapshot of what already holds a value now that the inputs are seeded. That snapshot
  // can't be a plain list of keys: createTree() already created an (empty) entry for every
  // port the tree remaps, so key existence means nothing here, only holding a value does.
  const bool report_new_entries = goal->output_keys.empty();
  const std::set<std::string> keys_before_run = report_new_entries ? valuedKeys(*blackboard) : std::set<std::string>{};

  auto collectOutputs = [&]
  {
    return report_new_entries ? newEntriesToJson(*blackboard, keys_before_run) :
                                entriesToJson(*blackboard, goal->output_keys);
  };

  ROS_INFO_STREAM_NAMED("bt_server", "Running subtree '" << goal->subtree << "' with input " << input_json.dump());

  using namespace std::chrono;
  const auto tick_period = duration_cast<system_clock::duration>(duration<double>(1.0 / tick_rate_));
  auto status = BT::NodeStatus::RUNNING;
  // Inputs aren't checked before the run: a node finds a missing or unparseable one when it
  // reads it, and requireInput throws. That, a scripting error or a node's own exception all
  // abort this goal with the message and leave the server serving the next one.
  try
  {
    while (ros::ok() && !BT::isStatusCompleted(status))
    {
      if (as_.isPreemptRequested())
      {
        ROS_WARN_STREAM_NAMED("bt_server", "Subtree '" << goal->subtree << "' preempted");
        tree->haltTree();
        result.success = false;
        result.json = collectOutputs().dump();
        as_.setPreempted(result);
        return;
      }

      const auto start_time = system_clock::now();
      status = tree->tickOnce();
      ros::spinOnce();

      const auto elapsed_time = system_clock::now() - start_time;
      const auto sleep_time = tick_period - elapsed_time;
      if (sleep_time > system_clock::duration::zero())
      {
        tree->sleep(sleep_time);
      }
      else
      {
        ROS_WARN_THROTTLE_NAMED(1.0, "bt_server",
                                "Missed desired tick rate of %.2fHz, most recent tick actually took %.2f seconds",
                                tick_rate_, duration_cast<duration<double>>(elapsed_time).count());
      }
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("bt_server", "Subtree '" << goal->subtree << "' threw: " << e.what());
    try
    {
      tree->haltTree();
    }
    catch (const std::exception& halt_error)
    {
      ROS_ERROR_STREAM_NAMED("bt_server", "Halting '" << goal->subtree << "' threw too: " << halt_error.what());
    }
    result.success = false;
    result.json = nlohmann::json{ { "error", std::string("subtree threw: ") + e.what() } }.dump();
    as_.setAborted(result);
    return;
  }

  result.success = status == BT::NodeStatus::SUCCESS;
  result.json = collectOutputs().dump();
  ROS_INFO_STREAM_NAMED("bt_server", "Subtree '" << goal->subtree << "' completed with status " << BT::toStr(status)
                                                 << ", output " << result.json);
  as_.setSucceeded(result);
}

}  // namespace thorp::bt
