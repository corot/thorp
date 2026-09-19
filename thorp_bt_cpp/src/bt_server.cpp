#include "thorp_bt_cpp/bt_server.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iterator>
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
namespace
{
/**
 * @brief The blackboard keys a tree reads but never writes: what its caller has to provide.
 *
 * Derived rather than declared, because a <BehaviorTree> definition carries no interface of
 * its own -- ports are bound by whoever instantiates it with <SubTree>, and running one as a
 * root tree means nobody did. Whatever the tree writes itself (an output port, or a key one
 * node fills in for a later one) isn't the caller's to supply, hence the difference.
 *
 * Only the root subtree is walked: a nested subtree's ports are bound by its parent's
 * remapping rather than by our caller, and its keys live on its own blackboard.
 */
std::set<std::string> requiredInputs(const BT::Tree& tree)
{
  std::set<std::string> read, written, required;
  if (tree.subtrees.empty())
  {
    return required;
  }

  for (const auto& node : tree.subtrees.front()->nodes)
  {
    // through a const reference on purpose: TreeNode's non-const config() is protected, and
    // a TreeNode::Ptr would select exactly that one
    const BT::TreeNode& tree_node = *node;

    std::set<std::string> node_reads, node_writes;
    // getRemappedKey gives the blackboard key a port is bound to: "{key}", or the port's own
    // name for the "{=}" shorthand. A literal attribute is bound to no key and yields an
    // error, since it asks nothing of the caller. Asking BT.CPP rather than reading the
    // braces ourselves is what keeps this from quietly disagreeing with the parser that
    // built the tree -- it also accepts a bare "=" and tolerates surrounding spaces, and a
    // spelling we resolved differently would put the wrong key in the required set.
    for (const auto& [port, remapping] : tree_node.config().input_ports)
    {
      if (auto key = BT::TreeNode::getRemappedKey(port, remapping))
      {
        node_reads.emplace(key->data(), key->size());
      }
    }
    for (const auto& [port, remapping] : tree_node.config().output_ports)
    {
      if (auto key = BT::TreeNode::getRemappedKey(port, remapping))
      {
        node_writes.emplace(key->data(), key->size());
      }
    }

    read.insert(node_reads.begin(), node_reads.end());
    // Only a *pure* write counts as the tree producing a key. A bidirectional port reads the
    // key before it writes it back, so it doesn't produce anything: GetPoseListFront pops
    // from {poses} and PopPoseFromList writes the shortened list back, but the caller still
    // has to supply that list in the first place. Counting those as produced would let a
    // goal missing its list through, which is exactly the case that crashes.
    std::set_difference(node_writes.begin(), node_writes.end(), node_reads.begin(), node_reads.end(),
                        std::inserter(written, written.end()));
  }

  std::set_difference(read.begin(), read.end(), written.begin(), written.end(),
                      std::inserter(required, required.end()));
  return required;
}
}  // namespace

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
  reject_missing_inputs_ = pnh_.param("reject_missing_inputs", true);
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

  // Refuse to tick a tree whose inputs we can't satisfy. Most nodes read a port as
  // *getInput<T>(...), and dereferencing that when the key was never set isn't an exception
  // we could catch below -- in a release build it's undefined behaviour that tends to take
  // the whole server down, which for an agent that merely forgot an argument is a poor
  // trade. Note this can be stricter than the tree really needs: a node that treats a port
  // as optional (checking the Expected rather than dereferencing it) still shows up here as
  // required. The error says exactly which keys are missing, so the caller can pass them; if
  // it gets in the way, ~reject_missing_inputs turns it off.
  if (reject_missing_inputs_)
  {
    std::vector<std::string> missing;
    for (const auto& key : requiredInputs(*tree))
    {
      auto locked_any = blackboard->getAnyLocked(key);
      if (!locked_any || locked_any->empty())
      {
        missing.push_back(key);
      }
    }
    if (!missing.empty())
    {
      ROS_ERROR_STREAM_NAMED("bt_server", "Subtree '" << goal->subtree
                                                      << "' needs input(s) the goal "
                                                         "didn't provide: "
                                                      << nlohmann::json(missing).dump());
      result.success = false;
      result.json = nlohmann::json{ { "error", "missing required input(s)" }, { "missing_inputs", missing } }.dump();
      as_.setAborted(result);
      return;
    }
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
  // A node can throw for reasons the pre-flight check above can't anticipate: an input that
  // is present but unparseable for the type the port wants, a scripting error, a node's own
  // exception. None of those are worth killing the server over, so they abort this goal and
  // leave it serving the next one.
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
