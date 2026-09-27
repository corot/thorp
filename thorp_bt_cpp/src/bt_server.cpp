#include "thorp_bt_cpp/bt_server.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

#include <thread>

#include "thorp_bt_cpp/bt_runner.hpp"  // owner of the shared, static bt_factory_
#include "thorp_bt_cpp/blackboard_json.hpp"

namespace fs = std::filesystem;

namespace thorp::bt
{
Server::Server(const rclcpp::Node::SharedPtr& node) : node_(node)
{
}

bool Server::loadTrees()
{
  if (!node_->get_parameter("bt_dir", bt_dir_))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Missing required parameter: bt_dir");
    return false;
  }
  tick_rate_ = node_->get_parameter_or("tick_rate", 10.0);
  if (tick_rate_ <= 0)
  {
    RCLCPP_WARN(node_->get_logger(), "Tick rate %.2f is non-positive, defaulting to 10 Hz", tick_rate_);
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
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Cannot list bt_dir '" << bt_dir_ << "': " << ec.message());
    return false;
  }
  if (tree_files.empty())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "No behavior tree xml files found in " << bt_dir_);
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
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to load " << file << ": " << e.what());
      return false;
    }
  }

  const auto tree_ids = Runner::bt_factory_.registeredBehaviorTrees();
  std::stringstream ids;
  for (const auto& id : tree_ids)
  {
    ids << id << " ";
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded " << tree_files.size() << " file(s) from " << bt_dir_ << ": "
                                               << tree_ids.size() << " subtree(s) available: " << ids.str());
  return true;
}

void Server::start()
{
  as_ = rclcpp_action::create_server<RunSubtree>(
      node_, "~/run_subtree",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const RunSubtree::Goal> goal) {
        if (busy_.exchange(true))
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Rejecting subtree '" << goal->subtree << "': already running one");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread([this, goal_handle]() {
          execute(goal_handle);
          busy_ = false;
        }).detach();
      });
  RCLCPP_INFO(node_->get_logger(), "Ready to run subtrees on action '%s/run_subtree'",
              node_->get_fully_qualified_name());
}

void Server::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<RunSubtree::Result>();

  nlohmann::json input_json;
  try
  {
    input_json = goal->json.empty() ? nlohmann::json::object() : nlohmann::json::parse(goal->json);
  }
  catch (const nlohmann::json::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid input json for subtree '" << goal->subtree << "': " << e.what());
    result->success = false;
    result->json = nlohmann::json{ { "error", std::string("invalid input json: ") + e.what() } }.dump();
    goal_handle->abort(result);
    return;
  }

  std::optional<BT::Tree> tree;
  try
  {
    tree = Runner::bt_factory_.createTree(goal->subtree, Runner::makeBlackboard(node_));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Cannot create subtree '" << goal->subtree << "': " << e.what());
    result->success = false;
    result->json = nlohmann::json{ { "error", std::string("unknown subtree: ") + e.what() } }.dump();
    goal_handle->abort(result);
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
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Cannot seed inputs for subtree '" << goal->subtree << "': " << e.what());
    result->success = false;
    result->json = nlohmann::json{ { "error", std::string("cannot seed inputs: ") + e.what() } }.dump();
    goal_handle->abort(result);
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

  RCLCPP_INFO_STREAM(node_->get_logger(), "Running subtree '" << goal->subtree << "' with input " << input_json.dump());

  using namespace std::chrono;
  const auto tick_period = duration_cast<system_clock::duration>(duration<double>(1.0 / tick_rate_));
  auto status = BT::NodeStatus::RUNNING;
  // Inputs aren't checked before the run: a node finds a missing or unparseable one when it
  // reads it, and requireInput throws. That, a scripting error or a node's own exception all
  // abort this goal with the message and leave the server serving the next one.
  try
  {
    while (rclcpp::ok() && !BT::isStatusCompleted(status))
    {
      if (goal_handle->is_canceling())
      {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Subtree '" << goal->subtree << "' preempted");
        tree->haltTree();
        result->success = false;
        result->json = collectOutputs().dump();
        goal_handle->canceled(result);
        return;
      }

      const auto start_time = system_clock::now();
      status = tree->tickOnce();

      const auto elapsed_time = system_clock::now() - start_time;
      const auto sleep_time = tick_period - elapsed_time;
      if (sleep_time > system_clock::duration::zero())
      {
        tree->sleep(sleep_time);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Missed desired tick rate of %.2fHz, most recent tick actually took %.2f seconds",
                             tick_rate_, duration_cast<duration<double>>(elapsed_time).count());
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Subtree '" << goal->subtree << "' threw: " << e.what());
    try
    {
      tree->haltTree();
    }
    catch (const std::exception& halt_error)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Halting '" << goal->subtree << "' threw too: " << halt_error.what());
    }
    result->success = false;
    result->json = nlohmann::json{ { "error", std::string("subtree threw: ") + e.what() } }.dump();
    goal_handle->abort(result);
    return;
  }

  result->success = status == BT::NodeStatus::SUCCESS;
  result->json = collectOutputs().dump();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Subtree '" << goal->subtree << "' completed with status "
                                                       << BT::toStr(status) << ", output " << result->json);
  goal_handle->succeed(result);
}

}  // namespace thorp::bt
