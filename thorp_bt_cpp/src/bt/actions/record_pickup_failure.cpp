#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"

namespace thorp::bt::actions
{
/**
 * Increase by one the number of picking failures for a given object.
 * If failures reach picking_max_failures, increase by one the number of given up objects.
 * Always returns SUCCESS
 */
class RecordPickupFailure : public BT::SyncActionNode
{
public:
  RecordPickupFailure(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("target_name"),                           //
             BT::BidirectionalPort<std::map<std::string, uint32_t>>("failures"),  //
             BT::BidirectionalPort<uint32_t>("given_up_count") };
  }

private:
  BT::NodeStatus tick() override
  {
    const auto target_name = *getInput<std::string>("target_name");
    auto failures = getInput<std::map<std::string, uint32_t>>("failures");
    ROS_ERROR_STREAM_COND(!failures, "empty");
    ROS_ERROR_STREAM_COND(failures && failures->find(target_name) == failures->end(), "not found");
    ROS_ERROR_STREAM_COND(failures && failures->find(target_name) != failures->end(),
                          " found  " << failures->find(target_name)->second);
    if (!failures)
      failures = std::map<std::string, uint32_t>{ { target_name, 1 } };
    else if (auto entry = failures->find(target_name); entry == failures->end())
      failures->insert({ target_name, 1 });
    else
      entry->second++;

    setOutput("failures", *failures);
    ROS_INFO_STREAM_NAMED(name(), "Pickup " << target_name << " failed " << failures->at(target_name) << " time(s)");

    const auto max_failures = ros::NodeHandle("~").param("picking_max_failures", 3);
    const auto failures_count = failures->find(target_name)->second;
    ROS_ASSERT_MSG(failures_count <= max_failures, "More failures than allowed?");
    if (failures_count == max_failures)
    {
      const auto given_up_count = getInput<uint32_t>("given_up_count");
      setOutput("given_up_count", given_up_count ? *given_up_count + 1 : 1);
      ROS_ERROR_STREAM("GIVEN UP " << (given_up_count ? *given_up_count + 1 : 1));
    }

    return BT::NodeStatus::SUCCESS;
  };

  BT_REGISTER_NODE(RecordPickupFailure);
};
}  // namespace thorp::bt::actions