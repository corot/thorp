#include <behaviortree_cpp_v3/action_node.h>

#include "thorp_bt_cpp/node_register.hpp"
#include "thorp_bt_cpp/service_client_node.hpp"

#include <std_srvs/Empty.h>

namespace thorp::bt::actions
{
class ClearCostmaps : public BT::RosServiceNode<std_srvs::Empty>
{
public:
  ClearCostmaps(const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode<std_srvs::Empty>(node_name, conf)
  {
  }

  void sendRequest(RequestType& request) override
  {
  }

  BT::NodeStatus onResponse(const ResponseType& response) override
  {
    return BT::NodeStatus::SUCCESS;
  }

private:
  BT_REGISTER_NODE(ClearCostmaps);
};
}  // namespace thorp::bt::actions
