// Template specialization of convertFromString for commonly used types

#include "behaviortree_cpp/behavior_tree.h"

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace BT
{
template <>
inline geometry_msgs::PoseStamped convertFromString(StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() == 4)
  {
    return ttk::createPose(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]),
                           convertFromString<double>(parts[2]), std::string(parts[3]));
  }
  if (parts.size() == 7)
  {
    return ttk::createPose(convertFromString<double>(parts[0]), convertFromString<double>(parts[1]),
                           convertFromString<double>(parts[2]), convertFromString<double>(parts[3]),
                           convertFromString<double>(parts[4]), convertFromString<double>(parts[5]),
                           std::string(parts[6]));
  }
  else
  {
    throw RuntimeError("Invalid number of inputs to build a PoseStamped)");
  }
}
}  // end namespace BT