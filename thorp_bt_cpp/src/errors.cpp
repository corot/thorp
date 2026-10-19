#include "thorp_bt_cpp/errors.hpp"

#include <fstream>
#include <map>
#include <regex>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/exceptions.h>

namespace thorp::bt
{
namespace
{
/**
 * The error codes by name, read from their message definitions, so a code added there is known without touching this
 */
const std::map<std::string, int32_t>& codes()
{
  static const std::map<std::string, int32_t> CODES = []
  {
    std::map<std::string, int32_t> codes;
    const std::regex constant(R"(^\s*int32\s+([A-Z][A-Z0-9_]*)\s*=\s*(-?[0-9]+))");
    for (const auto& [package, message] :
         { std::pair{ "thorp_msgs", "ThorpError" }, std::pair{ "moveit_msgs", "MoveItErrorCodes" } })
    {
      std::ifstream file(ament_index_cpp::get_package_share_directory(package) + "/msg/" + message + ".msg");
      std::string line;
      std::smatch match;
      while (std::getline(file, line))
      {
        if (std::regex_search(line, match, constant))
          codes.emplace(match[1], std::stoi(match[2]));
      }
    }
    return codes;
  }();
  return CODES;
}
}  // namespace

int32_t errorCode(const std::string& name)
{
  const auto found = codes().find(name);
  if (found == codes().end())
    throw BT::RuntimeError("unknown error code '", name, "': no ThorpError nor MoveItErrorCodes constant");
  return found->second;
}

}  // namespace thorp::bt
