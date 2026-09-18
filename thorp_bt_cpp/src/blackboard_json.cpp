#include "thorp_bt_cpp/blackboard_json.hpp"

#include <cstdint>
#include <typeinfo>
#include <vector>

#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <behaviortree_cpp/utils/demangle_util.h>

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt
{
namespace
{
// Renders a json scalar the same way it would look as a literal XML attribute, so it can
// be parsed back by whatever BT::convertFromString<T> a port applies to it.
std::string scalarToBTString(const nlohmann::json& value)
{
  if (value.is_string())
  {
    return value.get<std::string>();
  }
  if (value.is_boolean())
  {
    return value.get<bool>() ? "true" : "false";
  }
  // dump() renders numbers with the shortest round-trippable representation
  return value.dump();
}

// Poses go out as an object with the orientation as roll/pitch/yaw rather than a quaternion:
// the consumer is an agent reasoning about where the robot is, and rpy is what's legible to
// it. The shape is always the same seven fields, even for the planar poses where z/roll/pitch
// are all zero, so that whoever reads this doesn't have to handle two different pose shapes.
// Note this isn't quite the inverse of BT::convertFromString<PoseStamped>, which takes the
// same values ';'-separated rather than as an object.
nlohmann::json poseToJson(const geometry_msgs::Pose& pose)
{
  // quaternion to rpy readily yields -0.0, which is valid json but reads oddly
  auto tidy = [](double value) { return value == 0.0 ? 0.0 : value; };

  nlohmann::json json;
  json["x"] = tidy(pose.position.x);
  json["y"] = tidy(pose.position.y);
  json["z"] = tidy(pose.position.z);
  json["roll"] = tidy(ttk::roll(pose));
  json["pitch"] = tidy(ttk::pitch(pose));
  json["yaw"] = tidy(ttk::yaw(pose));
  return json;
}

nlohmann::json poseToJson(const geometry_msgs::PoseStamped& pose)
{
  nlohmann::json json = poseToJson(pose.pose);
  json["frame"] = pose.header.frame_id;
  return json;
}

// Serializes a single blackboard value, or a "<unsupported type: ...>" tag if we have no
// case for its type. Assumes the entry is not empty (callers only walk entries with a value).
nlohmann::json anyToJson(const BT::Any& any)
{
  if (any.isString())
  {
    return any.cast<std::string>();
  }
  if (any.type() == typeid(bool))
  {
    return any.cast<bool>();
  }
  if (any.type() == typeid(int))
  {
    return any.cast<int>();
  }
  if (any.type() == typeid(unsigned))
  {
    return any.cast<unsigned>();
  }
  if (any.type() == typeid(int64_t))
  {
    return any.cast<int64_t>();
  }
  if (any.type() == typeid(uint64_t))
  {
    return any.cast<uint64_t>();
  }
  if (any.type() == typeid(float))
  {
    return any.cast<float>();
  }
  if (any.type() == typeid(double))
  {
    return any.cast<double>();
  }
  if (any.type() == typeid(geometry_msgs::PoseStamped))
  {
    return poseToJson(any.cast<geometry_msgs::PoseStamped>());
  }
  if (any.type() == typeid(geometry_msgs::Pose))
  {
    return poseToJson(any.cast<geometry_msgs::Pose>());
  }
  if (any.type() == typeid(std::vector<geometry_msgs::PoseStamped>))
  {
    nlohmann::json poses = nlohmann::json::array();
    for (const auto& pose : any.cast<std::vector<geometry_msgs::PoseStamped>>())
    {
      poses.push_back(poseToJson(pose));
    }
    return poses;
  }
  return "<unsupported type: " + BT::demangle(any.type()) + ">";
}

// Adds `key` to `json` if the blackboard holds a value for it. False means there is no such
// entry, or it exists but was never written to (BT.CPP creates an empty entry for every port
// the tree remaps, so plenty of entries never receive a value).
bool addEntry(const BT::Blackboard& blackboard, const std::string& key, nlohmann::json& json)
{
  auto locked_any = blackboard.getAnyLocked(key);
  if (!locked_any || locked_any->empty())
  {
    return false;
  }

  try
  {
    json[key] = anyToJson(*locked_any.get());
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM_NAMED("blackboard_json", "Could not serialize blackboard key '" << key << "': " << e.what());
    json[key] = "<unreadable>";
  }
  return true;
}
}  // namespace

void blackboardFromJson(const nlohmann::json& json, BT::Blackboard& blackboard)
{
  if (json.is_null())
  {
    return;  // no input provided; nothing to seed
  }
  if (!json.is_object())
  {
    ROS_ERROR_STREAM_NAMED("blackboard_json", "Input json must be an object; got: " << json.dump());
    return;
  }

  for (auto it = json.begin(); it != json.end(); ++it)
  {
    const std::string& key = it.key();
    const nlohmann::json& value = it.value();
    if (value.is_object() || value.is_array())
    {
      ROS_WARN_STREAM_NAMED("blackboard_json",
                            "Skipping input key '" << key << "': nested objects/arrays are not supported yet");
      continue;
    }

    blackboard.set<std::string>(key, scalarToBTString(value));
  }
}

nlohmann::json entriesToJson(const BT::Blackboard& blackboard, const std::vector<std::string>& keys)
{
  nlohmann::json json = nlohmann::json::object();
  for (const auto& key : keys)
  {
    if (!addEntry(blackboard, key, json))
    {
      ROS_WARN_STREAM_NAMED("blackboard_json", "Requested output key '" << key << "' holds no value after the run");
    }
  }
  return json;
}

std::set<std::string> valuedKeys(const BT::Blackboard& blackboard)
{
  std::set<std::string> keys;
  for (const auto& key_view : blackboard.getKeys())
  {
    const std::string key(key_view);
    auto locked_any = blackboard.getAnyLocked(key);
    if (locked_any && !locked_any->empty())
    {
      keys.insert(key);
    }
  }
  return keys;
}

nlohmann::json newEntriesToJson(const BT::Blackboard& blackboard, const std::set<std::string>& known_keys)
{
  nlohmann::json json = nlohmann::json::object();

  for (const auto& key_view : blackboard.getKeys())
  {
    const std::string key(key_view);
    if (!known_keys.count(key))  // skip what already had a value before the run
    {
      addEntry(blackboard, key, json);
    }
  }
  return json;
}

}  // namespace thorp::bt
