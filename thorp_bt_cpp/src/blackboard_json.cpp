#include "thorp_bt_cpp/blackboard_json.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <typeindex>
#include <typeinfo>
#include <vector>

#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <behaviortree_cpp/utils/demangle_util.h>

// for parsePose: callers write poses in Thorp's short forms, "x;y;yaw;frame" and "x;y;z;roll;pitch;yaw;frame"
#include "thorp_bt_cpp/type_converters.hpp"

#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt
{
namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("blackboard_json");
}

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
nlohmann::json poseToJson(const geometry_msgs::msg::Pose& pose)
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

// Everything the caller sees is in the map frame; a pose in a sensor frame means nothing to
// it. Only the outgoing json is converted, not the blackboard.
geometry_msgs::msg::PoseStamped inMapFrame(const geometry_msgs::msg::PoseStamped& pose)
{
  if (pose.header.frame_id.empty() || pose.header.frame_id == "map")
  {
    return pose;
  }

  geometry_msgs::msg::PoseStamped in_map;
  if (!ttk::TF2::instance().transformPose("map", pose, in_map))
  {
    RCLCPP_WARN_STREAM(logger(), "Cannot transform pose from " << pose.header.frame_id
                                                               << " to map; reporting as is");
    return pose;
  }
  return in_map;
}

nlohmann::json poseToJson(const geometry_msgs::msg::PoseStamped& pose)
{
  const geometry_msgs::msg::PoseStamped in_map = inMapFrame(pose);
  nlohmann::json json = poseToJson(in_map.pose);
  json["frame"] = in_map.header.frame_id;
  return json;
}

// Tables are collision objects made of a single box; tabletop objects carry their meshes.
bool isTable(const moveit_msgs::msg::CollisionObject& object)
{
  return object.meshes.empty() && object.primitives.size() == 1 &&
         object.primitives.front().type == shape_msgs::msg::SolidPrimitive::BOX;
}

std::string colorFromMetadata(const moveit_msgs::msg::CollisionObject& object)
{
  const nlohmann::json metadata = nlohmann::json::parse(object.type.db, nullptr, false);
  return metadata.is_object() && metadata.contains("color") ? metadata["color"].get<std::string>() : std::string();
}

// A table as the trees use it: name, size, color and pose. Depth is the longest side, along the box x axis, and
// height the box thickness.
nlohmann::json tableToJson(const moveit_msgs::msg::CollisionObject& table)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = table.header;
  pose.pose = table.pose;

  const auto& dimensions = table.primitives.front().dimensions;
  nlohmann::json json;
  json["name"] = table.id;
  json["depth"] = dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
  json["width"] = dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
  json["height"] = dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
  json["color"] = colorFromMetadata(table);
  json["pose"] = poseToJson(pose);
  return json;
}

// A collision object as its name plus whatever the detector recorded in type.db, which today
// is the object's color. The geometry stays out: detection puts the objects themselves into the
// MoveIt planning scene, which is where it belongs and where it survives from one goal to the
// next, and repeating it here would go stale the moment anything moves. What the caller can't
// get anywhere else is what the detector decided to call them, and a name is all pickup_object,
// place_object and place_on_tray ever take.
nlohmann::json collisionObjectToJson(const moveit_msgs::msg::CollisionObject& object)
{
  if (isTable(object))
  {
    return tableToJson(object);
  }

  nlohmann::json json;
  json["name"] = object.id;
  if (const std::string color = colorFromMetadata(object); !color.empty())
  {
    json["color"] = color;
  }
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
  if (any.type() == typeid(uint16_t))  // Nav2's error codes
  {
    return any.cast<uint16_t>();
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
  if (any.type() == typeid(geometry_msgs::msg::PoseStamped))
  {
    return poseToJson(any.cast<geometry_msgs::msg::PoseStamped>());
  }
  if (any.type() == typeid(geometry_msgs::msg::Pose))
  {
    return poseToJson(any.cast<geometry_msgs::msg::Pose>());
  }
  if (any.type() == typeid(std::map<std::string, uint32_t>))
  {
    nlohmann::json json = nlohmann::json::object();
    for (const auto& [key, count] : any.cast<std::map<std::string, uint32_t>>())
    {
      json[key] = count;
    }
    return json;
  }
  if (any.type() == typeid(std::vector<geometry_msgs::msg::PoseStamped>))
  {
    nlohmann::json poses = nlohmann::json::array();
    for (const auto& pose : any.cast<std::vector<geometry_msgs::msg::PoseStamped>>())
    {
      poses.push_back(poseToJson(pose));
    }
    return poses;
  }
  if (any.type() == typeid(std::vector<moveit_msgs::msg::CollisionObject>))
  {
    nlohmann::json objects = nlohmann::json::array();
    for (const auto& object : any.cast<std::vector<moveit_msgs::msg::CollisionObject>>())
    {
      objects.push_back(collisionObjectToJson(object));
    }
    return objects;
  }
  if (any.type() == typeid(moveit_msgs::msg::CollisionObject))
  {
    return collisionObjectToJson(any.cast<moveit_msgs::msg::CollisionObject>());
  }
  return "<unsupported type: " + BT::demangle(any.type()) + ">";
}

// A pose as json: either a "x;y;yaw;frame" or "x;y;z;roll;pitch;yaw;frame" string, or the
// nested object this file emits (x, y, z, roll, pitch, yaw, frame).
//
// Both forms occur: a caller writing a goal by hand reaches for the string, and one feeding back
// a pose from an earlier result has the object, which is what lets capabilities compose.
//
// Returns nullopt for a shape that is neither, so the caller can report it rather than throw.
std::optional<geometry_msgs::msg::PoseStamped> poseFromJson(const nlohmann::json& json)
{
  if (json.is_string())
  {
    return thorp::bt::parsePose(json.get<std::string>());
  }
  if (!json.is_object())
  {
    return std::nullopt;
  }
  return ttk::createPose(json.value("x", 0.0), json.value("y", 0.0), json.value("z", 0.0),
                         json.value("roll", 0.0), json.value("pitch", 0.0), json.value("yaw", 0.0),
                         json.value("frame", std::string("map")));
}

// A table as tableToJson writes it: a box collision object named, sized and posed as given.
moveit_msgs::msg::CollisionObject tableFromJson(const nlohmann::json& json)
{
  moveit_msgs::msg::CollisionObject table;
  table.id = json.value("name", std::string());
  table.primitives.resize(1);
  table.primitives.front().type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives.front().dimensions = { json.value("depth", 0.0), json.value("width", 0.0),
                                          json.value("height", 0.0) };
  table.primitive_poses.resize(1);
  table.primitive_poses.front().orientation.w = 1.0;
  if (json.contains("pose"))
  {
    if (auto pose = poseFromJson(json["pose"]))
    {
      table.header = pose->header;
      table.pose = pose->pose;
    }
  }
  return table;
}

// Builds a structured json value as whatever type the port declared, and writes it to the
// blackboard. Returns false if we have no case for that type, or the json is the wrong shape
// for it. `type` comes from the entry BT.CPP created when the tree was built.
bool setStructured(BT::Blackboard& blackboard, const std::string& key, const nlohmann::json& value,
                   const std::type_index& type)
{
  if (type == typeid(geometry_msgs::msg::PoseStamped))
  {
    auto pose = poseFromJson(value);
    if (!pose)
    {
      return false;
    }
    blackboard.set(key, *pose);
    return true;
  }

  if (type == typeid(std::vector<geometry_msgs::msg::PoseStamped>))
  {
    if (!value.is_array())
    {
      return false;
    }
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (const auto& element : value)
    {
      auto pose = poseFromJson(element);  // a string as a literal attribute writes one, or an object as we emit one
      if (!pose)
      {
        return false;
      }
      poses.push_back(*pose);
    }
    blackboard.set(key, poses);
    return true;
  }

  if (type == typeid(std::map<std::string, uint32_t>))
  {
    if (!value.is_object())
    {
      return false;
    }
    std::map<std::string, uint32_t> counts;
    for (auto it = value.begin(); it != value.end(); ++it)
    {
      if (!it.value().is_number_unsigned())
      {
        return false;
      }
      counts[it.key()] = it.value().get<uint32_t>();
    }
    blackboard.set(key, counts);
    return true;
  }

  if (type == typeid(moveit_msgs::msg::CollisionObject))
  {
    if (!value.is_object())
    {
      return false;
    }
    blackboard.set(key, tableFromJson(value));
    return true;
  }

  return false;
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
    RCLCPP_WARN_STREAM(logger(), "Could not serialize blackboard key '" << key << "': " << e.what());
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
    RCLCPP_ERROR_STREAM(logger(), "Input json must be an object; got: " << json.dump());
    return;
  }

  for (auto it = json.begin(); it != json.end(); ++it)
  {
    const std::string& key = it.key();
    const nlohmann::json& value = it.value();

    // Poses as strings too: the trees parse strings in Nav2's pose format, not in the short forms callers write
    const BT::TypeInfo* string_info = value.is_string() ? blackboard.entryInfo(key) : nullptr;
    const bool pose_string = string_info && string_info->type() == typeid(geometry_msgs::msg::PoseStamped);

    if (value.is_object() || value.is_array() || pose_string)
    {
      // A structured value has to be built as its real type, which means knowing what that
      // type is. Ask the entry BT.CPP created for the port when it built the tree: no entry
      // means no port uses this key, so there's nothing to build it as.
      const BT::TypeInfo* info = blackboard.entryInfo(key);
      if (!info)
      {
        RCLCPP_WARN_STREAM(logger(), "Skipping input key '"
                                                     << key
                                                     << "': it is an object or array, and no port in this tree "
                                                        "uses that key, so there's no type to build it as");
      }
      else if (!setStructured(blackboard, key, value, info->type()))
      {
        RCLCPP_WARN_STREAM(logger(), "Skipping input key '" << key << "': can't build a "
                                                                        << BT::demangle(info->type()) << " out of "
                                                                        << value.dump());
      }
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
      RCLCPP_WARN_STREAM(logger(), "Requested output key '" << key << "' holds no value after the run");
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
