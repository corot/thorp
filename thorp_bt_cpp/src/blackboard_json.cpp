#include "thorp_bt_cpp/blackboard_json.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <typeindex>
#include <typeinfo>
#include <vector>

#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <rail_manipulation_msgs/SegmentedObject.h>

#include <behaviortree_cpp/utils/demangle_util.h>

// for convertFromString<PoseStamped>: a pose arrives in exactly the same "x;y;yaw;frame" form
// whether it came from a literal xml attribute or from a goal's json, and parsing it in one
// place is what keeps those two from drifting
#include "thorp_bt_cpp/type_converters.hpp"

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
  if (any.type() == typeid(std::map<std::string, uint32_t>))
  {
    nlohmann::json json = nlohmann::json::object();
    for (const auto& [key, count] : any.cast<std::map<std::string, uint32_t>>())
    {
      json[key] = count;
    }
    return json;
  }
  if (any.type() == typeid(rail_manipulation_msgs::SegmentedObject))
  {
    // the same fields seeding accepts, so a table can round-trip
    const auto object = any.cast<rail_manipulation_msgs::SegmentedObject>();
    geometry_msgs::Pose pose;
    pose.position = object.center;
    pose.orientation = object.orientation;

    nlohmann::json json;
    json["name"] = object.name;
    json["width"] = object.width;
    json["depth"] = object.depth;
    json["height"] = object.height;
    json["pose"] = poseToJson(pose);
    return json;
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
  if (any.type() == typeid(std::vector<moveit_msgs::CollisionObject>))
  {
    // Names only, on purpose. Detection puts the objects themselves into the MoveIt planning
    // scene, which is where their geometry belongs and where it survives from one goal to the
    // next; serialising it here would duplicate that, and go stale the moment anything moves.
    // What the caller genuinely can't get anywhere else is what the detector decided to call
    // them, and a name is all pickup_object, place_object and place_on_tray ever take.
    nlohmann::json names = nlohmann::json::array();
    for (const auto& object : any.cast<std::vector<moveit_msgs::CollisionObject>>())
    {
      names.push_back(object.id);
    }
    return names;
  }
  if (any.type() == typeid(moveit_msgs::CollisionObject))
  {
    return any.cast<moveit_msgs::CollisionObject>().id;
  }
  return "<unsupported type: " + BT::demangle(any.type()) + ">";
}

// A pose as json: either the "x;y;yaw;frame" string a literal xml attribute would use, or the
// nested object this file emits (x, y, z, roll, pitch, yaw, frame).
//
// Both forms are here because both occur. A caller writing a goal by hand reaches for the
// string; a caller feeding back a pose that came out of an earlier run has the object, because
// that is what we gave them. Accepting only the string made every pose we emit unusable as an
// input, which breaks the one thing capabilities are for: detect_table's table_pose could not
// be handed to poses_around_table, and its table could not be handed to anything at all.
//
// Returns nullopt for a shape that is neither, so the caller can report it rather than throw.
std::optional<geometry_msgs::PoseStamped> poseFromJson(const nlohmann::json& json)
{
  if (json.is_string())
  {
    return BT::convertFromString<geometry_msgs::PoseStamped>(json.get<std::string>());
  }
  if (!json.is_object())
  {
    return std::nullopt;
  }
  return ttk::createPose(json.value("x", 0.0), json.value("y", 0.0), json.value("z", 0.0),
                         json.value("roll", 0.0), json.value("pitch", 0.0), json.value("yaw", 0.0),
                         json.value("frame", std::string("map")));
}

// A table as the trees actually use it. They read width/depth (to judge whether the table is
// a usable size, and to work out poses around it), name, and the bounding volume's dimensions;
// detect_tables builds the table's pose out of center and orientation, so one pose input fills
// both of those. Everything else on the message -- the point cloud, image, grasps, colours --
// is left default: an agent has none of it, and nothing in the trees reads it.
rail_manipulation_msgs::SegmentedObject segmentedObjectFromJson(const nlohmann::json& json)
{
  rail_manipulation_msgs::SegmentedObject object;
  object.name = json.value("name", std::string());
  object.width = json.value("width", 0.0);
  object.depth = json.value("depth", 0.0);
  object.height = json.value("height", 0.0);

  if (json.contains("pose"))
  {
    if (auto pose = poseFromJson(json["pose"]))
    {
      object.center = pose->pose.position;
      object.orientation = pose->pose.orientation;
      object.bounding_volume.pose = *pose;
    }
  }

  // the same numbers again, in the form table_visited reads them; deriving it saves the
  // caller from stating the table's size twice and getting the two copies out of step
  object.bounding_volume.dimensions.x = object.width;
  object.bounding_volume.dimensions.y = object.depth;
  object.bounding_volume.dimensions.z = object.height;
  return object;
}

// Builds a structured json value as whatever type the port declared, and writes it to the
// blackboard. Returns false if we have no case for that type, or the json is the wrong shape
// for it. `type` comes from the entry BT.CPP created when the tree was built.
bool setStructured(BT::Blackboard& blackboard, const std::string& key, const nlohmann::json& value,
                   const std::type_index& type)
{
  if (type == typeid(geometry_msgs::PoseStamped))
  {
    auto pose = poseFromJson(value);
    if (!pose)
    {
      return false;
    }
    blackboard.set(key, *pose);
    return true;
  }

  if (type == typeid(std::vector<geometry_msgs::PoseStamped>))
  {
    if (!value.is_array())
    {
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> poses;
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

  if (type == typeid(rail_manipulation_msgs::SegmentedObject))
  {
    if (!value.is_object())
    {
      return false;
    }
    blackboard.set(key, segmentedObjectFromJson(value));
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
      // A structured value has to be built as its real type, which means knowing what that
      // type is. Ask the entry BT.CPP created for the port when it built the tree: no entry
      // means no port uses this key, so there's nothing to build it as.
      const BT::TypeInfo* info = blackboard.entryInfo(key);
      if (!info)
      {
        ROS_WARN_STREAM_NAMED("blackboard_json", "Skipping input key '"
                                                     << key
                                                     << "': it is an object or array, and no port in this tree "
                                                        "uses that key, so there's no type to build it as");
      }
      else if (!setStructured(blackboard, key, value, info->type()))
      {
        ROS_WARN_STREAM_NAMED("blackboard_json", "Skipping input key '" << key << "': can't build a "
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
