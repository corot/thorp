#pragma once

#include <set>
#include <string>
#include <vector>

#include <behaviortree_cpp/blackboard.h>
// BT.CPP vendors its own nlohmann/json and exposes it here. We must use that copy and not
// the system <nlohmann/json.hpp>: both headers share an include guard, so mixing them means
// whichever is included first wins per translation unit, and since 3.11 the version is part
// of an inline namespace baked into every mangled name -- two files disagreeing on which
// copy they got fail to link against each other, with no warning at compile time.
#include <behaviortree_cpp/json_export.h>

namespace thorp::bt
{
/**
 * @brief Seed a blackboard from a flat json object.
 *
 * Each entry is written to the blackboard as a plain string, using the same textual
 * convention a literal XML attribute would use (e.g. a pose is written as "x;y;yaw;frame",
 * exactly what <SomeNode pose="1;2;0;map"/> uses). This way, whatever type a port later
 * reads a key as goes through BT::convertFromString<T>, the very same mechanism BT.CPP
 * already uses to parse literal attributes -- so this function never needs to know what
 * type each key is "supposed" to be, and supporting a new type is just a matter of adding
 * a BT::convertFromString<T> specialization (as already done for geometry_msgs::msg::PoseStamped
 * in type_converters.hpp), with nothing to change here or in bt_server.
 *
 * Scalars go through that string path. A structured value -- a json array or object -- can't,
 * since there's no sensible way to flatten one into the textual form a port expects, so those
 * are built as the real type and written to the blackboard directly. That needs to know what
 * type the key is supposed to be, which BT.CPP can tell us: building a tree pre-creates an
 * entry per remapped port carrying its declared type, so the port itself says what to build.
 * Supported today are geometry_msgs::msg::PoseStamped and std::vector of them (pose strings or
 * objects), std::map<std::string, unsigned> (an object of counts), and
 * moveit_msgs::msg::CollisionObject for tables (an object of the fields the trees actually read).
 * A structured value for any other type is skipped with a warning naming the type, which is
 * the cue to add a case in blackboard_json.cpp.
 */
void blackboardFromJson(const nlohmann::json& json, BT::Blackboard& blackboard);

/**
 * @brief Dump the named blackboard entries into a json object keyed by those names.
 *
 * This is the normal way to report a run's outputs: the caller names what it wants, so the
 * result has the shape the caller asked for instead of one that depends on which branches
 * the tree took, and a key the tree merely updated (rather than created) is reported just
 * the same. Keys with no value -- never written, or not on the blackboard at all, e.g. a
 * misspelled one -- are left out of the object and logged, so the caller can spot them by
 * comparing what it asked for against what came back.
 *
 * See newEntriesToJson() below for how values are serialized.
 */
nlohmann::json entriesToJson(const BT::Blackboard& blackboard, const std::vector<std::string>& keys);

/**
 * @brief Keys of the blackboard entries that currently hold a value.
 *
 * Only needed for the newEntriesToJson() fallback below. Note that an entry merely existing
 * tells us nothing: when BT.CPP builds a tree it calls createEntry() for every port remapped
 * to a {key}, so the blackboard already holds an (empty) entry for every key the tree
 * mentions before the first tick. Only having a value is meaningful, hence this helper: take
 * a snapshot right before running a tree, and the keys that hold a value afterwards but
 * weren't in the snapshot are exactly the ones the run produced.
 */
std::set<std::string> valuedKeys(const BT::Blackboard& blackboard);

/**
 * @brief Dump into a flat json object the blackboard entries that hold a value and whose
 * key is not in `known_keys` -- i.e. what the run added, given a `known_keys` snapshot
 * taken with valuedKeys() before running.
 *
 * This is the fallback for a caller that didn't name the outputs it wants: useful to
 * discover what a tree produces, but note it reports only keys the run *created*, so a key
 * the tree updated in place (one of the seeded inputs, say) will not show up here.
 *
 * Entries whose runtime type we know how to serialize are reported as their value: bool, the
 * integral and floating point types and std::string map onto their json counterparts, while
 * a geometry_msgs::Pose / PoseStamped (or a vector of the latter) becomes an object with
 * x/y/z, roll/pitch/yaw and, when stamped, frame. Any other type -- any of the other ROS
 * messages a node may write via setOutput<T> -- is reported as the string
 * "<unsupported type: TYPE_NAME>" rather than silently dropped, so it's obvious both that
 * something was left out and which type needs a case added in blackboard_json.cpp.
 */
nlohmann::json newEntriesToJson(const BT::Blackboard& blackboard, const std::set<std::string>& known_keys);

}  // namespace thorp::bt
