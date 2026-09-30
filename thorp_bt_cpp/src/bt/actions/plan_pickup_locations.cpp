#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>

#include <behaviortree_cpp/action_node.h>

#include "thorp_bt_cpp/node_common.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <thorp_msgs/msg/pickup_location.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thorp_toolkit/geometry.hpp>
#include <thorp_toolkit/tf2.hpp>
namespace ttk = thorp::toolkit;

namespace thorp::bt::actions
{
/**
 * Group the objects on a table into pickup locations, the poses around it from where the arm reaches them, and sort
 * the locations to visit as few as possible, travelling the least: a pickup plan. Each object is picked from the
 * location where it's closest to the arm.
 */
class PlanPickupLocations : public BT::SyncActionNode
{
public:
  PlanPickupLocations(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),                 //
             BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("pickup_poses"),  //
             BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject>>("objects"),     //
             BT::OutputPort<std::vector<thorp_msgs::msg::PickupLocation>>("pickup_plan") };
  }

private:
  using Location = thorp_msgs::msg::PickupLocation;

  BT::NodeStatus tick() override
  {
    auto node = rosNode(*this);
    const std::string arm_frame = node->get_parameter_or<std::string>("pickup_planning_frame", "arm_base_link");
    const double pickup_dist = node->get_parameter_or("pickup_dist_to_table", 0.15);
    const double approach_offset = pickup_dist - node->get_parameter_or("approach_dist_to_table", 0.35);
    const double detach_offset = pickup_dist - node->get_parameter_or("detach_dist_from_table", 0.45);
    // pickup poses are reached with some tolerance, so the arm may end a bit farther from the objects
    const double max_reach =
        node->get_parameter_or("max_arm_reach", 0.3) - node->get_parameter_or("tight_dist_tolerance", 0.0);

    const auto robot_pose = toMap(requireInput<geometry_msgs::msg::PoseStamped>(*this, "robot_pose"));
    geometry_msgs::msg::TransformStamped base_to_arm;
    if (!ttk::TF2::instance().lookupTransform("base_footprint", arm_frame, base_to_arm))
      throw BT::RuntimeError(name(), ": cannot get the arm pose on the robot");
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> objects;
    for (const auto& object : requireInput<std::vector<moveit_msgs::msg::CollisionObject>>(*this, "objects"))
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = object.header;
      pose.pose = object.pose;
      objects.emplace_back(object.id, toMap(pose));
    }

    // A location for each pickup pose reaching some object, with the objects sorted by distance to the arm
    std::vector<Location> locations;
    for (const auto& pickup_pose : requireInput<std::vector<geometry_msgs::msg::PoseStamped>>(*this, "pickup_poses"))
    {
      Location location;
      location.pickup_pose = toMap(pickup_pose);
      location.arm_pose.header = location.pickup_pose.header;
      location.arm_pose.pose = composeWith(location.pickup_pose, base_to_arm);
      for (const auto& [id, pose] : objects)
      {
        const double distance = ttk::distance2D(pose, location.arm_pose);
        if (distance <= max_reach)
        {
          thorp_msgs::msg::ObjectToPickup object;
          object.name = id;
          object.distance = distance;
          object.pose = pose;
          location.objects.push_back(object);
        }
      }
      if (location.objects.empty())
        continue;
      std::sort(location.objects.begin(), location.objects.end(),
                [](const auto& a, const auto& b) { return a.distance < b.distance; });
      location.name = std::to_string(locations.size() + 1);
      location.distance = ttk::distance2D(location.pickup_pose, robot_pose);
      location.approach_pose = translated(location.pickup_pose, approach_offset);
      location.detach_pose = translated(location.pickup_pose, detach_offset);
      locations.push_back(location);
    }

    const auto plan = removeDuplicates(bestOrder(robot_pose, locations));
    std::ostringstream summary;
    for (const auto& location : plan)
    {
      summary << " " << location.name << " (";
      for (const auto& object : location.objects)
        summary << (&object == &location.objects.front() ? "" : ", ") << object.name;
      summary << ")";
    }
    RCLCPP_INFO(logger(*this), "Pickup plan of %zu locations, %.2f m to travel:%s", plan.size(),
                travelled(robot_pose, plan), summary.str().c_str());
    setOutput("pickup_plan", plan);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped toMap(const geometry_msgs::msg::PoseStamped& pose) const
  {
    geometry_msgs::msg::PoseStamped map_pose;
    if (!ttk::TF2::instance().transformPose("map", pose, map_pose))
      throw BT::RuntimeError(name(), ": cannot transform pose from ", pose.header.frame_id, " to map");
    return map_pose;
  }

  /** The pose of a frame given relative to the robot, when the robot is at pose */
  static geometry_msgs::msg::Pose composeWith(const geometry_msgs::msg::PoseStamped& pose,
                                              const geometry_msgs::msg::TransformStamped& relative)
  {
    tf2::Transform robot, offset;
    tf2::fromMsg(pose.pose, robot);
    tf2::fromMsg(relative.transform, offset);
    geometry_msgs::msg::Pose composed;
    tf2::toMsg(robot * offset, composed);
    return composed;
  }

  /** The pose moved along its own x axis */
  static geometry_msgs::msg::PoseStamped translated(const geometry_msgs::msg::PoseStamped& pose, double distance)
  {
    auto moved = pose;
    moved.pose.position.x += distance * std::cos(ttk::yaw(pose));
    moved.pose.position.y += distance * std::sin(ttk::yaw(pose));
    return moved;
  }

  static double travelled(const geometry_msgs::msg::PoseStamped& robot_pose, const std::vector<Location>& locations)
  {
    double distance = 0.0;
    const geometry_msgs::msg::PoseStamped* previous = &robot_pose;
    for (const auto& location : locations)
    {
      distance += ttk::distance2D(*previous, location.pickup_pose);
      previous = &location.pickup_pose;
    }
    return distance;
  }

  /**
   * Of all the orders to visit the locations, skipping those whose objects are all reachable from the others still
   * in the order, the one visiting fewer locations, and travelling less on a tie
   */
  static std::vector<Location> bestOrder(const geometry_msgs::msg::PoseStamped& robot_pose,
                                         const std::vector<Location>& locations)
  {
    std::vector<size_t> order(locations.size());
    std::iota(order.begin(), order.end(), 0);
    std::vector<Location> best;
    double best_distance = std::numeric_limits<double>::max();
    do
    {
      std::vector<Location> candidate;
      for (const size_t i : order)
        candidate.push_back(locations[i]);
      skipRedundant(candidate);
      const double distance = travelled(robot_pose, candidate);
      if (best.empty() || candidate.size() < best.size() ||
          (candidate.size() == best.size() && distance < best_distance))
      {
        best = candidate;
        best_distance = distance;
      }
    } while (std::next_permutation(order.begin(), order.end()));
    return best;
  }

  /** Traversing in order, discard the locations whose objects are all in the other remaining locations */
  static void skipRedundant(std::vector<Location>& locations)
  {
    std::set<std::string> skipped;
    for (const auto& location : locations)
    {
      std::set<std::string> elsewhere;
      for (const auto& other : locations)
        if (other.name != location.name && !skipped.count(other.name))
          for (const auto& object : other.objects)
            elsewhere.insert(object.name);
      if (std::all_of(location.objects.begin(), location.objects.end(),
                      [&](const auto& object) { return elsewhere.count(object.name); }))
        skipped.insert(location.name);
    }
    locations.erase(std::remove_if(locations.begin(), locations.end(),
                                   [&](const Location& location) { return skipped.count(location.name); }),
                    locations.end());
  }

  /** Keep each object only on the location where it's closest to the arm, dropping locations left empty */
  static std::vector<Location> removeDuplicates(std::vector<Location> locations)
  {
    std::map<std::string, std::pair<std::string, double>> closest;  // object -> location, distance
    for (const auto& location : locations)
      for (const auto& object : location.objects)
        if (!closest.count(object.name) || object.distance < closest[object.name].second)
          closest[object.name] = { location.name, object.distance };
    for (auto& location : locations)
      location.objects.erase(std::remove_if(location.objects.begin(), location.objects.end(),
                                            [&](const auto& object) {
                                              return closest[object.name].first != location.name;
                                            }),
                             location.objects.end());
    locations.erase(std::remove_if(locations.begin(), locations.end(),
                                   [](const Location& location) { return location.objects.empty(); }),
                    locations.end());
    return locations;
  }

  BT_REGISTER_NODE(PlanPickupLocations);
};

}  // namespace thorp::bt::actions
