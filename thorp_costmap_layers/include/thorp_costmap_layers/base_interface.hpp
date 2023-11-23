#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/layered_costmap.h>

#include <geometry_msgs/PoseStamped.h>

#include "thorp_costmap_layers/spatial_hash.hpp"
#include "thorp_costmap_layers/SemanticLayerConfig.h"

namespace thorp::costmap_layers
{
// Struct describing how to reflect an object of a given type on costmaps
struct ObjectType
{
  float width_padding = 0.0;   ///< width adjustment (m)
  float length_padding = 0.0;  ///< length adjustment (m)
  bool force_update = false;   ///< triggers map update
  bool use_maximum = false;    ///< overwrite current cost
  int precedence = 0;          ///< order when writing cost
  bool fill = false;           ///< set also enclosed cells
  float cost = 1.0f;           ///< 0: clean, 1: lethal
};

class BaseInterface
{
public:
  ~BaseInterface() = default;

  /**
   * Reconfigure dynamic parameters
   * @param config Updated configuration
   */
  virtual void reconfigure(const thorp_costmap_layers::SemanticLayerConfig& config) = 0;

  bool isEnabled() const
  {
    return enabled_;
  };

  /**
   * Get updated objects set and lock it until clearUpdatedIds is called, so it doesn't change while processing
   * @return Set with all the updated objects
   */
  std::set<std::shared_ptr<Object>> getUpdatedObjects();

  /**
   * Get removed objects set and lock it until clearRemovedIds is called, so it doesn't change while processing
   * @return Set with all the removed objects
   */
  std::set<std::shared_ptr<Object>> getRemovedObjects();

  /**
   * Get updated ids set and lock it until clearUpdatedIds is called, so it doesn't change while processing
   * @return Constant reference to the set of updated ids
   */
  const std::set<int>& getUpdatedIds()
  {
    updated_mutex_.lock();
    return ids_updated_;
  }

  /**
   * Get removed ids set and lock it until clearRemovedIds is called, so it doesn't change while processing
   * @return Constant reference to the set of removed ids
   */
  const std::set<int>& getRemovedIds()
  {
    removed_mutex_.lock();
    return ids_removed_;
  }

  /**
   * Clear updated ids set and unlock it so new ids can be added to the set
   */
  void clearUpdatedIds()
  {
    ids_updated_.clear();
    updated_mutex_.unlock();
  }

  /**
   * Clear removed ids set and unlock it so new ids can be added to the set
   */
  void clearRemovedIds()
  {
    for (const auto id_removed : ids_removed_)
      spatial_hash_.removeObject(id_removed);

    ids_removed_.clear();
    removed_mutex_.unlock();
  }

  /**
   * Returns a pointer to an object by its id
   */
  std::shared_ptr<Object> getObject(int id) const;

  /**
   * Returns all objects in a rectangular region. Unlike getUpdatedObjects, we actually copy the objects and lock
   * while so doing, so this is thread safe.
   * @param lower_left Lower-left corner of the region
   * @param upper_right Upper-right corner of the region
   * @return Vector with the objects within the region
   */
  std::vector<Object> getObjectsInRegion(const Point2d& lower_left, const Point2d& upper_right) const;

  /**
   * Provide current robot pose (static, as it's common for all interfaces) on costmap reference frame
   * @param robot_x Robot position x
   * @param robot_y Robot position y
   * @param robot_yaw  Robot heading
   */
  static void setRobotPose(double robot_x, double robot_y, double robot_yaw);

protected:
  BaseInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, costmap_2d::LayeredCostmap* layered_costmap);

  int makeHash(const std::string& id, int primitive) const;
  void contoursToHash(const std::vector<std::vector<geometry_msgs::PoseStamped>>& contours, const std::string& id,
                      const std::string& type = "default");
  bool removeContours(const std::string& id);
  bool removeAllContours();
  void updateObject(const Object& object);
  void removeObject(int hash);

  static std::string fixed_frame_;  ///< fixed reference frame (typically map) used to store objects on the spatial hash

  static std::map<std::string, ObjectType> object_types_;  ///< list of object types with common attributes

  static double robot_x_, robot_y_, robot_yaw_;  // current robot pose

  bool enabled_;

  ros::NodeHandle nh_;
  tf2_ros::Buffer& tf_;
  costmap_2d::LayeredCostmap* layered_costmap_;  ///< Pointer to the layered costmap

  std::unordered_map<std::string, size_t> primitives_count_;  ///< number of primitives for a given collision object

private:
  // updated and removed ids
  std::set<int> ids_updated_;
  std::set<int> ids_removed_;
  std::mutex updated_mutex_;
  std::mutex removed_mutex_;

  SpatialHash spatial_hash_{ 0.5 };

  /// Module name for logging
  static constexpr char LOGNAME[] = "base_interface";
};

}  // namespace thorp::costmap_layers
