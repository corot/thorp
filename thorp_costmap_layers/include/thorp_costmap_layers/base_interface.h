#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/PoseStamped.h>

#include "thorp_costmap_layers/spatial_hash.h"


namespace thorp_costmap_layers
{

// Struct describing how to reflect an object of a given type on costmaps
struct ObjectType
{
  float width_padding = 0.0;      ///< width adjustment (m)
  float length_padding = 0.0;     ///< length adjustment (m)
  bool force_update = false;      ///< triggers map update
  bool use_maximum = false;       ///< overwrite current cost
  int precedence = 0;             ///< order when writing cost
  bool fill = false;              ///< set also enclosed cells
  float cost = 1.0f;              ///< 0: clean, 1: lethal
};


class BaseInterface
{
public:

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

protected:
  BaseInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, const std::string& map_frame);
  ~BaseInterface();

  int makeHash(const std::string &id, int primitive) const;
  void contoursToHash(const std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                      const std::string& id, const ObjectType& type = ObjectType());
  bool removeContours(const std::string& id);
  void updateObject(const Object& object);
  void removeObject(int hash);

  ros::NodeHandle nh_;
  tf2_ros::Buffer& tf_;
  std::string map_frame_;
  std::string robot_frame_ = "base_footprint";

  std::map<std::string, ObjectType> object_types_;  ///< list of object types with common attributes    TODO STATIC!!!

  // save number of primitives for a given collision object
  std::unordered_map<std::string, size_t> primitives_count_;

private:
  // updated and removed ids
  std::set<int> ids_updated_;
  std::set<int> ids_removed_;
  std::mutex updated_mutex_;
  std::mutex removed_mutex_;

  SpatialHash spatial_hash_{0.5};
};

}
