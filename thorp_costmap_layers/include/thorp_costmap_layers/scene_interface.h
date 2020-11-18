#pragma once

#include <moveit_msgs/PlanningSceneWorld.h>

#include "thorp_costmap_layers/base_interface.h"


namespace thorp_costmap_layers
{

class SceneInterface : public BaseInterface
{
public:
  SceneInterface(ros::NodeHandle& nh, tf2_ros::Buffer& tf, const std::string& map_frame,
                 std::function<void(double, double, double)> update_map_callback);
  ~SceneInterface();

  void sceneMessageCallback(const moveit_msgs::PlanningSceneWorldConstPtr& planning_scene);
  void processCollisionObjs(const std::vector<moveit_msgs::CollisionObject>& collision_objects);

  bool getCallbackProcessed() { return callback_processed_; }

private:
  void collisionObjToContours(const moveit_msgs::CollisionObject& collision_object,
                              std::vector<std::vector<geometry_msgs::PoseStamped>>& contours,
                              double length_padding, double width_padding) const;
  void publishObject(const Object& object);
  void deleteObject(const Object& object);

  ros::Subscriber add_objs_sub_;
  ros::Publisher objects_pub_;
  bool callback_processed_;

  std::function<void(double, double, double)> update_map_callback_;
};

}
