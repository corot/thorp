/*
 * Author: Jorge Santos
 */

#include <stdexcept>

#include <omp.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

// input: RAIL segmentation and object recognition
#include <rail_manipulation_msgs/SegmentObjects.h>
#include <rail_mesh_icp/MatchTemplateAction.h>

// action server: make things easier for interactive manipulation
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/DetectObjectsAction.h>

// MoveIt!
#include <moveit_msgs/ObjectColor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// shapes
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

// auxiliary libraries
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;


namespace thorp_perception
{

class ObjectDetectionServer
{
private:
  typedef actionlib::ActionClient<rail_mesh_icp::MatchTemplateAction> MatchTemplateActionClient;

  // Service and action clients for RAIL object segmentation and template matching
  // We make parallel calls for template matching, so cannot use a simple action client
  ros::ServiceClient segment_srv_;
  MatchTemplateActionClient tm_ac_;
  ros::CallbackQueue callback_queue_;
  std::unique_ptr<boost::thread> spin_thread_;

  // Action server to handle it conveniently for our object manipulation demo
  actionlib::SimpleActionServer<thorp_msgs::DetectObjectsAction> od_as_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Planning scene doesn't show collision object names, so we add our own markers
  ros::Publisher markers_pub_;

  bool publish_static_tf_;      ///< Publish static transforms for the segmented objects and surface
  double max_matching_error_;   ///< Object recognition template matching threshold
  double redetect_tolerance_;   ///< ROI padding for matching an object to an existing one on planning scene
  double tabletop_vol_height_;  ///< Tabletop segmented volume maximum height
  std::string output_frame_;    ///< Recognized objects reference frame; we cannot use goal's field because
                                ///< RAIL segmentation zone's frame cannot be reconfigured on runtime

public:

  ObjectDetectionServer(const std::string& name) :
      tm_ac_("template_matcher/match_template", &callback_queue_),
      od_as_(name, boost::bind(&ObjectDetectionServer::executeCB, this, _1), false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("publish_static_tf", publish_static_tf_, false);  // good for debugging but pollutes a lot
    pnh.param("max_matching_error", max_matching_error_, 1e-5);
    pnh.param("redetect_tolerance", redetect_tolerance_, 1e-2);
    pnh.param("tabletop_vol_height", tabletop_vol_height_, 0.2);
    pnh.param("output_frame", output_frame_, std::string("base_footprint"));

    markers_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("visual_markers", 1);

    // Connect to rail_segmentation/segment_objects service
    segment_srv_ = nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
    ROS_INFO("[object detection] Waiting for rail_segmentation/segment_objects service to start...");
    if (!segment_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] rail_segmentation/segment_objects service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }
    ROS_INFO("[object detection] rail_segmentation/segment_objects service started; ready for sending goals");

    // Connect to template matcher action
    spin_thread_ = std::make_unique<boost::thread>([&](){
      while (ros::ok()) callback_queue_.callAvailable(ros::WallDuration(0.1f)); });
    ROS_INFO_STREAM("[object detection] Waiting for template matcher action server to start...");
    if (!tm_ac_.waitForActionServerToStart(ros::Duration(1.0)))
    {
      ROS_ERROR_STREAM("[object detection] Template matcher action server not available after 1 minute");
      ROS_ERROR_STREAM("[object detection] Shutting down node...");
      throw;
    }
    ROS_INFO_STREAM("[object detection] Template matcher action server started; ready for sending goals");

    // Wait for all RAIL services to connect before we provide our own action server
    od_as_.start();
  }

  void executeCB(const thorp_msgs::DetectObjectsGoal::ConstPtr& goal)
  {
    ros::Time time_start = ros::Time::now();

    ROS_INFO("[object detection] Received goal!");
    thorp_msgs::DetectObjectsResult result;

    // Clear results from previous goals
    if (goal->clear_scene)
      planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());

    // Call RAIL rail_segmentation/segment_objects service
    ROS_INFO("[object detection] Calling rail_segmentation/segment_objects service...");
    rail_manipulation_msgs::SegmentObjects srv;
    if (segment_srv_.call(srv))
    {
      ROS_INFO("[object detection] rail_segmentation/segment_objects service call succeeded");
    }
    else
    {
      ROS_ERROR("[object detection] rail_segmentation/segment_objects service call failed");
      od_as_.setAborted(result, "Segment objects service call failed");
      return;
    }

    if (srv.response.segmented_objects.objects.empty())
    {
      ROS_WARN("[object detection] No surface found");
      od_as_.setSucceeded(result, "No surface found");
      return;
    }

    // Process segmented support surface (normally a table) and add to the planning
    // scene as a MoveIt! collision object, so it gets filtered out from the octomap
    // note that we use centroid-z as z-position and 1 mm as height, to ensure we don't
    // subsume low-laying objects segmented together with the surface
    auto& table = srv.response.segmented_objects.objects.front();
    result.surface.id = "table";
    result.surface.operation = moveit_msgs::CollisionObject::ADD;
    result.surface.header = srv.response.segmented_objects.header;
    result.surface.pose.position = table.center;
    result.surface.pose.position.z = table.centroid.z;
    result.surface.pose.orientation = table.orientation;
    result.surface.primitives.resize(1);
    result.surface.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
    result.surface.primitives.front().dimensions = {table.depth, table.width, 0.001};
    ROS_INFO("[object detection] Adding table at %s as a collision object",
             ttk::point2cstr3D(result.surface.pose.position));
    std::vector<moveit_msgs::CollisionObject> new_scene_objs(1, result.surface);

    if (publish_static_tf_)
      ttk::TF2::instance().sendTransform(result.surface.pose, output_frame_, result.surface.id);

    // show collision objects with color
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = result.surface.id;
    obj_color.color = ttk::makeColor(table.rgb[0], table.rgb[1], table.rgb[2]);
    std::vector<moveit_msgs::ObjectColor> obj_colors{ obj_color };

    ROS_INFO("[object detection] %lu objects plus table segmented in %.2f seconds",
             srv.response.segmented_objects.objects.size() - 1, (ros::Time::now() - time_start).toSec());

    // Good time for serving preempt requests, as we will process segmented objects in parallel
    if (od_as_.isPreemptRequested())
    {
      od_as_.setPreempted(result, "Preempted before processing segmented objects");
      return;
    }

    // Required to avoid repeating existing object names on planning scene
    std::vector<std::string> extant_objs = std::move(planning_scene_interface_.getKnownObjectNames());

    visualization_msgs::MarkerArray markers;

    // Process segmented objects following the support surface, and add them to the planning scene
    #pragma omp parallel for    // NOLINT
    for (int i = 1; i < srv.response.segmented_objects.objects.size(); ++i)
    {
      auto& rail_obj = srv.response.segmented_objects.objects[i];
      // reject objects of incongruent dimensions or with impossible poses, e.g. floating or embedded in the table
      if (!validateObject(rail_obj, table))
        continue;

      moveit_msgs::CollisionObject co;
      co.header = srv.response.segmented_objects.header;
      co.pose.position = rail_obj.center;
      co.pose.orientation = rail_obj.orientation;
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.mesh_poses.resize(1);
      co.mesh_poses.front().position.z = - rail_obj.height / 2.0;  // meshes' origin is at z = 0
      co.mesh_poses.front().orientation.w = 1;

      // Identify object type and possibly update the pose with ICP template matching correction
      // we reject objects failing to match any template within our error threshold
      if (!identifyObject(rail_obj, co.pose))
      {
        ROS_WARN("[object detection] Object %d at %s discarded", i, ttk::point2cstr3D(rail_obj.center));
        continue;
      }

      // Our convention is that x-dimension is the longest one
      double length = std::max(rail_obj.depth, rail_obj.width);
      double width = std::min(rail_obj.depth, rail_obj.width);
      json metadata = {"size", {length, width, rail_obj.height}, "color", rail_obj.rgb};
      co.type.db = std::move(metadata.dump());

      // Load mesh from the identified object type and convert to mesh msg
      shapes::Mesh* mesh = shapes::createMeshFromResource("package://thorp_perception/meshes/" + rail_obj.name + ".stl");
      if (!mesh)
      {
        ROS_WARN("[object detection] Load mesh for %s failed", rail_obj.name.c_str());
        continue;
      }
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(mesh, shape_msg);
      co.meshes.emplace_back(boost::get<shape_msgs::Mesh>(shape_msg));

      #pragma omp critical    // NOLINT

      // Make a name for the new object; note that we must do under omp critical to keep extant_objs list meaningful
      co.id = getObjName(rail_obj, extant_objs);
      extant_objs.push_back(co.id);

      if (publish_static_tf_)
        ttk::TF2::instance().sendTransform(co.pose, output_frame_, co.id);

      // Use segmented pointcloud's color for the co
      obj_color.id = co.id;
      obj_color.color = ttk::makeColor(rail_obj.rgb[0], rail_obj.rgb[1], rail_obj.rgb[2]);

      ROS_INFO("[object detection] Object at %s classified as %s", ttk::pose2cstr2D(co.pose), rail_obj.name.c_str());
      result.objects.push_back(co);  // TODO   try emplace
      markers.markers.emplace_back(makeLabelMarker(obj_colors.size(), rail_obj.height, co, obj_color.color));
      markers.markers.emplace_back(makeVolumeMarker(obj_colors.size(), rail_obj.bounding_volume));
      obj_colors.push_back(obj_color);
    }

    // Remove all collision objects previously detected within the tabletop bounding volume
    rail_manipulation_msgs::BoundingVolume segmented_volume = table.bounding_volume;
    segmented_volume.dimensions.z += tabletop_vol_height_;
    segmented_volume.pose.pose.position.z += tabletop_vol_height_ / 2.0;
    markers.markers.emplace_back(makeVolumeMarker(0, segmented_volume));
    std::set<std::string> to_remove = std::move(getObjsInVolume(segmented_volume));

    // Exclude the table itself and all the objects that have been re-detected, as they will be replaced by the new ones
    // This avoids possible flickering between deletion and addition
    to_remove.erase(result.surface.id);
    std::for_each(result.objects.begin(), result.objects.end(), [&](const auto& co) { to_remove.erase(co.id); });
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>{to_remove.begin(), to_remove.end()});

    if (!result.objects.empty())
    {
      std::copy(result.objects.begin(), result.objects.end(), std::back_inserter(new_scene_objs));
      ROS_INFO("[object detection] Succeeded! %lu objects detected in %.2f seconds", result.objects.size(),
               (ros::Time::now() - time_start).toSec());
    }
    else
    {
      ROS_INFO("[object detection] Succeeded, but couldn't find any tabletop object (time: %.2f s)",
               (ros::Time::now() - time_start).toSec());
    }

    planning_scene_interface_.addCollisionObjects(new_scene_objs, obj_colors);
    markers_pub_.publish(markers);

    od_as_.setSucceeded(result);
  }

private:
  bool validateObject(const rail_manipulation_msgs::SegmentedObject& obj,
                      const rail_manipulation_msgs::SegmentedObject& table)
  {
    const double MIN_SIZE = 0.005;
    const double MAX_SIZE = 0.25;
    const double TOLERANCE = 0.01;

    // reject objects with incongruent dimensions
    if (std::min({obj.depth, obj.width, obj.height}) < MIN_SIZE ||
        std::max({obj.depth, obj.width, obj.height}) > MAX_SIZE)
    {
      ROS_WARN("[object detection] Object at %s discarded due to incongruent dimensions (%g x %g x %g; limits %g, %g)",
               ttk::point2cstr3D(obj.center), obj.depth, obj.width, obj.height, MIN_SIZE, MAX_SIZE);
      return false;
    }
    // reject objects embedded in the table (possibly table parts not properly removed)...
    if ((obj.center.z - obj.height / 2.0) < (table.centroid.z - TOLERANCE))
    {
      ROS_WARN("[object detection] Object at %s discarded as embedded in the table (%g < %g - %g)",
               ttk::point2cstr3D(obj.center), obj.center.z - obj.height / 2.0, table.centroid.z, TOLERANCE);
      return false;
    }
    // ...or floating above the table (possibly the gripper after a pick/place operation)
    if ((obj.center.z - obj.height / 2.0) > (table.centroid.z + TOLERANCE))
    {
      ROS_WARN("[object detection] Object at %s discarded as floating over the table (%g > %g + %g)",
               ttk::point2cstr3D(obj.center), obj.center.z - obj.height / 2.0, table.centroid.z, TOLERANCE);
      return false;
    }

    return true;
  }

  bool identifyObject(rail_manipulation_msgs::SegmentedObject& obj, geometry_msgs::Pose& pose)
  {
    // Call template matching action with the object pointcloud
    // Note that we log all errors multiplied by 1e6, as the values provided by the template matcher are very low
    // We provide the pointcloud center as the initial estimate for position, but use 0, 0, 0 for orientation, as
    // it's a good guideline only for elongated objects, where PCA is meaningful
    rail_mesh_icp::MatchTemplateGoal goal;
    goal.initial_estimate.translation.x = obj.center.x;
    goal.initial_estimate.translation.y = obj.center.y;
    goal.initial_estimate.translation.z = obj.center.z - obj.height / 2.0; // templates' base is at z = 0
    goal.initial_estimate.rotation.w = 1.0;
    goal.target_cloud = obj.point_cloud;
    MatchTemplateActionClient::GoalHandle goal_handle = tm_ac_.sendGoal(goal);
    const ros::Duration timeout(5);
    ros::Time t0 = ros::Time::now();
    while (goal_handle.getCommState() != actionlib::CommState::DONE && ros::Time::now() - t0 < timeout && ros::ok())
      ros::Duration(0.001).sleep();
    if (goal_handle.getCommState() != actionlib::CommState::DONE)
    {
      ROS_ERROR_STREAM("[object detection] Template match action timeout; aborting...");
      goal_handle.cancel();
      return false;
    }
    if (goal_handle.getTerminalState() != actionlib::TerminalState::SUCCEEDED)
    {
      ROS_ERROR_STREAM("[object detection] Template match action failed: " << goal_handle.getTerminalState().getText());
      return false;
    }

    rail_mesh_icp::MatchTemplateResult::ConstPtr result = goal_handle.getResult();

    // Check if best match is good enough
    if (result->match_error > max_matching_error_)
    {
      ROS_INFO_STREAM("[object detection] Best match (" << result->template_name << ") error above tolerance ("
                                                        << result->match_error * 1e6 << " > "
                                                        << max_matching_error_ * 1e6 << ")");
      return false;
    }

    ROS_INFO_STREAM("[object detection] Best match is " << result->template_name << ", with error "
                                                        << result->match_error * 1e6);

    obj.name = result->template_name;
    obj.recognized = true;

    // Use the matched template orientation to recalculate pointcloud dimensions
    recalcDimensions(obj, result->template_pose.transform);

    ttk::tf2pose(result->template_pose.transform, pose);
    pose.position.z += obj.height / 2.0;  // restore to object center (remember that templates' base is at z = 0)
    ROS_DEBUG("Yaw updated:    %f -> %f", tf::getYaw(obj.orientation), tf::getYaw(pose.orientation));

    return true;
  }

  void recalcDimensions(rail_manipulation_msgs::SegmentedObject& obj, const geometry_msgs::Transform& tf)
  {
    // Recenter object pointcloud in its estimated location (apply the inverse of its pose)
    pcl::PointCloud<pcl::PointXYZRGB> tmp_pc;
    pcl::fromROSMsg(obj.point_cloud, tmp_pc);
    tf::Transform transform;
    tf::transformMsgToTF(tf, transform);
    pcl_ros::transformPointCloud(tmp_pc, tmp_pc, transform.inverse());

    // Now max - min points provide the real size for each dimension, unlike the original values that provided the
    // size along the reference frame axis
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(tmp_pc, min_pt, max_pt);
    double new_width = max_pt[0] - min_pt[0];
    double new_depth = max_pt[1] - min_pt[1];
    ROS_DEBUG("Width updated:  %f -> %f", obj.width, new_width);
    ROS_DEBUG("Depth updated:  %f -> %f", obj.depth, new_depth);
    ROS_DEBUG("Area updated:   %f -> %f", obj.width * obj.depth, new_width * new_depth);
    obj.width = new_width;
    obj.depth = new_depth;
  }

  std::string getObjName(rail_manipulation_msgs::SegmentedObject& obj, const std::vector<std::string>& extant_objs)
  {
    // obj.name contains the best matching template; to complete an object name, we append an index to avoid repetitions
    // if there's already an object of the same type in approximately the same location, we just copy the name (with the
    // expectation that both are the same, detected on successive calls to segmentation), or...
    std::set<std::string> overlapping_objs = std::move(getObjsInVolume(obj.bounding_volume));
    const std::string& template_name = obj.name;
    for (const auto& obj_name : overlapping_objs)
    {
      if (obj_name.find(template_name) == 0)
      {
        ROS_DEBUG_STREAM("" << obj_name << " found on planning scene");
        return obj_name;
      }
    }

    // ...we just find an unused index in the extant objects list
    for (int index = 1; index < INT_MAX; ++index)
    {
      std::string obj_name = template_name + " " + std::to_string(index);
      if (std::find(extant_objs.begin(), extant_objs.end(), obj_name) == extant_objs.end())
      {
        ROS_DEBUG_STREAM("" << template_name << " not found on planning scene; renaming as " << obj_name);
        return obj_name;
      }
    }
    throw std::logic_error("There are INT_MAX objects of the given type???");
  }

  std::set<std::string> getObjsInVolume(const rail_manipulation_msgs::BoundingVolume& volume)
  {
    // Retrieve objects on planning scene within the given volume padded by redetect_tolerance_
    tf::Transform tf;
    ttk::pose2tf(volume.pose.pose, tf);
    double half_dim_x = (volume.dimensions.x / 2.0) + redetect_tolerance_;
    double half_dim_y = (volume.dimensions.y / 2.0) + redetect_tolerance_;
    double half_dim_z = (volume.dimensions.z / 2.0) + redetect_tolerance_;
    tf::Vector3 corner1 = tf * tf::Vector3(+half_dim_x, +half_dim_y, +half_dim_z);
    tf::Vector3 corner2 = tf * tf::Vector3(-half_dim_x, -half_dim_y, -half_dim_z);
    std::vector<std::string> objs =
        std::move(planning_scene_interface_.getKnownObjectNamesInROI(std::min(corner2.x(), corner1.x()),
                                                                     std::min(corner2.y(), corner1.y()),
                                                                     std::min(corner2.z(), corner1.z()),
                                                                     std::max(corner2.x(), corner1.x()),
                                                                     std::max(corner2.y(), corner1.y()),
                                                                     std::max(corner2.z(), corner1.z())));
    return {objs.begin(), objs.end()};
  }

  // visualization methods

  // Make a label to show over the collision object
  visualization_msgs::Marker makeLabelMarker(const size_t id, const double obj_height,
                                             const moveit_msgs::CollisionObject& co, const std_msgs::ColorRGBA& color)
  {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.header = co.header;
    m.ns = "labels";
    m.id = (int)id;
    m.text = co.id;
    m.scale.z = 0.035;
    m.color = color;
    m.pose = co.pose;
    m.pose.position.z += obj_height / 2.0 + 0.025;
    return m;
  }

  // Make a bounding volume marker for the segmented object
  visualization_msgs::Marker makeVolumeMarker(const size_t id, const rail_manipulation_msgs::SegmentedObject& obj)
  {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::CUBE;
    m.header = obj.bounding_volume.pose.header;
    m.ns = "b_volumes";
    m.id = (int)id;
    m.scale.x = obj.depth;
    m.scale.y = obj.width;
    m.scale.z = obj.height;
    m.color = ttk::makeColor(0.8f, 0.8f, 0.8f, 0.2f);
    m.pose.position = obj.center;
    m.pose.orientation = obj.orientation;
    m.lifetime.fromSec(2.0);
    return m;
  }

  // Make a bounding volume marker
  visualization_msgs::Marker makeVolumeMarker(const size_t id, const rail_manipulation_msgs::BoundingVolume& bv)
  {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::CUBE;
    m.header = bv.pose.header;
    m.ns = "b_volumes";
    m.id = (int)id;
    m.scale.x = bv.dimensions.x + redetect_tolerance_;
    m.scale.y = bv.dimensions.y + redetect_tolerance_;
    m.scale.z = bv.dimensions.z + redetect_tolerance_;
    m.color = ttk::makeColor(0.8f, 0.8f, 0.8f, 0.2f);
    m.pose = bv.pose.pose;
    m.lifetime.fromSec(2.0);
    return m;
  }

};

};  // namespace thorp_perception

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");

  thorp_perception::ObjectDetectionServer server("object_detection");
  ros::spin();

  return 0;
}
