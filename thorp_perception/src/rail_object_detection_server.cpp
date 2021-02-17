/*
 * Author: Jorge Santos
 */

#include <omp.h>
#include <ros/ros.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>

// input: RAIL segmentation and object recognition
#include <rail_manipulation_msgs/SegmentObjects.h>
#include <rail_mesh_icp/TemplateMatch.h>

// action server: make things easier for interactive manipulation
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/DetectObjectsAction.h>

// MoveIt!
#include <moveit_msgs/ObjectColor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// auxiliary libraries
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;


namespace thorp_perception
{

class ObjectDetectionServer
{
private:
  // Service clients for RAIL object segmentation and template matchers
  ros::ServiceClient segment_srv_;
  std::map<std::string, ros::ServiceClient> match_srvs_;

  // Action server to handle it conveniently for our object manipulation demo
  actionlib::SimpleActionServer<thorp_msgs::DetectObjectsAction> od_as_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  bool publish_static_tf_;      ///< Publish static transforms for the segmented objects and surface
  double max_matching_error_;   ///< Object recognition template matching threshold
  std::string output_frame_;    ///< Recognized objects reference frame; we cannot use goal's field because
                                ///< RAIL segmentation zone's frame cannot be reconfigured on runtime
  std::vector<std::string> obj_types_;  ///< Recognized object types: list of templates we try to match to

public:

  ObjectDetectionServer(const std::string& name) :
    od_as_(name, boost::bind(&ObjectDetectionServer::executeCB, this, _1), false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("publish_static_tf", publish_static_tf_, false);  // good for debugging but pollutes a lot
    pnh.param("max_matching_error", max_matching_error_, 1e-4);
    pnh.param("output_frame", output_frame_, std::string("base_footprint"));

    XmlRpc::XmlRpcValue otv;
    pnh.param("object_types", otv, otv);
    if (otv.size() == 0)
    {
      ROS_ERROR("[object detection] No recognized object types provided; aborting...");
      return;
    }
    ROS_INFO("[object detection] Recognized object types: ");
    for (int i = 0; i < otv.size(); i++)
    {
      obj_types_.push_back(otv[i]);
      ROS_INFO_STREAM("                   - " << obj_types_[i]);
    }

    // Connect to rail_segmentation/segment_objects service
    segment_srv_ = nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
    ROS_INFO("[object detection] Waiting for rail_segmentation/segment_objects service to start...");
    if (! segment_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] rail_segmentation/segment_objects service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }
    ROS_INFO("[object detection] rail_segmentation/segment_objects service started; ready for sending goals.");

    // Connect to template_matcher_<obj type>/match_template service for each object type
    for (const auto& obj_type: obj_types_)
    {
      std::ostringstream ss; ss << "template_matcher_" << obj_type << "/match_template";
      std::string service_name = ss.str();
      match_srvs_[obj_type] = nh.serviceClient<rail_mesh_icp::TemplateMatch>(service_name);
      ROS_INFO_STREAM("[object detection] Waiting for " << service_name << " service to start...");
      if (!match_srvs_[obj_type].waitForExistence(ros::Duration(60.0)))
      {
        ROS_ERROR_STREAM("[object detection] " << service_name << " service not available after 1 minute");
        ROS_ERROR_STREAM("[object detection] Shutting down node...");
        throw;
      }
      ROS_INFO_STREAM("[object detection] " << service_name << " service started; ready for sending goals.");
    }

    // Wait for all RAIL services to connect before we provide our own action server
    od_as_.start();
  }

  void executeCB(const thorp_msgs::DetectObjectsGoal::ConstPtr& goal)
  {
    ROS_INFO("[object detection] Received goal!");
    thorp_msgs::DetectObjectsResult result;

    // Clear results from previous goals
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
    std::vector<moveit_msgs::ObjectColor> obj_colors;  // show collision objects with color

    // Process segmented support surface (normally a table) and add to the planning
    // scene as a MoveIt! collision object, so it gets filtered out from the octomap
    auto& table = srv.response.segmented_objects.objects.front();
    result.surface.header = srv.response.segmented_objects.header;
    result.surface.id = "table";
    result.surface.operation = moveit_msgs::CollisionObject::ADD;
    result.surface.primitive_poses.resize(1);
    result.surface.primitive_poses.front().position = table.center;
    result.surface.primitive_poses.front().orientation = table.orientation;
    result.surface.primitives.resize(1);
    result.surface.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
    result.surface.primitives.front().dimensions = {table.depth, table.width, table.height};
    ROS_INFO("[object detection] Adding table at %s as a collision object",
             ttk::point2cstr3D(result.surface.primitive_poses[0].position));
    planning_scene_interface_.addCollisionObjects(std::vector<moveit_msgs::CollisionObject>(1, result.surface));

    if (publish_static_tf_)
      ttk::TF2::instance().sendTransform(result.surface.primitive_poses.front(),
                                         output_frame_, result.surface.id);
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = result.surface.id;
    obj_color.color.r = table.rgb[0];
    obj_color.color.g = table.rgb[1];
    obj_color.color.b = table.rgb[2];
    obj_color.color.a = 1.0;
    obj_colors.push_back(obj_color);

    std::map<std::string, unsigned int> detected;

    // Process segmented objects following the support surface, and add them to the planning scene
    for (int i = 1; i < srv.response.segmented_objects.objects.size(); ++i)
    {
      if (od_as_.isPreemptRequested())
      {
        od_as_.setPreempted(result, "Preempted after processing " + std::to_string(i) + "/"
                                  + std::to_string(srv.response.segmented_objects.objects.size())
                                  + " segmented objects");
        return;
      }

      auto& rail_obj = srv.response.segmented_objects.objects[i];
      // reject objects of incongruent dimensions or with impossible poses, e.g. floating or embedded in the table
      if (!validateObject(rail_obj, table))
        continue;

      moveit_msgs::CollisionObject co;
      co.header = srv.response.segmented_objects.header;
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitive_poses.resize(1);
      co.primitive_poses.front().position = rail_obj.center;
      co.primitive_poses.front().orientation = rail_obj.orientation;
      co.primitives.resize(1);
      co.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
      // identify and possibly update the pose with ICP template matching correction
      // we reject objects failing to match any template within our error threshold
      if (!identifyObject(rail_obj, co.primitive_poses.front()))
        continue;
      // our convention is that x-dimension is the longest one
      double length = std::max(rail_obj.depth, rail_obj.width);
      double width = std::min(rail_obj.depth, rail_obj.width);
      co.primitives.front().dimensions = { length, width, rail_obj.height };
      // we name objects with the best matching template's name and an index to avoid repetitions
      detected[rail_obj.name] += 1;
      co.id = rail_obj.name + " " + std::to_string(detected[rail_obj.name]);
      if (publish_static_tf_)
        ttk::TF2::instance().sendTransform(co.primitive_poses.front(), output_frame_, co.id);

      // matched poses have z = 0; raise the co's primitive to entirely cover the pointcloud
      co.primitive_poses.front().position.z += rail_obj.height / 2.0;
      result.objects.push_back(co);

      // use segmented pointcloud's color for the co
      obj_color.id = co.id;
      obj_color.color.r = rail_obj.rgb[0];
      obj_color.color.g = rail_obj.rgb[1];
      obj_color.color.b = rail_obj.rgb[2];
      obj_color.color.a = 1.0;
      obj_colors.push_back(obj_color);

      ROS_INFO("[object detection] Object at %s classified as %s",
               ttk::pose2cstr2D(co.primitive_poses.front()), rail_obj.name.c_str());
    }
    if (!result.objects.empty())
    {
      planning_scene_interface_.addCollisionObjects(result.objects, obj_colors);

      ROS_INFO("[object detection] Succeeded! %lu objects detected", result.objects.size());

    }
    else
    {
      ROS_INFO("[object detection] Succeeded, but couldn't find any object on the support surface");
    }

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
    if ((obj.center.z - obj.height / 2.0) < ((table.center.z + table.height / 2.0) - TOLERANCE))
    {
      ROS_WARN("[object detection] Object at %s discarded as embedded in the table (%g < %g - %g)",
               ttk::point2cstr3D(obj.center), obj.center.z - obj.height / 2.0, table.center.z + table.height / 2.0,
               TOLERANCE);
      return false;
    }
    // ...or floating above the table (possibly the gripper after a pick/place operation)
    if ((obj.center.z - obj.height / 2.0) > ((table.center.z + table.height / 2.0) + TOLERANCE))
    {
      ROS_WARN("[object detection] Object at %s discarded as floating over the table (%g > %g + %g)",
               ttk::point2cstr3D(obj.center), obj.center.z - obj.height / 2.0, table.center.z + table.height / 2.0,
               TOLERANCE);
      return false;
    }

    return true;
  }

  bool identifyObject(rail_manipulation_msgs::SegmentedObject& obj, geometry_msgs::Pose& pose)
  {
    std::map<std::string, double> score;
    std::map<std::string, geometry_msgs::TransformStamped> poses;

    // Call template matching services in parallel with the object pointcloud, one call per template
    #pragma omp parallel for    // NOLINT
    for (size_t i = 0; i < obj_types_.size(); ++i)
    {
      const auto& obj_type = obj_types_[i];

      // provide center as initial estimate for position, but use 0, 0, 0 for orientation,
      // as it's a good guideline only for elongated objects where PCA is meaningful
      rail_mesh_icp::TemplateMatch srv;
      srv.request.initial_estimate.translation.x = obj.center.x;
      srv.request.initial_estimate.translation.y = obj.center.y;
      srv.request.initial_estimate.translation.z = obj.center.z - obj.height / 2.0; // templates' base is at z = 0
      srv.request.initial_estimate.rotation.w = 1.0;
      srv.request.target_cloud = obj.point_cloud;
      if (match_srvs_[obj_type].call(srv))
      {
        ROS_INFO_STREAM("[object detection] " << obj_type << " template match service succeeded; score: "
                                              << score[obj_type]);
        #pragma omp critical
        score[obj_type] = srv.response.match_error;
        poses[obj_type] = srv.response.template_pose;
      }
      else
      {
        ROS_ERROR_STREAM("[object detection] " << obj_type << " template match service failed");
        #pragma omp critical
        score[obj_type] = 1.0;
      }
    }

    // choose the template with the lowest error (RAIL call it score, very bad name)
    const auto& best_match = std::min_element(score.begin(), score.end(),
                                              [](const auto& l, const auto& r) { return l.second < r.second; });
    // check if best match is good enough
    if (best_match->second > max_matching_error_)
    {
      ROS_INFO_STREAM("[object detection] Best match (" << best_match->first << ") error above tolerance ("
                      << best_match->second << " > " << max_matching_error_ << ")");
      return false;
    }

    ROS_INFO_STREAM("[object detection] Best match is " << best_match->first << ", with error " << best_match->second);

    obj.name = best_match->first;
    obj.recognized = true;

    // use the matched template orientation to recalculate pointcloud dimensions
    recalcDimensions(obj, poses[best_match->first].transform);

    ttk::tf2pose(poses[best_match->first].transform, pose);
    ROS_DEBUG("Yaw updated:    %f -> %f", tf::getYaw(obj.orientation), tf::getYaw(pose.orientation));

    return true;
  }

  void recalcDimensions(rail_manipulation_msgs::SegmentedObject& obj, const geometry_msgs::Transform& tf)
  {
    // recenter object pointcloud in its estimated location (apply the inverse of its pose)
    pcl::PointCloud<pcl::PointXYZRGB> tmp_pc;
    pcl::fromROSMsg(obj.point_cloud, tmp_pc);
    tf::Transform transform;
    tf::transformMsgToTF(tf, transform);
    pcl_ros::transformPointCloud(tmp_pc, tmp_pc, transform.inverse());

    // now max - min points provide the real size for each dimension, unlike the original values that provided the
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
};

};  // namespace thorp_perception

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_action_server");

  thorp_perception::ObjectDetectionServer server("object_detection");
  ros::spin();

  return 0;
}
