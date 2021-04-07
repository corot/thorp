/*
 * Author: Jorge Santos
 */

#include <omp.h>

#include <ros/ros.h>
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

// auxiliary libraries
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;


namespace thorp_perception
{

class ObjectDetectionServer
{
private:
  typedef actionlib::ActionClient<rail_mesh_icp::MatchTemplateAction> MatchTemplateActionClient;

  // Service clients for RAIL object segmentation and template matchers
  ros::ServiceClient segment_srv_;
  MatchTemplateActionClient tm_ac_;
  ros::CallbackQueue callback_queue_;
  boost::thread* spin_thread_;

  // Action server to handle it conveniently for our object manipulation demo
  actionlib::SimpleActionServer<thorp_msgs::DetectObjectsAction> od_as_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  bool publish_static_tf_;      ///< Publish static transforms for the segmented objects and surface
  double max_matching_error_;   ///< Object recognition template matching threshold
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
    pnh.param("max_matching_error", max_matching_error_, 1e-4);
    pnh.param("output_frame", output_frame_, std::string("base_footprint"));

    // Connect to rail_segmentation/segment_objects service
    segment_srv_ = nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
    ROS_INFO("[object detection] Waiting for rail_segmentation/segment_objects service to start...");
    if (! segment_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] rail_segmentation/segment_objects service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }
    ROS_INFO("[object detection] rail_segmentation/segment_objects service started; ready for sending goals");

    // Connect to template matcher action
    spin_thread_ = new boost::thread(boost::bind(&ObjectDetectionServer::spinThread, this));
    ROS_INFO_STREAM("[object detection] Waiting for template matcher action server to start...");
    if (!tm_ac_.waitForActionServerToStart(ros::Duration(1.0)))
    {
      ROS_ERROR_STREAM("[object detection] template matcher action server not available after 1 minute");
      ROS_ERROR_STREAM("[object detection] Shutting down node...");
      throw;
    }
    ROS_INFO_STREAM("[object detection] template matcher action server started; ready for sending goals");

    // Wait for all RAIL services to connect before we provide our own action server
    od_as_.start();
  }

  void executeCB(const thorp_msgs::DetectObjectsGoal::ConstPtr& goal)
  {
    ros::Time time_start = ros::Time::now();

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
    // show collision objects with color
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = result.surface.id;
    obj_color.color.r = table.rgb[0];
    obj_color.color.g = table.rgb[1];
    obj_color.color.b = table.rgb[2];
    obj_color.color.a = 1.0;
    std::vector<moveit_msgs::ObjectColor> obj_colors{ obj_color };

    ROS_INFO("[object detection] %lu objects plus table segmented in %.2f seconds",
             srv.response.segmented_objects.objects.size() - 1, (ros::Time::now() - time_start).toSec());

    // good time for serving preempt requests, as we will process segmented objects in parallel
    if (od_as_.isPreemptRequested())
    {
      od_as_.setPreempted(result, "Preempted before processing segmented objects");
      return;
    }

    std::map<std::string, unsigned int> detected;

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
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitive_poses.resize(1);
      co.primitive_poses.front().position = rail_obj.center;
      co.primitive_poses.front().orientation = rail_obj.orientation;
      co.primitives.resize(1);
      co.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
      // identify and possibly update the pose with ICP template matching correction
      // we reject objects failing to match any template within our error threshold
      if (!identifyObject(rail_obj, co.primitive_poses.front()))
      {
        ROS_WARN("[object detection] Object %d at %s discarded", i, ttk::point2cstr3D(rail_obj.center));
        continue;
      }
      // our convention is that x-dimension is the longest one
      double length = std::max(rail_obj.depth, rail_obj.width);
      double width = std::min(rail_obj.depth, rail_obj.width);
      co.primitives.front().dimensions = { length, width, rail_obj.height };
      // we name objects with the best matching template's name and an index to avoid repetitions
      co.id = rail_obj.name + " " + std::to_string(detected[rail_obj.name] + 1);
      if (publish_static_tf_)
        ttk::TF2::instance().sendTransform(co.primitive_poses.front(), output_frame_, co.id);

      // matched poses have z = 0; raise the co's primitive to entirely cover the pointcloud
      co.primitive_poses.front().position.z += rail_obj.height / 2.0;

      // use segmented pointcloud's color for the co
      obj_color.id = co.id;
      obj_color.color.r = rail_obj.rgb[0];
      obj_color.color.g = rail_obj.rgb[1];
      obj_color.color.b = rail_obj.rgb[2];
      obj_color.color.a = 1.0;

      ROS_INFO("[object detection] Object at %s classified as %s",
               ttk::pose2cstr2D(co.primitive_poses.front()), rail_obj.name.c_str());
      #pragma omp critical    // NOLINT
      detected[rail_obj.name] += 1;
      result.objects.push_back(co);
      obj_colors.push_back(obj_color);
    }
    if (!result.objects.empty())
    {
      planning_scene_interface_.addCollisionObjects(result.objects, obj_colors);

      ROS_INFO("[object detection] Succeeded! %lu objects detected in %.2f seconds", result.objects.size(),
               (ros::Time::now() - time_start).toSec());
    }
    else
    {
      ROS_INFO("[object detection] Succeeded, but couldn't find any tabletop object (time: %.2f s)",
               (ros::Time::now() - time_start).toSec());
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
    // Call template matching action with the object pointcloud
    // Note that we log all errors multiplied by 1e6, as the values provided by the template matcher are very low
    // provide center as initial estimate for position, but use 0, 0, 0 for orientation,
    // as it's a good guideline only for elongated objects, where PCA is meaningful
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
      ROS_ERROR_STREAM("[object detection] template match action timeout; aborting...");
      goal_handle.cancel();
      return false;
    }
    if (goal_handle.getTerminalState() != actionlib::TerminalState::SUCCEEDED)
    {
      ROS_ERROR_STREAM("[object detection] template match action failed: " << goal_handle.getTerminalState().getText());
      return false;
    }

    rail_mesh_icp::MatchTemplateResult::ConstPtr result = goal_handle.getResult();

    // check if best match is good enough
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

    // use the matched template orientation to recalculate pointcloud dimensions
    recalcDimensions(obj, result->template_pose.transform);

    ttk::tf2pose(result->template_pose.transform, pose);
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

  void spinThread()
  {
    while (ros::ok()) {
      {
        //boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
//        if (need_to_terminate_) {
//          break;
//        }
      }
      callback_queue_.callAvailable(ros::WallDuration(0.1f));
    }
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
