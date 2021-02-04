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
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// auxiliary libraries
#include <thorp_toolkit/tf2.hpp>
#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

// local includes for utility classes
//#include "thorp_perception/object_detection_bin.hpp"
//#include "thorp_perception/object_detection_table.hpp"


namespace thorp_perception
{

class ObjectDetectionServer
{
private:

  std::vector<std::string> obj_types_ = {"tower",
                                         "cube",
                                         "square",
                                         "rectangle",
                                         "triangle",
                                         "pentagon",
                                         "circle",
                                         "star",
                                         "diamond",
                                         "cross",
                                         "clover"};
  // Action client for the ORK object recognition and server
  ros::ServiceClient segment_srv_;
  std::map<std::string, ros::ServiceClient> match_srvs_;

  // Action server to handle it conveniently for our object manipulation demo
  actionlib::SimpleActionServer<thorp_msgs::DetectObjectsAction> od_as_;

  // Publishers and subscribers
  ros::Publisher clear_objs_pub_;
  ros::Publisher clear_table_pub_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Parameters from goal
  std::string output_frame_;

  // Object detection and classification parameters
  double max_matching_error_;

  double elongated_threshold_;
//  double ork_execute_timeout_;
//  double ork_preempt_timeout_;
//  int    recognize_objs_calls_;

//  std::unique_ptr<ObjectDetectionBins>  objs_detection_;
//  std::unique_ptr<ObjectDetectionTable> table_detection_;
//  std::shared_ptr<ObjectDetectionColor> color_detection_;

public:

  ObjectDetectionServer(const std::string& name) :
    od_as_(name, boost::bind(&ObjectDetectionServer::executeCB, this, _1), false)
  {
    ros::NodeHandle nh;

    // Wait for the rail_segmentation/segment_objects service to start before we provide our own service
    ros::NodeHandle pnh("~");
//    pnh.param("ork_execute_timeout",  ork_execute_timeout_,  5.0);
//    pnh.param("ork_preempt_timeout",  ork_preempt_timeout_,  1.0);
//    pnh.param("recognize_objs_calls", recognize_objs_calls_, 1);
    pnh.param("max_matching_error", max_matching_error_, 1e-4);
    pnh.param("elongated_threshold", elongated_threshold_, 1.2);

    segment_srv_ = nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
    ROS_INFO("[object detection] Waiting for rail_segmentation/segment_objects service to start...");
    if (! segment_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] rail_segmentation/segment_objects service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }
    ROS_INFO("[object detection] rail_segmentation/segment_objects service started; ready for sending goals.");

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

    // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
    od_as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this)); //TODO   seguro q no va,,,,
    od_as_.start();

    // Publish empty objects and table to clear ORK RViz visualizations
//    clear_objs_pub_ =
//        nh.advertise<object_recognition_msgs::RecognizedObjectArray>("/tabletop/recognized_object_array", 1, true);
//    clear_table_pub_ =
//        nh.advertise<object_recognition_msgs::TableArray>("/tabletop/table_array", 1, true);

//    color_detection_ = std::make_shared<ObjectDetectionColor>(nh);
//    objs_detection_.reset(new ObjectDetectionBins(color_detection_));
//    table_detection_.reset(new ObjectDetectionTable());
  }

  void executeCB(const thorp_msgs::DetectObjectsGoal::ConstPtr& goal)
  {
    ROS_INFO("[object detection] Received goal!");
    thorp_msgs::DetectObjectsResult result;

    // Clear results from previous goals
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());
//    table_detection_->clear();
//    objs_detection_->clear();

    output_frame_ = goal->output_frame;
//    table_detection_->setOutputFrame(output_frame_);

    // Call ORK tabletop action server and wait for the action to return
    // We do it CALLS_TO_TABLETOP times and accumulate the results on bins to provide more reliable results
    ROS_INFO("[object detection] Calling rail_segmentation/segment_objects service...");
//    for (int i = 0; i < recognize_objs_calls_; ++i)
//    {
///      ros::spinOnce();  // keep spinning so table messages callbacks are processed

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
//      std::vector<object_recognition_msgs::RecognizedObject> rec_objects(srv.response.segmented_objects.objects.size());
//      for (int i = 0; i < srv.response.segmented_objects.objects.size(); ++i)
//      {
//        auto& rail_obj = srv.response.segmented_objects.objects[i];
//        rec_objects[i].header = srv.response.segmented_objects.header;
////        rec_objects[i].type   LO USO???
//        rec_objects[i].confidence = rail_obj.confidence;
//        rec_objects[i].point_clouds.emplace_back(rail_obj.point_cloud);
////        rec_objects[i].bounding_mesh =   shape_msgs/Mesh   rail_obj.bounding_volume
////        geometry_msgs/Point[] bounding_contours
//        rec_objects[i].pose.header = rec_objects[i].header;
//        rec_objects[i].pose.pose.pose.position = rail_obj.centroid;  // or center???
//        rec_objects[i].pose.pose.pose.orientation = rail_obj.orientation;
//
//
//        mmm...  esto me da el color,,, y parece mas estable... quizas podria pasar de las otras clases y usar esto directamente
//        -->  rellenar result!!!
//        moveit_msgs/CollisionObject[] objects
//        string[] object_names
//        string   support_surf
//
    std::vector<moveit_msgs::ObjectColor> obj_colors;
//    ps.is_diff = true;

//    result.objects.resize(srv.response.segmented_objects.objects.size());
//    result.object_names.resize(srv.response.segmented_objects.objects.size());
//    obj_colors.resize(srv.response.segmented_objects.objects.size());

    auto& table = srv.response.segmented_objects.objects.front();
    result.support_surf.header = srv.response.segmented_objects.header;
    result.support_surf.id = "table";
    result.support_surf.operation = moveit_msgs::CollisionObject::ADD;
    result.support_surf.primitive_poses.resize(1);
    result.support_surf.primitive_poses.front().position = table.center;
    result.support_surf.primitive_poses.front().orientation = table.orientation;
    result.support_surf.primitives.resize(1);
    result.support_surf.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
    result.support_surf.primitives.front().dimensions = {table.depth, table.width, table.height};
    ROS_INFO("[object detection] Adding table at %s as a collision object",
             ttk::point2cstr3D(result.support_surf.primitive_poses[0].position));
    planning_scene_interface_.addCollisionObjects(std::vector<moveit_msgs::CollisionObject>(1, result.support_surf));

    ttk::TF2::instance().sendTransform(result.support_surf.primitive_poses.front(),
                                       "base_footprint", result.support_surf.id);
    moveit_msgs::ObjectColor obj_color;
    obj_color.id = result.support_surf.id;
    obj_color.color.r = table.rgb[0];
    obj_color.color.g = table.rgb[1];
    obj_color.color.b = table.rgb[2];
    obj_color.color.a = 1.0;
    obj_colors.push_back(obj_color);

    std::map<std::string, unsigned int> detected;

    for (int i = 1; i < srv.response.segmented_objects.objects.size(); ++i)
    {
      auto& rail_obj = srv.response.segmented_objects.objects[i];
      if (!validatePose(rail_obj, table))
        continue;

      moveit_msgs::CollisionObject co;
      co.header = srv.response.segmented_objects.header;
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitive_poses.resize(1);
//      co.primitive_poses.front().position = rail_obj.center;
//      co.primitive_poses.front().orientation = rail_obj.orientation;
      co.primitives.resize(1);
      co.primitives.front().type = shape_msgs::SolidPrimitive::BOX;
      if (!identifyObject(rail_obj, co.primitive_poses.front()))  // TODO move up if I don't pass the pose anymore
        continue;
      bool x_aligned = ttk::xAligned(co.primitive_poses.front());
      double length = std::max(rail_obj.depth, rail_obj.width);
      double width = std::min(rail_obj.depth, rail_obj.width);
      co.primitives.front().dimensions = { length, width, rail_obj.height };
      detected[rail_obj.name] += 1;
      co.id = rail_obj.name + " " + std::to_string(detected[rail_obj.name]);
      ROS_ERROR_STREAM(""<<co.id << "   x-a? " <<x_aligned << "   " << rail_obj.width << "   " << rail_obj.depth << "   -->    " <<length << "   " <<width);
      // matched poses have z = 0; raise the co's primitive to cover the pointcloud
      co.primitive_poses.front().position.z += rail_obj.height / 2.0;
      result.objects.push_back(co);
      result.object_names.push_back(co.id);

      obj_color.id = co.id;
      obj_color.color.r = rail_obj.rgb[0];
      obj_color.color.g = rail_obj.rgb[1];
      obj_color.color.b = rail_obj.rgb[2];
      obj_color.color.a = 1.0;
      obj_colors.push_back(obj_color);
      //result.objects[i].mesh_poses =
      //result.objects[i].meshes =
//        result.objects[i].confidence = rail_obj.confidence;
//        result.objects[i].point_clouds.emplace_back(rail_obj.point_cloud);
////        result.objects[i].bounding_mesh =   shape_msgs/Mesh   rail_obj.bounding_volume
////        geometry_msgs/Point[] bounding_contours
//        result.objects[i].pose.header = result.objects[i].header;
//        result.objects[i].pose.pose.pose.position = rail_obj.centroid;  // or center???
//        result.objects[i].pose.pose.pose.orientation = rail_obj.orientation;
//
//          moveit_msgs/CollisionObject[] objects
//        string[] object_names
//        string   support_surf

      ROS_INFO("[object detection] Object at %s classified as %s",
               ttk::pose2cstr2D(co.primitive_poses.front()), rail_obj.name.c_str());
//
//        sensor_msgs/Image image                                 # Segmented RGB image
//        geometry_msgs/Point centroid                            # Centroid of the point cloud
//        geometry_msgs/Point center                              # Center of the point cloud
//        rail_manipulation_msgs/BoundingVolume bounding_volume   # minimum bounding rectangular prism
//        float64 width                                           # The width of the object in meters (x in point cloud frame)
//        float64 depth                                           # The depth of the object in meters (y in point cloud frame)
//        float64 height                                          # The height of the object in meters (z in point cloud frame)
//        float32[] rgb                                           # Average color in RGB color space
//        float32[] cielab                                        # Average color in CIELAB color space
//        geometry_msgs/Quaternion orientation                    # Orientation of the object (typically from object rec)
//        bool recognized                                         # True if the object is recognized
//        string name                                             # Object name (if recognized)
//        uint32 model_id                                         # Object model ID (if recognized)
//        float64 confidence                                      # Recognition confidence value
//        Grasp[] grasps                                          # List of grasps (if recognized)
//        visualization_msgs/Marker marker                        # The downsampled visualization of the object
//        int32[] image_indices                                   # Indices of the segmented points in the 2D image coordinate
    }

//      // Classify objects detected in each call to tabletop into bins based on distance to bin's centroid
//      objs_detection_->addObservations(result->recognized_objects.objects);
//
//      ros::spinOnce();  // keep spinning so table messages callbacks are processed
//    }  // loop up to CALLS_TO_ORK_TABLETOP

    // Add a detected object per bin to the goal result, if the bin is consistent enough
    // Add a detected object per bin to the goal result and to the planning scene as a collision object
    // Only bins receiving detections on most of the ORK tabletop calls are considered consistent enough
    if (!result.objects.empty())
    {
      planning_scene_interface_.addCollisionObjects(result.objects, obj_colors);

      ROS_INFO("[object detection] Succeeded! %lu objects detected", result.objects.size());

      // Add also the table as a collision object, so it gets filtered out from MoveIt! octomap
  ////    addTable(result);

//      // Clear recognized objects and tables from RViz by publishing empty messages, so they don't
//      // mangle with interactive markers; shoddy... ORK visualizations should have expiration time
//      object_recognition_msgs::RecognizedObjectArray roa;
//      object_recognition_msgs::TableArray ta;
//
//      roa.header.frame_id = output_frame_;
//      ta.header.frame_id = output_frame_;
//
//      clear_objs_pub_.publish(roa);
//      clear_table_pub_.publish(ta);
    }
    else
    {
      ROS_INFO("[object detection] Succeeded, but couldn't find any object on the support surface");
    }

    od_as_.setSucceeded(result);
  }

  void preemptCB()
  {
    ROS_WARN("[object detection] Action preempted; cancel detection in course");
  }

private:
  bool validatePose(const rail_manipulation_msgs::SegmentedObject& obj,
                    const rail_manipulation_msgs::SegmentedObject& table)
  {
    double TOLERANCE = 0.01;
    // reject objects embedded in the table (possibly table parts not properly removed)...
    if ((obj.center.z - obj.height / 2.0) < (table.center.z + table.height / 2.0 - TOLERANCE))
    {
      ROS_WARN("[object detection] Object at %s discarded as embedded in the table (%g < %g - %g)",
               ttk::point2cstr3D(obj.center), obj.center.z - obj.height / 2.0, table.center.z + table.height / 2.0,
               TOLERANCE);
      return false;
    }
    // ...or floating above the table (possibly the gripper after a pick/place operation)
    if ((obj.center.z - obj.height / 2.0) > (table.center.z + table.height / 2.0 + TOLERANCE))
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

    #pragma omp parallel for    // NOLINT
    for (size_t i = 0; i < obj_types_.size(); ++i)
    {
      const auto& obj_type = obj_types_[i];
      rail_mesh_icp::TemplateMatch srv;
      srv.request.initial_estimate.translation.x = obj.center.x;
      srv.request.initial_estimate.translation.y = obj.center.y;
      srv.request.initial_estimate.translation.z = obj.center.z - obj.height / 2.0; // templates' base is at z = 0
      srv.request.initial_estimate.rotation = obj.orientation;
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

    geometry_msgs::Transform tf;
    tf.translation.x = obj.center.x;
    tf.translation.y = obj.center.y;
    tf.translation.z = obj.center.z - obj.height / 2.0; // templates' base is at z = 0
    tf.rotation = obj.orientation;
    ttk::TF2::instance().sendTransform(tf, "base_footprint", obj.name + "_PCA");       // TODO disable w/ param
    ttk::TF2::instance().sendTransform(poses[best_match->first].transform, "base_footprint", obj.name + "_TM");  // TODO remove
    ttk::tf2pose(tf, pose);
//    pose.position.x = obj.center.x;   /// TODO move this up and remove the pose argument
//    pose.position.y = obj.center.y;
//    pose.position.z = obj.center.z - obj.height / 2.0; // templates' base is at z = 0
//    pose.orientation = obj.orientation;

    // recalculate dimensions with the corrected orientation
    if (recalcDimensions(obj, tf))   // TODO pass
    {
      // matched tf orientation is almost always worse, son don't use it
      ////TODO  ttk::tf2pose(poses[best_match->first].transform, pose);
    }
    else
    {

    }

    double aspect_ratio = std::max(obj.width, obj.depth) / std::min(obj.width, obj.depth);

    if (aspect_ratio < elongated_threshold_)
    {
      ROS_INFO("Aspect ratio within elongated threshold (%f < %g); recalculate dimensions rotated 45 degrees",
               aspect_ratio, elongated_threshold_);

      tf.rotation = tf::createQuaternionMsgFromYaw(tf::getYaw(tf.rotation) + M_PI_4);
      if (recalcDimensions(obj, tf))
        pose.orientation = tf.rotation;

      ttk::TF2::instance().sendTransform(tf, "base_footprint", obj.name + "_R45");       // TODO disable w/ param
    }


    return true;
  }

  bool recalcDimensions(rail_manipulation_msgs::SegmentedObject& obj, const geometry_msgs::Transform& tf)
  {
    pcl::PointCloud<pcl::PointXYZRGB> tmp_pc;  // explain
    pcl::fromROSMsg(obj.point_cloud, tmp_pc);
    pcl_ros::transformPointCloud(tmp_pc, tmp_pc, tf);

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(tmp_pc, min_pt, max_pt);
    ROS_ERROR("%f -> %f", tf::getYaw(obj.orientation), tf::getYaw(tf.rotation));
    double new_width = max_pt[0] - min_pt[0];
    double new_depth = max_pt[1] - min_pt[1];
    if (new_width * new_depth < obj.width * obj.depth)  // TODO explain
    {
      ROS_ERROR("W:  %f -> %f", obj.width, new_width);
      ROS_ERROR("D:  %f -> %f", obj.depth, new_depth);
      ROS_ERROR("A:  %f -> %f", obj.width * obj.depth, new_width * new_depth);
      obj.width = new_width;
      obj.depth = new_depth;
      return true;    ///    esto tendria sentido en segmenter, igual que con la mesa
    }
    return false;
  }
/*  int addObjects(thorp_msgs::DetectObjectsResult& result)
  {
    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;

    // Add a detected object per bin to the goal result and to the planning scene as a collision object
    // Only bins receiving detections on most of the ORK tabletop calls are considered consistent enough
    objs_detection_->getDetectedObjects(recognize_objs_calls_/1.5, output_frame_,
                                        result.objects, result.object_names, ps.object_colors);
    if (result.objects.size() > 0)
    {
      planning_scene_interface_.addCollisionObjects(result.objects, ps.object_colors);
    }

    return result.objects.size();
  }

  void addTable(thorp_msgs::DetectObjectsResult& result)
  {
    moveit_msgs::CollisionObject table_co;
    if (table_detection_->getDetectedTable(table_co))
    {
      // Add the table as a collision object into the world, so it gets excluded from the collision map
      ROS_INFO("[object detection] Adding a table at %s as a collision object",
               ttk::point2cstr3D(table_co.primitive_poses[0].position));
      planning_scene_interface_.addCollisionObjects(std::vector<moveit_msgs::CollisionObject>(1, table_co));

      // Add "table" as the support surface on action result
      result.support_surf = table_co.id;
    }
    else
    {
      ROS_WARN("[object detection] No near-horizontal table detected!");
    }
  }*/
};

};  // namespace thorp_perception

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_action_server");

  thorp_perception::ObjectDetectionServer server("object_detection");
  ros::spin();

  return 0;
}
