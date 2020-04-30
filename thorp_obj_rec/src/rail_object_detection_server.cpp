/*
 * Author: Jorge Santos
 */

#include <ros/ros.h>

// input: RAIL segmentation and object recognition
#include <rail_manipulation_msgs/SegmentObjects.h>

// output: ORK's tabletop object recognition
#include <object_recognition_msgs/TableArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/GetObjectInformation.h>

// action server: make things easier for interactive manipulation
#include <actionlib/server/simple_action_server.h>
#include <thorp_msgs/DetectObjectsAction.h>

// MoveIt!
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// local includes for utility classes
#include "thorp_obj_rec/object_detection_bin.hpp"
#include "thorp_obj_rec/object_detection_table.hpp"


namespace thorp_obj_rec
{

class ObjectDetectionServer
{
private:

  // Action client for the ORK object recognition and server
  ros::ServiceClient rail_srv_;

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
  double ork_execute_timeout_;
  double ork_preempt_timeout_;
  int    recognize_objs_calls_;

  std::unique_ptr<ObjectDetectionBins>  objs_detection_;
  std::unique_ptr<ObjectDetectionTable> table_detection_;
  std::shared_ptr<ObjectDetectionColor> color_detection_;

public:

  ObjectDetectionServer(const std::string& name) :
    od_as_(name, boost::bind(&ObjectDetectionServer::executeCB, this, _1), false)
  {
    ros::NodeHandle nh;

    // Wait for the rail_segmentation/segment_objects service to start before we provide our own service
    ros::NodeHandle pnh("~");
//    pnh.param("ork_execute_timeout",  ork_execute_timeout_,  5.0);
//    pnh.param("ork_preempt_timeout",  ork_preempt_timeout_,  1.0);
    pnh.param("recognize_objs_calls", recognize_objs_calls_, 1);

    rail_srv_ = nh.serviceClient<rail_manipulation_msgs::SegmentObjects>("rail_segmentation/segment_objects");
    ROS_INFO("[object detection] Waiting for rail_segmentation/segment_objects service to start...");
    if (! rail_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] rail_segmentation/segment_objects service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }

    ROS_INFO("[object detection] rail_segmentation/segment_objects service started; ready for sending goals.");

    // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
    od_as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this));
    od_as_.start();

    // Publish empty objects and table to clear ORK RViz visualizations
//    clear_objs_pub_ =
//        nh.advertise<object_recognition_msgs::RecognizedObjectArray>("/tabletop/recognized_object_array", 1, true);
//    clear_table_pub_ =
//        nh.advertise<object_recognition_msgs::TableArray>("/tabletop/table_array", 1, true);

    color_detection_ = std::make_shared<ObjectDetectionColor>(nh);
    objs_detection_.reset(new ObjectDetectionBins(color_detection_));
    table_detection_.reset(new ObjectDetectionTable());
  }

  void executeCB(const thorp_msgs::DetectObjectsGoal::ConstPtr& goal)
  {
    ROS_INFO("[object detection] Received goal!");

    // Clear results from previous goals
    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());
    table_detection_->clear();
    objs_detection_->clear();

    output_frame_ = goal->output_frame;
    table_detection_->setOutputFrame(output_frame_);

    // Call ORK tabletop action server and wait for the action to return
    // We do it CALLS_TO_TABLETOP times and accumulate the results on bins to provide more reliable results
    ROS_INFO("[object detection] Calling rail_segmentation/segment_objects service for %d times...",
             recognize_objs_calls_);
//    for (int i = 0; i < recognize_objs_calls_; ++i)
//    {
///      ros::spinOnce();  // keep spinning so table messages callbacks are processed

      rail_manipulation_msgs::SegmentObjects srv;
      rail_manipulation_msgs::SegmentedObjectList segmented_objects;
      if (rail_srv_.call(srv))
      {
        ROS_INFO("[object detection] rail_segmentation/segment_objects service call succeeded");
      }
      else
      {
        ROS_WARN("[object detection] rail_segmentation/segment_objects service call failed");
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
      moveit_msgs::PlanningScene ps;
      ps.is_diff = true;

      thorp_msgs::DetectObjectsResult result;
      result.objects.resize(srv.response.segmented_objects.objects.size());
      result.object_names.resize(srv.response.segmented_objects.objects.size());
      ps.object_colors.resize(srv.response.segmented_objects.objects.size());
      // TODO  result.support_surf   = table_co.id;
      for (int i = 0; i < srv.response.segmented_objects.objects.size(); ++i)
      {
        auto& rail_obj = srv.response.segmented_objects.objects[i];
        result.objects[i].header = srv.response.segmented_objects.header;
        result.objects[i].id = std::to_string(i + 1);  // TODO rail_obj.name is empty,,,  I need to recognize!!!
        result.objects[i].operation = moveit_msgs::CollisionObject::ADD;
        result.objects[i].primitive_poses.resize(1);
        result.objects[i].primitive_poses.front().position = rail_obj.centroid;  // or center???
        result.objects[i].primitive_poses.front().orientation = rail_obj.orientation;
        result.objects[i].primitives.resize(1);
        result.objects[i].primitives.front().type = shape_msgs::SolidPrimitive::BOX;
        result.objects[i].primitives.front().dimensions = {rail_obj.width, rail_obj.depth, rail_obj.height};

        ps.object_colors[i].id = rail_obj.name;  // TODO probably empty
        ps.object_colors[i].color.r = rail_obj.rgb[0];
        ps.object_colors[i].color.g = rail_obj.rgb[1];
        ps.object_colors[i].color.b = rail_obj.rgb[2];
        ps.object_colors[i].color.a = 1.0;
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

        ROS_INFO("[object detection] Objects %s detected %d", rail_obj.name.c_str(), rail_obj.recognized);
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
    if (result.objects.size() > 0)
    {
      planning_scene_interface_.addCollisionObjects(result.objects, ps.object_colors);

      ROS_INFO("[object detection] Succeeded! %d objects detected", result.objects.size());

      // Add also the table as a collision object, so it gets filtered out from MoveIt! octomap
      addTable(result);

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
      ROS_INFO("[object detection] Succeeded, but couldn't find any object");
    }

    od_as_.setSucceeded(result);
  }

  void preemptCB()
  {
    ROS_WARN("[object detection] Action preempted; cancel detection in course");
  }

private:

  int addObjects(thorp_msgs::DetectObjectsResult& result)
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
  }
};

};  // namespace thorp_obj_rec

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_action_server");

  thorp_obj_rec::ObjectDetectionServer server("object_detection");
  ros::spin();

  return 0;
}
