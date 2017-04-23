/*
 * Author: Jorge Santos
 */

#include <ros/ros.h>

// action client: ORK's tabletop object recognition
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_recognition_msgs/TableArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/GetObjectInformation.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>

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
  actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ork_ac_;

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
  double confidence_threshold_;   // minimum confidence required to accept an object
  double clustering_threshold_;   // maximum acceptable distance to assign an object to a bin
  int    recognize_objs_calls_;

  std::unique_ptr<ObjectDetectionBins>  objs_detection_;
  std::unique_ptr<ObjectDetectionTable> table_detection_;
  std::shared_ptr<ObjectDetectionColor> color_detection_;

public:

  ObjectDetectionServer(const std::string& name) :
    od_as_(name, boost::bind(&ObjectDetectionServer::executeCB, this, _1), false),
    ork_ac_("tabletop/recognize_objects", true)
  {
    ros::NodeHandle nh;

    // Create the action client; spin its own thread
    ros::NodeHandle pnh("~");
    pnh.param("ork_execute_timeout",  ork_execute_timeout_,  5.0);
    pnh.param("ork_preempt_timeout",  ork_preempt_timeout_,  1.0);
    pnh.param("confidence_threshold", confidence_threshold_, 0.85);
    pnh.param("clustering_threshold", clustering_threshold_, 0.05);
    pnh.param("recognize_objs_calls", recognize_objs_calls_, 10);

    // Wait for the tabletop/recognize_objects action server to start before we provide our own service
    ROS_INFO("[object detection] Waiting for tabletop/recognize_objects action server to start...");
    if (! ork_ac_.waitForServer(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] tabletop/recognize_objects action server not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }

    ROS_INFO("[object detection] tabletop/recognize_objects action server started; ready for sending goals.");

    // Register feedback callback for our server; executeCB is run on a separated thread, so it can be cancelled
    od_as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this));
    od_as_.start();

    // Publish empty objects and table to clear ORK RViz visualizations
    clear_objs_pub_ =
        nh.advertise<object_recognition_msgs::RecognizedObjectArray>("/tabletop/recognized_object_array", 1, true);
    clear_table_pub_ =
        nh.advertise<object_recognition_msgs::TableArray>("/tabletop/table_array", 1, true);

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

    thorp_msgs::DetectObjectsResult result;

    output_frame_ = goal->output_frame;
    table_detection_->setOutputFrame(output_frame_);

    // Call ORK tabletop action server and wait for the action to return
    // We do it CALLS_TO_TABLETOP times and accumulate the results on bins to provide more reliable results
    ROS_INFO("[object detection] Sending %d goals to tabletop/recognize_objects action server...",
             recognize_objs_calls_);
    object_recognition_msgs::ObjectRecognitionGoal ork_goal;
    for (int i = 0; i < recognize_objs_calls_; ++i)
    {
      ros::spinOnce();  // keep spinning so table messages callbacks are processed

      if (od_as_.isPreemptRequested())
      {
        // set the action state to preempted
        od_as_.setPreempted();
        return;
      }

      actionlib::SimpleClientGoalState state =
          ork_ac_.sendGoalAndWait(ork_goal, ros::Duration(ork_execute_timeout_), ros::Duration(ork_preempt_timeout_));

      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("[object detection] Action successfully finished");
      }
      else
      {
        ROS_WARN("[object detection] Tabletop/recognize_objects action did not finish before the time out: %s",
                 state.toString().c_str());
        od_as_.setAborted(result, "Tabletop/recognize_objects action did not finish before the time out");
        continue;
      }

      object_recognition_msgs::ObjectRecognitionResultConstPtr result = ork_ac_.getResult();

      // Classify objects detected in each call to tabletop into bins based on distance to bin's centroid
      objs_detection_->addObservations(result->recognized_objects.objects, confidence_threshold_, clustering_threshold_);

      ros::spinOnce();  // keep spinning so table messages callbacks are processed
    }  // loop up to CALLS_TO_ORK_TABLETOP

    // Add a detected object per bin to the goal result, if the bin is consistent enough
    int added_objects = addObjects(result);
    if (added_objects > 0)
    {
      ROS_INFO("[object detection] Succeeded! %d objects detected", added_objects);

      // Add also the table as a collision object, so it gets filtered out from MoveIt! octomap
      addTable(result);

      // Clear recognized objects and tables from RViz by publishing empty messages, so they don't
      // mangle with interactive markers; shoddy... ORK visualizations should have expiration time
      object_recognition_msgs::RecognizedObjectArray roa;
      object_recognition_msgs::TableArray ta;

      roa.header.frame_id = output_frame_;
      ta.header.frame_id = output_frame_;

      clear_objs_pub_.publish(roa);
      clear_table_pub_.publish(ta);
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
               mcl::point2cstr3D(table_co.primitive_poses[0].position));
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
