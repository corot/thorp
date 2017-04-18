/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
///#include <thorp_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

#include "thorp_obj_rec/object_detection_color.hpp"


namespace thorp_obj_rec
{

/**
 * Class for clustering together the same object detected in consecutive observations
 * and provide a more accurate pose as the centroid of all observations.
 */
class ObjectDetectionBin
{
public:
  unsigned int id;
  unsigned int countObjects() const { return objects.size(); }
  const geometry_msgs::PoseStamped& getCentroid() const { return centroid; }
  const std::vector<cv::Vec3b>& getColorPoints() const  { return color_points; }

  void addObject(object_recognition_msgs::RecognizedObject object, const std::vector<cv::Vec3b>& colors)
  {
    objects.push_back(object);

    // recalculate centroid
    centroid = geometry_msgs::PoseStamped();
    double confidence_acc = 0.0, roll_acc = 0.0, pitch_acc = 0.0, yaw_acc = 0.0;
    for (const object_recognition_msgs::RecognizedObject& obj: objects)
    {
      centroid.header.stamp = ros::Time::now();
      centroid.header.frame_id = obj.pose.header.frame_id; // TODO could do some checking

      centroid.pose.position.x += obj.pose.pose.pose.position.x * obj.confidence;
      centroid.pose.position.y += obj.pose.pose.pose.position.y * obj.confidence;
      centroid.pose.position.z += obj.pose.pose.pose.position.z * obj.confidence;
      roll_acc                 += mtk::roll(obj.pose.pose.pose) * obj.confidence;
      pitch_acc                += mtk::pitch (obj.pose.pose.pose) * obj.confidence;
      yaw_acc                  += mtk::yaw(obj.pose.pose.pose) * obj.confidence;

      confidence_acc += obj.confidence;
    }

    centroid.pose.position.x /= confidence_acc;
    centroid.pose.position.y /= confidence_acc;
    centroid.pose.position.z /= confidence_acc;
    centroid.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_acc/confidence_acc,
                                                                        pitch_acc/confidence_acc,
                                                                        yaw_acc/confidence_acc);
    confidence = confidence_acc/(double)objects.size();

    // accumulate color points for calculate the average
    color_points.insert(color_points.end(), colors.begin(), colors.end());
  }

  std::string getName() const
  {
    std::map<std::string, unsigned int> key_occurences;
    for (const object_recognition_msgs::RecognizedObject& obj: objects)
      key_occurences[obj.type.key]++;

    std::string commonest_key;
    std::map<std::string, unsigned int>::iterator it;
    for(it = key_occurences.begin(); it != key_occurences.end(); it++)
    {
      if (it->second > key_occurences[commonest_key])
        commonest_key = it->first;
    }
    return commonest_key;
  }

  object_recognition_msgs::ObjectType getType() const
  {
    std::string commonest_key = getName();
    for (const object_recognition_msgs::RecognizedObject& obj: objects)
      if (obj.type.key == commonest_key)
        return obj.type;

    return object_recognition_msgs::ObjectType();
  }

private:
  double confidence;
  std_msgs::ColorRGBA avg_color;
  geometry_msgs::PoseStamped centroid;
  std::vector<cv::Vec3b> color_points;
  std::vector<object_recognition_msgs::RecognizedObject> objects;
};


/**
 * Class for clustering together the same object detected in consecutive observations
 * and provide a more accurate pose as the centroid of all observations.
 */
class ObjectDetectionBins
{
public:
  ObjectDetectionBins(std::shared_ptr<ObjectDetectionColor> color) :
      color_detection_(color)
  {
    ros::NodeHandle nh;

    // Wait for the get object information service (mandatory, as we need to know objects' mesh)
    obj_info_srv_ = nh.serviceClient<object_recognition_msgs::GetObjectInformation>("get_object_info");
    if (! obj_info_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] Get object information service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }
  }

  void addObservations(const std::vector<object_recognition_msgs::RecognizedObject>& objects,
                       double confidence_threshold, double clustering_threshold)
  {
    // Classify objects detected in each call to tabletop into bins based on distance to bin's centroid
    for (const object_recognition_msgs::RecognizedObject& obj: objects)
    {
      if (obj.confidence < confidence_threshold)
        continue;

      ros::Time t0 = ros::Time::now();
      bool assigned = false;
      for (ObjectDetectionBin& bin: detection_bins_)
      {
        if (mtk::distance3D(bin.getCentroid().pose, obj.pose.pose.pose) <= clustering_threshold)
        {
          ROS_DEBUG("Object with pose [%s] added to bin %d with centroid [%s] with distance [%f]",
                    mtk::pose2str3D(obj.pose.pose.pose).c_str(), bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(),
                    mtk::distance3D(bin.getCentroid().pose, obj.pose.pose.pose));
          ROS_ASSERT(obj.point_clouds.size() == 1);
          bin.addObject(obj, color_detection_->getColors(obj.point_clouds[0]));
          assigned = true;
          break;
        }
      }
      ROS_ERROR("%f", (ros::Time::now()-t0).toSec());

      if (! assigned)
      {
        // No matching bin; create a new one for this object
        ROS_DEBUG("Object with pose [%s] added to a new bin", mtk::pose2str3D(obj.pose.pose.pose).c_str());
        ObjectDetectionBin new_bin;
        new_bin.id = detection_bins_.size();
        new_bin.addObject(obj, color_detection_->getColors(obj.point_clouds[0]));
        detection_bins_.push_back(new_bin);
      }
    }
  }

  int getDetectedObjects(unsigned int min_detections, const std::string& output_frame,
                         std::vector<moveit_msgs::CollisionObject>& objects, std::vector<std::string>& obj_names,
                         std::vector<moveit_msgs::ObjectColor>& obj_colors)
  {
    std::map<std::string, unsigned int> obj_name_occurences;

    // Add a detected object per bin to the goal result and to the planning scene as a collision object
    // Only bins receiving detections on most of the ORK tabletop calls are considered consistent enough
    for (const ObjectDetectionBin& bin: detection_bins_)
    {
      if (bin.countObjects() < min_detections)
      {
        ROS_DEBUG("Bin %d with centroid [%s] discarded as it received %d objects (at least %d required)",
                   bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(), bin.countObjects(), min_detections);
        continue;
      }

      std_msgs::ColorRGBA color = color_detection_->getAvgColor(bin.getColorPoints());
      std::string color_label = color_detection_->getColorLabel(color);

      // Compose object name with the name provided by the database plus an index, starting with [1]
      object_recognition_msgs::ObjectInformation obj_info = getObjInfo(bin.getType());
      obj_name_occurences[obj_info.name]++;
      std::stringstream sstream;
      sstream << obj_info.name << " [" << obj_name_occurences[obj_info.name] << "] " << color_label;
      std::string obj_name = sstream.str();

      ROS_DEBUG("Bin %d with centroid [%s] and %d objects added as object '%s'",
                 bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(), bin.countObjects(), obj_name.c_str());

      geometry_msgs::Pose out_pose;
      thorp_toolkit::transformPose(bin.getCentroid().header.frame_id, output_frame,
                                   bin.getCentroid().pose, out_pose);
      ROS_INFO("[object detection] Adding '%s' object at %s",
               obj_name.c_str(), mtk::point2str3D(out_pose.position).c_str());

      moveit_msgs::CollisionObject co;
      co.id = obj_name;
      co.header.frame_id = output_frame;
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.meshes.resize(1, obj_info.ground_truth_mesh);
      co.mesh_poses.push_back(out_pose);

      obj_names.push_back(obj_name);
      objects.push_back(co);

      // Provide the detected color to the collision object
      moveit_msgs::ObjectColor oc;
      oc.id = co.id;
      oc.color = color;
      obj_colors.push_back(oc);
    }
  }

  void clear()
  {
    detection_bins_.clear();
  }

private:
  std::vector<ObjectDetectionBin> detection_bins_;
  std::shared_ptr<ObjectDetectionColor> color_detection_;

  // Get object information from database service and keep in a map
  ros::ServiceClient obj_info_srv_;
  std::map<std::string, object_recognition_msgs::ObjectInformation> objs_info_;


  const object_recognition_msgs::ObjectInformation& getObjInfo(const object_recognition_msgs::ObjectType& obj_type)
  {
    if (objs_info_.find(obj_type.key) == objs_info_.end() )
    {
      // Get object information from db using object's type
      object_recognition_msgs::GetObjectInformation srv;
      srv.request.type = obj_type;

      if (obj_info_srv_.call(srv))
      {
        ROS_DEBUG("Database information retrieved for object '%s'", obj_type.key.c_str());
        objs_info_[obj_type.key] = srv.response.information;
      }
      else
      {
        ROS_ERROR("Call to object information service with key '%s' failed", obj_type.key.c_str());
        throw;
      }
    }

    return objs_info_[obj_type.key];
  }
};

};  // namespace thorp_obj_rec
