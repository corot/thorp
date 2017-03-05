/*
 * Author: Jorge Santos
 */

#include <tf/tf.h>
#include <ros/ros.h>

// auxiliary libraries
///#include <thorp_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

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
  const std_msgs::ColorRGBA& getAvgColor() const  { return avg_color; }

  void addObject(object_recognition_msgs::RecognizedObject object, std_msgs::ColorRGBA color)
  {
    objects.push_back(object);
    colors.push_back(color);

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


    // recalculate average color
    avg_color = std_msgs::ColorRGBA();
    for (const std_msgs::ColorRGBA& col: colors)
    {
      avg_color.r += col.r;
      avg_color.g += col.g;
      avg_color.b += col.b;
      avg_color.a += col.a;
    }

    avg_color.r /= float(colors.size());
    avg_color.g /= float(colors.size());
    avg_color.b /= float(colors.size());
    avg_color.a /= float(colors.size());

    avg_color.a = 1; // JODER  XXX
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
  std::vector<std_msgs::ColorRGBA> colors;
  std::vector<object_recognition_msgs::RecognizedObject> objects;
};

};  // namespace thorp_obj_rec
