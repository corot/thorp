/*
 * Author: Jorge Santos
 */

#include <ros/ros.h>

// auxiliary libraries
//#include <thorp_toolkit/common.hpp>
//#include <yocs_math_toolkit/common.hpp>
//#include <yocs_math_toolkit/geometry.hpp>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>


namespace thorp_obj_rec
{

class ObjectDetectionColor
{
private:
  image_transport::ImageTransport image_trs_;
  image_transport::CameraSubscriber image_sub_;
  image_geometry::PinholeCameraModel cam_model_;
  sensor_msgs::Image rgb_image_;

  std_msgs::ColorRGBA getRandColor(float alpha = 1.0)
  {
    std_msgs::ColorRGBA color;
    color.r = float(rand())/RAND_MAX;
    color.g = float(rand())/RAND_MAX;
    color.b = float(rand())/RAND_MAX;
    color.a = alpha;
    return color;
  }

public:
  ObjectDetectionColor(ros::NodeHandle& nh) : image_trs_(nh)
  {
    // Create the action client; spin its own thread
    ros::NodeHandle pnh("~");
////    pnh.param("ork_execute_timeout",  ork_execute_timeout_,  5.0);

    // Subscribe to an RGB image to provide color to the detected objects
    image_sub_ = image_trs_.subscribeCamera("rgb_image", 1, &ObjectDetectionColor::imageCb, this);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // TODO deberia guardar un buffer y elegir la que tenga el stamp mas cercano al stamp de la deteccion

    cam_model_.fromCameraInfo(info_msg);
    rgb_image_ = *image_msg;
  }

  std_msgs::ColorRGBA getAvgColor(sensor_msgs::PointCloud2 pc_msg, float alpha = 1.0f)
  {

    std_msgs::ColorRGBA color;
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try
    {
      input_bridge = cv_bridge::toCvCopy(rgb_image_, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[object detection] Failed to convert RBG image");
      return color;
    }

    color.a = alpha;

    int points = 0;
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      cv::Point3d pt_cv(*iter_x, *iter_y, *iter_z);
      cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
      cv::Vec3b rgb = image.at < cv::Vec3b > (uv.y, uv.x);

      color.b += rgb[0] / 255.0f;
      color.g += rgb[1] / 255.0f;
      color.r += rgb[2] / 255.0f;

      points++;
    }
    color.r /= float(points);
    color.g /= float(points);
    color.b /= float(points);
    if (points == pc_msg.width * pc_msg.height)
      ROS_ERROR("iguales !!   %d  %d", points, pc_msg.width * pc_msg.height);

    return color;
  }
};

};  // namespace thorp_obj_rec
