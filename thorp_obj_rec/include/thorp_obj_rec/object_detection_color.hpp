/*
 * Author: Jorge Santos
 */

#include <mutex>
#include <unordered_map>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <mag_common_cpp_libs/common.hpp>
#include <mag_common_cpp_libs/geometry.hpp>
namespace mcl = mag_common_libs;

namespace thorp_obj_rec
{

class ObjectDetectionColor
{
public:
  ObjectDetectionColor(ros::NodeHandle& nh) : image_trs_(nh)
  {
    // Subscribe to an RGB image to provide color to the detected objects
    image_sub_ = image_trs_.subscribeCamera("rgb_image", 1, &ObjectDetectionColor::imageCb, this);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // TODO deberia guardar un buffer y elegir la que tenga el stamp mas cercano al stamp de la deteccion

    if (!cam_model_.initialized())
      cam_model_.fromCameraInfo(info_msg);
    image_mutex_.lock();
    rgb_image_ = *image_msg;
    image_mutex_.unlock();
  }

  /**
   * Obtain color from the latest RGB image for each point in a pointcloud
   */
  std::vector<cv::Vec3b> getColors(sensor_msgs::PointCloud2 pc_msg)
  {
    std::vector<cv::Vec3b> colors;

    cv::Mat image_bgr, image_hsv;
    cv_bridge::CvImagePtr input_bridge;
    try
    {
      image_mutex_.lock();
      input_bridge = cv_bridge::toCvCopy(rgb_image_, sensor_msgs::image_encodings::BGR8);
      image_mutex_.unlock();
      image_bgr = input_bridge->image;

      cv::cvtColor(image_bgr, image_hsv, cv::COLOR_BGR2HSV);
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[object detection] Failed to convert RBG image");
      return colors;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

    std::unordered_map<uint32_t, cv::Vec3b> color_map;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      cv::Point3d pt_cv(*iter_x, *iter_y, *iter_z);
      cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
      cv::Vec3b hsv = image_hsv.at<cv::Vec3b>(uv.y, uv.x);
      color_map[int(uv.y)<<16 + int(uv.x)] = hsv;
    }
    for (auto color: color_map)
      colors.emplace_back(color.second[0], color.second[1], color.second[2]);

    return colors;
  }

  /**
   * Calculate the average (median, in fact) color of a set of HSV color points
   */
  std_msgs::ColorRGBA getAvgColor(const std::vector<cv::Vec3b>& color_points, float alpha = 1.0f)
  {
    std_msgs::ColorRGBA color;

    std::vector<uint8_t> H;
    std::vector<uint8_t> S;
    std::vector<uint8_t> V;
    for (auto& hsv : color_points)
    {
      H.push_back(hsv[0]);
      S.push_back(hsv[1]);
      V.push_back(hsv[2]);
    }

    uint8_t h = mcl::median(H);
    uint8_t s = mcl::median(S);
    uint8_t v = mcl::median(V);
    if (v > 15)  // if not black, set value artificially high to make easier labeling and visualization
      v = 255;   // this trick works for black, white and colors, but any gray will be labeled as white

    cv::Vec3b rgb = convertColor(cv::Vec3b(h, s, v), cv::COLOR_HSV2RGB);
    ROS_DEBUG("%d  %d  %d  ->  %d  %d  %d", h, s, v, rgb[0], rgb[1], rgb[2]);

    color.r = rgb[0] / 255.0f;
    color.g = rgb[1] / 255.0f;
    color.b = rgb[2] / 255.0f;
    color.a = alpha;

    return color;
  }

  /**
   * Using Lab we can use euclidean distance to basic colors:
   * http://www.pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/
   */
  std::string getColorLabel(const std_msgs::ColorRGBA& color)
  {
    static std::map<std::string, cv::Vec3f> BASIC_LAB_COLORS =
      { {"black",   convertColor(cv::Vec3f(0.0, 0.0, 0.0), cv::COLOR_RGB2Lab)},
        {"white",   convertColor(cv::Vec3f(1.0, 1.0, 1.0), cv::COLOR_RGB2Lab)},
        {"red",     convertColor(cv::Vec3f(1.0, 0.0, 0.0), cv::COLOR_RGB2Lab)},
        {"lime",    convertColor(cv::Vec3f(0.0, 1.0, 0.0), cv::COLOR_RGB2Lab)},
        {"blue",    convertColor(cv::Vec3f(0.0, 0.0, 1.0), cv::COLOR_RGB2Lab)},
        {"yellow",  convertColor(cv::Vec3f(1.0, 1.0, 0.0), cv::COLOR_RGB2Lab)},
        {"cyan",    convertColor(cv::Vec3f(0.0, 1.0, 1.0), cv::COLOR_RGB2Lab)},
        {"magenta", convertColor(cv::Vec3f(1.0, 0.0, 1.0), cv::COLOR_RGB2Lab)} };
//        {"silver",  convertColor(cv::Vec3f(192,192,192), cv::COLOR_RGB2Lab)},
//        {"gray",    convertColor(cv::Vec3f(128,128,128), cv::COLOR_RGB2Lab)},
//        {"maroon",  convertColor(cv::Vec3f(128,  0,  0), cv::COLOR_RGB2Lab)},
//        {"olive",   convertColor(cv::Vec3f(128,128,  0), cv::COLOR_RGB2Lab)},
//        {"green",   convertColor(cv::Vec3f(  0,128,  0), cv::COLOR_RGB2Lab)},
//        {"purple",  convertColor(cv::Vec3f(128,  0,128), cv::COLOR_RGB2Lab)},
//        {"teal",    convertColor(cv::Vec3f(  0,128,128), cv::COLOR_RGB2Lab)},
//        {"navy",    convertColor(cv::Vec3f(  0,  0,128), cv::COLOR_RGB2Lab)} };

    std::string label;
    double min_dist = std::numeric_limits<double>::infinity();
    double lum_diff = std::numeric_limits<double>::infinity();
    cv::Vec3f lab_color = convertColor(cv::Vec3f(color.r, color.g, color.b), cv::COLOR_RGB2Lab);
    for (const auto& basic_color: BASIC_LAB_COLORS)
    {
      double dist = mcl::distance2D(lab_color[1], lab_color[2], basic_color.second[1], basic_color.second[2]);
      ROS_DEBUG("%f  %f  %f   %f  %f  %f  ->  %f  %f  %f   %f   < %f ?",
                color.r*1.0, color.g*1.0, color.b*1.0, lab_color[0], lab_color[1], lab_color[2],
                basic_color.second[0], basic_color.second[1], basic_color.second[2], dist, min_dist);
      if (dist == min_dist &&
          basic_color.second[1] == 0.0 && basic_color.second[2] == 0.0 &&
          std::abs(basic_color.second[0] - lab_color[0]) < lum_diff)
      {
        // Same distance, ergo must be a gray scale color; use luminosity do decide the right label
        ROS_DEBUG("%s  -> %s   %f  ->  %f", label.c_str(), basic_color.first.c_str(),
                  lum_diff, std::abs(basic_color.second[0] - lab_color[0]));
        lum_diff = std::abs(basic_color.second[0] - lab_color[0]);
        label = basic_color.first;
      }
      else if (dist < min_dist)
      {
        min_dist = dist;
        lum_diff = std::abs(basic_color.second[0] - lab_color[0]);
        label = basic_color.first;
      }
    }

    return label;
  }

private:
  image_transport::ImageTransport image_trs_;
  image_transport::CameraSubscriber image_sub_;
  image_geometry::PinholeCameraModel cam_model_;
  sensor_msgs::Image rgb_image_;
  std::mutex image_mutex_;


  cv::Vec3b convertColor(const cv::Vec3b& color, int code)
  {
    cv::Mat A(1, 1, CV_8UC3, color), B;
    cv::cvtColor(A, B, code);
    return B.at<cv::Vec3b>(0, 0);
  }

  cv::Vec3f convertColor(const cv::Vec3f& color, int code)
  {
    cv::Mat A(1, 1, CV_32FC3, color), B;
    cv::cvtColor(A, B, code);
    return B.at<cv::Vec3f>(0, 0);
  }

  std_msgs::ColorRGBA getRandColor(float alpha = 1.0)
  {
    std_msgs::ColorRGBA color;
    color.r = float(rand())/RAND_MAX;
    color.g = float(rand())/RAND_MAX;
    color.b = float(rand())/RAND_MAX;
    color.a = alpha;
    return color;
  }
};

};  // namespace thorp_obj_rec
