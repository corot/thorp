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
  std::vector<cv::Vec3f> getColors(sensor_msgs::PointCloud2 pc_msg)
  {
    std::vector<cv::Vec3f> colors;

    cv::Mat image_bgr, image_hsv;
    cv_bridge::CvImagePtr input_bridge;
    try
    {
      image_mutex_.lock();
      input_bridge = cv_bridge::toCvCopy(rgb_image_, sensor_msgs::image_encodings::BGR8);
      image_mutex_.unlock();
      image_bgr = input_bridge->image;

      cv::cvtColor(image_bgr, image_hsv, cv::COLOR_BGR2HSV);

      cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
      cv::imshow( "Display window", image_hsv );                   // Show our image inside it.

      cv::waitKey(100);
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[object detection] Failed to convert RBG image");
      return colors;
    }

//    color.a = alpha;

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

//      image2.at<cv::Vec3b>(uv.y, uv.x) = rgb;
//      if (uv.y < 0)
//        ROS_ERROR("--------------------------------------------------");   no ocurre...


//      color.b += rgb[0] / 225.0f;
//      color.g += rgb[1] / 225.0f;
//      color.r += rgb[2] / 225.0f;

//      colors.emplace_back(rgb[0] / 255.0f, rgb[1] / 255.0f, rgb[2] / 255.0f);
    }
    for (auto color: color_map)
      colors.emplace_back(color.second[0] / 255.0f, color.second[1] / 255.0f, color.second[2] / 255.0f);
//    color.b /= float(pc_msg.width * pc_msg.height);
//    color.g /= float(pc_msg.width * pc_msg.height);
//    color.r /= float(pc_msg.width * pc_msg.height);


    sensor_msgs::PointCloud2Iterator<float> iter_x1(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y1(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z1(pc_msg, "z");

    cv::Mat image2 = image_bgr;
    image2 = cv::Vec3b(255, 255, 255);
    std_msgs::ColorRGBA kk = getAvgColor(colors);
    for (; iter_x1 != iter_x1.end(); ++iter_x1, ++iter_y1, ++iter_z1)
    {
      cv::Point3d pt_cv(*iter_x1, *iter_y1, *iter_z1);
      cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
      image2.at<cv::Vec3b>(uv.y, uv.x) = cv::Vec3b(kk.b*255, kk.g*255, kk.r*255);
    }
//    for (auto color: color_map)
//      image.at<cv::Vec3b>(color.first>>16, color.first&0x0000FFFF) = cv::Vec3b(color.second[0] / 255.0f, color.second[1] / 255.0f, color.second[2] / 255.0f);
//esto peta,,,,    veo 4 opciones
//1 . investigar xq salen colores raros
//2 . median  OJO  tengo q pasar a HLS
//3 . coger solo puntos centricos (no se como)
//4 . x cada punto, coger median del vecindario

    cv::imshow( "Display window 2", image2 );                   // Show our image inside it.

    cv::waitKey(4000);

    return colors;
  }

  std_msgs::ColorRGBA getAvgColor(const std::vector<cv::Vec3f>& color_points, float alpha = 1.0f)
  {
    std_msgs::ColorRGBA color;

    static int kk = 0;
      static ros::WallDuration t{};
      ros::WallTime t0 = ros::WallTime::now();

    int K = 3;
    cv::Mat labels, centers;

    double compactness =
        cv::kmeans(color_points, K, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 0.0001),
                   3, cv::KMEANS_PP_CENTERS, centers);
    ROS_ERROR("compactness    %f", compactness);
    std::vector<int> cluster_size(K);
    for (int point = 0; point < color_points.size(); point++)
    {
      cluster_size[labels.at<int>(point)] ++;
    }

    std::sort(cluster_size.begin(), cluster_size.end(), std::greater<int>());

    int selected_cluster = 0;
    for (int cluster = 0; cluster < K; cluster++)
    {
      float gray_level_0 = centers.at<cv::Vec3f>(0)[0] + centers.at<cv::Vec3f>(0)[1] + centers.at<cv::Vec3f>(0)[2];
      float gray_level_i = centers.at<cv::Vec3f>(cluster)[0] + centers.at<cv::Vec3f>(cluster)[1] + centers.at<cv::Vec3f>(cluster)[2];

      ROS_INFO("Cluster %d: %d points with centroid %f, %f, %f (gray %f)", cluster, cluster_size[cluster],
               centers.at<cv::Vec3f>(cluster)[0], centers.at<cv::Vec3f>(cluster)[1], centers.at<cv::Vec3f>(cluster)[2],
               gray_level_i);

//      if (cluster > 0 && (cluster_size[0] - cluster_size[cluster])/float(cluster_size[0]) < 0.25)
//      {
//        if (gray_level_i > gray_level_0)
//        {
//          ROS_INFO("Selected cluster -> %d", cluster);
//          selected_cluster = cluster;
//        }
//      }
    }

    color.b = centers.at<cv::Vec3f>(selected_cluster)[0];
    color.g = centers.at<cv::Vec3f>(selected_cluster)[1];
    color.r = centers.at<cv::Vec3f>(selected_cluster)[2];
    color.a = alpha;

    t += ros::WallTime::now() - t0;
    kk++;
    if (kk%100 == 0){
      ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>    %f", t.toSec());
      t = ros::WallDuration{};
    }

    return color;
  }

private:
  image_transport::ImageTransport image_trs_;
  image_transport::CameraSubscriber image_sub_;
  image_geometry::PinholeCameraModel cam_model_;
  sensor_msgs::Image rgb_image_;
  std::mutex image_mutex_;

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
