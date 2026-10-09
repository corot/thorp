/*
 * Author: Jorge Santos
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <thorp_msgs/action/detect_objects.hpp>
#include <thorp_msgs/action/detect_tables.hpp>
#include <thorp_toolkit/common.hpp>
#include <thorp_toolkit/tray.hpp>

#include "thorp_perception/segmentation.hpp"
#include "thorp_perception/template_matcher.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::msg::ObjectColor;

namespace thorp::perception
{
namespace
{
// Objects dimensions we accept, and how much they can float over or sink into the table
constexpr double MIN_OBJECT_SIZE = 0.005;
constexpr double MAX_OBJECT_SIZE = 0.25;
constexpr double OBJECT_ON_TABLE_TOLERANCE = 0.01;
// Tables are boxes this thick, so they don't subsume low objects
constexpr double TABLE_THICKNESS = 0.001;

template <typename T>
T param(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  return node->declare_parameter(name, default_value);
}

/// Convert sRGB, in [0, 1], into CIELAB; adapted from pcl/registration/gicp6d
Eigen::Vector3f rgbToLab(const Eigen::Vector3f& rgb)
{
  auto linearize = [](double c) { return c > 0.04045 ? std::pow((c + 0.055) / 1.055, 2.4) : c / 12.92; };
  double r = linearize(rgb[0]), g = linearize(rgb[1]), b = linearize(rgb[2]);
  double x = (r * 0.4124 + g * 0.3576 + b * 0.1805) / 0.95047;
  double y = (r * 0.2126 + g * 0.7152 + b * 0.0722);
  double z = (r * 0.0193 + g * 0.1192 + b * 0.9505) / 1.08883;
  auto f = [](double t) { return t > 0.008856 ? std::cbrt(t) : 7.787 * t + 16.0 / 116.0; };
  return { static_cast<float>(116.0 * f(y) - 16.0), static_cast<float>(500.0 * (f(x) - f(y))),
           static_cast<float>(200.0 * (f(y) - f(z))) };
}

/// Mean color of a cloud, in [0, 1]
Eigen::Vector3f meanColor(const PointCloud& cloud)
{
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const auto& point : cloud.points)
    sum += Eigen::Vector3f(point.r, point.g, point.b);
  return sum / (255.0f * static_cast<float>(std::max<size_t>(cloud.size(), 1)));
}

/// What a collision object carries beyond its geometry: its size, x being the longest side, and color name
std::string metadata(double length, double width, double height, const Eigen::Vector3f& rgb)
{
  Eigen::Vector3f lab = rgbToLab(rgb);
  json data = { { "size", { length, width, height } }, { "color", toolkit::colorName(lab[0], lab[1], lab[2]) } };
  return data.dump();
}

geometry_msgs::msg::Pose makePose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

double yaw(const Eigen::Isometry3f& pose)
{
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

sensor_msgs::msg::PointCloud2 toMsg(const PointCloud& cloud, const std_msgs::msg::Header& header)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  return msg;
}

/// All the clusters in one cloud, with each point labeled with its cluster's index
sensor_msgs::msg::PointCloud2 toMsg(const std::vector<PointCloud::Ptr>& clusters, const std_msgs::msg::Header& header)
{
  pcl::PointCloud<pcl::PointXYZRGBL> labeled;
  for (size_t i = 0; i < clusters.size(); ++i)
    for (const auto& point : clusters[i]->points)
      labeled.emplace_back(point.x, point.y, point.z, point.r, point.g, point.b, static_cast<std::uint32_t>(i));
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(labeled, msg);
  msg.header = header;
  return msg;
}

visualization_msgs::msg::Marker makeLabel(const std::string& ns, int id, const std::string& text,
                                          const std_msgs::msg::ColorRGBA& color, const std_msgs::msg::Header& header,
                                          double x, double y, double z)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.text = text;
  marker.scale.x = marker.scale.z = 0.035;  // x is RViz's space width; left at zero, a space is about 1 m wide
  marker.color = color;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;
  return marker;
}

/// The surface's quadrilateral, as a closed line
visualization_msgs::msg::Marker makeOutline(const Surface& surface, const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "table";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.02;
  marker.color = toolkit::makeColor(1.0f, 1.0f, 0.0f);
  marker.pose.orientation.w = 1.0;
  for (size_t i = 0; i <= surface.corners.size(); ++i)
  {
    geometry_msgs::msg::Point point;
    point.x = surface.corners[i % surface.corners.size()].x();
    point.y = surface.corners[i % surface.corners.size()].y();
    point.z = surface.height(surface.corners[i % surface.corners.size()]);
    marker.points.push_back(point);
  }
  return marker;
}

}  // namespace

/**
 * Detect tables and tabletop objects on the Xtion point cloud, adding them to the planning scene as collision objects.
 */
class ObjectDetectionServer
{
public:
  using DetectObjects = thorp_msgs::action::DetectObjects;
  using DetectTables = thorp_msgs::action::DetectTables;

  explicit ObjectDetectionServer(const rclcpp::Node::SharedPtr& node)
    : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    output_frame_ = param<std::string>(node_, "output_frame", "base_footprint");
    auto crop_min = param<std::vector<double>>(node_, "crop.min", { 0.15, -1.0, 0.3 });
    auto crop_max = param<std::vector<double>>(node_, "crop.max", { 2.5, 1.0, 0.6 });
    segmentation_.crop_min = Eigen::Vector3d(crop_min[0], crop_min[1], crop_min[2]).cast<float>();
    segmentation_.crop_max = Eigen::Vector3d(crop_max[0], crop_max[1], crop_max[2]).cast<float>();
    segmentation_.min_surface_size = param(node_, "min_surface_size", segmentation_.min_surface_size);
    segmentation_.min_cluster_size = param(node_, "min_cluster_size", segmentation_.min_cluster_size);
    segmentation_.cluster_tolerance = param(node_, "cluster_tolerance", segmentation_.cluster_tolerance);
    IcpParams icp;
    icp.iterations = param(node_, "icp.iterations", icp.iterations);
    icp.max_distance = param(node_, "icp.max_distance", icp.max_distance);
    icp.transformation_epsilon = param(node_, "icp.transformation_epsilon", icp.transformation_epsilon);
    icp.fitness_epsilon = param(node_, "icp.fitness_epsilon", icp.fitness_epsilon);
    max_cluster_size_ = param(node_, "max_cluster_size", 10000);
    max_matching_error_ = param(node_, "max_matching_error", 4e-6);
    redetect_tolerance_ = param(node_, "redetect_tolerance", 0.01);
    tabletop_volume_height_ = param(node_, "tabletop_volume_height", 0.2);
    meshes_path_ = ament_index_cpp::get_package_share_directory("thorp_perception") + "/meshes";

    matcher_ = std::make_unique<TemplateMatcher>(param(node_, "templates_path", meshes_path_), icp);
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu object templates", matcher_->size());

    markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", toolkit::LATCHED);
    table_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/table", toolkit::LATCHED);
    clusters_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/clusters", toolkit::LATCHED);
    // Reliable, as the cloud publisher; with Cyclone DDS, a best effort subscription gets none of these ~10 MB clouds
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::QoS(1), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          cloud_ = msg;
          cloud_receipt_ = std::chrono::steady_clock::now();
          cloud_received_.notify_all();
        });
    // Named as on Noetic, where the executive found them
    objects_server_ = createServer<DetectObjects>("object_detection", &ObjectDetectionServer::detectObjects);
    tables_server_ = createServer<DetectTables>("object_detection/detect_tables", &ObjectDetectionServer::detectTables);
    RCLCPP_INFO(node_->get_logger(), "Object and table detection action servers ready");
  }

private:
  template <typename Action>
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  template <typename Action>
  typename rclcpp_action::Server<Action>::SharedPtr
  createServer(const std::string& name, void (ObjectDetectionServer::*execute)(std::shared_ptr<GoalHandle<Action>>))
  {
    return rclcpp_action::create_server<Action>(
        node_, name,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>) {
          if (busy_.exchange(true))
          {
            RCLCPP_ERROR(node_->get_logger(), "Rejecting goal: already detecting");
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<GoalHandle<Action>>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this, execute](std::shared_ptr<GoalHandle<Action>> goal_handle) {
          std::thread([this, execute, goal_handle]() {
            (this->*execute)(goal_handle);
            busy_ = false;
          }).detach();
        });
  }

  /**
   * Wait for a cloud captured after now, cropped to the segmentation region on the output frame.
   */
  PointCloud::Ptr getCloud(std_msgs::msg::Header& header, std::string& error)
  {
    const rclcpp::Time request_time = node_->now();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    {
      std::unique_lock<std::mutex> lock(cloud_mutex_);
      if (!cloud_received_.wait_for(lock, 5s, [&] { return cloud_ && rclcpp::Time(cloud_->header.stamp) >= request_time; }))
      {
        if (!cloud_)
        {
          error = "no point cloud received";
        }
        else
        {
          std::ostringstream text;
          text << std::fixed << std::setprecision(2) << "no point cloud captured after the request (at "
               << request_time.seconds() << " s) received in 5 s; the last one was captured at "
               << rclcpp::Time(cloud_->header.stamp).seconds() << " s and received "
               << std::chrono::duration<double>(std::chrono::steady_clock::now() - cloud_receipt_).count()
               << " s ago";
          error = text.str();
        }
        return nullptr;
      }
      msg = cloud_;
    }
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(output_frame_, msg->header.frame_id, msg->header.stamp, 1s);
    }
    catch (const tf2::TransformException& e)
    {
      error = e.what();
      return nullptr;
    }
    PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::transformPointCloud(cloud, cloud, tf2::transformToEigen(transform).cast<float>().matrix());
    header = msg->header;
    header.frame_id = output_frame_;
    return crop(cloud, segmentation_);
  }

  CollisionObject makeTable(const Surface& surface, const std_msgs::msg::Header& header) const
  {
    CollisionObject table;
    table.id = "table";
    table.header = header;
    table.operation = CollisionObject::ADD;
    table.pose = makePose(surface.center.x(), surface.center.y(), surface.height(surface.center), surface.yaw);
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = { surface.length, surface.width, TABLE_THICKNESS };
    // The box lies on the surface plane, while the table pose stays level for those navigating around it
    Eigen::Quaterniond level(Eigen::AngleAxisd(surface.yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond tilt =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), surface.plane.head<3>().cast<double>());
    table.primitive_poses.resize(1);
    table.primitive_poses[0].orientation = tf2::toMsg(level.inverse() * tilt * level);
    table.type.db = metadata(surface.length, surface.width, TABLE_THICKNESS, meanColor(*surface.cloud));
    return table;
  }

  void detectTables(std::shared_ptr<GoalHandle<DetectTables>> goal_handle)
  {
    auto result = std::make_shared<DetectTables::Result>();
    std::string error;
    std_msgs::msg::Header header;
    PointCloud::Ptr cloud = getCloud(header, error);
    if (!cloud)
    {
      RCLCPP_ERROR(node_->get_logger(), "Table detection failed: %s", error.c_str());
      goal_handle->abort(result);
      return;
    }
    std::optional<Surface> surface = findSurface(cloud, segmentation_, goal_handle->get_goal()->min_side, error);
    if (surface)
    {
      result->tables.push_back(makeTable(*surface, header));
      table_pub_->publish(toMsg(*surface->cloud, header));
      visualization_msgs::msg::MarkerArray markers;
      markers.markers.push_back(makeOutline(*surface, header));
      markers_pub_->publish(markers);
      RCLCPP_INFO(node_->get_logger(), "Table of %.2f x %.2f m detected at [%.2f, %.2f, %.2f]", surface->length,
                  surface->width, surface->center.x(), surface->center.y(), surface->height(surface->center));
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "No table detected: %s", error.c_str());
    }
    goal_handle->succeed(result);
  }

  void detectObjects(std::shared_ptr<GoalHandle<DetectObjects>> goal_handle)
  {
    const rclcpp::Time start = node_->now();
    auto result = std::make_shared<DetectObjects::Result>();
    if (goal_handle->get_goal()->clear_scene)
      planning_scene_.removeCollisionObjects(planning_scene_.getKnownObjectNames());

    std::string error;
    std_msgs::msg::Header header;
    PointCloud::Ptr cloud = getCloud(header, error);
    if (!cloud)
    {
      RCLCPP_ERROR(node_->get_logger(), "Object detection failed: %s", error.c_str());
      goal_handle->abort(result);
      return;
    }
    std::optional<Surface> surface = findSurface(cloud, segmentation_, 0.0, error);
    if (!surface)
    {
      RCLCPP_WARN(node_->get_logger(), "No surface found: %s", error.c_str());
      goal_handle->succeed(result);
      return;
    }
    result->surface = makeTable(*surface, header);
    table_pub_->publish(toMsg(*surface->cloud, header));
    std::vector<CollisionObject> scene_objects{ result->surface };
    std::vector<ObjectColor> colors{ objectColor(result->surface.id, meanColor(*surface->cloud)) };

    // Existing names, so new objects don't take them
    std::set<std::string> taken_names;
    for (const auto& name : planning_scene_.getKnownObjectNames())
      taken_names.insert(name);
    for (const auto& [name, attached] : planning_scene_.getAttachedObjects())
      taken_names.insert(name);

    std::vector<PointCloud::Ptr> clusters = extractClusters(cloud, *surface, segmentation_);
    clusters_pub_->publish(toMsg(clusters, header));
    RCLCPP_INFO(node_->get_logger(), "%zu objects plus table segmented in %.2f seconds", clusters.size(),
                (node_->now() - start).seconds());
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.emplace_back().action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(makeOutline(*surface, header));
    for (const auto& cluster : clusters)
    {
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
        return;
      }
      const int marker_id = static_cast<int>(markers.markers.size());
      std::string rejection;
      std::optional<CollisionObject> object = identifyObject(*cluster, *surface, header, taken_names, rejection);
      if (!object)
      {
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        markers.markers.push_back(makeLabel("rejected", marker_id, rejection, toolkit::makeColor(1.0f, 0.2f, 0.2f),
                                            header, (min_pt.x + max_pt.x) / 2.0, (min_pt.y + max_pt.y) / 2.0,
                                            (min_pt.z + max_pt.z) / 2.0 + 0.05));
        continue;
      }
      taken_names.insert(object->id);
      result->objects.push_back(*object);
      scene_objects.push_back(*object);
      colors.push_back(objectColor(object->id, meanColor(*cluster)));
      // The planning scene doesn't show object names, so we label them
      markers.markers.push_back(makeLabel("labels", marker_id, object->id, colors.back().color, header,
                                          object->pose.position.x, object->pose.position.y,
                                          object->pose.position.z + 0.05));
    }

    // Remove the objects previously detected on the table but not now; the redetected ones get replaced
    std::set<std::string> to_remove = objectsOnTable(*surface);
    to_remove.erase(result->surface.id);
    for (const auto& object : result->objects)
      to_remove.erase(object.id);
    for (const auto& name : to_remove)
    {
      CollisionObject remove;
      remove.id = name;
      remove.operation = CollisionObject::REMOVE;
      scene_objects.push_back(remove);
    }
    planning_scene_.applyCollisionObjects(scene_objects, colors);
    markers_pub_->publish(markers);
    RCLCPP_INFO(node_->get_logger(), "%zu objects detected in %.2f seconds", result->objects.size(),
                (node_->now() - start).seconds());
    goal_handle->succeed(result);
  }

  /**
   * Identify a cluster's type by template matching, and make a collision object with its template's mesh.
   * @param rejection Why the cluster isn't an object, in a few words
   * @return The object, or nothing if it has an impossible size or pose, or doesn't match any template
   */
  std::optional<CollisionObject> identifyObject(const PointCloud& cluster, const Surface& surface,
                                                const std_msgs::msg::Header& header,
                                                const std::set<std::string>& taken_names, std::string& rejection)
  {
    PointT min_pt, max_pt;
    pcl::getMinMax3D(cluster, min_pt, max_pt);
    Eigen::Vector3f size = max_pt.getVector3fMap() - min_pt.getVector3fMap();
    Eigen::Vector3f center = (max_pt.getVector3fMap() + min_pt.getVector3fMap()) / 2.0f;
    if (cluster.size() > static_cast<size_t>(max_cluster_size_))
    {
      RCLCPP_WARN(node_->get_logger(), "Object at [%.2f, %.2f, %.2f] discarded: %zu points, over %d", center.x(),
                  center.y(), center.z(), cluster.size(), max_cluster_size_);
      rejection = "size";
      return std::nullopt;
    }
    if (size.minCoeff() < MIN_OBJECT_SIZE || size.maxCoeff() > MAX_OBJECT_SIZE)
    {
      RCLCPP_WARN(node_->get_logger(), "Object at [%.2f, %.2f, %.2f] discarded: size %.3f x %.3f x %.3f", center.x(),
                  center.y(), center.z(), size.x(), size.y(), size.z());
      rejection = "size";
      return std::nullopt;
    }
    float base = std::numeric_limits<float>::max();
    float top = std::numeric_limits<float>::lowest();
    for (const auto& point : cluster.points)
    {
      base = std::min(base, surface.distance(point.getVector3fMap()));
      top = std::max(top, surface.distance(point.getVector3fMap()));
    }
    // Objects must rest on the table; others are probably table parts or the gripper after picking or placing
    if (base > OBJECT_ON_TABLE_TOLERANCE)
    {
      RCLCPP_WARN(node_->get_logger(), "Object at [%.2f, %.2f, %.2f] discarded: base %.3f m over the table",
                  center.x(), center.y(), center.z(), base);
      std::ostringstream text;
      text << std::fixed << std::setprecision(3) << "off the table, +" << base;
      rejection = text.str();
      return std::nullopt;
    }

    // Templates' origin is at their base center, which we put on the table; we give no initial orientation, as the
    // cluster's is only meaningful for elongated objects
    auto cluster_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(cluster, *cluster_xyz);
    Eigen::Isometry3f initial_estimate = Eigen::Isometry3f::Identity();
    initial_estimate.translation() = Eigen::Vector3f(center.x(), center.y(), surface.height(center.head<2>()));
    TemplateMatch match = matcher_->match(cluster_xyz, initial_estimate);
    if (match.error > max_matching_error_)
    {
      RCLCPP_INFO(node_->get_logger(), "Object at [%.2f, %.2f, %.2f] discarded: best match, %s, error %.2f > %.2f",
                  center.x(), center.y(), center.z(), match.name.c_str(), match.error * 1e6,
                  max_matching_error_ * 1e6);
      std::ostringstream text;
      text << std::fixed << std::setprecision(2) << match.name << "? " << match.error * 1e6;
      rejection = text.str();
      return std::nullopt;
    }

    // Dimensions along the matched template axes, with x the longest
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pcl::transformPointCloud(*cluster_xyz, aligned, match.pose.inverse().matrix());
    pcl::PointXYZ aligned_min, aligned_max;
    pcl::getMinMax3D(aligned, aligned_min, aligned_max);
    double length = std::max(aligned_max.x - aligned_min.x, aligned_max.y - aligned_min.y);
    double width = std::min(aligned_max.x - aligned_min.x, aligned_max.y - aligned_min.y);
    double height = top;

    CollisionObject object;
    object.header = header;
    object.operation = CollisionObject::ADD;
    object.pose = makePose(match.pose.translation().x(), match.pose.translation().y(),
                           match.pose.translation().z() + height / 2.0, yaw(match.pose));
    const shape_msgs::msg::Mesh* mesh = templateMesh(match.name);
    if (!mesh)
    {
      rejection = "no " + match.name + " mesh";
      return std::nullopt;
    }
    object.meshes.push_back(*mesh);
    object.mesh_poses.resize(1);
    object.mesh_poses[0].position.z = -height / 2.0;  // meshes' origin is at their base
    object.mesh_poses[0].orientation.w = 1.0;
    object.type.db = metadata(length, width, height, meanColor(cluster));
    object.id = objectName(match.name, object.pose, length, height, taken_names);
    RCLCPP_INFO(node_->get_logger(), "Object at [%.2f, %.2f, %.2f] classified as %s (error %.2f)",
                object.pose.position.x, object.pose.position.y, object.pose.position.z, object.id.c_str(),
                match.error * 1e6);
    return object;
  }

  /**
   * The name of an object of the same type already on the planning scene at about the same place, as that's
   * probably the same object; otherwise, the type plus the first free index.
   */
  std::string objectName(const std::string& type, const geometry_msgs::msg::Pose& pose, double length, double height,
                         const std::set<std::string>& taken_names)
  {
    double half_side = length / 2.0 + redetect_tolerance_;
    double half_height = height / 2.0 + redetect_tolerance_;
    for (const auto& name : objectsInVolume(
             Eigen::Vector3d(pose.position.x - half_side, pose.position.y - half_side, pose.position.z - half_height),
             Eigen::Vector3d(pose.position.x + half_side, pose.position.y + half_side, pose.position.z + half_height)))
    {
      if (name.rfind(type + " ", 0) == 0)
        return name;
    }
    for (int index = 1;; ++index)
    {
      std::string name = type + " " + std::to_string(index);
      if (taken_names.count(name) == 0)
        return name;
    }
  }

  /**
   * Objects on the planning scene within the volume over the table where we detect objects, but those on the tray:
   * docked at the table, under its eaves, the tray can be within that volume.
   */
  std::set<std::string> objectsOnTable(const Surface& surface)
  {
    Eigen::Vector2f min = surface.corners[0], max = surface.corners[0];
    for (const auto& corner : surface.corners)
    {
      min = min.cwiseMin(corner);
      max = max.cwiseMax(corner);
    }
    std::set<std::string> names =
        objectsInVolume(Eigen::Vector3d(min.x() - redetect_tolerance_, min.y() - redetect_tolerance_,
                                        surface.height(surface.center) - redetect_tolerance_),
                        Eigen::Vector3d(max.x() + redetect_tolerance_, max.y() + redetect_tolerance_,
                                        surface.height(surface.center) + tabletop_volume_height_));
    if (names.empty())
      return names;
    for (const auto& [name, object] : planning_scene_.getObjects({ names.begin(), names.end() }))
    {
      if (tray_.onTray(object))
        names.erase(name);
    }
    return names;
  }

  /**
   * Objects on the planning scene with their origin within an axis-aligned volume on the output frame.
   * PlanningSceneInterface::getKnownObjectNamesInROI takes the shapes' poses, relative to the object's pose, as
   * absolute; https://github.com/moveit/moveit2/pull/3884 fixes it.
   */
  std::set<std::string> objectsInVolume(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
  {
    std::set<std::string> names;
    for (const auto& [name, object] : planning_scene_.getObjects())
    {
      if (object.header.frame_id != output_frame_)
        continue;
      Eigen::Vector3d position(object.pose.position.x, object.pose.position.y, object.pose.position.z);
      if ((position.array() >= min.array()).all() && (position.array() <= max.array()).all())
        names.insert(name);
    }
    return names;
  }

  const shape_msgs::msg::Mesh* templateMesh(const std::string& type)
  {
    auto it = meshes_.find(type);
    if (it == meshes_.end())
    {
      std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource("file://" + meshes_path_ + "/" + type + ".stl"));
      if (!mesh)
      {
        RCLCPP_ERROR(node_->get_logger(), "Load mesh for %s failed", type.c_str());
        return nullptr;
      }
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(mesh.get(), shape_msg);
      it = meshes_.emplace(type, boost::get<shape_msgs::msg::Mesh>(shape_msg)).first;
    }
    return &it->second;
  }

  static ObjectColor objectColor(const std::string& id, const Eigen::Vector3f& rgb)
  {
    ObjectColor color;
    color.id = id;
    color.color = toolkit::makeColor(rgb[0], rgb[1], rgb[2]);
    return color;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  thorp::toolkit::Tray tray_;
  std::unique_ptr<TemplateMatcher> matcher_;
  std::map<std::string, shape_msgs::msg::Mesh> meshes_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr table_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
  rclcpp_action::Server<DetectObjects>::SharedPtr objects_server_;
  rclcpp_action::Server<DetectTables>::SharedPtr tables_server_;
  std::atomic<bool> busy_{ false };

  std::mutex cloud_mutex_;
  std::condition_variable cloud_received_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  std::chrono::steady_clock::time_point cloud_receipt_;

  std::string output_frame_;
  std::string meshes_path_;
  SegmentationParams segmentation_;
  int max_cluster_size_;
  double max_matching_error_;
  double redetect_tolerance_;
  double tabletop_volume_height_;
};

}  // namespace thorp::perception

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("object_detection");
  thorp::toolkit::init(node);
  thorp::perception::ObjectDetectionServer server(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
