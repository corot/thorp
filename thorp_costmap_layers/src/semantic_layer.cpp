#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "thorp_costmap_layers/srv_interface.hpp"
#include "thorp_costmap_layers/visualization.hpp"

#include "thorp_costmap_layers/semantic_layer.hpp"

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace thorp::costmap_layers
{

SemanticLayer::SemanticLayer() : old_bounds_(), dsrv_()
{
}

SemanticLayer::~SemanticLayer()
{
  delete dsrv_;
}

void SemanticLayer::onInitialize()
{
  ros::NodeHandle pnh("~" + name_);

  fixed_frame_ = pnh.param<std::string>("fixed_frame", fixed_frame_);

  visualization_ = std::make_unique<Visualization>(pnh);

  // so our stored costmap equals the world one
  matchSize();

  interfaces_.insert(std::make_unique<ServiceInterface>(pnh, *tf_, layered_costmap_));

  dsrv_ = new dynamic_reconfigure::Server<thorp_costmap_layers::SemanticLayerConfig>(pnh);
  dsrv_->setCallback(boost::bind(&SemanticLayer::reconfigureCB, this, _1, _2));

  current_ = true;
}

void SemanticLayer::matchSize()
{
  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
            master->getOriginY());
  ROS_DEBUG("[semantic_layer] resized to %dux%du (%gx%gm)  resolution: %f  origin: %f, %f", master->getSizeInCellsX(),
            master->getSizeInCellsY(), master->getSizeInMetersX(), master->getSizeInMetersY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void SemanticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                 double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  BaseInterface::setRobotPose(robot_x, robot_y, robot_yaw);

  geometry_msgs::TransformStamped cm_ff_tf;
  try
  {
    // Transform added, updated and removed objects from fixed to costmap frame to update the bounds to include them
    cm_ff_tf = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), fixed_frame_, ros::Time(0), ros::Duration(1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("[semantic_layer]  %s. Skipping update cycle", ex.what());
    return;
  }

  std::lock_guard<std::mutex> guard(mutex_);

  double prev_min_x = *min_x, prev_min_y = *min_y, prev_max_x = *max_x, prev_max_y = *max_y;
  ROS_DEBUG("[semantic_layer] Bounds before update: x: %g, %g   y: %g, %g  (size: %d x %d)", *min_x, *max_x, *min_y,
            *max_y, getSizeInCellsX(), getSizeInCellsY());

  // if local, need to keep updating
  current_ = true;

  // process updated and removed objects for each interface
  size_t changes_count = 0;
  for (const auto& interface_ptr : interfaces_)
  {
    if (!interface_ptr->isEnabled())
      continue;

    changes_count += processUpdated(cm_ff_tf, interface_ptr->getUpdatedObjects(), min_x, min_y, max_x, max_y);
    interface_ptr->clearUpdatedIds();

    changes_count += processRemoved(cm_ff_tf, interface_ptr->getRemovedObjects(), min_x, min_y, max_x, max_y);
    interface_ptr->clearRemovedIds();
  }

  visualization_->showUpdatedBounds(prev_min_x, prev_min_y, prev_max_x, prev_max_y, *min_x, *min_y, *max_x, *max_y,
                                    layered_costmap_->getGlobalFrameID());
  ROS_DEBUG("[semantic_layer] Bounds after update:  x: %g, %g   y: %g, %g  (%lu objects changed)", *min_x, *max_x,
            *min_y, *max_y, changes_count);
}

void SemanticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  geometry_msgs::TransformStamped cm_ff_tf, ff_cm_tf;
  try
  {
    // We need costmap to fixed frames transform to query the hash maps (on fixed frame) and the inverse to transform
    // the objects from fixed to costmap frame for painting them (both frames are probably the same on global costmap)
    cm_ff_tf = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), fixed_frame_, ros::Time(0), ros::Duration(1));
    ff_cm_tf = tf_->lookupTransform(fixed_frame_, layered_costmap_->getGlobalFrameID(), ros::Time(0), ros::Duration(1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("[semantic_layer]  %s. Skipping update cycle", ex.what());
    return;
  }

  // Create a rectangle containing the updated area transformed to fixed frame
  double edge_left = layered_costmap_->getCostmap()->getOriginX() + min_i * resolution_;
  double edge_right = layered_costmap_->getCostmap()->getOriginX() + max_i * resolution_;
  double edge_bottom = layered_costmap_->getCostmap()->getOriginY() + min_j * resolution_;
  double edge_top = layered_costmap_->getCostmap()->getOriginY() + max_j * resolution_;
  std::vector<Point2d> corners{ Point2d(edge_left, edge_bottom), Point2d(edge_left, edge_top),
                                Point2d(edge_right, edge_bottom), Point2d(edge_right, edge_top) };

  std::vector<geometry_msgs::Point> corners_transformed(4);
  for (size_t i = 0; i < corners.size(); i++)
  {
    tf2::doTransform(corners[i].toPointMsg(), corners_transformed[i], ff_cm_tf);
  }
  Point2d lower_left, upper_right;
  std::sort(corners_transformed.begin(), corners_transformed.end(),
            [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return (p1.x < p2.x); });
  lower_left.x = corners_transformed.front().x;
  upper_right.x = corners_transformed.back().x;
  std::sort(corners_transformed.begin(), corners_transformed.end(),
            [](const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) { return (p1.y < p2.y); });
  lower_left.y = corners_transformed.front().y;
  upper_right.y = corners_transformed.back().y;

  std::lock_guard<std::mutex> guard(mutex_);

  // Get all objects within the updated area and mark their boundaries with their associated cost, sorted by precedence
  // compose a polygon with the object's contour, enforcing map bounds but not the updated bounds to avoid creating fake
  // obstacles at the borders of the updated area; order by precedence
  // process updated and removed objects for each interface
  std::vector<Object> objects;
  for (const auto& interface_ptr : interfaces_)
  {
    if (!interface_ptr->isEnabled())
      continue;

    std::vector<Object> interface_objects = interface_ptr->getObjectsInRegion(lower_left, upper_right);
    objects.insert(objects.end(), interface_objects.begin(), interface_objects.end());
  }
  std::sort(objects.begin(), objects.end(),
            [](const Object& a, const Object& b) -> bool { return a.precedence < b.precedence; });

  for (const auto& object : objects)
  {
    std::vector<costmap_2d::MapLocation> points_in_map;
    std::vector<costmap_2d::MapLocation> polygon_cells;
    PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);

    costmap_2d::MapLocation p1_map, p2_map;
    geometry_msgs::Point p1_clip, p2_clip, p1_transformed, p2_transformed;
    for (auto p1 = object.contour_points.begin(), p2 = std::next(object.contour_points.begin());
         p1 != object.contour_points.end(); ++p1, ++p2)
    {
      if (p2 == object.contour_points.end())
        p2 = object.contour_points.begin();

      tf2::doTransform(p1->toPointMsg(), p1_transformed, cm_ff_tf);
      tf2::doTransform(p2->toPointMsg(), p2_transformed, cm_ff_tf);

      // Clip segments by the updated area bounds. Segments entirely out are skip,
      // what can create holes on filled objects if we add a smaller one within!
      // Note that at this point both edges and object contours are on fixed frame
      if (!clipSegment(edge_left, edge_right, edge_bottom, edge_top, p1_transformed.x, p1_transformed.y,
                       p2_transformed.x, p2_transformed.y, p1_clip.x, p1_clip.y, p2_clip.x, p2_clip.y))
        continue;

      master_grid.worldToMapEnforceBounds(p1_clip.x, p1_clip.y, (int&)p1_map.x, (int&)p1_map.y);
      master_grid.worldToMapEnforceBounds(p2_clip.x, p2_clip.y, (int&)p2_map.x, (int&)p2_map.y);

      if (object.fill)
      {
        // For filled objects, we need the polygon to obtain the filling cells
        points_in_map.push_back(p1_map);
        points_in_map.push_back(p2_map);
      }
      else
      {
        // For hollow ones, we get the cells for each non fully clipped segment
        raytraceLine(cell_gatherer, p1_map.x, p1_map.y, p2_map.x, p2_map.y);
      }
    }

    if (object.fill)
      convexFillCells(points_in_map, polygon_cells);

    auto new_cost = static_cast<unsigned char>(std::round(object.cost * LETHAL_OBSTACLE));
    for (const auto& cell : polygon_cells)
    {
      // enforce updated area bounds on cell coordinates because our spacial clipping can
      // let in invalid cells (on max_i and / or max_j) due to rounding (causing SW-12194)
      int x = cell.x;
      int y = cell.y;
      if ((x >= min_i && x < max_i && y >= min_j && y < max_j) &&
          (!object.use_maximum || master_grid.getCost(x, y) < new_cost))
        master_grid.setCost(x, y, new_cost);
    }
  }
}

size_t SemanticLayer::processUpdated(const geometry_msgs::TransformStamped& tf,
                                     const std::set<std::shared_ptr<Object>>& updated_objs,
                                     double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (const auto& updated_obj : updated_objs)
  {
    // set bounds to include updated objects
    transformAndTouch(tf, updated_obj->bounding_box, min_x, min_y, max_x, max_y);

    // if the obstacle has been updated, we also need to include previous location into the bounds to clear it
    if (old_bounds_.find(updated_obj->id) != old_bounds_.end())
    {
      transformAndTouch(tf, old_bounds_[updated_obj->id], min_x, min_y, max_x, max_y);
    }
    old_bounds_[updated_obj->id] = updated_obj->bounding_box;

    visualization_->showObjectContour(*updated_obj, fixed_frame_);
  }
  return updated_objs.size();
}

size_t SemanticLayer::processRemoved(const geometry_msgs::TransformStamped& tf,
                                     const std::set<std::shared_ptr<Object>>& removed_objs,
                                     double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (const auto& removed_obj : removed_objs)
  {
    // set bounds to include removed objects
    transformAndTouch(tf, removed_obj->bounding_box, min_x, min_y, max_x, max_y);

    // previous location not needed anymore
    old_bounds_.erase(removed_obj->id);

    visualization_->hideObjectContour(*removed_obj);
  }
  return removed_objs.size();
}

void SemanticLayer::transformAndTouch(const geometry_msgs::TransformStamped& tf, const Rectangle& rect,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
{
  geometry_msgs::Point corner_tf;
  for (const auto& corner : rect.corners())
  {
    tf2::doTransform(corner.toPointMsg(), corner_tf, tf);
    touch(corner_tf.x, corner_tf.y, min_x, min_y, max_x, max_y);
  }
}

void SemanticLayer::reconfigureCB(const thorp_costmap_layers::SemanticLayerConfig& config,
                                  [[maybe_unused]] uint32_t level)
{
  ROS_INFO("[semantic_layer] Received reconfigure callback");
  enabled_ = config.enabled;

  for (const auto& interface_ptr : interfaces_)
    interface_ptr->reconfigure(config);
}

bool SemanticLayer::clipSegment(double edge_left, double edge_right, double edge_bottom, double edge_top,
                                double x0src, double y0src, double x1src, double y1src,
                                double& x0clip, double& y0clip, double& x1clip, double& y1clip)
{
  double t0 = 0.0;
  double t1 = 1.0;
  double xdelta = x1src - x0src;
  double ydelta = y1src - y0src;
  double p, q, r;

  for (int edge = 0; edge < 4; edge++)
  {  // Traverse through left, right, bottom, top edges.
    if (edge == 0)
    {
      p = -xdelta;
      q = -(edge_left - x0src);
    }
    if (edge == 1)
    {
      p = xdelta;
      q = (edge_right - x0src);
    }
    if (edge == 2)
    {
      p = -ydelta;
      q = -(edge_bottom - y0src);
    }
    if (edge == 3)
    {
      p = ydelta;
      q = (edge_top - y0src);
    }
    r = q / p;
    if (p == 0 && q < 0)
      return false;  // Don't draw line at all. (parallel line outside)

    if (p < 0)
    {
      if (r > t1)
        return false;  // Don't draw line at all.
      else if (r > t0)
        t0 = r;  // Line is clipped!
    }
    else if (p > 0)
    {
      if (r < t0)
        return false;  // Don't draw line at all.
      else if (r < t1)
        t1 = r;  // Line is clipped!
    }
  }

  x0clip = x0src + t0 * xdelta;
  y0clip = y0src + t0 * ydelta;
  x1clip = x0src + t1 * xdelta;
  y1clip = y0src + t1 * ydelta;

  return true;  // (clipped) line is drawn
}

}  // namespace thorp::costmap_layers

PLUGINLIB_EXPORT_CLASS(thorp::costmap_layers::SemanticLayer, costmap_2d::Layer)
