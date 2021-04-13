#include <string>
#include <limits>

#include <ros/ros.h>
#include <json/json.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <shape_msgs/SolidPrimitive.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp> // OpenCV Rect and Point2f structs

#include <thorp_toolkit/geometry.hpp>
namespace ttk = thorp_toolkit;

#include "thorp_costmap_layers/semantic_layer.h"
#include "thorp_costmap_layers/visualization.h"

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace thorp_costmap_layers
{

SemanticLayer::SemanticLayer() :
    dsrv_(),
    old_bounds_(),
    scene_interface_()
{
}

SemanticLayer::~SemanticLayer()
{
  delete dsrv_;
}

void SemanticLayer::onInitialize()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~" + name_);

  visualization_ = std::make_unique<Visualization>(pnh);

  // so our stored costmap equals the world one
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<SemanticLayerConfig>(pnh);
  dsrv_->setCallback(boost::bind(&SemanticLayer::reconfigureCB, this, _1, _2));

  // Initialize scene obstacle topic interface to include scene information on the costmap; the callback allows
  // the interface to force updating the costmap without waiting for the update thread, when speed is critical
  namespace ph = std::placeholders;
  auto update_map_cb = std::bind(&costmap_2d::LayeredCostmap::updateMap, layered_costmap_, ph::_1, ph::_2, ph::_3);
  scene_interface_ = std::make_unique<SceneInterface>(pnh, *tf_, layered_costmap_->getGlobalFrameID(), update_map_cb);

  current_ = true;
}

void SemanticLayer::matchSize()
{
  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
  ROS_DEBUG("[semantic_layer] resized to %dux%du (%gx%gm)  resolution: %f  origin: %f, %f",
            master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getSizeInMetersX(), master->getSizeInMetersY(),
            master->getResolution(), master->getOriginX(), master->getOriginY());
}

void SemanticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                 double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  std::lock_guard<std::mutex> guard(mutex_);

  double prev_min_x = *min_x, prev_min_y = *min_y, prev_max_x = *max_x, prev_max_y = *max_y;
  ROS_DEBUG("[semantic_layer] Bounds before update:  x: %g, %g   y: %g, %g   (size: %d x %d)",
            *min_x, *max_x, *min_y, *max_y, getSizeInCellsX(), getSizeInCellsY());

  // if local, need to keep updating
  current_ = true;

  // process updated topic obstacles
  processUpdated(scene_interface_->getUpdatedObjects(), min_x, min_y, max_x, max_y);
  scene_interface_->clearUpdatedIds();

  // process removed topic obstacles
  processRemoved(scene_interface_->getRemovedObjects(), min_x, min_y, max_x, max_y);
  scene_interface_->clearRemovedIds();

  visualization_->showUpdatedBounds(prev_min_x, prev_min_y, prev_max_x, prev_max_y,
                                    *min_x, *min_y, *max_x, *max_y, layered_costmap_->getGlobalFrameID());
  ROS_DEBUG("[semantic_layer] Bounds after update:  x: %g, %g   y: %g, %g", *min_x, *max_x, *min_y, *max_y);
}

void SemanticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  std::lock_guard<std::mutex> guard(mutex_);

  cv::Point2f lower_left(layered_costmap_->getCostmap()->getOriginX() + min_i * resolution_,
                         layered_costmap_->getCostmap()->getOriginY() + min_j * resolution_);
  cv::Point2f upper_right(layered_costmap_->getCostmap()->getOriginX() + max_i * resolution_,
                          layered_costmap_->getCostmap()->getOriginY() + max_j * resolution_);

  // Get all objects within the updated area and mark their boundaries with their associated cost, sorted by precedence
  // compose a polygon with the object's contour, enforcing map bounds but not the updated bounds to avoid creating fake
  // obstacles at the borders of the updated area; order by precedence
  std::vector<Object> hash_objects = scene_interface_->getObjectsInRegion(lower_left, upper_right);
  std::sort(hash_objects.begin(), hash_objects.end(),
            [](const Object& a, const Object& b) -> bool { return a.precedence < b.precedence; });
  int lines_count = 0;
  for (const auto& object : hash_objects)
  {
    std::vector<costmap_2d::MapLocation> points_in_map;
    std::vector<costmap_2d::MapLocation> polygon_cells;
    costmap_2d::MapLocation p1_map, p2_map;
    geometry_msgs::Point p1_clip, p2_clip;
    for (auto p1 = object.contour_points.begin(), p2 = std::next(object.contour_points.begin());
         p1 != object.contour_points.end(); ++p1, ++p2)
    {
      if (p2 == object.contour_points.end())
        p2 = object.contour_points.begin();

      if (ttk::clipSegment(lower_left.x, upper_right.x, lower_left.y, upper_right.y, p1->x, p1->y, p2->x, p2->y,
                           p1_clip.x, p1_clip.y, p2_clip.x, p2_clip.y))
      {
        master_grid.worldToMapEnforceBounds(p1_clip.x, p1_clip.y, (int&)p1_map.x, (int&)p1_map.y);
        master_grid.worldToMapEnforceBounds(p2_clip.x, p2_clip.y, (int&)p2_map.x, (int&)p2_map.y);
        points_in_map.push_back(p1_map);
        points_in_map.push_back(p2_map);
        // TODO: in theory I could use worldToMapNoBounds, but it creates ghosts... no idea why
        // TODO: also the hash reports tons of obstacles within the bounds

        visualization_->showLineStrip({p1_clip, p2_clip}, 100 + ++lines_count,
                                      layered_costmap_->getGlobalFrameID(), "yellow");
      }
    }

    // ensure at least one of the obstacle points lies within the current window, do not draw if fully outside
    if (!points_in_map.empty())
    {
      // do not use convexFillCells as we can have concave obstacles in some circumstances
      // also I do not close the polygons, but still get sometimes a closing line! no idea why
      if (object.fill)
        convexFillCells(points_in_map, polygon_cells);
      else
        polygonOutlineCells(points_in_map, polygon_cells, false);
      for (const auto& cell : polygon_cells)
      {
        // enforce updated area bounds on cell coordinates because our spacial clipping can
        // let in invalid cells (on max_i and / or max_j) due to rounding (causing SW-12194)
        if ((cell.x >= min_i && cell.x < max_i && cell.y >= min_j && cell.y < max_j) &&
            (!object.use_maximum || master_grid.getCost(cell.x, cell.y) <
                                        object.cost * LETHAL_OBSTACLE))
          master_grid.setCost(cell.x, cell.y, object.cost * LETHAL_OBSTACLE);
      }
    }
  }
}

void SemanticLayer::processUpdated(const std::set<std::shared_ptr<Object>>& updated_objs,
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (const auto& updated_obj : updated_objs)
  {
    // set bounds to include updated objects
    touch(updated_obj->bounding_box.br().x, updated_obj->bounding_box.br().y, min_x, min_y, max_x, max_y);
    touch(updated_obj->bounding_box.tl().x, updated_obj->bounding_box.tl().y, min_x, min_y, max_x, max_y);

    // if the obstacle has been updated, we also need to include previous location into the bounds to clear it
    if (old_bounds_.find(updated_obj->id) != old_bounds_.end())
    {
      touch(old_bounds_[updated_obj->id].br().x, old_bounds_[updated_obj->id].br().y, min_x, min_y, max_x, max_y);
      touch(old_bounds_[updated_obj->id].tl().x, old_bounds_[updated_obj->id].tl().y, min_x, min_y, max_x, max_y);
      old_bounds_.erase(updated_obj->id);
    }
  }
}

void SemanticLayer::processRemoved(const std::set<std::shared_ptr<Object>>& removed_objs,
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  for (const auto& removed_obj : removed_objs)
  {
    // set bounds to include removed objects
    touch(removed_obj->bounding_box.br().x, removed_obj->bounding_box.br().y, min_x, min_y, max_x, max_y);
    touch(removed_obj->bounding_box.tl().x, removed_obj->bounding_box.tl().y, min_x, min_y, max_x, max_y);
  }
}

void SemanticLayer::reconfigureCB(const SemanticLayerConfig& config, uint32_t level)
{
  ROS_INFO("[semantic_layer] Received reconfigure callback");
  enabled_ = config.enabled;
}

} // end namespace

PLUGINLIB_EXPORT_CLASS(thorp_costmap_layers::SemanticLayer, costmap_2d::Layer)
