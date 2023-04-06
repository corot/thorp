#pragma once

#include <costmap_2d/costmap_layer.h>

#include "thorp_costmap_layers/SemanticLayerConfig.h"
#include "thorp_costmap_layers/visualization.hpp"

namespace thorp_costmap_layers
{

class SemanticLayer : public costmap_2d::CostmapLayer
{
public:
  SemanticLayer();

  ~SemanticLayer() override;

  void onInitialize() override;

  void matchSize() override;

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
  /**
   * @brief Touch the costmap bounds to include the updated obstacle areas
   */
  size_t processUpdated(const geometry_msgs::TransformStamped& tf, const std::set<std::shared_ptr<Object>>& objs,
                        double* min_x, double* min_y, double* max_x, double* max_y);

  /**
   * @brief Touch the costmap bounds to include the removed obstacle areas
   */
  size_t processRemoved(const geometry_msgs::TransformStamped& tf, const std::set<std::shared_ptr<Object>>& objs,
                        double* min_x, double* min_y, double* max_x, double* max_y);

  /**
   * @brief Touch the costmap bounds with each corner of the given rectangle transformed by tf
   */
  void transformAndTouch(const geometry_msgs::TransformStamped& tf, const Rectangle& rect,
                         double* min_x, double* min_y, double* max_x, double* max_y);

  std::set<std::unique_ptr<BaseInterface>> interfaces_;

  std::unique_ptr<Visualization> visualization_;  ///< Show updated bounds and object contours for debugging

  // for updated obstacles: back up old bounds to set removal bounds accordingly
  std::unordered_map<int, Rectangle> old_bounds_;

  std::mutex mutex_;  ///< Mutex locked on topic callback

  std::string fixed_frame_ = "map";  ///< fixed reference frame used to store objects on the spatial hash

  dynamic_reconfigure::Server<thorp_costmap_layers::SemanticLayerConfig>* dsrv_;
  void reconfigureCB(const thorp_costmap_layers::SemanticLayerConfig& config, uint32_t level);

  bool clipSegment(double edge_left, double edge_right, double edge_bottom, double edge_top,
                   double x0src, double y0src, double x1src, double y1src,
                   double& x0clip, double& y0clip, double& x1clip, double& y1clip);
};

}  // namespace thorp_costmap_layers
