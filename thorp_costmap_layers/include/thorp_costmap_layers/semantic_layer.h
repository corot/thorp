#pragma once

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include "thorp_costmap_layers/SemanticLayerConfig.h"
#include "thorp_costmap_layers/srv_interface.h"
#include "thorp_costmap_layers/visualization.h"

namespace thorp_costmap_layers
{

class SemanticLayer : public costmap_2d::CostmapLayer
{
public:
  SemanticLayer();

  ~SemanticLayer();

  void onInitialize();

  void matchSize();

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y);

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:

  // mark updated obstacle areas
  void processUpdated(const std::set<std::shared_ptr<Object>>& updated_objs,
                      double* min_x, double* min_y, double* max_x, double* max_y);

  // mark removed obstacle areas
  void processRemoved(const std::set<std::shared_ptr<Object>>& removed_objs,
                      double* min_x, double* min_y, double* max_x, double* max_y);

  std::unique_ptr<ServiceInterface> scene_interface_;
  std::unique_ptr<Visualization> visualization_;  ///< Show updated bounds and object contours for debugging

  // for updated obstacles: back up old bounds to set removal bounds accordingly
  std::unordered_map<int, Rectangle> old_bounds_;

  std::mutex mutex_;  ///< Mutex locked on topic callback

  dynamic_reconfigure::Server<SemanticLayerConfig> *dsrv_;
  void reconfigureCB(const SemanticLayerConfig& config, uint32_t level);
};

}
