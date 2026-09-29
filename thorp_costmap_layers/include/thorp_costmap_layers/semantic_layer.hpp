#pragma once

#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <thorp_costmap_layers/msg/object.hpp>
#include <thorp_costmap_layers/srv/query_objects.hpp>
#include <thorp_costmap_layers/srv/update_objects.hpp>

namespace thorp_costmap_layers
{
/**
 * Costmap layer marking known objects, as the tables the executive has found, that the sensors can't see well, or
 * clearing areas, as the approach to a table. Objects are rectangles, added and removed by name through the
 * ~/<layer>/update_objects service, and kept on a fixed frame. Each object has a type, with parameters saying how to
 * mark it: cost (0 free, 1 lethal), whether to fill it or only mark its outline, padding around it, and precedence,
 * as objects are painted in increasing precedence order; e.g. free space clears the obstacles painted before it.
 */
class SemanticLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SemanticLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override
  {
  }
  // Objects stay until removed, as the static map
  bool isClearable() override
  {
    return false;
  }

private:
  struct ObjectType
  {
    double cost = 1.0;
    bool fill = false;
    double length_padding = 0.0;
    double width_padding = 0.0;
    int precedence = 0;
    bool use_maximum = false;  ///< keep higher costs underneath instead of overwriting them
  };

  struct Object
  {
    msg::Object msg;                     ///< as added, to report it back on queries
    std::array<double, 8> corners;       ///< x, y pairs of the rectangle corners on the fixed frame, in order
    std::array<double, 4> bounding_box;  ///< min x, min y, max x, max y on the fixed frame
  };

  void updateObjects(const std::shared_ptr<srv::UpdateObjects::Request> request,
                     std::shared_ptr<srv::UpdateObjects::Response> response);
  void queryObjects(const std::shared_ptr<srv::QueryObjects::Request> request,
                    std::shared_ptr<srv::QueryObjects::Response> response);

  /** The object's rectangle, padded as its type says, on the fixed frame; none if its pose can't be transformed */
  std::optional<Object> makeObject(const msg::Object& msg, const ObjectType& type) const;

  /** Touch the bounds with a bounding box on the fixed frame, transformed to the costmap frame */
  void touchBox(const geometry_msgs::msg::TransformStamped& to_costmap, const std::array<double, 4>& box,
                double* min_x, double* min_y, double* max_x, double* max_y);

  std::string fixed_frame_ = "map";
  std::map<std::string, ObjectType> object_types_;

  std::mutex mutex_;  ///< the service callbacks run on other thread than the costmap updates
  std::map<std::string, Object> objects_;
  std::vector<std::array<double, 4>> removed_boxes_;  ///< areas to clear on the next update

  rclcpp::Service<srv::UpdateObjects>::SharedPtr update_srv_;
  rclcpp::Service<srv::QueryObjects>::SharedPtr query_srv_;
};

}  // namespace thorp_costmap_layers
