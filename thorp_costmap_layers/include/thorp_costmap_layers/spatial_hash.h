#pragma once

#include <mutex>
#include <unordered_map>

#include <opencv2/core/core.hpp> // OpenCV Rect and Point2f structs

#include <ros/ros.h>

#include <nav_msgs/MapMetaData.h>

namespace thorp_costmap_layers
{

// Object within the spatial hash
struct Object
{
  int id = -1;
  std::string type;
  cv::Point2d contour_center;
  cv::Rect_<float> bounding_box;
  std::list<cv::Point2f> contour_points;
  int update_score = 0;
  bool use_maximum = false;
  bool confirmed = false;
  bool fill = false;
  float cost = 1.0f;
};


class SpatialHash
{
public:
  SpatialHash(double cell_size, bool use_old_hash_function = true);

  ~SpatialHash();

  // puts objects into hash bins relevant to it, saves it in object hash with corresponding id
  void insertObject(const Object& object);

  // remove object based on ID
  bool removeObject(int id);

  // check if object with id exists in the spatial hash (true: yes, false: no)
  bool findObject(int id) const;

  // updates just score and confirmation state of an object
  void updateScore(const Object& object);

  // inserts object into the spatial hash, or updates entry with corresponding id if already in hash
  void updateObject(const Object& object);

  // returns all objects which are in the same hash buckets the object is in
  std::vector<Object> getNearby(const Object& object) const;

  const std::unordered_map<int, Object>& getObjectContainer() const;

  // returns ids of objects that are confirmed and have slipped below the removal threshold
  std::set<int> getObsolete(double score_threshold, double max_score);

  // returns an object by id
  std::shared_ptr<Object> getObject(int id);
  const std::shared_ptr<Object> getObject(int id) const;

  void setRemovalThreshold(int removal_threshold);

  void setConfirmationThreshold(int confirmation_threshold);

  void setMaxConfirmationScore(int max_confirmation_score);

  // returns objects in rectangular region
  std::vector<Object> getObjectsInRegion(const cv::Point2f& lower_left, const cv::Point2f& upper_right) const;
  std::vector<Object> getObjectsInRegion(const cv::Point2f& lower_left, double size_x, double size_y) const;

  // get ids of all objects in spatial hash
  std::set<int> getObjectIdsInSpatialHash() const;

  // updates object already stored in the hash
  void updateStoredObject(const Object& object);
  void clearCells();
  void clearCell(int index);
  std::set<int> getObjectsInCell(int id) const;
  // returns spatial hash cells that correspond to a rectangular region
  std::set<int> getCellsInRectangularRegion(const cv::Point2f bottom, double size_x, double size_y) const;
  // get ids of spatial hash cells that contain the object (rectangular region containing the polygonal object)
  std::set<int> getAllCellsOccupiedByPolygon(const Object& object) const;
  // hashes a point to corresponding spatial hash cell
  int getHashCell(const cv::Point2f& point) const;

  // callback for initially setting map boundaries to get collision-free
  void setMapBounds(const nav_msgs::MapMetaDataConstPtr& map_bounds);
  bool mapBoundsSet() {return map_bounds_set_;}

#ifdef DEBUG_DRAW_INTERNALS
  void getCellRect(const cv::Point2f& point_in_cell, cv::Rect& cell_rect, int cell_resolution);
#endif

private:
  // costmap-internal parameters
  double cell_size_;
  mutable std::mutex mutex_;  ///< Mutex locked in all public methods to make this class thread-safe

  // spatial hash position -> object ids in each bin
  std::unordered_map<int, std::set<int>> buckets_;
  // objects map containing all id-object pairs
  std::unordered_map<int, Object> objects_;

  // map boundary data
  bool map_bounds_set_;
  bool use_old_hash_function_;
  int map_height_;
  int map_width_;
  int times_;
};

} // namespace thorp_costmap_layers
