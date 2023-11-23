#pragma once

#include <mutex>
#include <unordered_map>

#include <ros/ros.h>

#include <nav_msgs/MapMetaData.h>

namespace thorp::costmap_layers
{
struct Point2d
{
  Point2d() : x(0), y(0)
  {
  }
  Point2d(double x, double y) : x(x), y(y)
  {
  }

  double x, y;  //< the point coordinates

  geometry_msgs::Point toPointMsg() const
  {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    return point;
  }
};

struct Rectangle
{
  Rectangle() : tl(Point2d()), br(Point2d())
  {
  }
  Rectangle(Point2d tl, Point2d br) : tl(tl), br(br)
  {
  }

  /**
   * Return rectangle's four corners: tl, tr, br, bl
   * @return Array containing the four corners
   */
  std::array<Point2d, 4> corners() const
  {
    return { tl, Point2d(br.x, tl.y), br, Point2d(tl.x, br.y) };
  }

  Point2d tl, br;  //< the rectangle corners
};

// Object within the spatial hash
struct Object
{
  int id = -1;
  std::string type;
  Rectangle bounding_box;
  std::list<Point2d> contour_points;
  int precedence = 0;
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

  // returns objects in rectangular region
  std::vector<Object> getObjectsInRegion(const Point2d& lower_left, const Point2d& upper_right) const;
  std::vector<Object> getObjectsInRegion(const Point2d& lower_left, double size_x, double size_y) const;

  // get ids of all objects in spatial hash
  std::set<int> getObjectIdsInSpatialHash() const;

  // updates object already stored in the hash
  void updateStoredObject(const Object& object);
  void clearCells();
  void clearCell(int index);
  std::set<int> getObjectsInCell(int id) const;
  // returns spatial hash cells that correspond to a rectangular region
  std::set<int> getCellsInRectangularRegion(const Point2d bottom, double size_x, double size_y) const;
  // get ids of spatial hash cells that contain the object (rectangular region containing the polygonal object)
  std::set<int> getAllCellsOccupiedByPolygon(const Object& object) const;
  // hashes a point to corresponding spatial hash cell
  int getHashCell(const Point2d& point) const;

  // callback for initially setting map boundaries to get collision-free
  void setMapBounds(const nav_msgs::MapMetaDataConstPtr& map_bounds);
  bool mapBoundsSet()
  {
    return map_bounds_set_;
  }

#ifdef DEBUG_DRAW_INTERNALS
  void getCellRect(const Point2d& point_in_cell, Rectangle& cell_rect, int cell_resolution);
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

}  // namespace thorp::costmap_layers
