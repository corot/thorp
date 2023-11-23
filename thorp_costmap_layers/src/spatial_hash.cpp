#include "thorp_costmap_layers/spatial_hash.hpp"

namespace thorp::costmap_layers
{

SpatialHash::SpatialHash(double cell_size, bool use_old_hash_function)
  : buckets_(100)
  , objects_(100)
  , map_bounds_set_(false)
  , use_old_hash_function_(use_old_hash_function)
  , map_height_(0)
  , map_width_(0)
  , times_(0)
{
  cell_size_ = cell_size;
}

SpatialHash::~SpatialHash()
{
}

void SpatialHash::clearCells()
{
  std::lock_guard<std::mutex> guard(mutex_);

  buckets_.clear();
  objects_.clear();
}

void SpatialHash::clearCell(int index)
{
  std::lock_guard<std::mutex> guard(mutex_);

  buckets_.erase(index);
}

// puts objects into hash bins relevant to it, saves it in object hash with corresponding id
void SpatialHash::insertObject(const Object& object)
{
  std::lock_guard<std::mutex> guard(mutex_);

  for (auto id : getAllCellsOccupiedByPolygon(object))
  {
    buckets_[id].insert(object.id);
  }

  objects_[object.id] = object;
}

// remove object based on ID
bool SpatialHash::removeObject(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);

  if (objects_.find(id) == objects_.end())
  {
    ROS_WARN_STREAM("The object with id " << id << " has already been removed from the hash");
    return false;
  }

  // delete all hash bucket entries pointing to the object
  std::set<int> bucket_ids = getAllCellsOccupiedByPolygon(objects_[id]);
  for (auto id : bucket_ids)
  {
    buckets_[id].erase(id);
  }

  objects_.erase(id);

  return true;
}

// check if object with id exists in the spatial hash (true: yes, false: no)
bool SpatialHash::findObject(int id) const
{
  std::lock_guard<std::mutex> guard(mutex_);

  return objects_.find(id) != objects_.end();
}

// updates just score and confirmation state of an object
void SpatialHash::updateScore(const Object& object)
{
  if (objects_.find(object.id) == objects_.end())
  {
    ROS_DEBUG("An object scheduled for score updating has been removed.");
    return;
  }

  objects_[object.id].update_score = object.update_score;
  objects_[object.id].confirmed = object.confirmed;
}

// inserts object into the spatial hash, or updates entry with corresponding id if already in hash
void SpatialHash::updateObject(const Object& object)
{
  if (objects_.find(object.id) == objects_.end())
  {
    // just insert new object if it is not in hash yet
    insertObject(object);
    return;
  }

  // WARN: Do not lock before, as insertObject also tries to lock the mutex and is not recursive!
  std::lock_guard<std::mutex> guard(mutex_);

  std::set<int> new_bucket_ids = getAllCellsOccupiedByPolygon(object);

  std::set<int> old_bucket_ids = getAllCellsOccupiedByPolygon(objects_[object.id]);

  // update object data
  updateStoredObject(object);

  // compare cell ids, if they changed, remove old and add new cell entries
  if (new_bucket_ids != old_bucket_ids)
  {
    // remove all bucket entries for object that are no longer valid (in case objects shrink)
    for (auto id : old_bucket_ids)
    {
      buckets_[id].erase(object.id);
    }

    // add in all valid new entries
    for (auto id : new_bucket_ids)
    {
      buckets_[id].insert(object.id);
    }
  }
}

// returns all objects which are in the same hash buckets the current object is in; use a set to avoid duplications
std::vector<Object> SpatialHash::getNearby(const Object& object) const
{
  std::lock_guard<std::mutex> guard(mutex_);

  std::vector<Object> nearby_objects;
  std::set<int> cell_ids = getAllCellsOccupiedByPolygon(object);
  for (auto cell_id : cell_ids)
  {
    try
    {
      for (auto object_id : buckets_.at(cell_id))
      {
        auto obj = objects_.find(object_id);
        if (obj != objects_.end() && object_id != object.id)
        {
          nearby_objects.push_back(obj->second);
        }
      }
    }
    catch (std::out_of_range& e)
    {
      // thrown by at if not found; should never happen because the given object is by definition in all the buckets
      // returned by getAllCellsOccupiedByPolygon... but it's not true, I think because we are not precise at all
      // ROS_WARN("SpatialHash::getNearby %d: %s", object.id, e.what());  commented, as happens all the time TODO XXX
    }
  }

  return nearby_objects;
}

// returns objects in rectangular region
std::vector<Object> SpatialHash::getObjectsInRegion(const Point2d& lower_left, const Point2d& upper_right) const
{
  return getObjectsInRegion(lower_left, upper_right.x - lower_left.x, upper_right.y - lower_left.y);
}

std::vector<Object> SpatialHash::getObjectsInRegion(const Point2d& lower_left, double size_x, double size_y) const
{
  std::lock_guard<std::mutex> guard(mutex_);

  // get buckets in the region
  std::set<int> hash_ids = getCellsInRectangularRegion(lower_left, size_x, size_y);

  // get all object ids from those buckets
  std::set<int> visible_object_ids;
  for (auto bucket_id : hash_ids)
  {
    try
    {
      for (auto object_id : buckets_.at(bucket_id))
      {
        auto obj = objects_.find(object_id);
        if (obj != objects_.end())
        {
          visible_object_ids.insert(object_id);
        }
      }
    }
    catch (std::out_of_range& e)
    {
    }  // this is fine; means this hash has been never used; but I wonder if this has any performance penalization...
  }

  // get those objects and return them
  std::vector<Object> visible_objects;
  for (auto object_id : visible_object_ids)
  {
    try
    {
      visible_objects.push_back(objects_.at(object_id));
    }
    catch (std::out_of_range& e)
    {
      // this should not happen: cannot find the object for an id added to a bucket, so there's an incoherence
      ROS_WARN("SpatialHash::getVisibleObjects: %s   object missing??? %d", e.what(), object_id);
    }
  }
  return visible_objects;
}

std::set<int> SpatialHash::getObjectsInCell(int id) const
{
  try
  {
    return buckets_.at(id);
  }
  catch (std::out_of_range& e)
  {
    return std::set<int>();  // this is fine; means we didn't use yet this bucket
  }
}

// returns spatial hash cells that correspond to a rectangular region
std::set<int> SpatialHash::getCellsInRectangularRegion(const Point2d bottom, double size_x, double size_y) const
{
  std::set<int> relevant_cells;
  for (float x = std::floor(bottom.x / cell_size_) * cell_size_;
       x < std::ceil((bottom.x + size_x) / cell_size_) * cell_size_; x += cell_size_)
  {
    for (float y = std::floor(bottom.y / cell_size_) * cell_size_;
         y < std::ceil((bottom.y + size_y) / cell_size_) * cell_size_; y += cell_size_)
    {
      relevant_cells.insert(getHashCell(Point2d(x, y)));
    }
  }
  return relevant_cells;
}

// hashes a point to corresponding spatial hash bucket
int SpatialHash::getHashCell(const Point2d& point) const
{
  int ix = std::floor(point.x / cell_size_);
  int iy = std::floor(point.y / cell_size_);

  // hash function from http://stackoverflow.com/questions/2634690/good-hash-function-for-a-2d-index
  int hash_id = 0;
  if (use_old_hash_function_)
    hash_id = (53 + ix) * 53 + iy;
  else
    hash_id = iy * times_ + ix;
  return hash_id;
}

void SpatialHash::setMapBounds(const nav_msgs::MapMetaDataConstPtr& map_bounds)
{
  if (map_bounds_set_)
    return;

  // calculate width and height in cells
  map_height_ = map_bounds->height * map_bounds->resolution / cell_size_;
  map_width_ = map_bounds->width * map_bounds->resolution / cell_size_;

  times_ = 1;
  while (times_ <= map_width_)
    times_ *= 10;

  map_bounds_set_ = true;
}

const std::unordered_map<int, Object>& SpatialHash::getObjectContainer() const
{
  return objects_;
}

// returns ids of objects that are confirmed and have slipped below the removal threshold
std::set<int> SpatialHash::getObsolete(double score_threshold, double max_score)
{
  std::lock_guard<std::mutex> guard(mutex_);

  std::set<int> obsolete_ids;

  auto it_object = objects_.begin();
  while (it_object != objects_.end())
  {
    // safety check: maintain maximum score threshold
    if (it_object->second.update_score > max_score)
    {
      it_object->second.update_score = max_score;
    }

    // get all confirmed objects and add their ids if they have a score below the removal threshold
    if (it_object->second.confirmed)
    {
      if (it_object->second.update_score < score_threshold)
      {
        obsolete_ids.insert(it_object->second.id);
        std::advance(it_object, 1);
      }
      else
      {
        std::advance(it_object, 1);
      }
    }
    else
    {
      // delete objects that have not been confirmed as well and are not being
      // found in map again (to prevent initially detected junk from accumulating)
      // fixed value for now, if it causes problems, tweak by parameter
      if (it_object->second.update_score < -50)
      {
        obsolete_ids.insert(it_object->second.id);
        std::advance(it_object, 1);
      }
      else
      {
        std::advance(it_object, 1);
      }
    }
  }
  return obsolete_ids;
}

// returns an object by id
std::shared_ptr<Object> SpatialHash::getObject(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);

  auto obj = objects_.find(id);
  if (obj == objects_.end())
    return nullptr;
  return std::make_shared<Object>(obj->second);
}

const std::shared_ptr<Object> SpatialHash::getObject(int id) const
{
  std::lock_guard<std::mutex> guard(mutex_);

  auto obj = objects_.find(id);
  if (obj == objects_.end())
    return nullptr;
  return std::make_shared<Object>(obj->second);
}

// get ids of spatial hash buckets that contain the object (rectangular region containing the polygonal object)
// XXX, WARN: note that it ignores the contour! only the bounding box is used (will try to fix this, though)
//            also, a 10 cm padding is applied to the bounding box, not sure why
std::set<int> SpatialHash::getAllCellsOccupiedByPolygon(const Object& object) const
{
  // get outer values of the polygon bounding box
  //  float min_x = std::numeric_limits<float>::max(), min_y = std::numeric_limits<float>::max();
  //  float max_x = std::numeric_limits<float>::lowest(), max_y = std::numeric_limits<float>::lowest();
  //  for (auto point : obstacle.contour_points) {
  //     if (point.x > max_x) {
  //       max_x = point.x;
  //     }

  //     if (point.y > max_y) {
  //       max_y = point.y;
  //     }

  //     if (point.x < min_x) {
  //       min_x = point.x;
  //     }

  //     if (point.y < min_y) {
  //       min_y = point.y;
  //     }
  //  }
  //  ROS_INFO_STREAM("In spatialHash::getAllCellsOccupiedByPolygon: bounding box tl is:
  //  "<<obstacle.bounding_box.tl()<<" bounding box br is: "<<obstacle.bounding_box.br());

  std::set<int> relevant_buckets;

  float min_x = object.bounding_box.tl.x;
  float min_y = object.bounding_box.tl.y;
  float max_x = object.bounding_box.br.x;
  float max_y = object.bounding_box.br.y;

  // safety check: both directions have failed, in that case we have to clean the directions a bit
  //  if (std::floor(min_x/cell_size_) == std::ceil(max_x/cell_size_) && std::floor(min_y/cell_size_) ==
  //  std::ceil(max_y/cell_size_)) {
  //      ROS_WARN("In getAllCellsOccupiedByPolygon (spatial hash), got an empty obstacle with id: %d, it has: %d
  //      points",obstacle.id, obstacle.contour_points.size()); ROS_INFO_STREAM("Cellsize is: "<<cell_size_);
  //      ROS_INFO_STREAM("Loop bounds for x would have been: "<<std::floor(min_x/cell_size_)*cell_size_<<" and
  //      "<<std::ceil(max_x/cell_size_)*cell_size_); ROS_INFO_STREAM("Loop bounds for y would have been:
  //      "<<std::floor(min_y/cell_size_)*cell_size_<<" and "<<std::ceil(max_y/cell_size_)*cell_size_); ROS_INFO("My
  //      points are: "); for (auto point: obstacle.contour_points) {
  //        ROS_INFO_STREAM("Point: "<<point);
  //      }
  // offset both boundaries so the object gets assigned to bins
  //      min_x = min_x-0.1;
  //      max_x = max_x+0.1;
  //      min_y = min_y-0.1;
  //      max_y = max_y=0.1;
  //      ROS_INFO_STREAM("Loop bounds for x now are: "<<std::floor(min_x/cell_size_)*cell_size_<<" and
  //      "<<std::ceil(max_x/cell_size_)*cell_size_); ROS_INFO_STREAM("Loop bounds for y now are:
  //      "<<std::floor(min_y/cell_size_)*cell_size_<<" and "<<std::ceil(max_y/cell_size_)*cell_size_);
  ////      return relevant_buckets;
  //    }

  // line object can have no extent along one of the axis possibly (problem with line objects remaining stuck within the
  // hash)
  if (std::floor(min_x / cell_size_) == std::ceil(max_x / cell_size_))
  {
    //    for (float y =std::floor(min_y/cell_size_)*cell_size_; y<std::ceil(max_y/cell_size_)*cell_size_;
    //    y+=cell_size_) {
    //      ROS_INFO_STREAM("Obstacle "<<obstacle.id<<" has x extent of 0");
    //      ROS_INFO_STREAM("Cellsize is: "<<cell_size_);
    //      ROS_INFO_STREAM("Loop bounds for x would have been: "<<std::floor(min_x/cell_size_)*cell_size_<<" and
    //      "<<std::ceil(max_x/cell_size_)*cell_size_); ROS_INFO("My points are: "); for (auto point:
    //      obstacle.contour_points) {
    //        ROS_INFO_STREAM("Point: "<<point);
    //      }
    //      relevant_buckets.insert(getHashBucket(Point2d(min_x, y)));
    //    ROS_DEBUG_STREAM("Floored obstacle with id "<<object.id);
    min_x = min_x - cell_size_ / 2.0;
    max_x = max_x + cell_size_ / 2.0;
    //      ROS_INFO_STREAM("Loop bounds for x now are: "<<std::floor(min_x/cell_size_)*cell_size_<<" and
    //      "<<std::ceil(max_x/cell_size_)*cell_size_);
    //    }
    //    return relevant_buckets;
  }

  if (std::floor(min_y / cell_size_) == std::ceil(max_y / cell_size_))
  {
    //    for (float x=std::floor(min_x/cell_size_)*cell_size_; x<std::ceil(max_x/cell_size_)*cell_size_; x+=cell_size_)
    //    {
    //      ROS_INFO_STREAM("Obstacle "<<obstacle.id<<" has y extent of 0");
    //      ROS_INFO_STREAM("Cellsize is: "<<cell_size_);
    //      ROS_INFO_STREAM("Loop bounds for y would have been: "<<std::floor(min_y/cell_size_)*cell_size_<<" and
    //      "<<std::ceil(max_y/cell_size_)*cell_size_); ROS_INFO("My points are: "); for (auto point:
    //      obstacle.contour_points) {
    //        ROS_INFO_STREAM("Point: "<<point);
    //      }
    //    ROS_DEBUG_STREAM("Floored obstacle with id: "<<object.id);
    min_y = min_y - cell_size_ / 2.0;
    max_y = max_y + cell_size_ / 2.0;
    //      ROS_INFO_STREAM("Loop bounds for y now are: "<<std::floor(min_y/cell_size_)*cell_size_<<" and
    //      "<<std::ceil(max_y/cell_size_)*cell_size_); relevant_buckets.insert(getHashBucket(Point2d(x, min_y)));
    //    }
    //    return relevant_buckets;
  }
  //  ROS_INFO_STREAM("Contour has "<<obstacle.contour_points.size()<< " points, which are: ");
  //  for (auto point : obstacle.contour_points)
  //      ROS_INFO_STREAM("point: "<<point);

  //  ROS_INFO_STREAM("Got min values of (x,y): "<<min_x<<","<<min_y<<" and max values of: "<<max_x<<","<<max_y);

  //  float size_x = max_x - min_x;
  //  float size_y = max_y - min_y;
  //  ROS_INFO_STREAM("X size is: "<<size_x<<" Y size is: "<<size_y<<" - x should run:
  //  "<<std::ceil(size_x/cell_size_)<<" times, y should run: "<<std::ceil(size_y/cell_size_)<<" times"); Rectangle
  //  bucket_rect;

  // line objects can have no extent along one of the axis possibly (problem with line objects remaining stuck within
  // the hash(

  for (float x = std::floor(min_x / cell_size_) * cell_size_; x < std::ceil(max_x / cell_size_) * cell_size_;
       x += cell_size_)
  {
    for (float y = std::floor(min_y / cell_size_) * cell_size_; y < std::ceil(max_y / cell_size_) * cell_size_;
         y += cell_size_)
    {
      // todo: check the values this returns
      //      ROS_INFO_STREAM("Testing point: "<< Point2d(x,y));
      //      ROS_INFO_STREAM("Point polygon test returns value of: "<<
      //      cv::pointPolygonTest(std::vector<Point2d>(obstacle.contour_points.begin(),obstacle.contour_points.end()),
      //                                                                                   Point2d(x, y), true)
      //                      << " and cell size is "<< cell_size_);
      //      ROS_INFO("/n");

      // draw a grid just for visual verification and confirm what happens

#ifdef DEBUG_DRAW_HASH_INTERNALS
      // so we do not draw before the hash has been properly initialize
      bool do_visualization = false;
      if (stage_width_ > 0 && stage_height_ > 0)
        do_visualization = true;
#endif

        //      int cell_resolution = 10;
        //      cv::Mat debug_image;

#ifdef DEBUG_DRAW_HASH_INTERNALS
      if (do_visualization)
      {
        int img_width = std::ceil(stage_width_ / cell_size_) * cell_resolution;
        int img_height = std::ceil(stage_height_ / cell_size_) * cell_resolution;
        debug_image = cv::Mat::zeros(img_height, img_width, CV_8UC3);
        // show grid lines
        for (int i = 0; i < img_width; i += cell_resolution)
          cv::line(debug_image, cv::Point(i, 0), cv::Point(i, img_height), cv::Scalar(255, 255, 255));

        for (int i = 0; i < img_height; i += cell_resolution)
          cv::line(debug_image, cv::Point(0, i), cv::Point(img_width, i), cv::Scalar(255, 255, 255));

        getBucketRect(Point2d(x, y), bucket_rect, cell_resolution);
      }
#endif

      // value is a bit higher, its better if we overrepresent obstacles at border than create doubled obstacles
      //      if
      //      (std::abs(cv::pointPolygonTest(std::vector<Point2d>(obstacle.contour_points.begin(),obstacle.contour_points.end()),
      //                                                                 Point2d(x,y), true))
      //                   < cell_size_*2.0) {
      relevant_buckets.insert(getHashCell(Point2d(x, y)));
#ifdef DEBUG_DRAW_HASH_INTERNALS
      //        ROS_INFO_STREAM("Confirmed bucket I got has br: "<<bucket_rect.br()<<" and tl: "<<bucket_rect.tl());
      if (do_visualization)
        cv::rectangle(debug_image, bucket_rect, cv::Scalar(0, 255, 0), CV_FILLED);
    }
    else
    {
      if (do_visualization)
        cv::rectangle(debug_image, bucket_rect, cv::Scalar(0, 0, 255), CV_FILLED);
#endif
        //      }

#ifdef DEBUG_DRAW_HASH_INTERNALS
      if (do_visualization)
      {
        std::vector<std::vector<cv::Point>> contour_points(1);
        for (auto point : object.contour_points)
        {
          contour_points[0].push_back(
              cv::Point(point.x / cell_size_ * cell_resolution, point.y / cell_size_ * cell_resolution));
        }
        //        ROS_INFO_STREAM("Size of first contour point container is: "<<contour_points[0].size());
        if (stage_width_ > 0 && stage_height_ > 0)
        {
          cv::drawContours(debug_image, contour_points, 0, cv::Scalar(255, 0, 0));
          cv::namedWindow("Spatial hash checks", CV_WINDOW_NORMAL);
          cv::imshow("Spatial hash checks", debug_image);
          // this will stall updating, so remove if you want the rest of the visualization in real-time
          cv::waitKey(0);
        }
      }
#endif
    }
  }

  return relevant_buckets;
}

// get ids of all objects in spatial hash
std::set<int> SpatialHash::getObjectIdsInSpatialHash() const
{
  std::set<int> objects_in_hash;
  for (const auto& object : objects_)
  {
    objects_in_hash.insert(object.first);
  }
  return objects_in_hash;
}

// updates object already stored in the hash
void SpatialHash::updateStoredObject(const Object& object)
{
  objects_[object.id].contour_points = object.contour_points;
  objects_[object.id].bounding_box = object.bounding_box;
  // added this (not before test)
  objects_[object.id].confirmed = object.confirmed;
  objects_[object.id].update_score = object.update_score;
}

#ifdef DEBUG_DRAW_INTERNALS
void SpatialHash::getCellRect(const Point2d& point_in_cell, cv::Rect& cell_rect, int cell_resolution)
{
  cell_rect = Rectangle(cv::Point((int)std::floor(point_in_cell.x / cell_size_) * cell_resolution,
                                  (int)std::floor(point_in_cell.y / cell_size_) * cell_resolution),
                        cv::Point((int)std::ceil((point_in_cell.x + 0.00001f) / cell_size_) * cell_resolution,
                                  (int)std::ceil((point_in_cell.y + 0.00001f) / cell_size_) * cell_resolution));
}
#endif

}  // namespace thorp::costmap_layers
