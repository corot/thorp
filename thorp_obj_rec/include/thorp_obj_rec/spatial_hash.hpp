/*
 * spatial_hash.hpp
 *
 *  Created on: May 20, 2017
 *      Author: jorge
 */

#pragma once


template <class T> class SpatialHash
{
public:
  /**
   * We can work out how many buckets we need by first calculating the rows and cols then simply create a new
   * dictionary of buckets to the tune of rows * cols. I also store these variables passed in for future use.
   */
  void setup(double scene_width, double scene_height, double cell_size)
  {
    cols_ = ceil(scene_width / cell_size);
    rows_ = ceil(scene_height / cell_size);

    scene_width_ = scene_width;
    scene_height_ = scene_height;
    cell_size_ = cell_size;
  }

  /**
   * Each update, we need to clear out the buckets.
   */
  void clear()
  {
    buckets_.clear();
  }

  /**
   * We now need a method to register a game object into the buckets it sits in.
   */
  void registerValue(double x, double y, double radius, T value)
  {
    std::set<int> cell_ids = getIdForObj(x, y, radius);
    for (int cell : cell_ids)
    {
      buckets_[cell].insert(value);
    }
  }

  std::set<T> getNearbyValues(double x, double y, double radius)
  {
    std::set<T> objects;
    std::set<int> bucket_ids = getIdForObj(x, y, radius);
    for (int id : bucket_ids)
    {
      objects.insert(buckets_[id].begin(), buckets_[id].end());
    }
    return objects;
  }

//As you can see, the code retrieves a list of cellids to add the game object to.
//
//In the getIdForObj method it calculates the cell id for each corner of the game object.
//
//If we were just checking the position of the game object the calculation would be.

//    int hash(geometry_msgs::Pose obj)
//    {
//        double width = scene_width_ / cell_size_; // 100 / 25
//
//        int hashid=(int)(
//
//            (Math.Floor(position.X / cell_size_)) +
//
//            (Math.Floor(position.Y / cell_size_)) * width);
//        return hashid;
//    }

private:
  uint8_t cols_;
  uint8_t rows_;
  uint8_t scene_width_;
  uint8_t scene_height_;
  double cell_size_;
  std::map<int, std::set<T>> buckets_;

//We need to do this for each corner and add our game to each bucket.
//
//The getIdForObj method looks like this.

  std::set<int> getIdForObj(double x, double y, double radius)
  {
    std::set<int> buckets_obj_is_in;

    double min_x = x - radius;
    double min_y = y - radius;
    double max_x = x + radius;
    double max_y = y + radius;

    double width = scene_width_ / cell_size_;
    //TopLeft
    addBucket(min_x, min_y, width, buckets_obj_is_in);
    //TopRight
    addBucket(max_x, min_y, width, buckets_obj_is_in);
    //BottomRight
    addBucket(max_x, max_y, width, buckets_obj_is_in);
    //BottomLeft
    addBucket(min_x, max_y, width, buckets_obj_is_in);

    return buckets_obj_is_in;
  }
//And here is the addBucket method which uses the calculation described above and adds it to the list of bucket id’s to add to.

  void addBucket(double x, double y, double width, std::set<int>& bucket_to_add_to)
  {
    int cell_position = (int)((floor(x / cell_size_)) + (floor(y / cell_size_)) * width);
    bucket_to_add_to.insert(cell_position);
  }
//Now that we have our grid of buckets. It’s a very simple retrieval process. I added a method to get the nearby objects of a given object. This uses the getIdForObj method and populates a list of geometry_msgs::Pose’s and returns once complete. This is the key part to this solution, you only now need to check items which are actually nearby and not items the other side of the theoretical world.
};

