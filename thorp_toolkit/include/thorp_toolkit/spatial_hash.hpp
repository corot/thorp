#include <cmath>
#include <iostream>
#include <vector>
#include <unordered_map>

struct Rectangle
{
  double x_min, x_max, y_min, y_max;

  Rectangle(double _x_min, double _x_max, double _y_min, double _y_max)
    : x_min(_x_min), x_max(_x_max), y_min(_y_min), y_max(_y_max)
  {
  }

  bool intersects(const Rectangle& other) const
  {
    return !(x_max < other.x_min || x_min > other.x_max || y_max < other.y_min || y_min > other.y_max);
  }
};

class SpatialHash
{
private:
  double cell_size_;
  std::unordered_map<int, std::vector<Rectangle>> grid;

  int hashFunction(int x, int y)
  {
    // Simple hash function for 2D points
    return x ^ (y << 16);
  }

public:
  SpatialHash(double cell_size) : cell_size_(cell_size)
  {
  }

  void insert(const Rectangle& rect)
  {
    int cell_x_min = static_cast<int>(std::floor(rect.x_min / cell_size_));
    int cell_y_min = static_cast<int>(std::floor(rect.y_min / cell_size_));
    int cell_x_max = static_cast<int>(std::floor(rect.x_max / cell_size_));
    int cell_y_max = static_cast<int>(std::floor(rect.y_max / cell_size_));

    for (int x = cell_x_min; x <= cell_x_max; ++x)
    {
      for (int y = cell_y_min; y <= cell_y_max; ++y)
      {
        int hash = hashFunction(x, y);
        grid[hash].push_back(rect);
      }
    }
  }

  std::vector<Rectangle> query(const Rectangle& rect)
  {
    std::vector<Rectangle> result;

    int cell_x_min = static_cast<int>(std::floor(rect.x_min / cell_size_));
    int cell_y_min = static_cast<int>(std::floor(rect.y_min / cell_size_));
    int cell_x_max = static_cast<int>(std::floor(rect.x_max / cell_size_));
    int cell_y_max = static_cast<int>(std::floor(rect.y_max / cell_size_));

    for (int x = cell_x_min; x <= cell_x_max; ++x)
    {
      for (int y = cell_y_min; y <= cell_y_max; ++y)
      {
        int hash = hashFunction(x, y);
        auto& cell = grid[hash];
        for (const auto& candidate : cell)
        {
          if (candidate.intersects(rect))
          {
            result.push_back(candidate);
          }
        }
      }
    }

    return result;
  }
};

int main()
{
  SpatialHash spatialHash(10.0);  // Define cell size

  spatialHash.insert(Rectangle(2.0, 5.0, 3.0, 6.0));
  spatialHash.insert(Rectangle(15.0, 20.0, 10.0, 15.0));

  // Query the grid for rectangles around a specific location
  std::vector<Rectangle> rectangles = spatialHash.query(Rectangle(4.0, 6.0, 4.0, 8.0));
  for (const auto& rect : rectangles)
  {
    std::cout << "Rectangle found: (" << rect.x_min << ", " << rect.y_min << ") - (" << rect.x_max << ", " << rect.y_max
              << ")" << std::endl;
  }

  return 0;
}
