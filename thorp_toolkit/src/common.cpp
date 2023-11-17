/*
 * Author: Jorge Santos
 */

#include "thorp_toolkit/common.hpp"

namespace thorp_toolkit
{

inline std_msgs::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline std_msgs::ColorRGBA randomColor(unsigned int seed, float alpha)
{
  srand(seed);
  std_msgs::ColorRGBA color;
  color.r = float(rand()) / RAND_MAX;
  color.g = float(rand()) / RAND_MAX;
  color.b = float(rand()) / RAND_MAX;
  color.a = alpha;
  return color;
}

inline std_msgs::ColorRGBA namedColor(const std::string& color_name, float alpha)
{
  static std::map<std::string, std_msgs::ColorRGBA> color_map;
  if (color_map.empty())
  {
    color_map["red"].r = 1.0f;
    color_map["blue"].b = 1.0f;
    color_map["green"].g = 1.0f;
    color_map["yellow"].r = color_map["yellow"].g = 1.0f;
    color_map["orange"].r = 1.0f;
    color_map["orange"].g = 0.65f;
    color_map["white"].r = color_map["white"].g = color_map["white"].b = 1.0f;
    color_map["gray"].r = color_map["gray"].g = color_map["gray"].b = 0.5f;
    color_map["beige"].r = color_map["beige"].g = 0.96f;
    color_map["beige"].b = 0.86f;
  }
  std_msgs::ColorRGBA& color = color_map.at(color_name);
  color.a = alpha;
  return color;
}

} /* namespace thorp_toolkit */
