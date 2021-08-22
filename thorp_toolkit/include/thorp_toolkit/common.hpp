/*
 * Author: Jorge Santos
 */

#pragma once


#include <std_msgs/ColorRGBA.h>


namespace thorp_toolkit
{

/**
 * Make a RGBA color msg
 * @param r red
 * @param g green
 * @param b blue
 * @param a alpha
 * @return RGBA color msg
 */
inline std_msgs::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * Make a random RGBA color msg
 * @param seed randomizer seed
 * @param alpha transparency
 * @return RGBA color msg
 */
inline std_msgs::ColorRGBA randomColor(unsigned int seed = 0, float alpha = 1.0f)
{
  srand(seed);
  std_msgs::ColorRGBA color;
  color.r = float(rand()) / RAND_MAX;
  color.g = float(rand()) / RAND_MAX;
  color.b = float(rand()) / RAND_MAX;
  color.a = alpha;
  return color;
}

/**
 * Make a RGBA color msg for a color name
 * @param color_name color by name
 * @param alpha transparency
 * @return RGBA color msg
 * @throw  std::out_of_range  If color_name doesn't exist
 */
inline std_msgs::ColorRGBA namedColor(const std::string& color_name, float alpha = 1.0f)
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
