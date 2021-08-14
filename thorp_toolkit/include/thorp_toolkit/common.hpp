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

} /* namespace thorp_toolkit */
