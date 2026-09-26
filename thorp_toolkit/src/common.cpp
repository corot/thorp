/*
 * Author: Jorge Santos
 */

#include "thorp_toolkit/common.hpp"
#include "thorp_toolkit/tf2.hpp"

#include <cmath>
#include <map>
#include <sstream>

#include <boost/algorithm/string/trim.hpp>

namespace thorp::toolkit
{

namespace
{
// weak, so the toolkit doesn't extend the node's lifetime
rclcpp::Node::WeakPtr toolkit_node;
}  // namespace

void init(const rclcpp::Node::SharedPtr& node)
{
  toolkit_node = node;
  // release the singletons using the node before the context shuts down; otherwise they would be destroyed
  // after main returns, when the middleware is already gone
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([]() { TF2::destroy(); });
}

rclcpp::Node::SharedPtr node()
{
  auto node = toolkit_node.lock();
  if (!node)
  {
    throw std::runtime_error("thorp_toolkit not initialized; call thorp::toolkit::init(node) first");
  }
  return node;
}

rclcpp::Logger logger()
{
  auto node = toolkit_node.lock();
  return node ? node->get_logger() : rclcpp::get_logger("thorp_toolkit");
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA randomColor(unsigned int seed, float alpha)
{
  srand48(seed);
  std_msgs::msg::ColorRGBA color;
  color.r = drand48();
  color.g = drand48();
  color.b = drand48();
  color.a = alpha;
  return color;
}

std_msgs::msg::ColorRGBA namedColor(const std::string& color_name, float alpha)
{
  // every name colorName can return, so the two remain inverses of each other
  static const std::map<std::string, std_msgs::msg::ColorRGBA> color_map = {
    { "black", makeColor(0.0f, 0.0f, 0.0f) },
    { "gray", makeColor(0.5f, 0.5f, 0.5f) },
    { "light gray", makeColor(0.75f, 0.75f, 0.75f) },
    { "white", makeColor(1.0f, 1.0f, 1.0f) },
    { "red", makeColor(1.0f, 0.0f, 0.0f) },
    { "orange", makeColor(1.0f, 0.65f, 0.0f) },
    { "yellow", makeColor(1.0f, 1.0f, 0.0f) },
    { "green", makeColor(0.0f, 1.0f, 0.0f) },
    { "cyan", makeColor(0.0f, 1.0f, 1.0f) },
    { "blue", makeColor(0.0f, 0.0f, 1.0f) },
    { "purple", makeColor(0.5f, 0.0f, 0.5f) },
    { "pink", makeColor(1.0f, 0.75f, 0.8f) },
    { "beige", makeColor(0.96f, 0.96f, 0.86f) }
  };
  std_msgs::msg::ColorRGBA color = color_map.at(color_name);
  color.a = alpha;
  return color;
}

// Lab rather than rgb: lightness is an axis of its own there, so dimming the scene moves L and
// mostly leaves the hue alone. Chroma decides first, so a dark but saturated object keeps its
// hue and only a colorless one falls back on lightness. The hue bands are where the RGB
// primaries and their midpoints land in Lab, which is nowhere near evenly spaced.
std::string colorName(float lightness, float a, float b)
{
  if (std::hypot(a, b) < 15.0f)  // no color to speak of; lightness is all there is to report
  {
    if (lightness > 85.0f)
    {
      return "white";
    }
    if (lightness > 60.0f)
    {
      return "light gray";
    }
    return lightness > 20.0f ? "gray" : "black";
  }

  double hue = std::atan2(b, a) * 180.0 / M_PI;
  if (hue < 0.0)
  {
    hue += 360.0;
  }
  for (const auto& [upper_bound, name] : { std::pair{ 19.0, "pink" }, std::pair{ 50.0, "red" },
                                           std::pair{ 82.0, "orange" }, std::pair{ 108.0, "yellow" },
                                           std::pair{ 166.0, "green" }, std::pair{ 212.0, "cyan" },
                                           std::pair{ 308.0, "blue" }, std::pair{ 332.0, "purple" } })
  {
    if (hue < upper_bound)
    {
      return name;
    }
  }
  return "pink";  // the last band wraps past 360 into the first
}

std::vector<std::string> tokenize(const std::string& csv)
{
  std::vector<std::string> result;

  std::stringstream ss(csv);
  while (ss.good())
  {
    std::string substr;
    getline(ss, substr, ',');
    boost::algorithm::trim(substr);
    result.push_back(substr);
  }
  return result;
}

} /* namespace thorp::toolkit */
