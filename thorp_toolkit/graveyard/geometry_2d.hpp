/*
 * Author: Jorge Santos
 */

#pragma once


#include <geometry_msgs/Pose.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/foreach.hpp>


struct Point2d
{
  Point2d() : x(0), y(0) {}
  Point2d(double x, double y) : x(x), y(y) {}

  double x, y; //< the point coordinates

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
  Rectangle() : tl(Point2d()), br(Point2d()) {}
  Rectangle(Point2d tl, Point2d br) : tl(tl), br(br) {}

  Point2d tl, br; //< the rectangle corners
};


namespace thorp_toolkit
{

bool intersection(const std::list<Point2d>& polygon1, const std::list<Point2d>& polygon2, std::list<Point2d>& output);

} /* namespace thorp_toolkit */
