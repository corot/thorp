/*
 * Author: Jorge Santos
 */

#include "thorp_toolkit/geometry_2d.hpp"

namespace thorp_toolkit
{

using bgPoint2d = boost::geometry::model::d2::point_xy<double>;
using bgPolygon = boost::geometry::model::polygon<bgPoint2d>;

bool intersection(const std::list<Point2d>& polygon1, const std::list<Point2d>& polygon2, std::list<Point2d>& output)
{
  std::deque<bgPoint2d> tmp_pol1, tmp_pol2;
  std::transform(polygon1.begin(), polygon1.end(), std::back_inserter(tmp_pol1),
                 [](const Point2d& pt) -> bgPoint2d { return bgPoint2d(pt.x, pt.y); });
  std::transform(polygon2.begin(), polygon2.end(), std::back_inserter(tmp_pol2),
                 [](const Point2d& pt) -> bgPoint2d { return bgPoint2d(pt.x, pt.y); });

  std::deque<bgPolygon> tmp_out;
  bool intersect = boost::geometry::intersection(bgPolygon{ {tmp_pol1.begin(), tmp_pol1.end()} },
                                                 bgPolygon{ {tmp_pol2.begin(), tmp_pol2.end()} }, tmp_out);
  if (intersect)
  {
   std::transform(tmp_out.front().outer().begin(), tmp_out.front().outer().end(), std::back_inserter(output),
                  [](const bgPoint2d& pt) -> Point2d { return Point2d(pt.x(), pt.y()); });
  }
  return intersect;

//  int i = 0;
//  std::cout << "green && blue:" << std::endl;
//  BOOST_FOREACH (Polygon2d const& p, output)
//  {
//    std::cout << i++ << ": " << boost::geometry::area(p) << std::endl;
//  }
}

} /* namespace thorp_toolkit */
