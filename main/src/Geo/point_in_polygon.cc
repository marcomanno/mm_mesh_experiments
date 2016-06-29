#pragma once

#include "point_in_polygon.hh"

namespace Geo
{
PointInPolygon::Classification PointInPolygon::classify(
    const std::vector<Geo::Vector3>& _poly,
    const Geo::Vector3& _pt,
    const double _tol)
{
  auto v0 = _poly.back() - _pt;
  double angl = 0;
  for (const auto& poly_pt : _poly)
  {
    auto v1 = poly_pt - _pt;
    angl += angle(v0, v1);
    v0 = v1;
  }
  return angl > 2 * M_PI ? Inside : Outside;
}

}//namespace Geo