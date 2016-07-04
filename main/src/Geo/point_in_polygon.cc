#pragma once

#include "point_in_polygon.hh"
#include "Geo/point_fitting.hh"
#include "Geo/vector.hh"

namespace Geo
{
PointInPolygon::Classification PointInPolygon::classify(
  const std::vector<Geo::Vector3>& _poly,
  const Geo::Vector3& _pt,
  const double& _tol)
{
  const auto tol_sq = Geo::sq(_tol);
  for (const auto& poly_pt : _poly)
  {
    if (length_square(poly_pt - _pt) < tol_sq)
      return On;
  }
  auto pl_fit = IPlaneFit::make();
  pl_fit->init(_poly.size());
  for (const auto pt : _poly)
    pl_fit->add_point(pt);
  Vector3 centr, norm;
  pl_fit->compute(centr, norm);
  auto v0 = _poly.back() - _pt;
  double angl = 0;
  for (const auto& poly_pt : _poly)
  {
    auto v1 = poly_pt - _pt;
    if (v0 * v1 < 0)
    {
      double h = 0.25 * length_square(v0 % v1) / 
        length_square(v0 + v1);
      if (h < tol_sq)
        return On;
    }
    auto curr_angl = signed_angle(v0, v1, norm);
    angl += curr_angl;
    v0 = v1;
  }
  return std::fabs(angl) > M_PI ? Inside : Outside;
}

}//namespace Geo