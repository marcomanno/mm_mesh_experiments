#pragma once

#include "vector.hh"
#include <vector>

namespace Geo
{
struct PointInPolygon
{
  enum Classification {Inside, Outside, On};
  static Classification classify(
    const std::vector<Geo::Vector3>& _poly,
    const Geo::Vector3& _pt,
    const double _tol);
};

}