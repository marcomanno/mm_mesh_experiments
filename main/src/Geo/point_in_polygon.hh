#pragma once

#include "vector.hh"
#include <vector>

namespace Geo
{
namespace PointInPolygon
{
enum Classification {Inside, Outside, On};
Classification classify(
  const std::vector<Geo::VectorD3>& _poly,
  const Geo::VectorD3& _pt,
  const double& _tol,
  const Geo::VectorD3* _norm = nullptr);

Classification classify(
  const std::vector<Geo::VectorD3>& _poly,
  const Geo::VectorD3& _pt,
  const Geo::VectorD3* _norm = nullptr);
};

}