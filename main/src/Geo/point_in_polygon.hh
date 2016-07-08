#pragma once

#include "vector.hh"
#include <vector>

namespace Geo
{
namespace PointInPolygon
{
enum Classification {Inside, Outside, On};
Classification classify(
  const std::vector<Geo::Vector3>& _poly,
  const Geo::Vector3& _pt,
  const double& _tol,
  const Geo::Vector3* _norm = nullptr);

Classification classify(
  const std::vector<Geo::Vector3>& _poly,
  const Geo::Vector3& _pt,
  const Geo::Vector3* _norm = nullptr);
};

}