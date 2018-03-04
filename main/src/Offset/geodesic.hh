#pragma once

#include "Geo/vector.hh"

#include <vector>
#include <vector>

namespace Offset
{
struct IGeodesic
{
  virtual bool compute() = 0;
  virtual bool find_points(double _dist, std::vector<Geo::VectorD>);
};

} // namespace Offset
