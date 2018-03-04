
#include "area.hh"

namespace Geo
{

double area(const std::vector<std::array<Geo::VectorD3, 3>>& _tris)
{
  double a = 0.;
  for (const auto tri : _tris)
    a += area(tri);

  return a;
}

}