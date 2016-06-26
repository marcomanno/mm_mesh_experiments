
#include "area.hh"

namespace Geo
{

double area_compute(const std::vector<std::array<Geo::Vector3, 3>>& _tris)
{
  double area = 0.;
  for (const auto tri : _tris)
    area += area_compute(tri);

  return area;
}

}