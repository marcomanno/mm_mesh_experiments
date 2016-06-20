#include "geo_vector.hh"
#include <memory>
#include <vector>

struct PolygonFil
{
  void init(const std::vector<Geo::Vector3>& _plgn) = 0;
  const std::vevtor<std::array<size_t, 3>>& triangles() const = 0;
  const std::vevtor<Geo::Vector3>& positions() const = 0;
  static std::shared_ptr<PolygonFil> make();
};//class PolygonFill