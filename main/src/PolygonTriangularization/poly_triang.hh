#include "geo_vector.hh"
#include <memory>
#include <vector>

struct PolygonFil
{
  virtual void init(const std::vector<Geo::Vector3>& _plgn) = 0;
  virtual const std::vector<std::array<size_t, 3>>& triangles() const = 0;
  virtual const std::vector<Geo::Vector3>& positions() const = 0;
  virtual double area_compute() const = 0;
  static std::shared_ptr<PolygonFil> make();
};//class PolygonFill