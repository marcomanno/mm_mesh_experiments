#include "Geo/vector.hh"
#include <memory>
#include <vector>

struct PolygonFil
{
  virtual void add(const std::vector<Geo::Vector3>& _plgn) = 0;
  virtual const std::vector<std::array<size_t, 3>>& triangles() = 0;
  virtual double area() = 0;
  virtual const std::vector<Geo::Vector3>& polygon() = 0;
  static std::shared_ptr<PolygonFil> make();
};//class PolygonFill