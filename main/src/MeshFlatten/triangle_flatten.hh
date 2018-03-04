
#pragma once

#include "Geo\vector.hh"
#include <memory>

namespace MeshFlatt
{
struct IMF
{
  virtual void add_point(const Geo::VectorD3& _pt) = 0;
  virtual void add_facet(std::array<int, 3>& _facet) = 0;
  virtual void compute() = 0;
  virtual const Geo::VectorD3& get_point(int _i) = 0;
  static std::shared_ptr<IMF> make();
};

} // namespace MeshFlatt
