#pragma once

#include "Geo/vector.hh"
#include "Topology/topology.hh"


#include <vector>
#include <vector>

namespace Offset
{
struct IGeodesic
{
  virtual bool compute(const Topo::Wrap<Topo::Type::VERTEX>& _v) = 0;
  virtual bool find_points(
    double _dist,
    std::vector<std::vector<Geo::VectorD3>>& loops) = 0;
  static std::shared_ptr<IGeodesic> make();
};

} // namespace Offset
