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
  virtual bool find_graph(
    double _dist,
    std::vector<Geo::VectorD3>& _pts,
    std::vector<std::array<size_t, 2>>& _inds) = 0;
  static std::shared_ptr<IGeodesic> make();
};

} // namespace Offset
