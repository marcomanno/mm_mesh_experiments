#pragma once

#include "topology.hh"
#include <memory>

namespace Topo {

struct ISplitChain
{
  virtual void add_chain(const VertexChain _chain) = 0;
  virtual void add_connection(const Topo::Wrap<Topo::Type::VERTEX>& _v0,
                              const Topo::Wrap<Topo::Type::VERTEX>& _v1,
                              bool _bidirectional = true) = 0;
  enum class ConnectionCheck {OK, EXISTING, INVALID};
  virtual ConnectionCheck check_new_connection(
    const Topo::Wrap<Topo::Type::VERTEX>& _v0,
    const Topo::Wrap<Topo::Type::VERTEX>& _v1) const = 0;
  virtual void compute() = 0;
  virtual const VertexChains& boundaries() const = 0;
  virtual const VertexChains* boundary_islands(
    size_t _bondary_ind) const = 0;
  static std::shared_ptr<ISplitChain> make();
};

} // namespace Topo