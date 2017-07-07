
#pragma once

#include "iterator.hh"
#include <vector>

namespace Topo {

// Organize a set of vertices in a chain using existing edges.
bool connect_entities(
  const Wrap<Type::VERTEX> _from, const Wrap<Type::VERTEX> _to,
  VertexChain& _using_verts,
  VertexChain& _conn,
  const size_t _min_size = 0
  );

}//namespace Topo
