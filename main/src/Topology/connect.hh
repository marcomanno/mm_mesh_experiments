
#pragma once

#include "iterator.hh"
#include <vector>

namespace Topo {

bool connect_entities(
  Wrap<Type::VERTEX> _from, Wrap<Type::VERTEX> _to,
  VertexChain& _using_verts,
  VertexChain& _conn);

}//namespace Topo
