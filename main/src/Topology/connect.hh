
#pragma once

#include "iterator.hh"
#include <vector>

namespace Topo {

bool connect_entities(
  const Wrap<Type::VERTEX> _from, const Wrap<Type::VERTEX> _to,
  VertexChain& _using_verts,
  VertexChain& _conn,
  const bool _allow_direct = true
  );

}//namespace Topo
