
#include "connect.hh"
#include "shared.hh"
#include "Utils/error_handling.hh"

namespace Topo {

bool connect_entities(
  Wrap<Type::VERTEX> _from, Wrap<Type::VERTEX> _to,
  VertexChain& _using_verts,
  VertexChain& _conn,
  const size_t _min_size)
{
  _conn.push_back(_from);
  auto using_verts_backup = _using_verts;
  for (;;)
  {
    if (_conn.size() >= _min_size)
    {
      auto shrd_edges = shared_entities<Type::VERTEX, Type::EDGE>(_conn.back(), _to);
      if (!shrd_edges.empty())
      {
        _conn.push_back(_to);
        return true;
      }
    }
    bool advancing = false;
    for (auto it = _using_verts.begin(); it != _using_verts.end(); ++it)
    {
      if (*it == _conn.back())
        continue;
      auto shrd_edges = 
        shared_entities<Type::VERTEX, Type::EDGE>(_conn.back(), *it);
      if (shrd_edges.empty())
        continue;
      THROW_IF(shrd_edges.size() > 1, "Too many edges between two vertices.");
      _conn.push_back(*it);
      _using_verts.erase(it);
      advancing = true;
      break;
    }
    if (!advancing)
    {
      _using_verts = using_verts_backup;
      return false;
    }
  }
}

}//namespace Topo
