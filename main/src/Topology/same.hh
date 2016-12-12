#pragma  once

#include "Topology.hh"
#include "Iterator.hh"

namespace Topo {

bool same(Topo::Wrap<Topo::Type::FACE>& _face0,
  Wrap<Topo::Type::FACE>& _face1, bool* _reversed = nullptr);

}
