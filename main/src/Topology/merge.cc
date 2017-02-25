#include "merge.hh"
#include "shared.hh"


namespace Topo {

template<> bool merge<Type::VERTEX>(
  Wrap<Type::VERTEX> _a, Wrap<Type::VERTEX> _b)
{
  if (_a == _b)
    return false;
  // TODO: delete shared edges
  Geo::Point pt[2];
  _a->geom(pt[0]);
  _b->geom(pt[1]);
  Topo::Iterator<Topo::Type::VERTEX, Topo::Type::FACE> it_fa(_a);
  for (auto f : it_fa)
    f->replace_child(_a.get(), _b.get());
  _b->set_geom((pt[0] + pt[1]) / 2.);
  _b->set_tolerance(
    std::max({ Geo::length(pt[0] - pt[1]),_a->tolerance(), _b->tolerance() }));
  return true;
}


}
