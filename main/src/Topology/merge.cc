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
  auto faces = shared_entities<Type::VERTEX, Type::FACE>(_a, _b);
  for (auto f : faces)
    f->replace_child(_b.get(), _a.get());
  _a->set_geom((pt[0] + pt[1]) / 2.);
  _a->set_tolerance(std::max(_a->tolerance(), _b->tolerance()));
  return true;
}


}
